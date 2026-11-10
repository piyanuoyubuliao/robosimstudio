"""Capture one unchanged policy episode through ctl eval; compose stills offline."""

# ruff: noqa: RUF001 -- Chinese captions use Chinese punctuation.

import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "evidence" / "keyframes"
STAGES = {
    "01_initial": ("初始观测", "单件平放，标签朝下"),
    "02_handoff": ("右臂推送交接", "包裹中心进入左臂抓取范围"),
    "03_pinch": ("长边中点夹持", "夹爪闭合，准备提起"),
    "04_lift": ("提起校验通过", "包裹离桌，抓取材料点保持跟踪"),
    "05_turn": ("翻转中途", "包裹接近竖直，随夹爪产生形变"),
    "06_face_up": ("正面校验通过", "释放落桌，标签朝上"),
    "07_belt_push": ("左臂推进入带", "根据当前边界继续分段推送"),
    "08_stable": ("稳定入带完成", "正面朝上，通过连续稳定性检查"),
}


def cameras():
    import robosimstudio as rss

    return [
        rss.CameraSpec(
            name="perspective",
            width=1440,
            height=960,
            eye=(-0.6, 1.3, 2.5),
            look_at=(-0.22, -0.05, 0.93),
            fov_y=44,
        ),
        rss.CameraSpec(
            name="detail",
            width=1200,
            height=900,
            eye=(-0.22, 0.10, 2.20),
            look_at=(-0.22, 0.10, 0.8),
            up=(0, 1, 0),
            fov_y=53,
        ),
    ]


class Recorder:
    def __init__(self, simulation, destination, backend):
        self.sim = simulation
        self.destination = destination
        self.cameras = cameras()
        self.renderer = simulation.make_renderer(backend, cameras=self.cameras, channels=["rgb"])
        self.renderer.render(simulation.state)
        self.frames = {}
        self.manifest = {
            "kind": "single_episode_keyframes_not_acceptance_statistics",
            "case": {"center": [0.08, 0.10], "yaw": 0.0, "face_up": False},
            "renderer": backend,
            "cameras": [asdict(camera) for camera in self.cameras],
            "sha256": {
                str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in [*sorted(ROOT.glob("*.py")), ROOT / "graph/workflow.json"]
            },
            "frames": [],
        }

    def save(self, key, policy, node, observation=None):
        if key in self.frames:
            return
        observation = observation or policy.observer.observe()[0]
        self.sim.sync()
        rendered = self.renderer.render(self.sim.state)
        files = {}
        for camera in self.cameras:
            name = f"{key}_{camera.name}.png"
            Image.fromarray(rendered[camera.name]["rgb"]).save(self.destination / name)
            files[camera.name] = name
        title, description = STAGES[key]
        self.frames[key] = {
            "key": key,
            "title": title,
            "description": description,
            "time": float(self.sim.time),
            "node": node,
            "observation": observation.record(),
            "images": files,
            "left_gripper_command": float(policy.robot.grip["left"]),
        }
        self.manifest["frames"] = [self.frames[name] for name in sorted(self.frames)]
        self.flush()
        print(f"KEYFRAME {key} t={self.sim.time:.2f}", flush=True)

    def flush(self):
        (self.destination / "manifest.json").write_text(json.dumps(self.manifest, ensure_ascii=False, indent=2) + "\n")

    def during(self, policy, node):
        if node == "transfer" and "02_handoff" not in self.frames:
            observation = policy.observer.observe()[0]
            if policy.reachable(observation):
                self.save("02_handoff", policy, node, observation)
        elif node == "grasp" and "03_pinch" not in self.frames:
            if policy.robot.grip["left"] >= policy.config.pinch - 0.001:
                self.save("03_pinch", policy, node)
        elif node == "flip" and "05_turn" not in self.frames:
            observation = policy.observer.observe()[0]
            if abs(observation.face_up) < 0.10:
                self.save("05_turn", policy, node, observation)
        elif node == "belt" and "07_belt_push" not in self.frames:
            observation = policy.observer.observe()[0]
            if 0.45 <= observation.belt_coverage < 0.95:
                self.save("07_belt_push", policy, node, observation)


def capture(page_sim=None, destination=OUTPUT, backend="rtx"):
    """Use: ./ctl --run-dir run/validation_live eval '... capture(sim)' --wait."""
    import policy as policy_module
    from cell import make_scene

    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    if (destination / "manifest.json").exists():
        raise FileExistsError(f"Keep the previous capture; choose a new destination: {destination}")
    original = policy_module.FeedbackPolicy
    simulation = make_scene().build(control_mode="joint_position")
    recorder = None
    generator = None
    try:
        recorder = Recorder(simulation, destination, backend)

        class CapturedPolicy(original):
            def dispatch(self, node):
                if node == "observe" and not recorder.frames:
                    recorder.save("01_initial", self, node)
                parent = super().dispatch(node)
                count = 0
                while True:
                    try:
                        value = next(parent)
                    except StopIteration as completed:
                        result = completed.value
                        break
                    count += 1
                    if count % 6 == 0:
                        recorder.during(self, node)
                    yield value
                if node == "verify_lift" and result["status"] == "flip":
                    recorder.save("04_lift", self, node)
                elif node == "verify_flip" and result["status"] == "belt":
                    recorder.save("06_face_up", self, node)
                elif node == "verify_belt" and result["status"] == "success":
                    recorder.save("08_stable", self, node)
                return result

        policy_module.FeedbackPolicy = CapturedPolicy
        generator = policy_module.run_graph(simulation, trace_dir=destination / "trace")
        batches = 0
        while True:
            try:
                for _ in range(24):
                    next(generator)
            except StopIteration as completed:
                result = completed.value
                break
            batches += 1
            if page_sim is not None and batches % 30 == 0:
                page_sim.step()
            yield
        (destination / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        recorder.manifest["success"] = result["success"]
        recorder.manifest["sim_seconds"] = float(simulation.time)
        recorder.flush()
        missing = set(STAGES) - set(recorder.frames)
        if not result["success"] or missing:
            raise RuntimeError(f"Capture incomplete: success={result['success']}, missing={sorted(missing)}")
        compose(destination)
        print(f"KEYFRAMES COMPLETE {destination / 'storyboard.png'}", flush=True)
        return {"success": True, "keyframes": len(recorder.frames), "output": str(destination)}
    finally:
        if generator is not None:
            generator.close()
        policy_module.FeedbackPolicy = original
        if recorder is not None:
            recorder.renderer.close()
        simulation.close()


def detail_crop(image, observation, camera):
    # Only crop the rendered pixels. No pose/state is changed to frame the parcel.
    camera_point = np.linalg.inv(camera.world_matrix) @ np.append(observation["center"], 1)
    pixel = camera.intrinsics @ camera_point[:3]
    center = pixel[:2] / pixel[2]
    width, height = 340, 340
    left = max(0, min(image.width - width, round(center[0] - width / 2)))
    top = max(0, min(image.height - height, round(center[1] - height / 2)))
    return image.crop((left, top, left + width, top + height))


def compose(destination=OUTPUT):
    """Compose already captured stills; no live process or physics is needed."""
    destination = Path(destination)
    manifest = json.loads((destination / "manifest.json").read_text())
    regular = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    bold = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

    def font(size, heavy=False):
        return ImageFont.truetype(bold if heavy else regular, size, index=2)

    background, ink, muted, accent = "#eef2f6", "#102233", "#536879", "#137e85"
    card_width, card_height, gap, margin = 1000, 785, 24, 40
    width = 2 * card_width + gap + 2 * margin
    height = 200 + 4 * card_height + 3 * gap + 100
    board = Image.new("RGB", (width, height), background)
    draw = ImageDraw.Draw(board)
    draw.text((margin, 28), "双臂 xArm7 · 包裹翻面分拣", font=font(47, True), fill=ink)
    draw.text((margin, 94), "同一轮仿真 · 8 个关键帧 · 右臂交接 → 左臂翻面 → 稳定入带", font=font(27), fill=muted)
    draw.text(
        (margin, 139), "初始 xy = (0.08, 0.10) m，yaw = 0°，标签朝下  |  图中时间为仿真时间", font=font(23), fill=muted
    )
    detail_camera = cameras()[1]
    frames = manifest["frames"]
    for index, frame in enumerate(frames):
        x = margin + (index % 2) * (card_width + gap)
        y = 200 + (index // 2) * (card_height + gap)
        card = Image.new("RGB", (card_width, card_height), "white")
        cd = ImageDraw.Draw(card)
        cd.rectangle((0, 0, 8, 78), fill=accent)
        cd.text((24, 13), f"{index + 1:02d}  {frame['title']}", font=font(31, True), fill=ink)
        stamp = f"t = {frame['time']:.1f} s"
        cd.text((card_width - 24 - cd.textlength(stamp, font=font(25)), 19), stamp, font=font(25), fill=muted)
        perspective = Image.open(destination / frame["images"]["perspective"]).convert("RGB")
        perspective = perspective.resize((card_width, 667), Image.Resampling.LANCZOS)
        card.paste(perspective, (0, 78))
        detail = Image.open(destination / frame["images"]["detail"]).convert("RGB")
        detail = detail_crop(detail, frame["observation"], detail_camera)
        detail = detail.resize((280, 280), Image.Resampling.LANCZOS)
        card.paste(detail, (704, 116))
        cd.rectangle((702, 86, 986, 398), outline="#c7d9df", width=2)
        cd.rectangle((704, 88, 984, 115), fill="#102233")
        cd.text((713, 87), "俯视局部 · 同一时刻", font=font(18), fill="white")
        # A caption on an opaque footer keeps the actual rendered state unaltered.
        cd.rectangle((0, 669, card_width, card_height), fill="white")
        cd.text((24, 681), frame["description"], font=font(25), fill=ink)
        obs = frame["observation"]
        if frame["key"] == "04_lift":
            metric = f"包裹最低点离桌 {(obs['lower'][2] - 0.78) * 1000:.0f} mm"
        elif frame["key"] in {"07_belt_push", "08_stable"}:
            metric = f"入带粒子覆盖率 {obs['belt_coverage']:.0%}"
        elif frame["key"] == "05_turn":
            metric = f"正面法向 z = {obs['face_up']:+.2f}（接近 0 为竖直）"
        else:
            metric = f"中心 xy = ({obs['center'][0]:+.3f}, {obs['center'][1]:+.3f}) m"
        cd.text((24, 728), metric, font=font(23), fill=muted)
        board.paste(card, (x, y))
        card.save(destination / f"{frame['key']}.png")
    draw.text(
        (margin, height - 68),
        "单件同尺寸平放包裹；静止接收带。此组为过程可视化，独立于多位姿验收统计。",
        font=font(24),
        fill=muted,
    )
    board.save(destination / "storyboard.png")
    return destination / "storyboard.png"


if __name__ == "__main__":
    print(compose())
