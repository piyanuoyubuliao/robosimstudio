import json
from datetime import UTC, datetime
from pathlib import Path
from types import MethodType

import numpy as np

from robosimstudio_studio.panels import Panel

ROOT = Path(__file__).resolve().parent


class PanelTeaching(Panel):
    def __init__(self, session):
        super().__init__(session)
        self.host = session.host
        self.sim = self.host.sim
        self.active = False
        self.needs_reset = False
        self.closed = False
        self.document = None
        self.path = None
        self.drivers = {}
        for arm in session.arms.values():
            original_drive = arm._drive

            def drive_hand(panel, env, original_drive=original_drive):
                if self.active and panel.name in self.drivers:
                    self.drivers[panel.name].drive(env)
                else:
                    original_drive(env)

            arm._drive = MethodType(drive_hand, arm)

    def build(self, gui):
        with self.session.host_folder:
            with gui.add_folder("轨迹示教", expand_by_default=True):
                gui.add_markdown(
                    "停止原例程后，拖动双臂末端坐标轴，并用 Grip 调整夹爪。逐点记录，结束后交给 agent 重写轨迹。"
                )
                self.takeover_button = gui.add_button("停止并接管")
                self.label = gui.add_text("关键点名称", initial_value="关键点 1", disabled=True)
                self.capture_button = gui.add_button("记录关键点", disabled=True)
                self.finish_button = gui.add_button("结束示教", disabled=True)
                self.message = gui.add_markdown("等待接管。接管只终止自动轨迹，物理仿真继续运行。")
        self.takeover_button.on_click(lambda _: self._request("takeover"))
        self.capture_button.on_click(lambda _: self._request("capture"))
        self.finish_button.on_click(lambda _: self._request("finish"))

    def _request(self, action):
        self.host.bus.call_starting_within(30, self._invoke, action)

    def _invoke(self, action):
        if self.closed:
            return
        try:
            return getattr(self, action)()
        except Exception as error:
            self.message.content = f"操作未完成：{error}"
            return None

    def _buttons(self):
        self.takeover_button.disabled = self.active or self.needs_reset
        self.capture_button.disabled = not self.active
        self.finish_button.disabled = not self.active
        self.label.disabled = not self.active
        button = self.session.buttons.get("routine")
        if button is not None:
            button.disabled = self.needs_reset

    def snapshot(self):
        from motion import mailer_state

        self.sim.sync()
        joints = self.env.state.joint_q.numpy()
        targets = self.env.control.joint_target_q.numpy()
        arms = {}
        for name, panel in self.session.arms.items():
            position, quaternion = panel.ik.ee_pose(self.env.state)
            gripper = panel.gripper
            arms[name] = {
                "tcp_position": np.asarray(position).tolist(),
                "tcp_quaternion_xyzw": np.asarray(quaternion).tolist(),
                "joint_coordinates": list(panel.ik.arm_coords),
                "joint_positions": joints[panel.ik.arm_coords].tolist(),
                "joint_targets": targets[panel.ik.arm_coords].tolist(),
                "gizmo_position": np.asarray(panel.cursor_pos).tolist(),
                "gizmo_quaternion_xyzw": np.asarray(panel.cursor_quat).tolist(),
                "position_error_m": float(np.linalg.norm(np.asarray(position) - panel.cursor_pos)),
                "grip_command": float(gripper.command),
                "grip_requested": float(
                    panel.grip_request if panel.grip_request is not None else panel.gripper_command
                ),
                "gripper_joint_positions": joints[gripper.coords].tolist(),
                "gripper_opening_m": float(gripper.width(self.env.state)),
            }
        return {
            "step": int(self.sim.step_count),
            "time_s": float(self.sim.time),
            "routine_progress": dict(getattr(self.sim, "_teaching_progress", {})),
            "arms": arms,
            "parcel": mailer_state(self.sim),
        }

    def _save(self):
        payload = json.dumps(self.document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        for destination in (self.path, self.path.parent / "latest.json"):
            temporary = destination.with_suffix(".tmp")
            temporary.write_text(payload, encoding="utf-8")
            temporary.replace(destination)

    def takeover(self):
        from teaching_motion import TeachingArm

        if self.active or self.needs_reset:
            return
        scripts = [
            {
                key: value
                for key, value in self.host.script_meta.get(id(script), {}).items()
                if key in ("name", "serial", "steps")
            }
            for script in self.host.script
        ]
        self.session.set_paused(True)
        self.host.stop_scripts("manual teaching takeover")
        self.active = self.needs_reset = True
        self._buttons()
        for panel in self.session.arms.values():
            panel.ik.hold_into(self.env.state.joint_q, self.env.control.joint_target_q)
            panel.gripper_command = float(panel.gripper.command)
            panel.resume()
            self.drivers[panel.name] = TeachingArm(panel)
        stamp = datetime.now(UTC)
        self.path = ROOT / "assets" / "teaching" / f"{stamp.strftime('%Y%m%dT%H%M%S_%fZ')}.json"
        self.document = {
            "schema_version": 1,
            "purpose": "replace_routine_from_takeover",
            "status": "collecting",
            "created_at": stamp.isoformat(),
            "frame": "world",
            "units": {
                "position": "metres",
                "joint_angles": "radians",
                "quaternion": "xyzw",
                "grip": "0=open, 1=closed",
            },
            "source": {
                "scene": str(self.host.path),
                "scene_hash": self.host.scene_id,
                "generation": self.host.generation,
                "scripts": scripts,
            },
            "takeover": self.snapshot(),
            "keyframes": [],
        }
        self._save()
        self.session.set_paused(False)
        self.label.value = "关键点 1"
        self.message.content = "已终止原例程。拖动末端坐标轴，调整夹爪，填写名称后记录关键点。"

    def capture(self):
        if not self.active:
            return
        point = self.snapshot()
        point["name"] = self.label.value.strip() or f"关键点 {len(self.document['keyframes']) + 1}"
        self.document["keyframes"].append(point)
        try:
            self._save()
        except Exception:
            self.document["keyframes"].pop()
            raise
        count = len(self.document["keyframes"])
        self.label.value = f"关键点 {count + 1}"
        gap = max(arm["position_error_m"] for arm in point["arms"].values()) * 1000
        self.message.content = f"已保存 {count} 个关键点：{point['name']}。机械臂距拖拽目标最大 {gap:.1f} mm。"
        return point

    def finish(self):
        if not self.active:
            return
        self.session.set_paused(True)
        self.document["end"] = self.snapshot()
        self.document["status"] = "completed"
        self._save()
        self.active = False
        for panel in self.session.arms.values():
            panel.suspend()
        self._buttons()
        count = len(self.document["keyframes"])
        self.message.content = (
            f"示教已结束，保存 {count} 个关键点。场景已暂停。\n\n"
            f"文件：`assets/teaching/{self.path.name}`\n\n"
            "请让 agent 读取 `assets/teaching/latest.json` 重写后续轨迹，再 Reset 后从头验证。"
        )
        return str(self.path)

    def prepare_routine(self):
        if self.needs_reset:
            raise RuntimeError("接管已终止原轨迹。请先结束示教，修改轨迹并 Reset，再从头运行。")
        for panel in self.session.arms.values():
            panel.suspend()

    def on_reset(self, observation, info):
        if self.active and self.document is not None:
            self.document["status"] = "interrupted_by_reset"
            self._save()
        self.active = self.needs_reset = False
        self.sim._teaching_progress = {}
        self._buttons()
        self.message.content = "场景已重置，可以从头运行 routine 后接管。已保存的示教文件保留。"

    def close(self):
        self.closed = True
        if self.active and self.document is not None:
            self.document["status"] = "interrupted_by_reload_or_shutdown"
            self._save()
        self.active = False
