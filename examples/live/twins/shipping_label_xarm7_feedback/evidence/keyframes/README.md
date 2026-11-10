# 分拣过程关键帧

[打开关键帧总览](storyboard.png)。按从左到右、从上到下阅读；每张图右上角为同一时刻的俯视局部。
主图从工作台前方看向机器人，因此右臂位于画面左侧，左臂位于画面右侧。

| 帧 | 阶段 | 图像 |
| --- | --- | --- |
| 01 | 初始观测，标签朝下 | [单帧](01_initial.png) |
| 02 | 右臂交接，中心进入左臂抓取范围 | [单帧](02_handoff.png) |
| 03 | 左臂夹持长边中点 | [单帧](03_pinch.png) |
| 04 | 提起状态校验通过 | [单帧](04_lift.png) |
| 05 | 翻转中途，包裹接近竖直 | [单帧](05_turn.png) |
| 06 | 放回工作台，正面校验通过 | [单帧](06_face_up.png) |
| 07 | 左臂继续分段推进入带 | [单帧](07_belt_push.png) |
| 08 | 正面朝上，稳定入带完成 | [单帧](08_stable.png) |

图像来自同一轮独立物理仿真，以原有 GaP Policy 逐步执行；额外操作仅为读取状态与渲染。
包裹由夹爪接触搬运，没有通过坐标系拖拽或写入包裹状态制造姿态。
固定的两个相机使用 RTX 渲染；局部图仅裁剪同一时刻的俯视图。
`*_perspective.png` 和 `*_detail.png` 是未叠加文字、未裁剪的原始渲染。
`manifest.json` 保存帧时间、粒子观测、相机参数和来源哈希；`result.json` 与 `trace/` 保存此轮执行结果。

适用范围为单件 240 × 320 × 30 mm 平放包裹，此轮初始 xy = (0.08, 0.10) m、yaw = 0°、标签朝下。
接收带是静止平面，完成表示包裹稳定落在带面上，不表示已随运行传送带输送。
该演示独立于多位姿验收统计；此前两次受人工干扰的回合仍按 `../exclusions.json` 排除。

从任务目录重新采集到新目录（需要已有 live 实例）：

```bash
./ctl --run-dir run/validation_live eval 'from keyframes import capture; capture(page_sim=sim, destination="run/keyframes_new")' --wait --wait-limit 7200
```

仅重新拼图，无需运行物理仿真：

```bash
../../../../.venv/bin/python keyframes.py
```
