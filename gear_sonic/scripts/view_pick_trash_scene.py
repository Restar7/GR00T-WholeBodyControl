#!/usr/bin/env python3
"""在 MuJoCo 里打开「拿瓶子→踩踏板→丢垃圾桶」场景。

两种用法：

    # 1) 交互窗口（可以拖鼠标转视角、Ctrl+拖拽平移、右键拖物体）
    python gear_sonic/scripts/view_pick_trash_scene.py

    # 2) 只出一张无头渲染图（没有图形界面时用）
    python gear_sonic/scripts/view_pick_trash_scene.py --shot /tmp/scene.png

默认会让机器人以「钉住基座 + 关节 PD 保持」的姿势站着（不然自由落体会立刻倒地），
想看真实物理就加 --no-hold。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE = (
    REPO_ROOT / "gear_sonic/data/robot_model/model_data/g1/scene_pick_trash_43dof.xml"
)
WBC_YAML = REPO_ROOT / "gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml"

# 初始自由相机（跟场景里的 pick_trash_preview 差不多，但可自由拖动）
CAM = {"lookat": (0.45, -0.05, 0.72), "distance": 2.7, "azimuth": 150.0, "elevation": -18.0}


def load_wbc_gains():
    """从 wbc yaml 拿 29 个身体关节的 KP/KD 和目标角度。"""
    with open(WBC_YAML) as f:
        cfg = yaml.safe_load(f)
    return (
        [float(v) for v in cfg["MOTOR_KP"]],
        [float(v) for v in cfg["MOTOR_KD"]],
        [float(v) for v in cfg["DEFAULT_DOF_ANGLES"]],
    )


def build_hold_map(mujoco, model):
    """返回 {actuator_id: (kp, kd, q_des)}，只覆盖 29 个身体关节。"""
    kp, kd, q_des = load_wbc_gains()
    hand_joints = set()
    for i in range(len(kp), model.nu):
        jid = int(model.actuator_trnid[i, 0])
        if jid >= 0:
            hand_joints.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid))
    body_actuators = []
    for i in range(model.nu):
        jid = int(model.actuator_trnid[i, 0])
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) if jid >= 0 else None
        if jname and jname not in hand_joints:
            body_actuators.append(i)
    if len(body_actuators) != len(kp):
        print(f"[warn] 身体执行器 {len(body_actuators)} 个，wbc yaml 给了 {len(kp)} 组增益，按较短的来")
    hold = {}
    for k, aid in enumerate(body_actuators[: len(kp)]):
        hold[aid] = (kp[k], kd[k], q_des[k])
    return hold


def compute_controls(mujoco, model, data, hold):
    data.ctrl[:] = 0.0
    for aid, (kp, kd, q_des) in hold.items():
        jid = int(model.actuator_trnid[aid, 0])
        q = float(data.qpos[model.jnt_qposadr[jid]])
        qd = float(data.qvel[model.jnt_dofadr[jid]])
        tau = kp * (q_des - q) - kd * qd
        lo, hi = model.actuator_forcerange[aid]
        if hi > lo:  # 有限制时夹一下
            tau = max(lo, min(hi, tau))
        data.ctrl[aid] = tau


def pin_base(data, height):
    """把浮动基座钉在原点（相当于吊着看），否则机器人会自由落体。"""
    data.qpos[0:3] = (0.0, 0.0, height)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[0:6] = 0.0


def set_camera(cam):
    cam.type = 0  # mjCAMERA_FREE
    cam.lookat[:] = CAM["lookat"]
    cam.distance = CAM["distance"]
    cam.azimuth = CAM["azimuth"]
    cam.elevation = CAM["elevation"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default=str(DEFAULT_SCENE), help="场景 XML 路径")
    ap.add_argument("--shot", default=None, help="只渲染保存一张图，不开窗口")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=860)
    ap.add_argument("--no-hold", action="store_true", help="不钉基座、不做 PD 保持（真实物理，会倒）")
    ap.add_argument("--headless", action="store_true", help="强制无头模式（用 egl/osmesa）")
    ap.add_argument("--panels", action="store_true", help="显示 simulate 的左右控制面板（默认关）")
    ap.add_argument("--width-win", type=int, default=1600, help="窗口宽（像素）")
    ap.add_argument("--height-win", type=int, default=1000, help="窗口高（像素）")
    ap.add_argument("--azimuth", type=float, default=None, help="相机方位角（度）")
    ap.add_argument("--elevation", type=float, default=None, help="相机俯仰角（度）")
    ap.add_argument("--distance", type=float, default=None, help="相机距离（米）")
    args = ap.parse_args(argv)
    for key, val in (("azimuth", args.azimuth), ("elevation", args.elevation), ("distance", args.distance)):
        if val is not None:
            CAM[key] = float(val)

    if args.headless:
        os.environ["MUJOCO_GL"] = "egl"
    else:
        os.environ.setdefault("MUJOCO_GL", "glfw")

    import mujoco
    import numpy as np

    scene = Path(args.scene)
    if not scene.exists():
        print(f"场景不存在: {scene}")
        return 1
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    hold = build_hold_map(mujoco, model)

    base_height = float(model.body("pelvis").pos[2])
    if not args.no_hold:
        for _ in range(200):  # 先稳住 0.4 s 再显示，避免一开窗就在抖
            compute_controls(mujoco, model, data, hold)
            pin_base(data, base_height)
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    if args.shot:
        renderer = mujoco.Renderer(model, args.height, args.width)
        cam = mujoco.MjvCamera()
        set_camera(cam)
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        try:
            from PIL import Image

            Image.fromarray(np.asarray(img)).save(args.shot)
        except ImportError:
            import imageio.v2 as imageio

            imageio.imwrite(args.shot, img)
        renderer.close()
        print(f"已保存 {args.shot}")
        return 0

    import mujoco.viewer

    print("窗口已打开：鼠标左键拖动转视角 / Ctrl+左键平移 / 滚轮缩放 / 右键拖动物体 / Esc 退出")
    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=args.panels, show_right_ui=args.panels
    ) as viewer:
        set_camera(viewer.cam)
        step_dt = model.opt.timestep * 5
        while viewer.is_running():
            t0 = time.perf_counter()
            for _ in range(5):
                if not args.no_hold:
                    compute_controls(mujoco, model, data, hold)
                    pin_base(data, base_height)
                mujoco.mj_step(model, data)
            viewer.sync()
            sleep = step_dt - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)
    return 0


if __name__ == "__main__":
    sys.exit(main())
