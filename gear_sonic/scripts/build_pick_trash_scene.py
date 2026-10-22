#!/usr/bin/env python3
"""生成 / 校验 / 渲染 "G1 拿瓶子 → 踩踏板 → 丢进垃圾桶" 的 MuJoCo 场景。

场景 = 机器人(G1 29dof + 灵巧手) + 0.8 m 实木桌 + 农夫山泉 550 mL 空瓶 + 12 L 脚踏垃圾桶。

用法（在仓库根目录）::

    python gear_sonic/scripts/build_pick_trash_scene.py build          # 生成场景 XML
    python gear_sonic/scripts/build_pick_trash_scene.py check          # 物理/几何自检
    python gear_sonic/scripts/build_pick_trash_scene.py render --out-dir /tmp/preview
    python gear_sonic/scripts/build_pick_trash_scene.py all            # build + check + render

布局参数在 ``gear_sonic/scripts/pick_trash_scene.yaml``，改完重跑 ``build`` 即可，
也可以用 ``--set table.center=0.70,0.26`` 临时覆盖。

为什么资产放在 ``.../g1/meshes/pick_trash/`` 下面：MuJoCo 处理 ``<include>`` 时会把被包含
文件里的 ``<compiler meshdir="meshes"/>`` 当成全局设置，于是主文件里声明的 mesh 路径也会
被强制加上 ``meshes/`` 前缀。把资产放进 ``meshes/`` 里、用 ``pick_trash/...`` 引用，是在
不改动机器人 XML 的前提下唯一稳的做法（已实测）。
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
G1_DIR = REPO_ROOT / "gear_sonic/data/robot_model/model_data/g1"
ASSETS = G1_DIR / "meshes/pick_trash"  # 相对 meshes/ 的路径前缀见 ASSET_PREFIX
ASSET_PREFIX = "pick_trash"
DEFAULT_CONFIG = REPO_ROOT / "gear_sonic/scripts/pick_trash_scene.yaml"

# 原始资产包里的 XML（含完整 body/mesh/material 定义），生成时按需裁剪
SRC = {
    "table": ASSETS / "table/asset_source.xml",
    "bottle": ASSETS / "bottle/asset_source.xml",
    "bin": ASSETS / "bin/asset_source.xml",
}

INSTALL_HINT = """\
资产目录不完整：{missing}

这些 mesh/贴图不在 git 里（.gitignore 忽略了 data/ 下的 robot_model），
需要从原始资产包拷一次（在仓库根目录执行）：

  D=~/下载
  G=gear_sonic/data/robot_model/model_data/g1/meshes/pick_trash
  mkdir -p $G/{{bottle,bin/visual,bin/collision,table}}
  cp $D/nongfu_spring_550ml_empty_mujoco_isaaclab/nongfu_550_empty/nongfu_550_empty.xml $G/bottle/asset_source.xml
  cp $D/nongfu_spring_550ml_empty_mujoco_isaaclab/nongfu_550_empty/meshes/* $G/bottle/
  cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/mujoco/pedal_bin_12l.xml" $G/bin/asset_source.xml
  cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/meshes/visual/"*.obj $G/bin/visual/
  cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/meshes/collision/"*.stl $G/bin/collision/
  cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/textures/front_label.png" $G/bin/
  cp "$D/table_0p8m_simulation_package(1)/table_0p8m/table_mujoco.xml" $G/table/asset_source.xml
"""


def _preflight_assets() -> None:
    missing = [str(p.relative_to(REPO_ROOT)) for p in SRC.values() if not p.exists()]
    if missing:
        raise SystemExit(INSTALL_HINT.format(missing="\n  ".join(missing)))


# --------------------------------------------------------------------------- #
# 配置
# --------------------------------------------------------------------------- #
def _load_yaml(path: Path) -> dict[str, Any]:
    import yaml

    with open(path) as f:
        return yaml.safe_load(f)


def _deep_update(dst: dict, src: dict) -> dict:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_update(dst[k], v)
        else:
            dst[k] = v
    return dst


def _apply_set_overrides(cfg: dict, items: list[str] | None) -> dict:
    """--set table.center=0.70,0.26 之类的临时覆盖。"""
    import yaml

    for item in items or []:
        if "=" not in item:
            raise SystemExit(f"--set 需要 key=value 形式，收到: {item}")
        key, raw = item.split("=", 1)
        node = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = yaml.safe_load(raw)
    return cfg


def load_config(path: Path | str = DEFAULT_CONFIG, overrides: list[str] | None = None) -> dict:
    cfg = _load_yaml(Path(path))
    defaults = {
        "table": {"friction": [1.0, 0.02, 0.002]},
        "bottle": {
            "clearance": 0.0005,
            "solref": [0.004, 1.0],
            "solimp": [0.9, 0.95, 0.001],
            "friction": [1.0, 0.005, 0.0001],
        },
        "bin": {"pedal_lid_ratio": 4.5},
        "render": {"width": 960, "height": 720},
    }
    cfg = _deep_update(defaults, cfg)
    return _apply_set_overrides(cfg, overrides)


# --------------------------------------------------------------------------- #
# XML 片段工具
# --------------------------------------------------------------------------- #
def _fmt(v: Any) -> str:
    if isinstance(v, (list, tuple)):
        return " ".join(_fmt(x) for x in v)
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def _indent_element(el: ET.Element, indent: str) -> str:
    """把 element 序列化成带缩进的文本块（去掉源文件带来的空白 tail）。"""
    el = copy.deepcopy(el)

    def _strip_tail(node: ET.Element) -> None:
        node.tail = None
        node.text = None if (node.text or "").strip() == "" else node.text
        for child in node:
            _strip_tail(child)

    _strip_tail(el)
    ET.indent(el, space="  ")
    text = ET.tostring(el, encoding="unicode")
    return "\n".join(indent + line for line in text.splitlines() if line.strip())


def _yaw_quat(deg: float) -> str:
    if abs(deg) < 1e-9:
        return "1 0 0 0"
    h = math.radians(deg) / 2.0
    return _fmt([math.cos(h), 0.0, 0.0, math.sin(h)])


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v] if n > 0 else v


def _look_at_camera(name: str, pos, target, fovy: float) -> ET.Element:
    """生成朝 target 看的相机（MuJoCo 相机沿自身 -Z 看，X 向右，Y 向上）。"""
    fwd = _normalize([t - p for t, p in zip(target, pos)])
    right = _normalize(_cross(fwd, [0.0, 0.0, 1.0]))
    up = _cross(right, fwd)
    cam = ET.Element("camera", {
        "name": name,
        "pos": _fmt(pos),
        "xyaxes": _fmt(right + up),
        "fovy": _fmt(float(fovy)),
    })
    return cam


# --------------------------------------------------------------------------- #
# 从原始资产里抽取 asset / body
# --------------------------------------------------------------------------- #
def _mesh_path_map(obj: str) -> dict[str, str]:
    """原始 mesh 文件名 -> 场景里的 file 属性（相对 meshes/）。"""
    mapping: dict[str, str] = {}
    if obj == "bottle":
        for p in sorted((ASSETS / "bottle").glob("*.obj")):
            mapping[p.name] = f"{ASSET_PREFIX}/bottle/{p.name}"
    elif obj == "bin":
        for p in sorted((ASSETS / "bin/visual").glob("*.obj")):
            mapping[p.name] = f"{ASSET_PREFIX}/bin/visual/{p.name}"
        for p in sorted((ASSETS / "bin/collision").glob("*.stl")):
            mapping[p.name] = f"{ASSET_PREFIX}/bin/collision/{p.name}"
    return mapping


def _texture_path_map(obj: str) -> dict[str, str]:
    """原始贴图 -> 场景里的 file（相对模型目录，texturedir 为空）。"""
    out: dict[str, str] = {}
    for p in sorted((ASSETS / obj).glob("*.png")):
        out[p.name] = f"meshes/{ASSET_PREFIX}/{obj}/{p.name}"
    return out


def _collect_assets(obj: str) -> list[ET.Element]:
    root = ET.parse(SRC[obj]).getroot()
    meshes = _mesh_path_map(obj)
    textures = _texture_path_map(obj)
    out: list[ET.Element] = []
    asset_el = root.find("asset")
    if asset_el is None:
        return out
    for el in asset_el:
        el = copy.deepcopy(el)
        if el.tag == "mesh":
            name = Path(el.get("file")).name
            if name not in meshes:
                raise SystemExit(f"{obj}: mesh {name} 不在本地资产里")
            el.set("file", meshes[name])
        elif el.tag == "texture" and el.get("file"):
            name = Path(el.get("file")).name
            if name not in textures:
                raise SystemExit(f"{obj}: texture {name} 不在本地资产里")
            el.set("file", textures[name])
        out.append(el)
    return out


def _default_geom_attrs(obj: str) -> dict[str, str]:
    """原始 XML 的 <default><geom .../> —— 单独 include 会丢，这里手工补回。"""
    root = ET.parse(SRC[obj]).getroot()
    d = root.find("default/geom")
    return dict(d.attrib) if d is not None else {}


def _object_body(obj: str) -> ET.Element:
    root = ET.parse(SRC[obj]).getroot()
    bodies = root.find("worldbody").findall("body")
    if len(bodies) != 1:
        raise SystemExit(f"{obj}: 期望 worldbody 里正好一个 body，实际 {len(bodies)}")
    return copy.deepcopy(bodies[0])


def _apply_geom_defaults(el: ET.Element, attrs: dict[str, str], only_if_missing: bool = True) -> None:
    for g in el.iter("geom"):
        for k, v in attrs.items():
            if k in ("name", "class"):
                continue
            if only_if_missing and k in g.attrib:
                continue
            g.set(k, v)


def _strip_inertial(el: ET.Element) -> None:
    for parent in el.iter():
        for child in list(parent):
            if child.tag == "inertial":
                parent.remove(child)


def _build_room(room: dict) -> list[ET.Element]:
    """生成房间外壳（4 面墙 + 天花板），全部是静态 box。

    墙的内表面正好落在 size/2 上，所以房间内部净尺寸就是配置里的 size。
    """
    sx, sy = float(room["size"][0]), float(room["size"][1])
    cx, cy = float(room["center"][0]), float(room["center"][1])
    h = float(room["height"])
    t = float(room["wall_thickness"])
    geoms: list[ET.Element] = []
    walls = {
        "wall_back": [cx + sx / 2 + t / 2, cy, h / 2, t / 2, sy / 2 + t, h / 2],
        "wall_front": [cx - sx / 2 - t / 2, cy, h / 2, t / 2, sy / 2 + t, h / 2],
        "wall_left": [cx, cy + sy / 2 + t / 2, h / 2, sx / 2, t / 2, h / 2],
        "wall_right": [cx, cy - sy / 2 - t / 2, h / 2, sx / 2, t / 2, h / 2],
    }
    for name, (x, y, z, hx, hy, hz) in walls.items():
        geoms.append(ET.Element("geom", {
            "name": name, "type": "box",
            "pos": _fmt([x, y, z]), "size": _fmt([hx, hy, hz]),
            "material": "wall_mat",
        }))
    if room.get("ceiling", True):
        geoms.append(ET.Element("geom", {
            "name": "ceiling", "type": "box",
            "pos": _fmt([cx, cy, h + t / 2]), "size": _fmt([sx / 2 + t, sy / 2 + t, t / 2]),
            "material": "wall_mat", "contype": "0", "conaffinity": "0",
        }))
    return geoms


# --------------------------------------------------------------------------- #
# 组装场景
# --------------------------------------------------------------------------- #
def build_scene_xml(cfg: dict) -> str:
    _preflight_assets()

    scene_cfg = cfg["scene"]
    table_cfg, bottle_cfg, bin_cfg = cfg["table"], cfg["bottle"], cfg["bin"]

    table_top_z = float(table_cfg["height"])
    table_root_z = table_top_z - 0.80  # 原始桌子 XML 里桌面顶面在 z=0.802（root 0.002 + 0.785 + 0.015）

    # ---- 桌子：静态（去掉 freejoint/inertial） ----
    table_body = _object_body("table")
    for j in list(table_body.findall("freejoint")):
        table_body.remove(j)
    _strip_inertial(table_body)
    table_body.set("name", "table")
    table_body.set("pos", _fmt([table_cfg["center"][0], table_cfg["center"][1], table_root_z]))
    if table_cfg.get("yaw_deg"):
        table_body.set("quat", _yaw_quat(table_cfg["yaw_deg"]))
    _apply_geom_defaults(table_body, _default_geom_attrs("table"))
    for g in table_body.iter("geom"):
        g.set("friction", _fmt(table_cfg["friction"]))

    # ---- 瓶子：自由落体，放在桌面上 ----
    bottle_body = _object_body("bottle")
    bottle_body.set(
        "pos",
        _fmt([
            bottle_cfg["pos"][0],
            bottle_cfg["pos"][1],
            table_top_z + float(bottle_cfg["clearance"]),
        ]),
    )
    if bottle_cfg.get("yaw_deg"):
        bottle_body.set("quat", _yaw_quat(bottle_cfg["yaw_deg"]))
    _apply_geom_defaults(bottle_body, {"condim": "4"})
    for g in bottle_body.iter("geom"):
        if g.get("contype") == "0" and g.get("conaffinity") == "0":
            continue  # 纯视觉 mesh
        g.set("solref", _fmt(bottle_cfg["solref"]))
        g.set("solimp", _fmt(bottle_cfg["solimp"]))
        g.set("friction", _fmt(bottle_cfg["friction"]))

    # ---- 垃圾桶：底座静态，踏板/盖子是铰链 ----
    bin_body = _object_body("bin")
    bin_body.set("pos", _fmt([bin_cfg["center"][0], bin_cfg["center"][1], 0.0]))
    if bin_cfg.get("yaw_deg"):
        bin_body.set("quat", _yaw_quat(bin_cfg["yaw_deg"]))
    _apply_geom_defaults(bin_body, _default_geom_attrs("bin"))
    for g in bin_body.iter("geom"):
        g.set("group", g.get("group", "3"))
    pedal_joint = bin_body.find(".//joint[@name='pedal_joint']")
    if pedal_joint is not None:
        pedal_joint.set("damping", _fmt(float(bin_cfg["pedal_damping"])))
        pedal_joint.set("frictionloss", _fmt(float(bin_cfg["pedal_frictionloss"])))

    # ---- asset 区 ----
    assets: list[ET.Element] = []
    for obj in ("table", "bottle", "bin"):
        assets.extend(_collect_assets(obj))

    skybox = ET.Element("texture", {
        "type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0",
        "width": "512", "height": "3072",
    })
    # 地面：房间开着时用木地板配色，关掉时退回原来的棋盘格
    room_cfg = cfg["room"]
    room_on = bool(room_cfg.get("enabled", True))
    if room_on:
        floor_tex = ET.Element("texture", {
            "type": "2d", "name": "floor_tiles", "builtin": "checker",
            "rgb1": _fmt(room_cfg["floor_rgba"][:3]),
            "rgb2": _fmt(room_cfg["floor_checker_rgba"][:3]),
            "width": "128", "height": "128",
        })
        floor_mat = ET.Element("material", {
            "name": "floor_mat", "texture": "floor_tiles", "texuniform": "true",
            "texrepeat": _fmt([room_cfg["floor_texrepeat"]] * 2), "reflectance": "0.03",
        })
    else:
        floor_tex = ET.Element("texture", {
            "type": "2d", "name": "floor_tiles", "builtin": "checker", "mark": "edge",
            "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
            "width": "300", "height": "300",
        })
        floor_mat = ET.Element("material", {
            "name": "floor_mat", "texture": "floor_tiles", "texuniform": "true",
            "texrepeat": "5 5", "reflectance": "0.2",
        })
    wall_mat = ET.Element("material", {"name": "wall_mat", "rgba": _fmt(room_cfg["wall_rgba"])})
    room_geoms = _build_room(room_cfg) if room_on else []

    # ---- body 顺序：地板/墙 → 桌子 → 瓶子 → 垃圾桶 ----
    cam_cfg = cfg["cameras"]["preview"]
    preview_cam = _look_at_camera(
        "pick_trash_preview",
        cam_cfg["pos"],
        cam_cfg["target"],
        cam_cfg.get("fovy", 45.0),
    )

    equality = ET.Element("equality")
    ET.SubElement(equality, "joint", {
        "name": "pedal_to_lid",
        "joint1": "lid_joint",
        "joint2": "pedal_joint",
        "polycoef": _fmt([0.0, -float(bin_cfg["pedal_lid_ratio"]), 0.0, 0.0, 0.0]),
        "solref": ".004 1.0",
        "solimp": ".99 .999 .0005",
    })

    parts: list[str] = []
    parts.append(f'<mujoco model="{scene_cfg["model_name"]}">')
    parts.append('  <!-- ============================================================')
    parts.append('       由 gear_sonic/scripts/build_pick_trash_scene.py 生成，不要手改。')
    parts.append('       改布局请编辑 gear_sonic/scripts/pick_trash_scene.yaml 后重新 build。')
    parts.append('       ============================================================ -->')
    parts.append(f'  <include file="{scene_cfg["robot_include"]}"/>')
    parts.append('  <statistic center="0.6 0 0.6" extent="2.5"/>')
    parts.append('')
    render_cfg = cfg["render"]
    fb_w = max(int(render_cfg["width"]), 1280)
    fb_h = max(int(render_cfg["height"]), 1024)
    parts.append('  <visual>')
    parts.append('    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0 0 0"/>')
    parts.append('    <rgba haze="0.15 0.25 0.35 1"/>')
    parts.append(f'    <global azimuth="-130" elevation="-20" offwidth="{fb_w}" offheight="{fb_h}"/>')
    parts.append('    <quality shadowsize="4096"/>')
    parts.append('  </visual>')
    parts.append('')
    parts.append('  <asset>')
    for el in [skybox, floor_tex, floor_mat, wall_mat, *assets]:
        parts.append(_indent_element(el, '    '))
    parts.append('  </asset>')
    parts.append('')
    parts.append('  <worldbody>')
    parts.append('    <light pos="0.5 0.2 2.55" dir="0 0 -1" directional="true" diffuse="0.55 0.55 0.55"/>')
    parts.append('    <light pos="1.4 -1.1 2.3" dir="-0.4 0.35 -1" diffuse="0.35 0.35 0.35"/>')
    parts.append('    <geom name="floor" type="plane" size="0 0 0.05" material="floor_mat"/>')
    if room_geoms:
        parts.append('')
        parts.append('    <!-- 房间外壳（静态墙 + 天花板；天花板不参与碰撞） -->')
        for g in room_geoms:
            parts.append(_indent_element(g, '    '))
    parts.append('')
    parts.append('    <!-- 0.8 m 实木桌（静态） -->')
    parts.append(_indent_element(table_body, '    '))
    parts.append('')
    parts.append('    <!-- 农夫山泉 550 mL 空瓶（21.9 g） -->')
    parts.append(_indent_element(bottle_body, '    '))
    parts.append('')
    parts.append('    <!-- 12 L 脚踏垃圾桶：踩 pedal_joint，盖子按 ratio 联动打开 -->')
    parts.append(_indent_element(bin_body, '    '))
    parts.append('')
    parts.append('    <!-- 预览相机（数据采集用的是机器人自带的 head_camera） -->')
    parts.append(_indent_element(preview_cam, '    '))
    parts.append('  </worldbody>')
    parts.append('')
    parts.append(_indent_element(equality, '  '))
    parts.append('</mujoco>')
    parts.append('')
    return "\n".join(parts)


def write_scene(cfg: dict, out_path: Path | None = None) -> Path:
    out = Path(out_path) if out_path else REPO_ROOT / cfg["scene"]["out"]
    out.parent.mkdir(parents=True, exist_ok=True)
    text = build_scene_xml(cfg)
    out.write_text(text, encoding="utf-8")
    return out


# --------------------------------------------------------------------------- #
# 自检
# --------------------------------------------------------------------------- #
def _import_mujoco():
    os.environ.setdefault("MUJOCO_GL", "glfw")
    import mujoco  # noqa: WPS433

    return mujoco


def _props_only_xml(scene_path: Path) -> Path:
    """去掉机器人 include 的版本，用来单独验证桌子/瓶子/垃圾桶的静置行为。

    注意：meshdir="meshes" 是随机器人 include 一起进来的全局设置，删掉 include 之后
    资产路径会解析错，所以这里显式补一条 <compiler>。
    """
    text = scene_path.read_text(encoding="utf-8")
    lines = [ln for ln in text.splitlines() if "<include" not in ln]
    lines.insert(1, '  <compiler angle="radian" meshdir="meshes"/>')
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".xml", dir=scene_path.parent, delete=False, encoding="utf-8"
    )
    tmp.write("\n".join(lines))
    tmp.close()
    return Path(tmp.name)


def check_scene(cfg: dict, scene_path: Path) -> dict:
    mujoco = _import_mujoco()
    import numpy as np

    report: dict[str, Any] = {}
    print(f"=== 场景: {scene_path.relative_to(REPO_ROOT)} ===")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(f"  nq={model.nq} nv={model.nv} nu={model.nu} ngeom={model.ngeom} "
          f"nmesh={model.nmesh} nbody={model.nbody}")

    actuators = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    extra = [a for a in actuators if a in ("pedal_step_torque",)]
    report["nu"] = model.nu
    report["actuators_ok"] = not extra
    print(f"  执行器 {model.nu} 个{'（无多余踏板电机 ✓）' if not extra else f'（多余: {extra} ✗）'}")

    # 关节
    joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    for name in ("floating_base_joint", "floating_bottle", "pedal_joint", "lid_joint"):
        print(f"  关节 {name:22s} {'✓' if name in joints else '✗ 缺失'}")
    report["joints_ok"] = all(n in joints for n in
                              ("floating_base_joint", "floating_bottle", "pedal_joint", "lid_joint"))

    # 关键尺寸
    bottle_z = data.body("pet_bottle").xpos[2]
    table_top = data.geom("tabletop").xpos[2] + model.geom("tabletop").size[2]
    pedal_xy = data.body("pedal_link").xpos
    print(f"  桌面上表面 z={table_top:.3f} m，瓶底 z={bottle_z:.3f} m")
    print(f"  踏板铰链 (x,y,z)=({pedal_xy[0]:.3f}, {pedal_xy[1]:.3f}, {pedal_xy[2]:.3f})")
    report["table_top"] = float(table_top)
    report["bottle_z"] = float(bottle_z)
    report["bottle_on_table"] = abs(bottle_z - table_top) < 0.005

    # 可达性：肩到瓶、髋到踏板
    def pos(name: str) -> np.ndarray:
        return np.array(data.body(name).xpos)

    bottle_p = pos("pet_bottle")
    reach = 0.0
    for a, b in zip(
        ["right_shoulder_pitch_link", "right_elbow_link", "right_wrist_yaw_link"],
        ["right_elbow_link", "right_wrist_yaw_link", "right_hand_middle_1_link"],
    ):
        reach += float(np.linalg.norm(pos(b) - pos(a)))
    for hand, shoulder in (("right", "right_shoulder_pitch_link"), ("left", "left_shoulder_pitch_link")):
        d = float(np.linalg.norm(bottle_p - pos(shoulder)))
        ratio = d / reach * 100 if reach else 0
        print(f"  {shoulder:26s} → 瓶子 {d:.3f} m = 手臂可达 {ratio:.0f}% (最大 {reach:.3f} m)")
    report["arm_reach_m"] = reach
    for hip, foot, target in (("right_hip_pitch_link", "right_ankle_roll_link", pedal_xy),
                              ("left_hip_pitch_link", "left_ankle_roll_link", pedal_xy)):
        d = float(np.linalg.norm(np.array(target) - pos(foot)))
        print(f"  {foot:26s} → 踏板铰链 直线距离 {d:.3f} m  (足部需前伸踩下)")

    # 干涉检查：桌腿/桌面对瓶子和垃圾桶
    table_x = data.geom("tabletop").xpos
    top_half = model.geom("tabletop").size
    in_table = (
        abs(bottle_p[0] - table_x[0]) < top_half[0]
        and abs(bottle_p[1] - table_x[1]) < top_half[1]
    )
    print(f"  瓶子在桌面范围内: {'✓' if in_table else '✗ 掉出桌沿'}")
    report["bottle_in_tabletop"] = bool(in_table)

    bin_p = pos("base_link")
    gap = float(np.hypot(bottle_p[0] - bin_p[0], bottle_p[1] - bin_p[1]))
    print(f"  垃圾桶中心 ({bin_p[0]:.3f}, {bin_p[1]:.3f})，与瓶子平面距离 {gap:.3f} m")
    report["bottle_bin_distance"] = gap

    # ---- 静置物理：去掉机器人，跑 3 s 看道具会不会自己动 ----
    props_path = _props_only_xml(scene_path)
    try:
        pm = mujoco.MjModel.from_xml_path(str(props_path))
        pd = mujoco.MjData(pm)
        mujoco.mj_forward(pm, pd)
        bottle_q0 = pd.qpos[0:7].copy()
        for _ in range(int(3.0 / pm.opt.timestep)):
            mujoco.mj_step(pm, pd)
        bottle_q1 = pd.qpos[0:7].copy()
        drift = float(np.linalg.norm(bottle_q1[:3] - bottle_q0[:3]))
        tilt = float(2 * math.degrees(math.acos(min(1.0, abs(bottle_q1[3])))))
        jnames = [mujoco.mj_id2name(pm, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(pm.njnt)]
        pedal_q = float(pd.qpos[pm.jnt_qposadr[jnames.index("pedal_joint")]])
        lid_q = float(pd.qpos[pm.jnt_qposadr[jnames.index("lid_joint")]])
        max_vel = float(np.abs(pd.qvel).max())
        print(f"  静置 3 s：瓶子漂移 {drift * 1000:.2f} mm，倾角 {tilt:.2f}°，"
              f"踏板 {pedal_q:+.3f} rad，盖子 {lid_q:+.3f} rad，最大速度 {max_vel:.3f}")
        ok = drift < 0.01 and tilt < 3.0 and max_vel < 1.0
        print(f"  道具静置稳定: {'✓' if ok else '✗'}")
        report.update({
            "props_drift_mm": drift * 1000,
            "props_tilt_deg": tilt,
            "props_max_vel": max_vel,
            "props_stable": bool(ok),
        })
    finally:
        props_path.unlink(missing_ok=True)

    report["ok"] = bool(
        report["actuators_ok"]
        and report["joints_ok"]
        and report["bottle_on_table"]
        and report["props_stable"]
    )
    print(f"=== 结论: {'通过 ✓' if report['ok'] else '有问题 ✗'} ===")
    return report


# --------------------------------------------------------------------------- #
# 渲染
# --------------------------------------------------------------------------- #
def render_scene(cfg: dict, scene_path: Path, out_dir: Path) -> list[Path]:
    mujoco = _import_mujoco()
    import numpy as np
    from PIL import Image

    out_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    w = int(cfg["render"]["width"])
    h = int(cfg["render"]["height"])
    renderer = mujoco.Renderer(model, h, w)
    saved: list[Path] = []
    for cam, fname in (("pick_trash_preview", "scene_preview.png"), ("head_camera", "ego_view.png")):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
            print(f"  [跳过] 没有相机 {cam}")
            continue
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        p = out_dir / fname
        Image.fromarray(np.asarray(img)).save(p)
        saved.append(p)
        print(f"  已保存 {p}")

    # 一个“踩下踏板”的姿态截图，方便确认盖子确实打开
    jnames = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    ratio = float(cfg["bin"]["pedal_lid_ratio"])
    pedal_q = -0.245
    data.qpos[model.jnt_qposadr[jnames.index("pedal_joint")]] = pedal_q
    # 盖子和踏板由 equality 联动，mj_forward 不会解约束，这里手动摆到位
    data.qpos[model.jnt_qposadr[jnames.index("lid_joint")]] = -ratio * pedal_q
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera="pick_trash_preview")
    img = renderer.render()
    p = out_dir / "scene_pedal_pressed.png"
    Image.fromarray(np.asarray(img)).save(p)
    saved.append(p)
    print(f"  已保存 {p}（踏板 {pedal_q:+.2f} rad -> 盖子 {-ratio * pedal_q:+.2f} rad）")
    renderer.close()
    return saved


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs="?", default="all", choices=["build", "check", "render", "all"])
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--set", dest="overrides", action="append", help="临时覆盖，如 table.center=0.70,0.26")
    ap.add_argument("--out", default=None, help="场景 XML 输出路径（默认读配置）")
    ap.add_argument("--out-dir", default="/tmp/pick_trash_preview", help="渲染输出目录")
    args = ap.parse_args(argv)

    cfg = load_config(args.config, args.overrides)
    scene_path = Path(args.out) if args.out else REPO_ROOT / cfg["scene"]["out"]

    if args.command in ("build", "all"):
        p = write_scene(cfg, scene_path)
        print(f"[build] 已生成 {p.relative_to(REPO_ROOT) if p.is_relative_to(REPO_ROOT) else p}")
    if args.command in ("check", "all"):
        report = check_scene(cfg, scene_path)
        if args.command == "check" and not report["ok"]:
            return 1
    if args.command in ("render", "all"):
        render_scene(cfg, scene_path, Path(args.out_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
