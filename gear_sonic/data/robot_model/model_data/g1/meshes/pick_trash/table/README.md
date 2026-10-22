# 800 mm 实木桌 | MuJoCo + Isaac Lab + CAD

这是 **0.8 m × 0.8 m × 0.8 m** 的工程化刚性桌子模型，包含命名的 9 个实体零件、CAD STEP 装配文件、STL/GLB 网格、MuJoCo MJCF、两版 URDF、质量与惯量、物理参数和检查脚本。

## 1. 文件清单

| 路径 | 用途 |
| --- | --- |
| `table_assembly_800mm.step` | 真正的 9 零件 STEP CAD 装配，含装配名称；**尺寸单位 mm** |
| `table_assembled.glb` | 彩色整桌可视化 3D 模型；长度单位 m |
| `meshes_m/*.stl` | 每个零件的独立网格，原点在各零件中心；**长度数值单位 m** |
| `table_mujoco.xml` | 带地面、光源及可移动桌子的完整 MuJoCo 场景 |
| `table_mujoco_asset.xml` | 无地面的 MuJoCo 桌子资产，用于合并到已有仿真环境 |
| `table_isaaclab.urdf` | 9 个 link + 8 个 fixed joint 的装配体 URDF |
| `table_isaaclab_single_link.urdf` | 优化的单 link 刚体 URDF：9 个碰撞形状 + 1 组整体惯量 |
| `parameters_physics.json` | 精确的各零件尺寸、位姿、质量、局部惯量、整体质心和惯量 |
| `preview.png` | 装配图与零件爆炸图 |
| `build_table.py` | 可复现的完整生成代码（需 cadquery、trimesh、numpy、matplotlib、lxml） |
| `validate_model.py` | 跨格式静态校验；安装 MuJoCo 后可选做 3 秒落地仿真 |
| `verification.json` | 已做与尚未做的验证项目 |

## 2. 已确定的物理设计

- 全局坐标：中心落地位置为 `(0, 0, 0)`，`+X` 向右、`+Y` 向后、`+Z` 向上。
- 总体外包框：`x = [-0.4, 0.4] m`，`y = [-0.4, 0.4] m`，`z = [0, 0.8] m`。
- 桌面：`0.8 × 0.8 × 0.03 m`，中心高度 `z=0.785 m`。
- 桌腿：4 根 `0.05 × 0.05 × 0.77 m`，腿中心 `x,y = ±0.33 m`；四脚底 `z=0`。
- 桌裙横梁：4 根，长度 `0.61 m`、厚度 `0.025 m`、高度 `0.11 m`，顶面高度 `0.77 m`；横梁端面抵住桌腿内侧，没有体积重叠。
- 实木密度：**假设** `650 kg/m³`。
- 计算总质量：**21.846500 kg**。
- 整体质心：`(0, 0, 0.6793856) m`。
- 以整体质心为原点、全局 XYZ 轴为主轴的整体惯量约为 `Ixx=2.34801516`、`Iyy=2.34801516`、`Izz=3.03381177 kg·m²`，非对角项为零（对称结构）。
- 装配连接：桌面、四腿和四根桌裙之间在动力学上视作完全刚性连接。实际木作的螺钉/胶层变形、木纹方向、柔性、老化等不纳入模型。
- MuJoCo 设置：`gravity = -9.81 m/s²`、时间步长 `0.002 s`；木材假设滑动摩擦系数 `0.68`，地面假设为 `0.90`。这是工程初值，并非表面实测值。
- 桌面承载/抗弯强度 **没有做有限元验证**，不能把刚体模型当作结构强度证明。

## 3. MuJoCo：直接加载

装好 MuJoCo Python 包后，在该目录执行：

```bash
pip install mujoco
python -m mujoco.viewer --mjcf=table_mujoco.xml
```

`table_mujoco.xml` 已带地面和可运动的 free joint 桌子。模型初始足底在 `z=0.002 m`，留 2 mm 接触沉降空间，**桌子本身高度依旧精确为 800 mm**。桌面、腿、裙板均有明确质量、惯性和可碰撞的盒体，没有用一个实心大盒子替代镂空桌子。

对于现有 MuJoCo 工作空间，参考 `table_mujoco_asset.xml` 复用桌子 body/geom；不要把带地面的完整场景直接重复插入原场景。

可做附加的仿真落地验证：

```bash
python validate_model.py
python validate_model.py --simulate
```

`--simulate` 需要安装 `mujoco`；它会编译 MJCF 并执行 3 秒物理仿真，检查数值有限、四脚接触、整体姿态与最终位置。

## 4. Isaac Lab：先 URDF，再 USD

**Isaac Lab 通常使用 USD 资产，而非直接用 `.urdf` 文件生成环境中的刚体。** 推荐两种导入方式：

**A. 普通桌子障碍物或可移动刚体：** 推荐 `table_isaaclab_single_link.urdf`。这是已经合并的单刚体，9 个碰撞盒子与整体真实质量/惯量在同一 link 中，比较适合 `RigidObject` 工作流。

**B. 需要保留桌面、腿、横梁的装配关系：** 使用 `table_isaaclab.urdf`，含 8 个 `fixed` 关节；导入时勾选 Merge Fixed Joints，合并成单个刚体。若不合并，部分物理后端可能创建额外的关节/刚体结构。

在较新 Isaac Lab 源码目录运行（命令参数以安装版本为准）：

```bash
uv run --extra importers isaaclab -p scripts/tools/convert_urdf.py \
  /ABS/PATH/table_isaaclab_single_link.urdf \
  /ABS/PATH/converted_output
```

或者导入保留装配关节的版本：

```bash
uv run --extra importers isaaclab -p scripts/tools/convert_urdf.py \
  /ABS/PATH/table_isaaclab.urdf \
  /ABS/PATH/converted_output \
  --merge_joints
```

使用较旧的 Isaac Lab / Isaac Sim 时，启动命令和开关可能是 `./isaaclab.sh`、`--merge-joints`；请对照你安装版本的 `convert_urdf.py --help`。在 GUI 中也可通过 Isaac Sim **URDF Importer** 读取并导出 USD。

随后根据实际生成的 `.usd` / `.usda` 路径，通过 `UsdFileCfg` 引用到 Isaac Lab 场景。若作为可移动的 `RigidObject`，**不要**选择 Fix Base；若将其作为永远不动的场景家具，则选择固定基座，或在 USD 中设为静态碰撞物体。

### Isaac Lab 物理摩擦

URDF 标准本身不包含跨 PhysX 和 MuJoCo 完全等价的材料接触参数。导入 USD 后应对碰撞材质检查并设置，例如：
- 静摩擦：`0.70`
- 动摩擦：`0.60`
- 弹性系数（restitution）：`0.0`
- 保留源文件质量与惯量；不要再叠加密度导致质量重复计算。
- 桌子的地面参考高度是 `z=0`；动态生成时可给 2 mm 初始间隙，例如 `pos=(0, 0, 0.002)`。

不同物理引擎的接触模型、摩擦组合规则和求解容差不同，不能假定两种引擎给出数值完全相同的滑动或撞击轨迹。

Isaac Lab 官方导入说明：
https://isaac-sim.github.io/IsaacLab/main/source/how-to/import_new_asset.html

MuJoCo 官方参考：
https://mujoco.readthedocs.io/en/latest/XMLreference.html

## 5. 单位特别提醒

**STEP 文件是毫米（mm）**：在 CAD 中查看整体尺寸为 `800 × 800 × 800 mm`。

**URDF、MJCF、GLB 和 `meshes_m/*.stl` 使用米（m）**：在仿真中尺寸为 `0.8 × 0.8 × 0.8 m`。STL 格式本身不保存单位；如果放进默认以毫米解释 STL 的 CAD 软件，需进行 ×1000 单位转换。

## 6. 可信度与验证范围

本套模型并非扫描某一实际生产型号，而是基于现实木桌常见构造生成的**参数化刚体数字样机**。

已验证：STEP 重载/实心数量、外包框尺寸、9 零件之间无穿透、GLB 边界、跨 XML/URDF 质量与惯量、固定装配关系、整体 COM、XML 语法解析、碰撞几何位置。

未验证：当前生成环境没有 `mujoco` 与 Isaac Lab/Isaac Sim，因此**尚未进行真实 MuJoCo 引擎编译/动力学落地测试，也尚未将 URDF 实际转换为 USD**。`validate_model.py --simulate` 可在用户本机 MuJoCo 环境继续检查；最终用于机器人碰撞实验前还应在目标 Isaac Lab 版本进行一次加载、接触和稳定性实验。
