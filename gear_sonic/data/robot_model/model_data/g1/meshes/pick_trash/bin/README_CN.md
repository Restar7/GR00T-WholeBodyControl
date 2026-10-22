# 12L 脚踏式垃圾桶｜MuJoCo + Isaac Lab 仿真资产

**状态：基于用户照片的外观与功能近似重建（不是原厂测绘 CAD）**

本资产以图片中的米白色圆形脚踏桶为参考，制作了：
- 可拆分识别的 **外桶、带底的空心内桶、上口包边、掀盖、后铰链、脚踏板、脚踏传动机构示意件、贴纸**；
- 独立的物理碰撞壳：外桶和内桶各 **24 段凸多面体侧壁** + 碰撞底面；
- **脚踏板转动关节**和**桶盖后铰链关节**，用物理传动约束关联，空桶可接收动态垃圾物体；
- MuJoCo **XML/MJCF**、Isaac Lab/Isaac Sim 可导入的 **URDF**，附预览图和测试脚本。

## 重要：真实性与可验证范围

本模型并非照片对应实物的尺寸测绘；**只有一张正面产品照无法读出背部铰链、连杆机械结构、塑件厚度、摩擦阻尼和真实重心**。外观、标贴与形状按照片重绘，内部传动用 **恒定比值 -4.5 的理想脚踏联动机构**简化；不是原厂四连杆逆向复刻。几何/连通/碰撞凸性已通过静态检验；**交互动力学需要在安装 MuJoCo 或 Isaac Sim/PhysX 的设备上实际运行本包附带的 demo 验收**。

## 几何与功能参数

| 参数 | 建模值 |
|---|---|
| 坐标 | Z 向上；-X 面向脚踏板；+X 指向后铰链；角度弧度 |
| 单位 | 米、千克、秒、弧度 |
| 桶体最大直径 | 约 0.284 m（桶盖约 0.290 m） |
| 桶体含盖高度 | 约 0.363 m |
| 内桶有效空腔容积 | **约 12.18 L**（截锥估算） |
| 脚踏角度 | 从 0 至 -0.245 rad（约下压 14°） |
| 开盖角度 | 从 0 至 +1.1025 rad（约 63.2°） |
| 关节传动 | `lid_joint = -4.5 * pedal_joint` |
| 内桶底部 | z 约 0.043 m，实际空心容器 |
| 可投放垃圾 | 直径 3–4 cm 的球/盒可从开启口进入并落到底部 |
| 总体质量（估计） | 1.365 kg |

### 为什么做 48 个分段侧壁碰撞？

MuJoCo 对普通三角网格的碰撞按凸包处理。若直接把整个桶体外表面设为一个 mesh collision，凸包会封住桶口、导致垃圾**掉不进去**。本包将外壁 24 段、内壁 24 段分别构建成凸几何，内桶底部另加实体碰撞盘：**外壳是空的，内桶也是空的，但容器侧壁与底板能挡住垃圾**。URDF 的 `<collision>` 同样是这些凸分段，从而避免 Isaac/PhysX 导入后的整体凸包填洞。注意，不要在导入软件里将 48 段强制重新合并为一个整体凸包。

## 文件组织

```text
foot_pedal_bin_12l/
├── mujoco/
│   ├── pedal_bin_12l.xml          # 主场景：地面+固定底座+可踩踏盖
│   ├── pedal_bin_12l_demo.xml     # 物理鞋踏板+垃圾球测试场景
│   ├── pedal_bin_12l_floating.xml # 自由移动的桶体+地面
│   └── pedal_bin_12l_asset.xml    # 无地面、可自行嵌入场景的自由桶体
├── isaaclab/
│   ├── pedal_bin_12l.urdf         # URDF：带负比值 mimic 联动
│   └── IMPORT_GUIDE_CN.md
├── meshes/
│   ├── visual/                    # OBJ + 曲面贴标 MTL
│   └── collision/                 # STL 凸网格
├── textures/front_label.png
├── scripts/
│   ├── geometry_check.py         # 不需要 MuJoCo，可离线检查
│   ├── demo_mujoco.py            # 真实接触踩踏+投垃圾+自动回落演示
│   ├── isaaclab_mimic_fallback.py
│   └── render_preview_vtk.py     # 可选，可重渲染预览图
└── preview/
    ├── comparison.png            # 关闭/脚踏开启并排对比
    ├── closed.png, open.png, interior.png
    └── closed.glb, open.glb       # 静态姿势预览；不包含物理关节
```

请保持相对目录，不要仅移动 `.xml`/`.urdf` 而不移动 `meshes` 和 `textures`。

## MuJoCo 使用说明

```bash
pip install mujoco numpy trimesh lxml
python scripts/geometry_check.py
python scripts/demo_mujoco.py --strict
python scripts/demo_mujoco.py --viewer
```

进入解压后根目录运行。`demo_mujoco.py` 使用 **运动学脚部碰撞体**向下踩真实的脚踏碰撞几何，`data.ctrl` 始终为 0；被踩动的踏板经 `<equality><joint>` 抬起桶盖，在开盖后释放直径 42mm 垃圾球，最后移开脚部检查桶盖回落。由于运行时没有 MuJoCo 可供本次会话实测，这个 demo 必须在目标设备上运行，并可能需微调接触和摩擦参数。

也可以在 MuJoCo viewer 或脚本中加载 `mujoco/pedal_bin_12l.xml`，通过模型的 `pedal_step_torque` 马达施加**负的踏板转矩**进行等效操控，**无需驱动 lid_joint**。零控制输入时依靠重力关盖。

- 主 XML 为**固定桶体**，更适合外部机器人踩踏测试；脚踏不必克服整桶的倾倒。
- 若需把整个桶也当做可移动刚体，使用 `pedal_bin_12l_floating.xml`；应设好地面摩擦和机器人足部作用力。
- 若要将桶放入既有仿真场景，自由桶体无地面版本为 `pedal_bin_12l_asset.xml`。
- 让垃圾物体从前部开启口（示例 x=-0.067, y=0）释放，避免在还没开足的桶盖上方落下。

## Isaac Lab / Isaac Sim 使用说明

**Isaac Lab 正常使用的是 USD，而不是把 URDF 当作最终场景模型直接运行。先用 URDF importer 转成 USD**，再添加到 Isaac Lab 环境。详细步骤参见 `isaaclab/IMPORT_GUIDE_CN.md`。

在 Isaac Sim 的 URDF 导入窗口：
1. 指向 `isaaclab/pedal_bin_12l.urdf`，可勾选 **Fix Base** 用于稳固的踩踏测试。
2. **Ignore Mimic 必须保持关闭/不勾选**。这样 `lid_joint` 中的 `<mimic joint="pedal_joint" multiplier="-4.5">` 才会转换为 PhysX 的 MimicJointAPI。
3. 保留**多个独立的 collision 子网格**、使用凸包或单个凸网格，而非将桶身整组网格合并成一个凸包。
4. 关闭刚体链接自身碰撞（Self collision off），防止相邻的铰链件与脚踏产生无谓碰撞。
5. 为物理踏板测试关闭主驱动的强制位置保持，或将踏板的驱动刚度设置为 0；给桶内物品开启刚体+碰撞+重力。
6. 若某导入器忽略 URDF mimic，则使用 `scripts/isaaclab_mimic_fallback.py` 的每步 lid target 更新法。**fallback 只是控制器近似，不等价于真正的物理联动约束**；优先使用原生 PhysX mimic。

## 已做的离线质量检查

- 所有 XML、URDF 文法可解析，所有相对 mesh/texture 文件存在。
- 外桶和内桶曲面均为闭合带厚度的可视化网格，开口明确。
- 所有碰撞 STL 网格既 `watertight`，又是凸体。
- 48 个环形扇段 + 2 个底面，不会把内部腔体变成实心。
- 0、1/4、1/2、3/4、满踏板行程下足踏不会穿过 z=0 地面。
- 由传动约束得到的最大开盖 63.2°；渲染展示了脚踏闭合/打开状态。
- **本环境无法安装 MuJoCo、也未运行 Isaac Sim，因此不能把静态检查说成已通过目标物理引擎的实测。**

## 调参位置

- 脚踏行程：XML `pedal_joint.range`、URDF `pedal_joint/limit`。
- 开盖范围/传动比：XML `equality/joint.polycoef` 与 URDF `lid_joint/mimic.multiplier` 须同步。
- 桶体碰撞摩擦：XML 所有 `c_inner_wall_*` / `c_outer_wall_*`，URDF 导入后 PhysX material。
- 弹性、回位速度：关节 damping/frictionloss、各刚体质量/转动惯量以及碰撞参数。
- 桶大小：目前是近似尺寸，**不要仅在某个软件里直接缩放视觉 mesh 而不缩放碰撞、关节原点、惯性**。

## 参考

- MuJoCo 文档： https://mujoco.readthedocs.io/en/stable/computation/
- MuJoCo joint equality： https://mujoco.readthedocs.io/en/latest/XMLreference.html
- Isaac Sim URDF 导入： https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_asset_importer_urdf.html
- Isaac Lab USD/URDF 资产导入： https://isaac-sim.github.io/IsaacLab/develop/source/how-to/import_new_asset.html
