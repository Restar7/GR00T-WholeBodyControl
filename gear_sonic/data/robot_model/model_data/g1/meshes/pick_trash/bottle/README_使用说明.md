# 农夫山泉 550 mL 红盖瓶（空瓶）— MuJoCo / Isaac Lab

> **模型性质：工程近似，不是农夫山泉官方 CAD、3D 扫描或实际样品称重结果。**
> 水质量 = **0 kg**；不创建水的网格、材质、液体粒子或填充刚体。装配包含 PET 中空瓶身、带防拆环造型的红色 HDPE 瓶盖及印刷薄膜标签。

## 1. 文件

- `nongfu_550_empty.xml` — MuJoCo MJCF：只有自由运动的空瓶，可嵌入你的场景；默认世界高度 0.26 m。
- `demo_mujoco.xml` — MuJoCo 直接预览版：含地面、灯光和自由空瓶。
- `nongfu_550_empty.urdf` — Isaac Sim / Isaac Lab 导入用 URDF，瓶身、瓶盖、标签三个 link 通过两个 fixed joint 组装。
- `meshes/bottle_hollow.obj` — 真实中空、开口在瓶颈处的 PET 壳体，包括肩部浮雕、底部握持肋条和五瓣瓶底。
- `meshes/cap_red.obj` — 螺纹盖视觉造型、72 条竖向防滑纹与防拆环轮廓。
- `meshes/label_wrap.obj`, `meshes/label.mtl`, `meshes/label_texture.png` — 红白绿标签网格与自制近似印刷贴图。
- `physical_metadata.json` — 质量、惯量、尺寸与参考资料。
- `preview_empty_bottle.png` — 供肉眼核对外观的程序化预览，**不是仿真引擎截图**。
- `build_nongfu.py` — 可以修改参数并重建 OBJ、XML、URDF 的生成器。
- `validate_model.py`, `validation_report.json` — 静态结构与几何一致性测试。
- `render_preview.py` — 重绘概念预览图。

**注意：** URDF 和 XML 引用 `meshes/` 的相对路径。不要只复制一个 XML 或 URDF 文件；应连同 `meshes` 目录一起放在同一文件夹结构下。

## 2. 原型尺寸与模型物性

| 指标 | 模型值 | 依据与可信度 |
|---|---:|---|
| 产品规格 | 550 mL | 产品标称水容量，**不代表瓶内有水** |
| 含盖总高度 | 227 mm | 公开非官方人工测量结果；不同代次会有差异 |
| 瓶身标称外径 | 64 mm | 公开非官方人工测量结果；模型最大约 64 mm |
| 瓶盖外径 | 32 mm | 公开非官方人工测量结果；模型局部约 31.7 mm |
| 瓶盖高度 | 13 mm | 公开非官方人工测量结果 |
| 标签高度 | 64 mm | 依据产品照片比例近似 |
| PET 壳质量 | **19.4 g** | 按 3D 壳体体积约 14.06 cm³ × PET 密度 1.38 g/cm³ 推算 |
| HDPE 红盖质量 | **1.8 g** | 30/25 mm 同类防拆水瓶盖供应商典型值；并与本模型 HDPE 网格体积约 1.895 cm³ 对应 |
| 薄膜标签质量 | **0.7 g** | 基于印刷薄膜面积和厚度的工程估算 |
| **装配空质量** | **21.9 g = 0.0219 kg** | 上述三项之和；**没有水的 550 g** |
| 瓶身质心 z | 92.03 mm（从瓶底算） | 依据非均匀壁厚的壳体积分估算 |
| 原始材料类型 | PET / HDPE / 印刷薄膜 | 常用包装材料假设 |

- 全部模型几何单位为 **米 (m)**、质量 **千克 (kg)**、惯量 **kg·m²**。
- Z 轴向上，瓶底为 `z=0`，含盖最高点 `z=0.227`。
- 外形旋转积分得到的外轮廓体积约 **595.5 mL**；扣除 PET 材料体积后，接近瓶口的空腔有效几何空间约 581 mL（只是估算，用于检查 550 mL 标称容量是否合理；**未模拟水**）。
- PET 壁厚随高度非均匀：侧面约 0.22–0.45 mm，底部约 0.8 mm，瓶颈约 1.0 mm。不是来自工厂模具数据。
- 惯量是按**空心壳体**及各部件独立质量计算的；不是装了 550 mL 水的实心圆柱。

## 3. MuJoCo 使用

在安装了 MuJoCo Python 包的环境，进入解压目录：

```bash
python -m mujoco.viewer --mjcf=demo_mujoco.xml
```

若用于已有机器人操作场景，选用 `nongfu_550_empty.xml`（无地面版本），把其中 `<asset>` 的网格与材质定义、以及 `<worldbody>` 内的 `pet_bottle` body 复制进你的模型；确保网格路径从 XML 文件位置可解析。`demo_mujoco.xml` 自带地面，避免与已有地面重叠。

- 碰撞代理为多段圆柱；Z 方向底部从 0 开始以保证正确落地。碰撞代理不等于完美贴合每个 PET 表面肋纹。
- visual mesh 不参与接触，碰撞 geom 不参与质量计算：全部质量由显式 `<inertial>` 指定。
- 若用于夹持、挤压和变形研究，**空 PET 瓶实际可屈曲**，本 MJCF 为**刚体近似**，需要额外的柔性建模和材料标定。

## 4. Isaac Lab / Isaac Sim 使用

URDF 是输入格式，Isaac Lab 实际应用通常要先将 URDF 转成 **USD**，不是把 `.urdf` 直接填入 `UsdFileCfg`。

**Isaac Lab 3.x（参考官方新流程）：**

```bash
# 从 IsaacLab 项目根目录执行。把 /ABS/PATH 替换为实际解压目录。
uv run --extra importers isaaclab -p scripts/tools/convert_urdf.py \
  /ABS/PATH/nongfu_550_empty/nongfu_550_empty.urdf \
  /ABS/PATH/converted_assets \
  --merge_joints
```

**Isaac Lab 2.x（旧版脚本调用通常为）：**

```bash
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  /ABS/PATH/nongfu_550_empty/nongfu_550_empty.urdf \
  /ABS/PATH/converted_assets/nongfu_550_empty.usd \
  --merge-joints
```

参数会随发行版演变，按当前版本的 `convert_urdf.py --help` 调整。用于可掉落、可抓取的自由物体时不要加 `--fix-base`。建议启用合并固定关节，否则某些 PhysX 导入流程会把这个物体识别为多-link articulation。之后在 Isaac Lab 中用转换后 USD 路径生成 object。

URDF 声明了 3 个 link / 2 个固定关节。OBJ 贴图使用 `label.mtl` 和 `label_texture.png`；不同 Isaac Sim 版本对 URDF 纹理的导入支持不完全一样。如纹理丢失，在 USD Stage 中把对应 PNG 重新绑定到标签材质。不要将 label 复制为实体圆柱：它不需要独立接触碰撞。

## 5. 如何校准真实瓶子

要让你的**某一生产批次**尺寸、质量和质心精确匹配，最可靠的步骤是：

1. 用 0.01 g 精度电子秤分别称 **干燥空 PET 瓶（无盖、无标签）**、瓶盖、标签。
2. 卡尺测含盖总高、瓶身最大直径、盖尺寸、标签上下边缘位置。
3. 修改 `build_nongfu.py` 中的 `PET_MASS`, `CAP_MASS`, `LABEL_MASS`, `PROFILE`, `CAP_Z`, `CAP_HEIGHT`, `LABEL_BOTTOM`, `LABEL_TOP`；若修改高度/瓶径，应同步更新相关采样区间及碰撞片段，而不只改数字标题。
4. 重建并验证：

   ```bash
   python build_nongfu.py
   python validate_model.py
   python render_preview.py
   ```

5. 如果做机器人抓取摩擦、夹持失稳或压溃仿真，还需测量实际 PET 局部壁厚、弹性与接触摩擦，并用柔性体/薄壳方法替换刚体代理。

## 6. 公开参考资料（并非农夫山泉官方工程图）

1. 瓶高、外径、瓶盖尺寸的非官方手工测量： https://zhida.baidu.com/question/413060760.html
2. 佛山市 Honsen 30/25 mm 同类水瓶盖，质量 1.8 ± 0.2 g： https://www.hspreform.com/product/30mm-plastic--caps/30mm%283025%29Cap.html
3. 同类 500 mL 饮用水瓶胚质量的制造商范围： https://www.hspreform.com/news/industry-news/factors-to-consider-before-ordering-pet-preforms.html
4. Isaac Lab 当前导入说明： https://isaac-sim.github.io/IsaacLab/develop/source/how-to/import_new_asset.html
5. MuJoCo XML 模型格式： https://mujoco.readthedocs.io/en/latest/XMLreference.html

## 7. 验证范围和限制

`validation_report.json` 验证 OBJ 的闭合性、接近标称的尺度、碰撞底面、URDF/MJCF XML 完整性、质量一致性及正定惯量。本运行环境**未安装 MuJoCo 和 Isaac Lab**，因此尚不能声称经过这两个仿真引擎的实际加载、可视化或落地测试。后续应在你的安装版本里进行引擎导入及动力学测试。

瓶盖的真实内螺纹配合、批次包装质量、商标精确印刷位置、材料压屈及瓶底局部纹样，均需要实物测绘才能达到工业级还原。这是**有尺寸/质量依据的可用仿真近似件**，不是未经授权宣称的官方原始 CAD。
