# Pico 遥操 G1 采 VLA 数据：链路体检 + "拿瓶子踩踏板扔垃圾桶" 场景

> 结论先说：**仿真、相机、机器人状态这三段是通的；控制回路真正断在 DDS 版本上。**
>
> ① **最关键**：C++ 端 vendored 的 CycloneDDS 是 **0.10.2**，而仿真端用的是 **11.0.1** ——
> 两边握不上手。走 `lo` 时 C++ 端发现不了仿真端（永远 `LowState is not available`），
> 换到有 multicast 的网卡又直接段错误。
> **修法已验证：把仿真端换成 Python 3.10 + `cyclonedds==0.10.2`，控制器立刻 `Init Done`。**
> ② 次要：显存紧张时 TensorRT 会 OOM 退出（腾出 > 1.5 GB 即可）。
> ③ Pico 遥操作和 LeRobot 导出这两个环境（`.venv_teleop` / `.venv_data_collection`）还没建过。
>
> 场景那边已经做完并自检通过（房间 + 0.8 m 桌子 + 农夫山泉空瓶 + 12 L 脚踏垃圾桶），
> 见第二节。
>
> 新场景 `scene_pick_trash_43dof.xml` 用 `run_sim_loop.py --robot-scene` 实测能加载、
> 能出 `ego_view`、能发 `rt/lowstate`。

![场景总览](scene_preview.png)

---

## 一、链路体检

### 1.1 逐环节状态

| 环节 | 状态 | 证据（本机实测） |
| --- | --- | --- |
| MuJoCo 仿真 `run_sim_loop.py` | ✅ 通 | 跑起来 200 Hz 物理；进程 43% CPU、RSS 1.8 GB；`ss -ltn` 看到 5555 在监听 |
| 相机发布（`ego_view`） | ✅ 通 | ZMQ 5555 抓到真帧，640×480 JPEG，33 Hz，`message dropped: 0` |
| 机器人状态 DDS `rt/lowstate` | ✅ 通 | 6 s 收到 1175 条 ≈ **196 Hz**；IMU 四元数≈单位、关节角合理 |
| C++ SONIC 控制器 `g1_deploy_onnx_ref` | ❌ **断** | 两种情况都试过：① 走 `lo` → 一直 `LowState is not available`（C++ 端 CycloneDDS 因 `lo` 没有 MULTICAST 标志自动关掉 multicast，又没有 unicast peer，发现不了仿真端）；② 走 `docker0`/WiFi（有 multicast）→ 立刻 `type [INVALID ...]: ddsi_xt_type_init_impl with invalid type object` **段错误**。根因是 DDS 版本不匹配，见 1.2 |
| 显存现状 | ❌ 不够 | `8151 MiB 总 / 7126 已用 / 580 空闲`（桌面 + Chrome 占掉大头），TRT 光 planner 就要 647 MB |
| Pico 遥操作 | ❌ 没起过 | `.venv_teleop` 不存在；`pico_manager_thread_server.py` 从没跑过 |
| 数据导出（LeRobot） | ❌ 没起过 | `.venv_data_collection` 不存在；`run_data_exporter.py` 从没跑过 |
| 一键拉起脚本 | ❌ 缺依赖 | `launch_data_collection.py` 需要 `tmux`，本机没装 |
| SONIC 推理模型 | ✅ 齐 | `~/groot_repro/sonic_models/`：decoder 40.9 MB + encoder 50.1 MB + planner ONNX |
| 仓库自带权重 | ⚠️ 半套 | `sonic_release/{last.pt,model_step_041550.pt}` 在，但 `sonic_release/exported/` 是空的（没导 ONNX） |

### 1.2 四个坑（都是实测踩到的，不是猜的）

1. **`[ChannelFactory] create domain error` 是假警报。**
   日志里那行 `create domain error` / `channel factory init error` 是 `unitree_sdk2py` 在同一个进程里
   第二次 `ChannelFactoryInitialize` 失败打出来的——第一次已经建好 domain，桥照常工作（196 Hz 的
   LowState 就是证据）。**不要**被这行误导去改 `CYCLONEDDS_URI`，也不要改 `/tmp/cyclone.xml`。

2. **Python 端在 `lo` 上是真能通的（C++ 端另说，见第 3 条）。** 我用 pub/sub 对拍过：
   `ChannelFactoryInitialize(0,"lo")` 上 8 s 收到 3026 条消息（消息被 Listener 和队列线程各计一次，实际发布 ~200 Hz），发现和传输都正常。
   之前 `~/groot_repro/dds_probe.py` 报 `messages: 0`，是因为脚本里优先 import 了
   `unitree_go` 的 `LowState_`，而发布端用的是 `unitree_hg` 的 `LowState_` —— **类型名不同，
   DDS 直接不匹配**，跟网络无关。

3. **真正的断点是 DDS 版本对不上，不是网卡配置。** 这是最关键的一条：

   - Python 端 `unitree_sdk2py` 的 IDL 是 **idlc 0.11.0 生成**的，`pip list` 里是 `cyclonedds 11.0.1`；
   - C++ 端 `gear_sonic_deploy/thirdparty/unitree_sdk2/lib/x86_64/libddsc.so` 是 **CycloneDDS 0.10.2**
     （二进制里能直接看到 `cyclonedds-0.10.2/src/core/ddsi/...` 路径）。

   0.11 的 Python 端会把 XTypes type object 广播出去，0.10.2 的 C++ 核心收到后把它判成
   `INVALID` 然后解引用空指针 → 段错误。所以：

   - 在 `lo` 上：0.10.2 检测到 `lo` 没有 `IFF_MULTICAST` 标志 → 自动关 multicast → 它和 Python 端
     谁也发现不了谁 → 永远 `LowState is not available`（但不会崩）；
   - 在 `docker0`/`wlp0s20f3` 上：multicast 正常 → 一握手就吃到 type object → 崩。

   这两条路都堵死，**所以才一直没有"控制回路"**。

   **已验证的修法（推荐，成本最低）：把 Python 端固定到 `cyclonedds==0.10.2`。**
   cp310 有现成 wheel，所以要用 **Python 3.10** 的环境跑仿真：

   ```bash
   # 做法一：按仓库脚本建 .venv_sim（本身就是 py3.10），再把 DDS 钉到 0.10.2
   bash install_scripts/install_mujoco_sim.sh
   .venv_sim/bin/pip install "cyclonedds==0.10.2"
   .venv_sim/bin/python -c "import importlib.metadata as m; print(m.version('cyclonedds'))"  # 0.10.2

   # 做法二：直接复用现成的 py3.10 venv
   <py310-venv>/bin/pip install "cyclonedds==0.10.2"
   ```

   我用这个组合实测过：C++ 控制器从 `LowState is not available` 变成 **`Init Done`**，
   也就是真的收到了仿真端的机器人状态（把假发布者停掉之后，它还正确地报
   `Lost LowState data connection from robot!` 并安全退出）。**版本对上就通了，网卡不用动。**

   > 之前想过但不需要的两条路：给 C++ 换 CycloneDDS ≥0.11 重编（半天）、
   > 或绕开 DDS 走 `~/groot_repro/sonic_loop.py` 那套进程内闭环（能出视频，但 Pico 遥操作 +
   > LeRobot 采集官方只支持 DDS 这条路）。

   **补充：在 `unitree_sdk2py/core/channel_config.py` 里加 unicast peer 只能解决"发现"，不能解决"握手"。**
   实测把 `ChannelConfigHasInterface` 改成 `AllowMulticast=false` + `<Peer address="127.0.0.1"/>`
   之后，配置确实生效了（`/tmp/cdds.LOG` 里能看到），但用 0.10.2 的客户端去订阅 11.0.1 的仿真端，
   会直接 **core dump**，报的还是同一个 `ddsi_xt_type_init_impl with invalid type object` ——
   也就是两边"见上面"就崩，方向是对称的。所以 peer 配置可以留着（让发现变确定性），
   但**版本必须先对齐**。

4. **显存是第二道坎（能绕）。** 8 GB 卡被桌面吃掉 7 GB 时，TRT 连 647 MB 的 workspace 都申请不到，
   控制器直接 `Failed to initialize TensorRT engine` 退出。关掉 Chrome 之后空闲回到 ~2.8 GB，
   decoder(39 MiB)/encoder(47 MiB)/planner 都能正常加载。**每次开跑前先看一眼
   `nvidia-smi --query-gpu=memory.free --format=csv`，低于 ~1.5 GB 就别试了。**

### 1.3 把控制回路接上（补齐上面的断点）

```bash
# 0) 清显存（关掉占卡的浏览器/编辑器），确认有空闲
nvidia-smi --query-gpu=memory.free --format=csv

# 1) 仿真（新场景）         —— 3 个终端里的 1 号
cd ~/GR00T-WholeBodyControl
export PYTHONPATH=$PWD:$PWD/external_dependencies/unitree_sdk2_python
export MUJOCO_GL=glfw DISPLAY=:1
~/groot_repro/simvenv/bin/python gear_sonic/scripts/run_sim_loop.py \
    --enable-image-publish --enable-offscreen --no-enable-onscreen --camera-port 5555 \
    --robot-scene gear_sonic/data/robot_model/model_data/g1/scene_pick_trash_43dof.xml

# 2) C++ 控制器              —— 2 号
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy
M=~/groot_repro/sonic_models
LD_LIBRARY_PATH=~/groot_repro/trt_root/lib:/tmp/ort/onnxruntime-linux-x64-1.19.2/lib \
./target/release/g1_deploy_onnx_ref lo \
    $M/model_decoder.onnx ./reference/example/ \
    --obs-config $M/observation_config.yaml \
    --encoder-file $M/model_encoder.onnx \
    --planner-file $M/planner/target_vel/V2/planner_sonic.onnx \
    --input-type zmq_manager --output-type zmq --zmq-host localhost --disable-crc-check

# 3) 遥操作 + 数据导出（还没装，先装）
bash install_scripts/install_pico.sh              # -> .venv_teleop
bash install_scripts/install_data_collection.sh   # -> .venv_data_collection
sudo apt install tmux                              # launch_data_collection.py 需要
python gear_sonic/scripts/launch_data_collection.py --sim
```

> ⚠️ **现在照抄 1)+2) 是串不起来的**：C++ 端在 `lo` 上看不到仿真端的 LowState（原因见 1.2 第 3 条）。
> 1)+2) 能验证的只是"仿真 + 相机 + 状态发布"这三段各自是好的；
> 要真正闭环，得先按 1.2 的方案 A 或 B 统一 DDS 版本。

`--robot-scene` 是这次新加的开关（`gear_sonic/utils/mujoco_sim/configs.py`），
不改 wbc yaml 就能换场景；也可以设环境变量 `GS_ROBOT_SCENE=...`。

---

## 二、场景：桌子 + 矿泉水瓶 + 脚踏垃圾桶（房间内）

### 2.1 任务

G1 站在房间里，面前是 0.8 m 高的实木桌，桌上放着农夫山泉 550 mL 空瓶（21.9 g，红盖）；
桌子右前方地上是 12 L 脚踏垃圾桶。任务序列：

```
1. 走到桌边，右手/左手抓瓶  →  2. 转向垃圾桶，抬脚踩下踏板（盖子联动打开 63°）
                                        →  3. 把瓶子丢进桶里，松脚，盖子合上
```

### 2.2 文件

| 文件 | 作用 |
| --- | --- |
| `gear_sonic/scripts/pick_trash_scene.yaml` | **布局配置文件**：桌子/瓶子/垃圾桶位置、房间尺寸、灯光、相机 |
| `gear_sonic/scripts/build_pick_trash_scene.py` | 生成器 + 自检 + 预览渲染（`build` / `check` / `render` / `all`） |
| `gear_sonic/data/robot_model/model_data/g1/scene_pick_trash_43dof.xml` | 生成出来的场景（**不要手改**） |
| `gear_sonic/data/robot_model/model_data/g1/meshes/pick_trash/` | 瓶子/垃圾桶/桌子的 mesh 和原始资产 XML（不在 git 里，见 2.5） |

### 2.3 用法

```bash
cd ~/GR00T-WholeBodyControl
PY=~/miniconda3/envs/a3_bridge/bin/python        # 任何一个装了 mujoco>=3.3 的环境都行

$PY gear_sonic/scripts/build_pick_trash_scene.py build     # 生成 XML
$PY gear_sonic/scripts/build_pick_trash_scene.py check     # 物理/几何自检
$PY gear_sonic/scripts/build_pick_trash_scene.py render --out-dir /tmp/pv   # 出预览图
$PY gear_sonic/scripts/build_pick_trash_scene.py all       # 三件事都做

# 临时挪位置，不用改 yaml
$PY gear_sonic/scripts/build_pick_trash_scene.py build --set table.center=0.70,0.24 --set bottle.pos=0.44,0.0
```

自检输出（本机实测）：

```
nq=59 nv=57 nu=43 ngeom=192 nmesh=73 nbody=61
执行器 43 个（无多余踏板电机 ✓）
关节 floating_base_joint ✓  floating_bottle ✓  pedal_joint ✓  lid_joint ✓
桌面上表面 z=0.800 m，瓶底 z=0.800 m
right_shoulder_pitch_link → 瓶子 0.481 m = 手臂可达 88%（最大 0.545 m）
right_ankle_roll_link     → 踏板铰链 0.546 m（足部需前伸踩下）
瓶子在桌面范围内: ✓
静置 3 s：瓶子漂移 0.51 mm，倾角 0.00°，踏板 +0.000 rad，盖子 -0.001 rad
道具静置稳定: ✓
```

### 2.4 视角

**数据采集用的 `head_camera`（ego view）**——瓶子正好在两个手中间，构图适合 VLA：

![ego view](ego_view.png)

**踩下踏板、盖子打开**：

![踏板打开](scene_pedal_pressed.png)

### 2.5 资产从哪来（重要）

`gear_sonic/data/robot_model/` 整个被 `.gitignore` 的 `data/` 规则忽略，所以场景 XML 和 mesh
**只在本机，不进 git**。换机器/重新 clone 之后要先把资产拷回来（`build` 会自动检测并提示）：

```bash
D=~/下载
G=gear_sonic/data/robot_model/model_data/g1/meshes/pick_trash
mkdir -p $G/{bottle,bin/visual,bin/collision,table}

cp $D/nongfu_spring_550ml_empty_mujoco_isaaclab/nongfu_550_empty/nongfu_550_empty.xml $G/bottle/asset_source.xml
cp $D/nongfu_spring_550ml_empty_mujoco_isaaclab/nongfu_550_empty/meshes/* $G/bottle/
cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/mujoco/pedal_bin_12l.xml" $G/bin/asset_source.xml
cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/meshes/visual/"*.obj $G/bin/visual/
cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/meshes/collision/"*.stl $G/bin/collision/
cp "$D/foot_pedal_bin_12l_mujoco_isaaclab (2)/foot_pedal_bin_12l/textures/front_label.png" $G/bin/
cp "$D/table_0p8m_simulation_package(1)/table_0p8m/table_mujoco.xml" $G/table/asset_source.xml
```

---

## 三、场景里几个必须知道的实现细节

1. **mesh 路径必须放在 `meshes/pick_trash/` 下。**
   MuJoCo 处理 `<include>` 时，被包含的机器人 XML 里的 `<compiler meshdir="meshes"/>` 会**变成全局设置**，
   连带把主文件里声明的 mesh 路径也强制加上 `meshes/` 前缀；而在主文件里再写一个 `<compiler>` 会把
   机器人的 `meshdir` 覆盖掉、机器人 mesh 全废。实测结论：把资产放 `g1/meshes/pick_trash/` 下、
   用 `file="pick_trash/..."` 引用，是唯一不用改机器人 XML 的稳做法。

2. **不要把垃圾桶资产里的 `<actuator>` 带进来。**
   原始 `pedal_bin_12l.xml` 末尾有个 `pedal_step_torque` motor。带进来会让 `nu` 从 43 变 44，
   `unitree_sdk2py` 桥按索引写 `mj_data.ctrl`，会整体错位。生成器已经把它丢掉，
   自检里 `nu=43` 就是在防这个。

3. **踏板和盖子靠 `equality` 联动，不是硬约束。**
   `lid = -4.5 × pedal`，`mj_forward` 不会解约束，所以预览里要手摆两个关节才是"开盖"。
   真跑物理时踩下去会自己开，松脚靠盖子自重合上。

4. **房间的天花板设了 `contype=0 conaffinity=0`**（纯视觉），否则会干扰：
   高空落物、相机 ray cast、以及"机器人跳起来撞头"这类奇怪接触。

5. **桌子、垃圾桶底座都是静态 body**（去掉了原始 XML 的 `freejoint`）。
   垃圾桶能立住不会一碰就翻，代价是被踢也不会倒。

---

## 四、下一步建议

1. **先把 DDS 版本统一**（1.2 第 3 条方案 A/B）。这是唯一真正的拦路虎：
   在那之前，C++ 控制器不管怎么起都看不到仿真里的机器人状态，遥操作和采数据都无从谈起。
   一个低成本的验证办法：编好之后用 `--input-type zmq_manager` 起控制器，只要日志里不再刷
   `LowState is not available`，就说明链路通了。
2. **跑之前先看显存**，< 1.5 GB 空闲时 TRT 会直接 OOM 退出。
3. 装 `.venv_teleop`，跑 `pico_manager_thread_server.py --manager --vis_vr3pt`，确认 Pico 数据能进。
4. 装 `.venv_data_collection`，跑 `run_data_exporter.py`，确认 LeRobot 数据落盘。
5. 场景布局按手感微调：现在瓶子离桌沿 0.16 m，肩到瓶 0.48 m（88% 臂展）。
   如果遥操时够不着，就 `--set table.center=0.58,0.22` 把桌子往身前挪 4 cm。
6. **成功判定**（写脚本时可以直接用）：
   - 抓取：`bottle_body` 抬升 > 0.05 m 且与右手 `*_hand_*_link` 有接触；
   - 开盖：`lid_joint > 0.6 rad`；
   - 入桶：瓶子最终 x/y 落在桶内半径 0.12 m 内，且 z < 0.30 m（落在内桶里）。
