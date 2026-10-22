# PICO 全身遥操完整流程（MuJoCo + C++ 控制器 + PICO）

> 适用仓库：`/home/wusichen/GR00T-WholeBodyControl`
> 本文所有命令都在 **本机实测过**（DDS、仿真、C++ 控制器、PICO 流服务启动链路）。
> 最后更新：2026-10-22

---

## 0. 结论先看

| 项目 | 状态 | 说明 |
|---|---|---|
| 是不是 conda 环境？ | **不是** | 仓库全部用 `uv` 建独立 venv：`.venv_sim` / `.venv_teleop` / `.venv_data_collection` |
| DDS 修好了吗？ | **修好了（已实测）** | Python 端 `cyclonedds` binding 固定 0.10.2；C++ 端 `Init Done` 正常打印 |
| C++ 控制器能收到仿真状态吗？ | **能** | `rt/lowstate` 实测约 190–200 Hz；控制器从 `LowState is not available` 进入 `Init Done` |
| PICO 能出数据吗？ | **取决于头显连接** | 头显没连/没开 Send 时，脚本会一直停在 `waiting for body data...`，这是正常的等待状态 |
| 还需要 `just` / `tmux` 吗？ | **不需要** | 直接调已编译好的二进制 + 三个独立终端即可；`tmux` 只有官方一键脚本才需要 |

---

## 1. 环境说明：不是 conda，是 uv venv

仓库的安装脚本（`install_scripts/*.sh`）都用 `uv` 创建 Python 3.10 虚拟环境，**不会碰 conda**：

| venv | 用途 | 关键依赖 |
|---|---|---|
| `.venv_sim` | MuJoCo 仿真 + Unitree DDS 桥 | `mujoco`、`unitree_sdk2py`、`cyclonedds==0.10.2` |
| `.venv_teleop` | PICO 遥操流 | `torch`、`pyzmq`、`pin`、`pyvista`、`xrobotoolkit_sdk` |
| `.venv_data_collection` | LeRobot 数据导出 | `lerobot`、`datasets`、`av`、`opencv` |

检查当前解释器：

```bash
cd ~/GR00T-WholeBodyControl
.venv_sim/bin/python -c "import sys; print(sys.executable)"
.venv_sim/bin/python -c "import importlib.metadata as m; print('cyclonedds', m.version('cyclonedds'))"
# 期望：.../.venv_sim/bin/python
# 期望：cyclonedds 0.10.2
```

> **重要：不要 `pip install -U cyclonedds`。**
> C++ 控制器自带的是 CycloneDDS **0.10.2**；Python 端 binding 一旦升到 0.11.x，会在握手时崩溃：
> `ddsi_xt_type_init_impl with invalid type object` → 段错误。

---

## 2. 一次性安装（已装可跳过）

```bash
cd ~/GR00T-WholeBodyControl

# 仿真环境（会重建 .venv_sim）
bash install_scripts/install_mujoco_sim.sh

# PICO 遥操环境（会重建 .venv_teleop；跳过 sim/unitree，避免重复装 DDS）
SKIP_SIM_AND_UNITREE=1 bash install_scripts/install_pico.sh

# 数据导出环境（可选，录数据才需要）
bash install_scripts/install_data_collection.sh
```

说明：

- `install_pico.sh` 会自动编译 `external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64`，并安装 `isaacteleop[cloudxr]`。
- `install_data_collection.sh` 会尝试用 `apt` 装 `espeak`（语音提示）；没有 sudo 也不影响录制，只是没有语音。
- 官方 `deploy.sh` 依赖 `just`，本机没装；**本文直接调用编译好的二进制，绕过 `just`**。
- 官方 `launch_data_collection.py` 依赖 `tmux`，本机没装；**本文用三个独立终端手动启动**。

验证三个环境：

```bash
.venv_sim/bin/python -c "import mujoco, unitree_sdk2py, cyclonedds; print('sim OK')"
.venv_teleop/bin/python -c "import torch, zmq, pinocchio, pyvista, xrobotoolkit_sdk; print('teleop OK')"
.venv_data_collection/bin/python -c "import lerobot, cv2, zmq; print('data OK')"   # 可选
```

---

## 3. DDS 修复：原因、改动、验证

### 3.1 原因

| 端 | 版本 | 结果 |
|---|---|---|
| C++ 控制器 `gear_sonic_deploy` | 自带 `libddsc.so.0` = **0.10.2** | 老版本，不认新 XTypes type object |
| Python 仿真（修复前） | `cyclonedds` binding **0.11.x** | 会广播 type object，C++ 解析时崩溃 |
| Python 仿真（修复后） | `cyclonedds` binding **0.10.2** | 握手正常，C++ 打印 `Init Done` |

关键点：

1. **决定兼容性的是 Python binding 版本**，不是 wheel 里那个 `.so` 的文件名。
   实测 `cyclonedds==0.10.2` 的 wheel 内部仍带 `cyclonedds.libs/libddsc-*.so.0.11.0`，
   但 binding 0.10.2 不会发送新格式的 type object，C++ 0.10.2 可以正常接收。
2. `unitree_sdk2_python/setup.py` 里已经 pin 了 `cyclonedds==0.10.2`，所以
   `install_mujoco_sim.sh` 装出来的环境天然是对的。
3. `lo` 不支持组播，仓库里对 `channel_config.py` 做了本地 patch：
   `AllowMulticast=false` + `<Peer address="127.0.0.1"/>`，让回环发现走 unicast，结果是确定的。

### 3.2 本仓库的两处 patch

```text
external_dependencies/unitree_sdk2_python/unitree_sdk2py/core/channel_config.py
  + <AllowMulticast>false</AllowMulticast>
  + <Discovery><Peers><Peer address="127.0.0.1"/></Peers></Discovery>
      -> lo 不支持组播，改用 unicast peer，让发现确定

external_dependencies/unitree_sdk2_python/unitree_sdk2py/core/channel.py
  ChannelFactory.Init() 幂等：相同 (domain_id, interface) 重复调用直接复用
      -> 修掉 run_sim_loop.py 里 SimWrapper + BaseSimulator 重复建 Domain 的
         "create domain error" 警告
```

> 场景不要写死到 wbc yaml 里；用 `--robot-scene` 传，换场景不动配置文件。

### 3.3 验证 DDS（仿真开着的时候执行）

先启动仿真（见第 4 节 Terminal 1），然后另开终端：

```bash
cd ~/GR00T-WholeBodyControl
.venv_sim/bin/python - <<'PY'
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_   # 必须是 unitree_hg！

n = 0
def cb(msg):
    global n
    n += 1

ChannelFactoryInitialize(0, "lo")
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(cb, 10)
time.sleep(5)
print("LowState messages:", n)
PY
```

期望：**约 900–1000 条 / 5 秒（≈200 Hz）**。

> 常见坑：用 `unitree_go` 的 `LowState_` 订阅会得到 **0 条**，不是 DDS 不通，是 IDL 类型用错了。
> G1 用的是 `unitree_hg`。

### 3.4 `create domain error` 已经被彻底修掉

修复前 Terminal 1 启动会打印：

```text
[ChannelFactory] create domain error. msg: ... cyclonedds.domain.Domain
Note: Channel factory initialization attempt: channel factory init error.
```

原因：`run_sim_loop.py` 在 `SimWrapper` 里初始化一次 ChannelFactory，
`BaseSimulator.__init__` 里又初始化一次；CycloneDDS 同进程重复建 Domain 返回 `PRECONDITION_NOT_MET`。

已给 `unitree_sdk2_python/unitree_sdk2py/core/channel.py` 加了幂等处理：
相同的 `(domain_id, interface)` 重复 `Init()` 直接复用已有 domain/participant，不再重复创建。
修复后启动日志只剩：

```text
[scene] ROBOT_SCENE override -> ...
```

不再有 `create domain error`。如果换了 domain 或网卡，会明确提示并返回失败，不会静默错配。

---

## 4. 启动流程（三个终端）

### 总体顺序

```text
Terminal 1  MuJoCo 仿真（发布 rt/lowstate + 相机 5555）
      ↓
Terminal 2  C++ 控制器（订阅 lowstate，等 "Init Done"；发布 ZMQ 5557）
      ↓
Terminal 3  PICO 遥操流（绑 ZMQ 5556，等头显 body data）
      ↓
PICO 头显连 PC → 校准 → A+B+X+Y 启动 → A+X 进入 POSE / 左摇杆按下进 VR_3PT
```

### Terminal 1 — MuJoCo 仿真

```bash
cd ~/GR00T-WholeBodyControl
source .venv_sim/bin/activate
export MUJOCO_GL=egl

python gear_sonic/scripts/run_sim_loop.py \
  --enable-image-publish \
  --enable-offscreen \
  --no-enable-onscreen \
  --camera-port 5555 \
  --robot-scene /home/wusichen/groot_repro/scenes/livingroom_bed_compact.xml
```

- 想要本机 MuJoCo 窗口：把 `--no-enable-onscreen` 去掉，改成 `--enable-onscreen`，
  并设 `export MUJOCO_GL=glfw DISPLAY=:1`。
- 无头模式（推荐）：保持上面的 `--no-enable-onscreen`，用第 7 节的相机窗口看画面。
- 正常标志：`ss -ltn | grep 5555` 有监听；日志里相机发布频率约 30 Hz。

### Terminal 2 — C++ 控制器

```bash
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy

M=/home/wusichen/groot_repro/sonic_models
export LD_LIBRARY_PATH=/home/wusichen/groot_repro/trt_root/lib:/home/wusichen/groot_repro/ort_lib:$LD_LIBRARY_PATH

./target/release/g1_deploy_onnx_ref lo \
  $M/model_decoder.onnx ./reference/example/ \
  --obs-config $M/observation_config.yaml \
  --encoder-file $M/model_encoder.onnx \
  --planner-file $M/planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager \
  --output-type zmq \
  --zmq-host localhost \
  --disable-crc-check
```

**必须注意的路径坑：**

- `--planner-file` 的路径里必须包含 **`V2`**。
  控制器用路径字符串判断 planner 版本：
  `planner_sonic.onnx` → `Unsupported planner version`；
  `.../target_vel/V2/planner_sonic.onnx` → 正常。
- `LD_LIBRARY_PATH` 必须包含 TensorRT 和 ONNX Runtime：
  - TensorRT：`/home/wusichen/groot_repro/trt_root/lib`
  - ORT：`/home/wusichen/groot_repro/ort_lib`
  否则报 `libonnxruntime.so.1: cannot open shared object file`。
- 显存：开跑前保证空闲显存 **≥ 1.5 GB**（decoder 39 MiB + encoder 47 MiB + planner 714 MiB，另有 workspace）。

正常日志末尾：

```text
[ZMQ Output] Robot config cached ...
[DEBUG] G1Deploy object created successfully!
Init Done
```

看到 `Init Done` 才说明 C++ 真的收到了仿真端的 `LowState` 并完成初始站立插值。
如果一直打印 `LowState is not available, waiting for robot to be ready`，回到第 3.3 节查 DDS。

### Terminal 3 — PICO 遥操流

```bash
cd ~/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
export DISPLAY=:1          # 需要 --vis_vr3pt 窗口时

python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt
```

正常启动顺序：

1. 自动执行 `/opt/apps/roboticsservice/runService.sh` 拉起 XRoboToolkit PC 服务；
2. 打印 `Waiting for body tracking data...`；
3. 头显连上并开始推流后，打印 `ZMQ socket bound to port 5556`；
4. 出现 G1 可视化窗口（`--vis_vr3pt`）。

如果只看到 `waiting for body data...` 一直循环，说明 **头显没连上/没点 Send**，不是程序坏了。

> 本机验证补注：脚本在 `_init_input_source()` 里会**先阻塞等 body data，拿到数据后才 bind 5556**。
> 因此“5556 没监听”本身不代表脚本启动失败，只代表还没收到头显数据。
> 排查时看 `pgrep -af pico_manager` 进程是否还活着，以及日志是否还在刷 `waiting for body data...`。

> 如果 `import xrobotoolkit_sdk` 报找不到 `.so`，在启动前加：
> `export LD_LIBRARY_PATH=/opt/apps/roboticsservice/SDK/x64:$LD_LIBRARY_PATH`

---

## 5. PICO 头显设置（一次性 + 每次开机）

### 5.1 网络与 PC 服务

1. PC 和头显连 **同一个 Wi-Fi**（本机无线 IP：`192.168.76.22`，网卡 `wlp0s20f3`）。
2. 确认 PC 服务在跑：
   ```bash
   pgrep -af RoboticsServiceProcess
   ss -ltnp | grep 60061 || true
   ```
   Terminal 3 会自动拉起；也可以手动：
   ```bash
   cd /opt/apps/roboticsservice && ./runService.sh
   ```
3. 头显里打开 **XRoboToolkit** app：
   - `PC Service` 填 `192.168.76.22`
   - `Status` 显示 **WORKING**（不行就点 `Reconnect`）

### 5.2 头显内勾选项

- Tracking：**Head** ✅、**Controller** ✅
- Data/Control：**Send** ✅
- Pico Motion Tracker：**Full body** ✅

### 5.3 脚踝 tracker

1. 左右脚踝各绑一个 tracker，指示灯朝上；**穿紧身裤**，裤脚不要遮住 tracker。
2. PICO 设置 → 开发者（Developer）→ **Safeguard 关闭**。
3. Wi-Fi 设置里点开设备图 → 选 Motion Tracker → 对每个 tracker 点 `i` → **Unpair all**。
4. 右上角 `Pair`，按住 tracker 顶部按钮 **6 秒**，红蓝闪即配对模式。
5. 戴头显，点蓝色 **Calibrate**，做两段校准：
   - 第一段：站直，双手控制器自然下垂；
   - 第二段：低头看脚踝 tracker，让头显相机识别到。
6. 校准后可把头显架在额头上（镜头朝前），继续让 tracker 可见。

### 5.4 首次 `CALIB_FULL` 参考姿态（非常重要）

第一次按 **A+B+X+Y** 之前，必须摆成机器人“全零参考姿态”：

1. 站直，双脚并拢，眼睛看正前方；
2. 上臂自然下垂、贴近身体；
3. 前臂向前弯 **90°**（肘部 L 形），掌心相对。

按 **A+B+X+Y** 后系统会做 `CALIB_FULL`（头 + 双腕对全零姿态）。
姿态不对，后面 POSE / VR_3PT 会明显错位甚至突然剧烈运动。

---

## 6. 控制方式与模式切换

### 6.1 按键总表

| 操作 | 按键 | 说明 |
|---|---|---|
| 启动 / 停止 policy | **A+B+X+Y** | 第一次：启动 + `CALIB_FULL`；再按：急停 → OFF |
| POSE ↔ PLANNER | **A+X** | 从 VR_3PT（经 PLANNER 进入）也可回 POSE |
| POSE ↔ PLANNER_FROZEN_UPPER | **B+Y** | 冻结上半身，只让 planner 管下肢 |
| 进入 / 退出 VR_3PT | **左摇杆按下（L-Stick Click）** | 进入时做 `CALIB`（双腕对机器人当前姿态） |
| 移动 | **左摇杆** | 前后 / 左右平移 |
| 转向 | **右摇杆水平** | 连续累积 yaw |
| 切换 locomotion 模式 | **A+B** 下一个 / **X+Y** 上一个 | 0=Idle, 1=Slow Walk, 2=Walk, 3=Run … |
| 手抓握 | **扳机（Trigger）** | 对应手闭合；松开张开 |
| 开始 / 停止录数据 | **左 grip + A** | 需要 data exporter 在跑 |
| 丢弃当前 episode | **左 grip + B** | 需要 data exporter 在跑 |
| 急停（键盘） | C++ 终端按 **`O`** | 立即 stop |

### 6.2 模式状态机（简化）

```text
OFF ──(A+B+X+Y，CALIB_FULL)──► PLANNER ──(A+X)──► POSE
                                  │  ▲               │
                                  │  └────(A+X)───────┘
                                  │
                          (左摇杆按下，CALIB)
                                  ▼
                               VR_3PT ──(左摇杆按下)──► 回 PLANNER

POSE ──(B+Y)──► PLANNER_FROZEN_UPPER ──(左摇杆按下，CALIB)──► VR_3PT
  ▲                    │                                          │
  └──────(B+Y)─────────┴──────────────(B+Y / A+X)────────────────┘

任意模式 ──(A+B+X+Y)──► OFF
```

### 6.3 安全规则（务必遵守）

- 场地清空，旁边必须有人手放在 C++ 终端上准备按 `O`。
- **切 POSE 前**：身体姿态先和机器人当前姿态对齐，否则机器人会瞬间大幅吸附过去。
- **切 VR_3PT 前**：双臂先和机器人当前姿态对齐，再按左摇杆；否则会乱动。
- 第一次启动一定站在 `CALIB_FULL` 参考姿态。
- 穿紧身裤，保证脚踝 tracker 视线；tracker 丢失会导致下肢动作异常。
- 不建议在 POSE / VR_3PT 之间频繁快速切换；每次切换都先对齐。

### 6.4 VR_3PT 校准失败怎么恢复

1. 再按一次 **左摇杆**，退回 PLANNER（冻结上半身）；
2. 把双臂摆到机器人当前姿态；
3. 按 **A+X** 切到 POSE 重置；
4. 重新对齐后再进 VR_3PT。

---

## 7. 数据采集与相机

### 7.1 先确认三个端口

```bash
ss -ltn | grep -E '5555|5556|5557'
# 5555: MuJoCo 相机
# 5556: PICO pose/planner
# 5557: C++ 机器人状态 / robot_config
```

### 7.2 相机预览（可选）

```bash
cd ~/GR00T-WholeBodyControl
source .venv_data_collection/bin/activate
python gear_sonic/scripts/run_camera_viewer.py --camera-host localhost --camera-port 5555
```

窗口里应看到 `ego_view` 等画面；按 `R` 录制，按 `Q` 退出。

### 7.3 数据导出（LeRobot 格式）

在 C++ 已 `Init Done`、PICO 已绑定 5556 之后，再开一个终端：

```bash
cd ~/GR00T-WholeBodyControl
source .venv_data_collection/bin/activate

python gear_sonic/scripts/run_data_exporter.py \
  --task-prompt "pick up the bottle and throw it into the trash bin" \
  --data-collection-frequency 50 \
  --camera-host localhost --camera-port 5555 \
  --root-output-dir outputs
```

- 输出目录：`outputs/<自动时间戳>/`（LeRobot dataset）。
- 录制开关：**左 grip + A** 开始/停止；**左 grip + B** 丢弃当前 episode。
- 没有 `espeak` 时语音提示会缺失；可加 `--no-text-to-speech` 避免相关报错。
- exporter 启动时会等 C++ 的 `robot_config`；如果一直等不到，检查 C++ 是否在跑、5557 是否监听。

---

## 8. 故障排查表

| 现象 | 原因 | 处理 |
|---|---|---|
| C++ 一直 `LowState is not available` | 仿真没开 / DDS 版本不对 / 接口不对 | 跑第 3.3 节探针；确认 `.venv_sim` 的 `cyclonedds==0.10.2`；确认 WBC yaml `INTERFACE: "lo"` |
| `ddsi_xt_type_init_impl with invalid type object` + 段错误 | Python binding 是 0.11.x | `bash install_scripts/install_mujoco_sim.sh` 重建；不要升级 cyclonedds |
| 探针 0 条消息 | 用了 `unitree_go` 的 `LowState_` | 改成 `from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_` |
| 仿真启动打印 `create domain error` | 同进程重复初始化 Domain | 无害；以探针 ~200 Hz 为准 |
| `Address already in use (tcp://*:5555)` | 上一个仿真没退干净 | `pkill -f run_sim_loop.py`，再用 `ss -ltn` 确认 5555 已释放 |
| `libonnxruntime.so.1: cannot open shared object file` | 没设 `LD_LIBRARY_PATH` | 加上 `/home/wusichen/groot_repro/ort_lib` 和 `trt_root/lib` |
| `Unsupported planner version` | planner 路径不含 `V2` | 用 `$M/planner/target_vel/V2/planner_sonic.onnx` |
| TensorRT `Failed to initialize` / OOM | 显存不足 | 关浏览器/编辑器，确认 `nvidia-smi` 空闲 ≥1.5 GB |
| PICO 一直 `waiting for body data...` / 5556 不监听 | PC 服务没起 / 头显没连 / 没点 Send / tracker 没校准 | 查 `pgrep RoboticsServiceProcess`、头显 Status=WORKING、Send ✅、Full body ✅ |
| `xrt.get_headset_pose()` 全 0 | 头显没推流 | 同上；先让 body data available |
| `import xrobotoolkit_sdk` 失败 | 找不到 `libPXREARobotSDK.so` | `export LD_LIBRARY_PATH=/opt/apps/roboticsservice/SDK/x64:$LD_LIBRARY_PATH` |
| `just: command not found` | 没装 just | 本文直接跑 `target/release/g1_deploy_onnx_ref`，不用 `deploy.sh` |
| `tmux: command not found` | 没装 tmux | 本文用三个独立终端，不用 `launch_data_collection.py` |
| 相机 5555 没画面 | 仿真没开图像发布 | 加 `--enable-image-publish --enable-offscreen` |
| `ModuleNotFoundError: unitree_sdk2py` | 用错 venv | 用 `.venv_sim/bin/python` 或 `source .venv_sim/bin/activate` |
| 数据 exporter 一直等 `robot_config` | C++ 没起 / 5557 没监听 | 先让 C++ `Init Done`，再启动 exporter |

---

## 9. 启动检查清单

```text
[ ] nvidia-smi 空闲显存 ≥ 1.5 GB
[ ] Terminal 1: 5555 监听，相机频率 ~30 Hz
[ ] DDS 探针：LowState ~900+ 条 / 5 s
[ ] Terminal 2: 日志出现 "Init Done"
[ ] Terminal 2: 5557 监听
[ ] Terminal 3: 日志出现 "ZMQ socket bound to port 5556"
[ ] PICO 头显 Status=WORKING，Send ✅，Full body ✅
[ ] 头显 body tracking 有数据（不再打印 waiting）
[ ] 可视化窗口里 G1 姿态正常
[ ] 站到 CALIB_FULL 参考姿态，再按 A+B+X+Y
[ ] 先 A+X 进 POSE 小幅测试，再考虑 VR_3PT
```

---

## 10. 停止与清理

按顺序：

1. PICO 上按 **A+B+X+Y** 停止 policy（或 C++ 终端按 `O`）；
2. `Ctrl+C` 结束 Terminal 3（PICO 流）；
3. `Ctrl+C` 结束 Terminal 2（C++）；
4. `Ctrl+C` 结束 Terminal 1（仿真）；
5. 如还有残留：

```bash
pkill -f pico_manager_thread_server.py
pkill -f g1_deploy_onnx_ref
pkill -f run_sim_loop.py
pkill -f RoboticsServiceProcess
```

确认端口释放：

```bash
ss -ltn | grep -E '5555|5556|5557' || echo "ports free"
```

---

## 11. 备注

- 本机场景 `livingroom_bed_compact.xml` 里机器人 pelvis 初始位置是 **`(1.40, 2.75, 0.781)`**，不是 `(0, 0)`。
  这是 MuJoCo 场景坐标，和 PICO 遥操无关；如果下游 GR00T 推理/评测假设 base 在 `(0,0)`，需要在那边单独对齐。
- 官方文档里的 `./deploy.sh --input-type zmq_manager sim` 依赖 `just`，本机不可用；
  本文的直调命令与它等价（接口 `lo`、`--input-type zmq_manager`、`--disable-crc-check`）。
- C++ 控制器第一次启动会花几十秒编译/加载 TensorRT engine（首次还会把 ONNX 转 TRT），
  之后有缓存会快很多；这段时间不要以为卡死。
