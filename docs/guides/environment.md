# Dexmate reset/step 与同步序列

入口：`Isaacsim_tactile_env/dexmate_env.py`。原生 Isaac Sim 单场景环境，复用已验证的机器人、桌子、碰撞和触觉配置。
不需要安装整套 Isaac Lab 或 Gymnasium；接口采用常见的五项 step 返回值，但当前未注册为 Gym 环境。

## 运行完整示例

```bash
conda activate dexmate_isaacsim
./tools/run_dexmate_sim.sh tools/run_dexmate_episodes.py \
  --workaround-r535-vulkan --episodes 3 \
  --output Isaacsim_tactile_env/output/dexmate_episodes_new
```

输出目录必须不存在。一个进程只创建一次场景，连续执行三轮 reset 和完整抓取；共享导入资产不按轮重复导出。
`--smoke --episodes 2` 仅验证两次重置及每轮八个物理步，不代替完整抓取验收。

### 小范围位置／朝向变化

```bash
conda activate dexmate_isaacsim
./tools/run_dexmate_sim.sh tools/run_dexmate_episodes.py \
  --workaround-r535-vulkan --pose-sweep \
  --output Isaacsim_tactile_env/output/dexmate_pose_sweep_new
```

`--pose-sweep` 使用固定十轮清单（替代 `--episodes` 数量）：基线、X/Y 各 ±20 mm、yaw ±10°、
两组 ±10 mm 与 ±5° 的组合、最后重复基线。清单位于 `tools/dexmate_variations.py`。
Yaw 绕物体自身中心的世界 Z 轴旋转；夹爪相对物体的偏移与朝向一起旋转，重新求解 IK。
目前仅接受各轴 ±30 mm、yaw ±15° 内的显式变化，不进行随机采样。

`env.reset(variation={"name": "case", "offset_xy_m": [0.01, 0], "yaw_deg": 5})`
每次都基于原始配置计算，变化不会逐轮累计；参数保存在每轮 `episode_report.json` 的 `grasp_config`。
重置后核对请求的物体 XY 与朝向；同一参数再次出现时核对重置一致性。

完整运行的抓取验收失败会保留全部记录并继续下一轮，汇总为 `completed_with_failures`、退出码 2；
程序异常或重置校验错误仍中止并记录错误。每轮生成 `four_cameras_preview.mp4/png`，
上方 2×2 四相机 RGB，下方同帧四片触觉；首轮另有 `four_cameras.gif`。
这仍是目标位姿已知的脚本抓取，不是视觉闭环策略或通用避障规划。

采集完成后生成汇总页面，并通过共用离线读取器核对各轮窗口和图像（从本仓库根目录）：

```bash
conda run -n dexmate_isaacsim python tools/review_dexmate_sweep.py /path/to/sweep_output
PYTHONPATH=../dex_wire/src conda run -n robot python tools/audit_dexmate_sweep.py /path/to/sweep_output
```

查看 `review.html` 或 `sweep_overview.jpg`；数值汇总在 `sweep_summary.json`，
离线读取审计在 `offline_audit.json`（拒绝覆盖已有审计）。审计的“可读取”与抓取验收结果分别报告。

## 环境接口

先创建环境，再导入其他 Isaac Sim API。环境负责启动 SimulationApp；同一进程只维护一个实例。

```python
from Isaacsim_tactile_env.dexmate_env import DexmateEnv

env = DexmateEnv("Isaacsim_tactile_env/output/api_new", workaround_r535_vulkan=True)
try:
    observation, info = env.reset(seed=0)
    action = env.initial[env.driven_ids].copy()  # 示例：保持初始姿态
    observation, reward, terminated, truncated, info = env.step(action)
finally:
    env.close()
```

- `reset(seed=..., variation=None)`：重置机器人与动态物体、速度、接触计数、任务进度和观测缓存；稳定 120 步后清除残余速度，
  将新一轮时间和步号置零，返回更新后的触觉与图像。当前没有随机化，seed 仅保留在返回信息中。
  稳定阶段也按每四步一次渲染；最后一步必定渲染，物理步数与 dt 不变。
- `step(action)`：推进且仅推进一个 1/120 s 物理步。动作按 `env.action_names` 排列，转动关节单位 rad，
  `Lift` 单位 m；排除两个 mimic 从动关节。形状错误、非有限值和越限目标会在执行动作前拒绝。
- 可选 `step(action, phase="grasp_hold")` 仅为记录标记阶段，不会替调用者生成动作。
- 返回 `(observation, reward, terminated, truncated, info)`；默认一轮上限 2160 步。
  完成持续抬升并稳定释放后，在轮次结束时 `terminated=True, reward=1`；超时未完成则 `truncated=True`。
  非有限关节状态或物体掉落超过基线 10 cm 会提前失败。`info.task_success` 表示任务进度条件，
  完整示例还独立检查滑移、意外碰撞、视频与时钟，见 `episode_report.json`。
- 结束后继续 step 会报错，需要再次 reset。`close()` 结束 Isaac Sim 应用进程，释放录像资源由示例记录器负责。

## observation 的时间约定

| 字段 | 内容 |
| --- | --- |
| `episode_id`, `step`, `physics_step` | 当前轮次与物理步号，reset 后从 0 开始 |
| `time_s`, `sim_time_s` | 本轮相对时间、Isaac Sim 当前物理时间 |
| `joint_position`, `joint_velocity` | 全部关节实际值，顺序 `env.robot.dof_names` |
| `object_position_m`, `object_orientation_wxyz` | 自由物体世界位姿 |
| `object_velocity_m_s`, `object_angular_velocity_rad_s` | 物体速度 |
| `gripper_pose`, `object_in_gripper_m` | 左夹爪世界矩阵及物体在夹爪坐标中的位置 |
| `tactile` | float32 `(4,12,32)`，顺序 L1、L2、R1、R2 |
| `rgb` | 四路机器人相机，加一台观察相机；RGB uint8 数组 |
| `rgb_updated`, `rgb_step`, `rgb_time_s` | 图像是否刚更新、对应本轮步号和时间；中间三个物理步复用最近图像 |
| `rgb_sim_time_s`, `rgb_reference_time` | 每台相机的渲染器实际物理时间及原始有理数帧时间 |
| `rgb_camera_to_world` | 本次图像对应的相机世界矩阵；USD 相机坐标 +X 右、+Y 上、−Z 前 |

头部两路为 640×480，C960 腕相机两路为 640×360，观察相机为 800×600。
腕相机采用 90° 对角视场、居中无畸变针孔模型，fx=fy≈367.151 px；光心／朝向由 STL 镜头前端估算。
内外参均为名义配置，未做实机标定；每轮 `data_contract.camera_models` 保存四路相机的 K 和来源。
组合预览保持原图比例，16:9 腕相机画面在 4:3 格子中留边，不拉伸。
观察相机近裁剪 1 cm，已修复旧 GIF 的桌边裁切问题。

### 本机真实腕相机采集检查

本机两台 C960 沿用 `/dev/dexmate-wrist-left` 和 `/dev/dexmate-wrist-right`，
配置在 `Isaacsim_tactile_env/dexmate_wrist_cameras.json`。脚本在打开前核对每个别名的序列号。

```bash
conda run --no-capture-output -n dex_wire python tools/probe_dexmate_wrist_cameras.py \
  --apply-controls --output Isaacsim_tactile_env/output/wrist_camera_capture_new
```

输出目录必须不存在；`--apply-controls` 应用 profile 中已保存的曝光／白平衡设置，省略则保留设备控件现状。
采集模式为 1280×720 MJPG／30 fps、四个缓冲区；最终短测双台约 30 fps。
输出原尺寸 PNG、左右组合图、UVC／udev 信息与主机读图时间；它不是硬件同步录像或内参标定程序。
采集需独占设备并持续读帧；当前脚本是有限帧数检查，如设备断连导致驱动阻塞，可用系统 `timeout` 限制总运行时间。
实机使用 1280×720，仿真腕相机使用同为 16:9 的 640×360。实机配置中的未知标定值保持 null；
仿真使用 workspace 中单独声明的规格估算模型，不将其反写为实机标定。
详情和早期单缓冲掉帧对照见[设备报告](../reports/2026-09-16_wrist_camera_devices.md)。

### 如何保证同步

新录制采用 `data_contract.version=2`：RGB、触觉、动作、关节和物体状态统一保存为 **30 Hz**。
物理、控制与内部验收仍为 120 Hz，每四步在相机对应时刻保存一组数据，不做平均或插值。
性能优化后每四个物理步执行一次正常渲染，中间三个步只推进物理；USD 位姿与触觉仍每步更新。
启用 Isaac Sim 5.1 的同步渲染完成设置，采集时读取每台相机的 `ReferenceTime`，
通过 Isaac Sim 映射到实际物理时间；必要时调用仅渲染的刷新，直到五台相机与当前物理时间一致。
刷新期间断言物理步号不变。这里没有靠提前或滞后填写标签来掩盖图像延迟。
此外要求原始渲染帧时间严格前进：跳过渲染时，Isaac Sim 可能把旧引用映射成当前物理时间，
因此仅比较映射后的时间戳不足以证明图像更新。

## 每轮输出

`episode_000/`、`episode_001/` 等分别包含：

- `trajectory.npz`：540 条 30 Hz 记录，包含动作、关节状态、物体状态、完整触觉、受力标志、任务状态、阶段和时间。
- `camera_index.npz`：540 条相机帧记录；`camera_names` 给出名称顺序，
  第 k 个视频帧对应 `state_row[k]` 行的状态与动作，v2 中 `state_row[k] == k`。
  原物理步号仍为 4、8、12……，另存真实渲染时间、原始帧时间和相机世界矩阵。
- `head_left.mp4`、`head_right.mp4`、`wrist_left.mp4`、`wrist_right.mp4`：每路 540 帧、30 fps。
- `observer.mp4`：修正裁剪后的观察视角。
- 首轮额外导出 `observer_preview.mp4`（30 fps）、`observer.gif`（10 fps）和接触帧 `observer_preview.png`：
  **上方 RGB、下方四片 12×32 触觉热图**，由 `camera_index.state_row` 关联同一时刻，固定色标 0–1。
  可通过 `tools/dexmate_episode_recording.py` 的 `make_review(output)` 从已有记录离线重建。
- `reset_*.png`：每次 reset 的初始图像，与视频分开；视频从本轮物理第 4 步开始。
- `tactile_contact.png`、`tactile_stages.png`：同帧四片热图及开始／接触／结束对比。
- `episode_report.json`：抓取判据、重置状态、真实时间对齐误差及解码核验的视频帧数。
  `frame_count` 是保存的 30 Hz 行数；`validation_frame_count` / `validation_hz` 是内部验收步数／频率。
  两个录制时刻之间提前结束时，由 `final_physics_state` 保留最终步号与任务标志，不伪造额外视频帧。

视频采用 H.264 CRF18，属于有损可视化记录；数值数组未做有损压缩。需要无损训练图像时应另加无损编码，
不要把 MP4 解码帧宣称为渲染器原始 RGB。

一行 `trajectory` 的动作先作用一个物理步，再记录该行状态，即动作与其**执行后**的观测成对。
这是 120 Hz 控制指令的 30 Hz 抽样，不能解释成整个 1/30 s 内保持不变的指令。
抓取验收仍使用内存中的全频数据，因此 30 Hz 保存不会漏掉验收中的短暂失稳；全频序列不再写入 NPZ。
轮次初始状态在 `episode_report.reset_state`。不同轮次可有相同相对时间，必须同时使用轮次编号。

读取一帧的对应触觉：

```python
import numpy as np

with np.load("episode_000/trajectory.npz") as states, np.load("episode_000/camera_index.npz") as frames:
    k = 100  # 视频第 101 帧
    row = frames["state_row"][k]
    tactile = states["tactile"][row]
    action = states["action"][row]
    assert states["step"][row] == frames["step"][k]
```

顶层 `run_report.json` 汇总连续重置一致性、每轮抓取与同步结果；`scene_setup.json` 保存场景配置及碰撞设置。
本例覆盖左手既定轨迹与单一物体；多进程基准另覆盖相同轨迹并行回放，未验证随机初始状态或其他任务。

## 与实机共用离线读取

读取实现统一维护在相邻 `dex_wire/src/dex_wire/data/offline.py`，使用 `robot` 环境，无需启动 Isaac Sim。
从本仓库根目录运行（两个仓库为相邻目录）：

```bash
PYTHONPATH=../dex_wire/src conda run -n robot python -c '
from dex_wire.data.offline import open_episode
episode = open_episode("Isaacsim_tactile_env/output/dexmate_episodes_20260915_v1/episode_000")
clip = episode.window(8.0, 2.0, include_images=False)
print(clip["frames"]["observation.state"].shape)
print(clip["signals"]["observation.tactile"].shape)
'
```

上述旧 v1 记录输出 `(60,16)` 与 `(240,4,12,32)`；新 v2 记录则为 `(60,16)` 与 `(60,4,12,32)`。
`episode.frame(k)` 默认同时读取四路 RGB。读取器兼容两版，历史输出不改写。
共用命名为 `observation.images.left_wrist/right_wrist`，与原文件名 `wrist_left/right` 对应。
完整说明见 [Dex Wire 离线读取指南](../../../dex_wire/docs/offline_reading.zh-CN.md)。
磁盘记录、原始采样率、完整关节和动作时间关系均保留；读取接口不自动转换为策略训练样本。

## 性能与诊断

采集环境不再创建三路 1200×900 的摆放检查视角；单独场景验证器仍保留这些视角。
无窗口采集默认关闭闲置 GUI 视口更新；五个相机的独立渲染输出继续工作。
四路机器人 RGB 与观察相机仍全部保存，分辨率、物理参数、控制和触觉计算频率不变。
同一进程内，仅当物体变化参数和初始关节状态逐字节相同时复用已求解的脚本动作。

- `--render-mode camera`：每四个物理步渲染一次。
- `--zero-delay` / `--no-zero-delay`：启用／关闭同步渲染完成设置。
- `--diagnostic-cameras`：重新启用三路摆放检查视角，仅供对照。
- `--no-preview`：跳过组合 MP4/GIF，五路源视频和数值仍保存，之后可用 `make_review()` 离线生成。
- `--cpu-threads N`：全局 Carbonite/TBB 工作线程上限，默认 32；8／16／32 对照未见明显差异。
- `--physics-threads N`：显式指定 PhysX 工作线程；省略沿用安装默认，本机实测为 8，单线程没有更快。
- `--viewport-updates`：重新启用 GUI 视口，默认关闭。
- `run_report.performance`：启动、reset、轨迹求解、步进、录制、预览计时，以及实际物理设备和刷新次数。
  计时含嵌套区间，例如 capture 已包含 render_flush，不能相加重复计算；GPU 等待也包含在调用耗时中。

旧渲染流程的对照参数为 `--render-mode physics --diagnostic-cameras --no-zero-delay --viewport-updates`；
接触统计已优化，当前源码不会复现旧版 Python 向量遍历的耗时。
实测耗时见 [首次性能报告](../reports/2026-09-16_performance.md) 和
[接触回调／线程检查](../reports/2026-09-16_contact_performance.md)。

专项检查可直接回放已有的完整 **120 Hz** 动作记录，避开 IK 和预览干扰；30 Hz 记录不能用于该回放：

```bash
./tools/run_dexmate_sim.sh tools/benchmark_dexmate_cpu.py \
  --workaround-r535-vulkan --profile --cpu-threads 32 --physics-threads 8 \
  --disable-viewport-updates \
  --reference Isaacsim_tactile_env/output/dexmate_pose_sweep_20260916_v1/episode_000/trajectory.npz \
  --output Isaacsim_tactile_env/output/cpu_benchmark_new
```

输出五路视频、正常验收报告、`benchmark.json` 和可选 `step_profile.pstats`。
`benchmark.json` 同时记录与源轨迹全部物理步的数值差异；该检查工具默认保留 GUI 视口以方便做控制变量对照。

### 单 GPU 多进程吞吐实验

```bash
conda activate dexmate_isaacsim
./tools/run_dexmate_sim.sh tools/benchmark_dexmate_parallel.py \
  --workers 2 --episodes 2 --cpu-threads 8 --physics-threads 8 \
  --workaround-r535-vulkan \
  --reference Isaacsim_tactile_env/output/dexmate_pose_sweep_20260916_v1/episode_000/trajectory.npz \
  --output Isaacsim_tactile_env/output/parallel_benchmark_new
```

`--workers` 支持 1／2／4。每个独立进程建立自己的场景、物理和五路相机，共享本机 GPU；
这不是 Isaac Lab 的同场景克隆／批量渲染。为减少同时初始化 Kit/RTX 时的延迟，逐个加载场景，
所有进程完成首次 reset 后一起起跑，每进程连续录制 `--episodes` 条。
原 120 Hz 动作回放只用于比较吞吐和正确性，重复记录不能算不同示范；不会重新求解 IK 或生成组合预览。

- 每进程输出 `worker_00/` 等独立目录，其中 `episode_000/` 等沿用标准 30 Hz 数据格式。
- `parallel_benchmark.json`：以最早开始到最晚结束的墙钟时间计算合格条数／分钟；
  包含录制、轮次间 reset、写盘与验收，不包含首次加载／reset、IK、组合预览。
  `total_wall_seconds` 另含所有进程加载、首次 reset 和退出。
- `worker_startup` 与每进程 `ready.json`：环境构建、首次 reset、进程到就绪的耗时及实际驱动缓存路径。
  就绪文件在写完后原子发布，协调器逐个打印进度。
- `gpu_telemetry.json`：约每秒采样整张显卡的显存与利用率；不是精确逐帧峰值，也未扣除桌面占用。
- 每个 `benchmark.json` 保留每条的完整 2160 步数值对比、抓取／碰撞／同步检查。
  `all_numeric_samples_identical` 与任务通过分开报告；提前结束不算完整参考回放。
- 默认总超时 600 秒；异常保留日志并清理本次创建的子进程。使用新输出目录重试。
- 离线审计仍按每个 worker 运行：`tools/audit_dexmate_sweep.py /path/to/output/worker_00`。

测量结果和范围见 [并行采集报告](../reports/2026-09-16_parallel_performance.md)。

首次扩展到四进程时曾因 RTX 管线编译使准备时间达到约 190 秒；复用持久化驱动缓存后约 40 秒。
详见[初始化缓存复测](../reports/2026-09-16_startup_cache.md)。保留安装目录的 `kit/cache/nv_shadercache`，
它与本项目的采集 `output/` 分开；清理采集输出不会要求删除这个缓存。
缓存清空、驱动或渲染配置变化后，首次运行仍可能重新编译；比较性能时应分开报告初次编译和缓存已建立的运行。

### GPU 物理对照

环境构造支持 `physics_mode="gpu"`，默认仍为 `"cpu"`。GPU 模式在首次模拟前启用 GPU dynamics 和 GPU broadphase，
保留 NumPy、CPU 状态读回及 USD 位姿更新，因此不属于数据全程驻留 GPU 的接口。
GPU 的 aggregate pair 缓冲容量设为 4096，避免当前场景在 reset 时超过默认容量而漏掉交互。

使用上面的协调器命令，加上 `--physics-mode gpu --no-ccd`；CPU 对照使用 `--physics-mode cpu --no-ccd`。
Isaac Sim 的 GPU dynamics 会关闭 CCD；构造参数 `ccd=None` 表示沿用该物理模式的默认值。
协调器还会扫描原生 PhysX 错误，出现错误的运行不会算作合格吞吐；单独 worker 报告中的脚本通过不替代这一检查。

`runtime.gpu_dynamics_enabled` 与 `broadphase` 标识实际 PhysX 配置。
历史 `physics_device` 字段来自 SimulationManager，在保留 CPU 读回时仍显示 `cpu`，不能据此认定 GPU 求解没有启用。
`suppress_readback=false`、backend `numpy` 表示当前保留的数据路径。
实测结果及适用范围见 [GPU PhysX 对照](../reports/2026-09-16_gpu_physics.md)。
