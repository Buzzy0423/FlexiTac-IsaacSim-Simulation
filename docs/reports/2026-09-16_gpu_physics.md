# GPU PhysX 对照：保留 CPU 状态读回

## 结论

**当前单场景、CPU 状态读回的数据接口下，GPU PhysX 更慢，保留 CPU 物理作为默认。**
两组各连续录制两条完整 18 秒抓取，均通过物理和同步检查，且修正容量后的运行没有原生 PhysX 错误。

| 同条件配置（均关闭 CCD） | 核心步进／条 | 采集墙钟折算／条 | 合格条数／分钟 | 两条含启动总耗时 | 显存峰值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| CPU dynamics + MBP | 29.15 s | 31.57 s | 1.90 | 75.87 s | 3.61 GiB |
| GPU dynamics + GPU broadphase，CPU 读回 | 109.24 s | 113.45 s | 0.53 | 243.61 s | 3.98 GiB |

GPU 采集墙钟约为 CPU 的 **3.59 倍**，没有继续增加 GPU PhysX 并发度。
采集墙钟包含写盘、文件验收和两条之间 reset，不包含首次启动／reset、IK、组合预览或退出。
两条总耗时另包含启动、首次 reset 和退出。CPU 默认开启 CCD 的历史结果为约 31.62 s／条，
本次关闭 CCD 后为 31.57 s／条，且全部内部数值与原 CPU 基线相同。

### 时间增加在哪里

| 每次调用的墙钟均值 | CPU | GPU |
| --- | ---: | ---: |
| 只推进物理的 step | 5.07 ms | 42.29 ms |
| 物理＋渲染 step | 28.99 ms | 65.37 ms |
| 相机图像读取／同步 | 6.14 ms | 6.13 ms |
| FlexiTac 更新 | 0.610 ms | 0.654 ms |

增加的成本主要落在物理步进路径。GPU 利用率采样均值约 80.9%，CPU 物理对照约 29.6%；
这不能单独证明瓶颈就是读回或某个 CUDA 内核。本轮未做 GPU 内核级剖析。
小场景中的 GPU 求解、同步和状态回读成本可能抵消并行收益，这一解释是推断。
上述结果不代表数据全程驻留 GPU、同场景批量环境也会更慢。

### 正确性和差异

- GPU 两条抬升范围约 **49.921–49.960 mm**，保持阶段最大滑移 **0.0791 mm**；全部原抓取判据通过。
- 两种模式共 4/4 条通过，视频真实时钟误差为 0，无额外同步刷新；每台相机每条 540 帧。
- CPU 对照全部 2160 步状态与触觉和原参考逐项相同。
- GPU 与 CPU 有差异：物体单坐标最大绝对差约 **0.291 mm**；关节位置数组最大差 0.005345，
  关节速度数组最大差 1.01072；四片归一化触觉最大绝对差 **0.0885**。关节数组按原定义含 rad 与 Lift 的 m 单位。
  两种引擎不能被声明为数值等价，也不应无标识混用为同一动力学基线。
- 每种模式自身的两条 30 Hz 保存数值逐项一致；共用离线读取四条通过，RGB 内容变化检查通过。
- 已查看 GPU 首条的 RGB＋下方四片触觉预览；24 项单元检查和相关 Ruff 检查通过。

## 测试范围

本轮将同一场景的 PhysX 动力学和 broadphase（碰撞候选配对）切换为 GPU，
保留现有 NumPy 控制／观测、CPU 状态读回和每物理步的 USD 位姿更新。
四路机器人 RGB、观察相机与 FlexiTac 原本已使用 GPU，本轮没有改变它们的分辨率、计算方式或频率。
GPU 全程驻留的 Torch/Fabric 数据接口、同场景批量环境不在本轮范围。

基于已安装 Isaac Sim 5.1 的 `PhysicsContext` / `SimulationManager` API，
在首次 reset 前启用 GPU dynamics 和 GPU broadphase；`suppressReadback=False`、backend 为 NumPy。
运行报告同时记录 `gpu_dynamics_enabled`、`broadphase`、`ccd_enabled`、`solver_type` 和缓冲容量。
历史 `physics_device` 字段来自 `SimulationManager.get_physics_sim_device()`，
该接口在 CPU 读回模式下返回 `cpu`，因此不能只靠这个字段判断 PhysX 求解器是否已切到 GPU。

CPU 与 GPU 都采用 TGS、32／4 次关节求解迭代、每进程 8 个全局及 8 个 PhysX 工作线程。
物理／控制／内部触觉 120 Hz，保存 30 Hz。原资产、凸分解参数、接触偏移、摩擦、驱动及验收阈值不变。
本机 Isaac Sim 的 GPU 开关会关闭 CCD，因此 CPU 对照也显式使用 `--no-ccd`。
CPU 默认配置不变；此前默认开启 CCD 的基线保留，用完整内部参考序列检查关闭 CCD 的影响。

GPU 碰撞与 CPU 碰撞可能有数值差异，不要求两种求解器逐位一致；
仍使用同一抬升、保持、滑移、物理接触、触觉、释放、自碰撞及视频同步验收。
GPU 也可能将不适合的凸包接触生成留在 CPU，不能称作全部计算都在 GPU。
相关能力边界见 [PhysX GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GPURigidBodies.html)。

## 首次无效运行与修正

`output/dexmate_physx_gpu_20260916_v1/` 在首次模拟／reset 报错：
`foundLostAggregatePairsCapacity` 默认 1024，PhysX 要求至少 1313，否则漏掉交互。
已主动中止并标记无效，不纳入正确性或性能结论。

GPU 分支在首次模拟前将容量提高至 **4096**，保留余量；CPU 分支不变。
协调器新增原生 PhysX 错误检查：即使脚本抓取判据通过，只要原生日志出现 PhysX 错误，
该运行仍不算有效合格吞吐。测试覆盖了缓冲区不足的真实错误文本与普通 USD 警告的区分。

## 复现

从仓库根目录执行，分别使用新目录与 `--physics-mode cpu` / `gpu`：

```bash
conda activate dexmate_isaacsim
./tools/run_dexmate_sim.sh tools/benchmark_dexmate_parallel.py \
  --workers 1 --episodes 2 --physics-mode gpu --no-ccd \
  --workaround-r535-vulkan \
  --reference Isaacsim_tactile_env/output/dexmate_pose_sweep_20260916_v1/episode_000/trajectory.npz \
  --output Isaacsim_tactile_env/output/gpu_physics_new
```

基准仍回放完整 2160 步 120 Hz 动作，记录五路视频并做原有全频验收；不含 IK 与组合预览。
CPU 是默认物理模式。GPU 仅作为显式实验选项，不自动提升为默认采集配置。

## 证据

- 有效 GPU：`output/dexmate_physx_gpu_20260916_v2/`。
- 同条件 CPU：`output/dexmate_physx_cpu_control_20260916_v1/`。
- 原生错误检查位于协调器；`physical_passed_count` 与最终有效 `passed_count` 分开，出现原生错误时有效吞吐为 0。
- [数值、计时、离线审计与证据哈希](2026-09-16_gpu_physics_summary.json)。
