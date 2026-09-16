# 当前状态

更新：2026-09-16。目标：Dexmate + FlexiTac 单场景环境，为固定四相机布局的视觉／触觉序列研究提供数据。

## 当前可用

- 独立 `dexmate_isaacsim` 环境：Isaac Sim 5.1.0、CAD 转换、RTX 渲染可用；未安装整套 Isaac Lab。
- 官方 Vega-1U 定制副本，当前手指 `pad_66x26_v3`：凸起 66×26 mm，仿真／打印比原 STL 降低 0.5／0.8 mm。
  官方 Vention 桌子与桌板已接入；机器人居中面向短边、底座贴桌架；[固定决策](decisions.md)。
- 自碰撞开启，左手方块夹持→抬升 5 cm→保持 2 秒→放回／松开通过；[抓取报告](reports/2026-09-15_dynamic_grasp.md)。
- 四片 FlexiTac 沿凸起外法线偏移 2.1 mm，每指 12×32 点、2 mm 间距；[触觉指南](guides/tactile.md)。
- 原生 `DexmateEnv.reset()/step(action)` 支持 21 个独立关节位置目标与带时间观测。
  recorder v2 将 RGB／状态／动作／触觉统一保存为 30 Hz，物理／控制及内部验收为 120 Hz。
  仿真和实机共用 Dex Wire 离线读取，兼容旧记录；[环境指南](guides/environment.md)。
- **自装 C960 腕相机已激活**：视觉／碰撞及 STL 名义光学坐标接入；640×360、90° 对角视场。
  候选两轮＋默认入口一轮抓取 **3/3 通过**，同步误差 0、最大滑移 0.0514 mm；
  [仿真报告](reports/2026-09-16_wrist_camera_simulation.md)。实机 udev 双路 720p 约 30 fps；
  [设备报告](reports/2026-09-16_wrist_camera_devices.md)。内外参仍为名义值，当前不以精确标定为前置条件。

## 验证范围与历史基线

- 原连续三轮 reset／抓取通过；原相机配置下位置／朝向变化 **10/10 轮通过**，最大滑移 0.156 mm。
  新 C960 配置目前仅覆盖上述三轮固定轨迹；[十轮报告](reports/2026-09-16_pose_sweep.md)。
- CPU 接触优化后核心采集约 **29.8 s／轮**，保留原数值和判据；
  [性能报告](reports/2026-09-16_contact_performance.md)。单 GPU 四进程约 3.47 条／分钟，
  [并行报告](reports/2026-09-16_parallel_performance.md)；持久化 RTX 缓存启动约 40.4 s，
  [缓存复测](reports/2026-09-16_startup_cache.md)。当前 CPU 读回接口下 GPU PhysX 更慢，继续默认 CPU；
  [物理对照](reports/2026-09-16_gpu_physics.md)。这些性能数值来自更换腕相机前的运行。

## 当前入口与证据

- 启动与参数：[运行指南](guides/runtime.md)；环境：`Isaacsim_tactile_env/dexmate_env.py`；
  多轮记录：`tools/run_dexmate_episodes.py`。两者与旧验证入口共用 `create_scene()`；原 ALOHA 示例独立保留。
- 最新腕相机数据：`output/wrist_camera_active_20260916_v1/episode_000/`，组合视频 `four_cameras_preview.mp4`。
- 十轮历史数据：`output/dexmate_pose_sweep_20260916_v1/review.html`；优化基线：`output/dexmate_cpu_final_20260916_v1/`。
- 完整本地导航：[output/README](../Isaacsim_tactile_env/output/README.md)；全量输出不提交，长期摘要保留在 `reports/`。
- 本次清理释放 **3.17 GiB**：移除重复导入网格、USD 导出和三份相同预览文件；所有视频、数值记录、
  报告与源／维护资产保留并核对哈希；[清理记录](reports/2026-09-16_output_cleanup.json)。历史场景需重新导出后才能加载。

## 尚未覆盖

1. **任务**：其他物体、右手／双手、全关节避碰；新腕相机下的十轮位置／朝向变化尚未复测。
2. **相机**：内参、光轴／外参和裁剪为名义值；视频为有损 H.264。用户提供的实机录像为 960×600，
   与当前仿真 640×360 宽高比不同，模型输入的裁剪／缩放方式待统一；外观相似不等于像素配准。
3. **物理**：驱动、惯量、摩擦为初值；触觉是距离响应，不能直接等同真实力或压力。
4. **规模**：每进程仍为单场景；未实现同场景批量渲染、随机化、Gym 注册或并行训练。

## 下次从这里继续

1. 沿用名义相机参数，复测十轮位置／朝向变化，检查视野、碰撞、夹持、同步、reset 和离线读取。
2. 明确实机／仿真的图像处理、时间采样与数据划分；重复及近邻姿态不要随意拆到训练／验证两侧。
3. 扩展物体或右手前补充对应回归；扩规模先沿用 CPU 物理＋多进程，验证常驻、多姿态稳定性。
