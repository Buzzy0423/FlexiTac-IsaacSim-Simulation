# 项目结构与职责

本仓库保留上游 Isaac Lab/FlexiTac 代码，并在项目层增加 Dexmate 资产和验证工具。
当前进度只维护在 [STATUS.md](STATUS.md)。

| 路径 | 职责／何时读取 |
| --- | --- |
| `AGENTS.md`、`docs/` | 工作入口、当前状态、按需指南和历史证据 |
| `environment_dexmate.yml` | 新仿真环境依赖声明 |
| `Isaacsim_tactile_env/dexmate_workspace.json` | 桌子来源、工作区坐标、机器人位姿、触觉与仿真相机参数 |
| `Isaacsim_tactile_env/dexmate_wrist_cameras.json` | 实机腕相机 udev 映射、采集配置与待标定字段 |
| `Isaacsim_tactile_env/dexmate_flexitac.py` | 原生 Isaac Sim 四指触觉适配器，复用 ALOHA 的距离内核与弹簧响应 |
| `Isaacsim_tactile_env/dexmate_env.py` | 单场景 reset/step、动作边界检查、任务进度及真实相机时间同步 |
| `tools/run_dexmate_episodes.py`、`tools/dexmate_episode_recording.py` | 多轮抓取演示、重置一致性、流式视频和状态索引 |
| `tools/dexmate_variations.py` | 小范围物体 XY/yaw 清单及绕物体中心的抓取位姿变换 |
| `tools/review_dexmate_sweep.py`、`tools/audit_dexmate_sweep.py` | 四相机／触觉汇总查看页，以及使用 Dex Wire 共用读取器的离线审计 |
| `tools/dexmate_tactile_validation.py` | 逐物理步记录触觉、接触／释放断言和热图输出 |
| `tools/dexmate_grasp_motion.py`、`tools/dexmate_grasp_regression.py` | 左臂 URDF IK／轨迹及自由物体夹持、抬升、放回验证 |
| `Isaacsim_tactile_env/assets/dexmate/` | 官方机器人快照、当前定制 URDF、手指与 C960 安装件、原始输入及来源清单 |
| `Isaacsim_tactile_env/assets/VentionAssembly_312098_v6.STEP` | 用户提供的官方桌子源文件 |
| `Isaacsim_tactile_env/assets/vention_312098_v6/` | 受维护的桌子网格和转换 manifest |
| `tools/build_dexmate_assets.py` | 从指定官方版本与原始 STL 建立资产快照 |
| `tools/build_dexmate_wrist_camera.py`、`tools/dexmate_camera_model.py` | 自装 C960 派生／激活及名义内参计算 |
| `tools/reduce_dexmate_bump.py` | 分别生成仿真 −0.5 mm、打印 −0.8 mm 指体 |
| `tools/preview_dexmate.py`、`tools/dexmate_preview.html` | 无 Isaac Sim 的几何与截面预览 |
| `tools/convert_vention_step.py` | 官方 STEP 的实体及开放壳体转换 |
| `tools/run_dexmate_sim.sh` | 专用 Conda 环境的进程级启动设置 |
| `tools/check_dexmate_runtime.py` | 基础 PhysX 与真实 RTX 渲染检查 |
| `tools/validate_dexmate_scene.py` | 生成运行用 URDF／网格、导入场景、开合与相机验证 |
| `Isaacsim_tactile_env/aloha_tactile_{cfg,env}.py` | 原 ALOHA 触觉环境，可按需参考其传感器生命周期 |
| `source/isaaclab/isaaclab/sensors/warp_sdf_tactile/` | FlexiTac 距离／刚度触觉实现，接入传感器时读取 |
| `Isaacsim_tactile_env/output/` | 每次运行独立目录；USD、图像、日志、全量报告，不提交 |

## 数据流

源 STEP／STL／官方 URDF → 项目维护的网格与 manifest → 工作区配置 → 运行副本 → PhysX／RTX 验证。

运行副本烘焙 GLB 节点变换，并覆盖零驱动限值，不回写源 URDF。
其中 `create_scene()` 同时供旧验证入口和新环境使用，避免两套机器人／碰撞配置分叉。
源文件、受维护的派生资产和哈希清单一起提交；运行导入网格与 USD 可重建，视频、数值和报告作为证据保留。
USD 默认忽略；明确维护的 USD 资产仅允许放在 `Isaacsim_tactile_env/assets/`。

相邻 `dex_wire` 仓库负责研究计划和数据／模型工作，本仓库负责仿真实现。
研究背景文件名为 `2026-09-15_temporal_multimodal_concepts.zh-CN.md`；它不属于恢复仿真工作的必读上下文。
