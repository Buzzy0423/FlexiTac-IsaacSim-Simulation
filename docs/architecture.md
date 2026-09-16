# 项目结构与职责

本仓库保留上游 Isaac Lab/FlexiTac 代码，并在项目层增加 Dexmate 资产和验证工具。
当前进度只维护在 [STATUS.md](STATUS.md)。

| 路径 | 职责／何时读取 |
| --- | --- |
| `AGENTS.md`、`docs/` | 工作入口、当前状态、按需指南和历史证据 |
| `environment_dexmate.yml` | 新仿真环境依赖声明 |
| `Isaacsim_tactile_env/dexmate_workspace.json` | 桌子来源、工作区坐标、机器人位姿 |
| `Isaacsim_tactile_env/assets/dexmate/` | 官方机器人快照、当前定制 URDF、两版手指及来源清单 |
| `Isaacsim_tactile_env/assets/VentionAssembly_312098_v6.STEP` | 用户提供的官方桌子源文件 |
| `Isaacsim_tactile_env/assets/vention_312098_v6/` | 受维护的桌子网格和转换 manifest |
| `tools/build_dexmate_assets.py` | 从指定官方版本与原始 STL 建立资产快照 |
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
源文件、受维护的派生资产和哈希清单一起提交，运行输出可重建。
USD 默认忽略；明确维护的 USD 资产仅允许放在 `Isaacsim_tactile_env/assets/`。

相邻 `dex_wire` 仓库负责研究计划和数据／模型工作，本仓库负责仿真实现。
研究背景文件名为 `2026-09-15_temporal_multimodal_concepts.zh-CN.md`；它不属于恢复仿真工作的必读上下文。
