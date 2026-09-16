# 文档入口

**恢复工作先读 [当前状态](STATUS.md)**。它是当前进展和下一步的唯一维护入口。
无需默认阅读全文、历史日志或上游 Isaac Lab 文档。

| 当前任务 | 按需阅读 |
| --- | --- |
| 了解目录职责、寻找代码入口 | [项目结构](architecture.md) |
| 修改手指、桌子位置或模态定义 | [已确认决策](decisions.md) |
| 安装、启动、查看运行结果 | [运行指南](guides/runtime.md) |
| 修改 Dexmate 触觉采样、目标物体和输出接口 | [触觉接入指南](guides/tactile.md) |
| 验证可移动物体的夹持、抬升和释放 | [抓取验证指南](guides/grasp.md) |
| 使用 reset/step 或读取四相机同步序列 | [环境接口与数据指南](guides/environment.md) |
| 修改／重建机器人、打印件、桌子资产 | [资产指南](guides/assets.md) |
| 查看自装腕相机效果、名义内参与实机设备 | [仿真报告](reports/2026-09-16_wrist_camera_simulation.md)、[设备报告](reports/2026-09-16_wrist_camera_devices.md) |
| 寻找本机视频、数值记录或清理记录 | [本地输出索引](../Isaacsim_tactile_env/output/README.md)、[清理摘要](reports/2026-09-16_output_cleanup.json) |
| 查看首轮成功及失败证据 | [2026-09-15 验证报告](reports/2026-09-15_first_validation.md) |
| 追查指体初版来源和修改过程 | [资产准备归档](reports/2026-09-15_asset_preparation.md) |

## 维护规则

- `STATUS.md`：保持约 70 行以内，只写当前状态、限制和下一步；完成任务后更新。
- `decisions.md`：只记录仍有效的用户决定与约束，修改时说明日期与理由。
- `guides/`：操作方法和参数解释，不维护另一份进度表。
- `reports/`：按日期保留测量结果、失败与验证条件；更正历史事实时注明更正。
- 全量 JSON、日志、USD 和相机帧放在被忽略的 `output/`；有长期价值的证据抽取成简短报告。
- `output/README.md` 仅为本机导航，未随仓库分发；历史运行的导入网格／USD 可清理，原始输入、维护资产及采集证据保留。
- 阅读预算：默认只读根目录 `AGENTS.md` 和 `STATUS.md`，再按任务打开一个指南。

根目录 [README](../README.md) 保留上游 ALOHA 使用说明；[旧入口](dexmate_sim_setup.zh-CN.md) 仅作导航。
