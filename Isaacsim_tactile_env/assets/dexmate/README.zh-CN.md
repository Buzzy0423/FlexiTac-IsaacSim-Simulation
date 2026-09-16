# Dexmate Vega-1U 定制资产

当前入口：`vega_1u_gripper_flexitac.urdf`，几何版本 `bump_v2`。
基线是 `dexmate-urdf==0.8.4`；原版资源与 LICENSE 保留在 `upstream/`。

- 四根手指均使用定制 26 mm TPU 指体，保留金属件与安装关系。
- 仿真凸起相对原始 STL 减少 **0.5 mm**，位于 `meshes/bump_v2/simulation/`。
- [打印 STL](meshes/bump_v2/printing/gripper_finger_26mm_bump_minus_0p8mm_print.stl) 相对同一原件减少 **0.8 mm**，单位毫米。
- `manifest.json` 和 `meshes/bump_v2/revision.json` 保存来源、变换及哈希；原始输入和历史版本不覆盖。
- 零位中心截面间隙约 0.009 mm，仅为几何证据；完整接触验证状态见下方入口。

[当前状态](../../../docs/STATUS.md) · [已确认决策](../../../docs/decisions.md) ·
[资产维护与重建](../../../docs/guides/assets.md) · [准备过程归档](../../../docs/reports/2026-09-15_asset_preparation.md)
