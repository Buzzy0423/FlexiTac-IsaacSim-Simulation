# Dexmate Vega-1U 定制资产

当前入口：`vega_1u_gripper_flexitac.urdf`，手指版本 `pad_66x26_v3`，腕相机版本 `wrist_c960_v1`。
基线是 `dexmate-urdf==0.8.4`；原版资源与 LICENSE 保留在 `upstream/`。

- 四根手指均使用定制 26 mm TPU 指体，保留金属件与安装关系。
- 凸起平面 **66 × 26 mm**，适配用户提供的 64 × 24 mm FlexiTac 外形，居中四周各留 1 mm。
- [仿真 STL](meshes/pad_66x26_v3/simulation/gripper_finger_pad_66x26_bump_minus_0p5mm_sim.stl) 保持相对原始 STL 低 **0.5 mm**；URDF 四指视觉／碰撞均已切换到该版本。
- [打印 STL](meshes/pad_66x26_v3/printing/gripper_finger_pad_66x26_bump_minus_0p8mm_print.stl) 保持相对同一原件低 **0.8 mm**，单位毫米。
- `manifest.json` 和 `meshes/pad_66x26_v3/revision.json` 保存来源、变换及哈希；原始输入与 `bump_v2` 历史版本保留。
- 零位中心截面间隙约 0.009 mm，仅为几何证据；完整接触验证状态见下方入口。
- 左右腕使用用户 C960 外壳／支架，移除旧安装件；[版本记录](meshes/wrist_c960_v1/revision.json)
  保存镜头前端名义光心、坐标变换、碰撞近似与源哈希，修改前 URDF／manifest 保留在同目录。
  相机内参采用官方 90° 对角视场估算，不是实机标定；验证范围见[仿真报告](../../../docs/reports/2026-09-16_wrist_camera_simulation.md)。

[当前状态](../../../docs/STATUS.md) · [已确认决策](../../../docs/decisions.md) ·
[资产维护与重建](../../../docs/guides/assets.md) · [准备过程归档](../../../docs/reports/2026-09-15_asset_preparation.md)
