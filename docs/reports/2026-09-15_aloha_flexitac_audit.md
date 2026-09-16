# ALOHA FlexiTac 配置核对

核对范围：当前源码默认配置及网格离线测量；本次未启动 ALOHA 仿真，未修改其配置。

## 尺寸与采样

- `assets/meshes/aloha_flat_pad.obj` 外包络为 X=±13、Y=±1.5、Z=±33.325 mm，
  即 **宽 26 × 长 66.65 × 厚 3 mm**。四片垫片引用同一模型。
- 环境默认每片 **12 × 32 = 384 点**，点距 2 mm，共四片；观测形状 `(4, 12, 32)`。
- 网格生成函数舍去扩展网格两端，实际点中心范围为短向 ±11 mm、长向 ±31 mm，
  首末点跨度 **22 × 62 mm**；不能把垫片外形尺寸、点中心跨度和阵列分辨率混为一谈。
- 默认 `normal_axis=0`、`normal_offset=+3.6 mm`，patch 四元数为绕 Z -90°。
  在默认独立 elastomer 连杆坐标中，采样面因此位于 **Y=-3.6 mm**；对应网格外表面 Y=-1.5 mm，
  采样面比几何表面突出约 **2.1 mm**。这是采样偏移，不是垫片厚度。

## 计算方法

传感器为 `WarpSdfTactileSensor`：将每个采样点随指体转换至世界坐标，再转换至目标网格坐标，
使用 Warp 查询最近表面距离及内外符号。默认使用有符号距离（winding 模式）。

对每点：`penetration = max(-signed_distance, 0)`；
`force_like = clamp(5000 * penetration, 0, 10)`；最终返回 `force_like / 10`，范围 0–1。
若距离以米计，0.1／0.5／1／2 mm 的侵入深度分别得到 0.05／0.25／0.5／1。
代码未乘采样面积，所以不是压强模型，不能把点值相加直接视为 PhysX 测得的夹持力。

`mesh_shell_thickness=1 mm` 仅在无符号距离模式中用于 `max(shell-distance, 0)`；默认有符号模式不使用它。
法向平滑只影响 `normal` 符号模式，默认 winding 不使用平滑法向分支。
每个环境步更新，默认物理步长 1/120 s；没有该传感器内部的切向力、迟滞或弹性体变形求解。

## 接入 Dexmate 前需保留的边界

- ALOHA 环境覆盖传感器通用默认值：实际 `max_force=10`、`normal_offset=+3.6 mm`，
  不能误用传感器基类中的 2.4 和 -3.2 mm。
- 默认左手只查询 Socket、右手只查询 Plug；解析器选择目标下第一个 Mesh，未自动合并所有网格或感知所有场景物体。
- 查询点会除以目标 scale，但距离结果未乘回世界尺度；缩放目标（如默认 Plug scale=1.06）需要在接入时统一单位。
- 这是基于几何距离的触觉信号合成；即便订阅了 ContactSensor，也只借用位姿，不读取 PhysX 接触力来生成点值。
- Dexmate 的凸起已经表示最终传感器／防滑层外表面，应按实际凸起建立采样面，单独决定有效感知偏移，
  不直接照搬 ALOHA 的 2.1 mm 外凸采样面或重复增加实体垫层。

源码入口：
[环境配置](../../Isaacsim_tactile_env/aloha_tactile_cfg.py)、
[环境绑定](../../Isaacsim_tactile_env/aloha_tactile_env.py)、
[传感器实现](../../source/isaaclab/isaaclab/sensors/warp_sdf_tactile/warp_sdf_tactile_sensor.py)。
