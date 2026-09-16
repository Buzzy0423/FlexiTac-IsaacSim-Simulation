# Dexmate 自碰撞修复

范围：保持近端桌板和底座贴桌布局，修复当前初始臂姿态下的自碰撞；不改变源 URDF、手指 STL 或官方 CAD。

## 原因与修正

1. **固定中间帧漏过滤**：`torso_flip_link → arm_center → 肩部／head_l1` 中，
   `arm_center` 是固定中间帧。导入后的直接关节过滤未覆盖这个装配关系。
   现在按 URDF 固定连接归并刚体组，仅过滤组内及直接相邻组。保留所有非相邻关系和同手两指关系。
2. **接触偏移未生效**：碰撞体位于实例子树，旧 `Usd.PrimRange` 遍历跳过实例内部，
   实际写到的碰撞体数量为零。现在只解除 28 个碰撞子树的实例化，并确认全部 57 个碰撞体
   设置 `contactOffset=0.0002 m`、`restOffset=0`。视觉实例保留。
3. **凸分解填实空隙**：原网格在名义姿态下头二连杆与躯干约有 9.71 mm 最近距离，
   原近似仍产生接触。启用 shrink wrap、1% 误差、最多 64 个凸包、0.05 mm 最小厚度后，异常接触消失。
4. **mimic 耦合过软**：导入器自然频率 25 rad/s、阻尼比 0.005 导致受阻时从动指偏差约 0.1 rad。
   改为 5000 rad/s、阻尼比 1，保留单驱动＋mimic，接触下两指角差降至约 0.001 rad。
   这些是数值仿真参数，未声明为实机刚度标定值。

同时在 `World.reset()` 内部第一次物理步之前写入初始关节状态，避免先在全部零位求解碰撞。
运行层修正由 `tools/dexmate_collision.py` 维护；拓扑覆盖与接触事件分类有轻量单元测试。

过滤 API 的语义参考 [NVIDIA pairwise filtering 文档](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/collision.html#pairwise-collision-filtering)。
凸分解和 mimic 默认值同时核对了本机 Isaac Sim 5.1 导入结果及扩展定义。

## 诊断证据

本机目录前缀：`Isaacsim_tactile_env/output/dexmate_collision_20260915_`，原始输出不进入 Git。

| 运行 | 条件／结论 |
| --- | --- |
| d1 | 原始碰撞，短时诊断；装配内部碰撞在初始化阶段出现，失败 |
| d2 / d4 | 修正相邻过滤／初始状态；可稳定开合，但未触及实例内部，仍有假接触 |
| d3 | 直接修改实例代理被 USD 拒绝；作为失败证据保留 |
| d5 | 全部 57 个接触偏移生效；暴露头部近似碰撞，最大误差约 0.0349 rad，失败 |
| d6 | 凸分解细化后，开启自碰撞的自由开合通过；只出现同手 TPU 指面接触 |
| d7 / d8 | 固定块正向接触测试暴露过软 mimic；从动指不能持续受力，失败 |
| d9 | mimic 修正后联动正常；固定块只能维持单侧受力，双侧同时保持的夹具判据不适用 |
| d10 | 完整开启自碰撞、四相机及两个位置的固定块接触／释放回归通过，退出码 0 |

固定块无法像可移动物体一样自行居中，因此最终回归使用两个相差 2 mm 的横向位置，
分别确认四根手指均能持续产生接触冲量、重新张开后冲量消失。
这属于碰撞功能回归，不是成功抓取物体的证明。

最终结果保存于[精简证据 JSON](2026-09-15_self_collision_summary.json)：

- USD 的 `enabledSelfCollisions=true`，全部 57 个碰撞体的偏移配置已生效。
- 三段自由开合最大关节目标误差约 0.01063（角度 rad／升降位移 m），低于既定 0.03 阈值。
- 自接触仅出现于左右手各自的 TPU 对向指面；没有底座／躯干、头部／躯干等异常接触。
- 两种固定块位置下，每根手指至少一次在保持段末尾持续产生冲量；最终释放段末尾全部无冲量。
- 两种受力位置下最大 mimic 角差分别约 0.000812 rad 和 0.0000721 rad。
- 板上方块静止接触、场景三视图和四路 640×480 相机输出通过。
- 三项离线单元测试、Ruff、资产哈希及文档链接检查通过。

复现（在 `dexmate_isaacsim` 环境中，输出目录必须不存在）：

```bash
./tools/run_dexmate_sim.sh tools/validate_dexmate_scene.py \
  --workaround-r535-vulkan --four-cameras --contact-regression \
  --output Isaacsim_tactile_env/output/dexmate_collision_new
```

## 仍未覆盖

- 全关节范围、运动规划路径及随机初始姿态。
- 自由物体的抓取保持、真实摩擦／柔性、实机力标定。
- FlexiTac 四片采样面和触觉响应；相机参数也仍为名义值。

后续使用[运行指南](../guides/runtime.md)的默认自碰撞配置；不要重新使用全局关闭来绕过后续任务中的碰撞。
