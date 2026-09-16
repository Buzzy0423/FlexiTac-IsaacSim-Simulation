# Dexmate FlexiTac 接入

## 参数和坐标

参数集中在 `Isaacsim_tactile_env/dexmate_workspace.json` 的 `tactile` 段。
适配器为 `Isaacsim_tactile_env/dexmate_flexitac.py`，在 SimulationApp 启动后创建实例。

- 四指顺序：`L_gripper_l1`、`L_gripper_l2`、`R_gripper_l1`、`R_gripper_l2`。
- 每片 12×32 点，行沿宽度、列沿长度，间距 2 mm；中心跨度 22×62 mm。
- 从当前 manifest 对应的 STL 读取 66×26 mm 凸起，以面中心为基准向外偏移 2.1 mm。
- 两种手指均使用 manifest 中的源网格到连杆变换，包括第二指的镜像与微小装配偏移。
- 每次更新使用四指当前世界位姿；实体 URDF、视觉、碰撞网格无需因触觉接入改变。

这里只放置一片虚拟采样面，没有另建 2.1 mm 厚的实体垫片或体素体积。

## 距离与输出

ALOHA 和 Dexmate 共用 `source/isaaclab/isaaclab/sensors/warp_sdf_tactile/warp_sdf_kernels.py`。
Dexmate 默认只查询配置中明确列出的 `target_prim_paths`；测试回归额外注册两个固定测试块。
多个目标取逐点最小有符号距离。距离单位为米：

```text
penetration = max(-distance, 0)
value = clamp(5000 * penetration, 0, 10) / 10
```

点位于物体外时输出 0；进入物体 0.5、1、2 mm 时分别输出 0.25、0.5、1。
这是单点距离响应，不能将数值相加当成真实总力或压力。
由于采样面在实体外侧，输出可以先于实体接触出现，与原 ALOHA 的处理一致。

## 原生 Isaac Sim 调用

在 `SimulationApp`、场景和机器人初始化完成后：

```python
from pathlib import Path
from Isaacsim_tactile_env.dexmate_flexitac import DexmateFlexiTac

sensor = DexmateFlexiTac(
    stage, "/World/Dexmate",
    Path("Isaacsim_tactile_env/assets/dexmate"), scene_config["tactile"],
)
world.step(render=True)
grid = sensor.update()  # numpy float32 (4, 12, 32), [0, 1]
points = sensor.tactile_points_w  # (1, 1536, 4), world xyz metres + value
```

每个物理步之后调用一次；当前验证场景为 120 Hz。
验证入口将 `physics_dt` 与 `rendering_dt` 均设为 `1/120`，并断言同阶段相邻记录的物理步号差为 1。
Torch 与 Warp 使用同一 CUDA stream，读回 NumPy 时完成同步。
该接口服务当前单场景，不是 Isaac Lab 的并行训练环境／Gym step-reset 接口。
带 reset/step 和四相机同步记录的上层封装见[环境指南](environment.md)。

### 新增目标物体

将目标刚体的根 prim 路径加入 `target_prim_paths`。当前支持该刚体下的闭合、朝向一致的 Mesh 和 Cube。
几何子节点变换及缩放烘焙为米制查询网格，根节点的平移／旋转随物体更新；缩放改变需要重建传感器。
guide purpose 的碰撞副本跳过，避免和视觉面重复。
目标根应与运动刚体一致，不支持变形网格、开放曲面或含多运动刚体的装配体。
桌子和机器人不会自动成为触觉目标；机器人及其祖先／子节点不能注册。

## 验证及输出

按[运行指南](runtime.md)启用 `--contact-regression`，输出包含：

- `tactile_sequence.npz`：`tactile` 为 `(frames, 4, 12, 32)`；另存阶段、阶段内步号、指名和局部采样位置，
  `time_s` 为仿真时间、`physics_step` 为全局物理步号。阶段间生成预览会推进仿真，跨阶段时间不保证连续。
- `tactile_metadata.json`：配置、目标、四指局部坐标、各阶段峰值与归零断言。
- `tactile_contact.png`：同一时刻的四指横向 12×32 热图，固定 0～1 色标、蓝色为零。
- `tactile_stages.png`：记录开始、接触、记录结束的三组热图；接触回归中对应空夹、接触、释放。
  早期 v1～v3 输出的 `tactile_peaks.png` 是旧版竖向显示，分别选各指峰值帧，不能当作同时读数。
- `validation.json`：同时检查实际接触冲量和触觉输出，防止把虚拟距离响应误当成物理接触证据。

设置 `tactile.enabled=false` 可关闭适配器。最新通过范围见 [STATUS](../STATUS.md)。

已有 NPZ 可离线重新绘图，无需启动 Isaac Sim：

```bash
python tools/dexmate_tactile_preview.py \
  Isaacsim_tactile_env/output/dexmate_tactile_20260915_v3/tactile_sequence.npz \
  --output Isaacsim_tactile_env/output/tactile_review_new --gif
```

`--gif` 额外生成 `tactile_replay.gif`，从 120 Hz 数据每六帧取一帧，以 20 fps 回放。
图中每格对应一个采样点，没有空间插值；行沿指面宽度、列沿长度。
原 ALOHA 演示窗口按各片当前最大值重新映射颜色；本预览固定 0～1 色标，方便比较四指和不同时刻的强度。
