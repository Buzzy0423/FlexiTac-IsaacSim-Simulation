# 2026-09-15 指体资产准备记录（归档）

本文件保存资产准备阶段的说明，包含当时尚未安装 Isaac Sim 的历史状态。
当前进展以 [STATUS.md](../STATUS.md) 为准；文内命令与相对路径按原资产目录解释，
后续操作请使用 [资产指南](../guides/assets.md)。本记录不进入默认阅读范围。

# 本机 Dexmate Vega-1U 仿真资产

**2026-09-15 后续运行验证：** 已创建独立 Isaac Sim 环境并完成导入及部分驱动／渲染检查，
接入官方 Vention STEP。自碰撞开启时仍有内部干涉，FlexiTac 触觉尚未接入。
见 [仿真配置与验证范围](../guides/runtime.md)。下文“尚未导入”的描述保留为资产创建阶段记录。

此目录由 FlexiTac 仿真项目维护，作为后续定制的起点。2026-09-15 创建；
当前完成资产和指体几何适配，尚未通过 Isaac Sim 导入、动力学或实机接触验证。

**当前版本：bump_v2。** 按用户确认，仿真凸起相对最初 STL 减少 **0.5 mm**，
打印凸起相对同一 STL 减少 **0.8 mm**。下方早期闭合重叠记录属于初版，不代表当前模型。

## 当前仿真与打印文件

- 仿真入口仍为 `vega_1u_gripper_flexitac.urdf`，四根手指的视觉与碰撞资源均已切换到
  `meshes/bump_v2/simulation/`。
- 打印文件：
  [`gripper_finger_26mm_bump_minus_0p8mm_print.stl`](../../Isaacsim_tactile_env/assets/dexmate/meshes/bump_v2/printing/gripper_finger_26mm_bump_minus_0p8mm_print.stl)。
  **STL 坐标单位为毫米**，宽度 26 mm；这是打印件本体，未增加额外垫片。
- 仿真用毫米 STL：
  [`gripper_finger_26mm_bump_minus_0p5mm_sim.stl`](../../Isaacsim_tactile_env/assets/dexmate/meshes/bump_v2/simulation/gripper_finger_26mm_bump_minus_0p5mm_sim.stl)。
- 原始 STL、初版生成网格和官方资产均保留；`meshes/bump_v2/revision.json` 记录减量与哈希。
  `previous_urdf.xml` / `previous_manifest.json` 保存激活前的文本快照；其中相对路径仍以本资产根目录解释。

两个减量都以用户最初提供的 `raise_2mm.stl` 为基准，不是先减 0.5 mm 再减 0.8 mm。
仅将矩形凸起接触面的四个角点沿局部 -X 移动，连接三角面随之缩短。
其余顶点、孔位、26 mm 宽度、镜像变换、安装关系和关节参数保持不变。
凸起最高面的 X 从 26.296568 mm 改为仿真版 25.796568 mm、打印版 25.496568 mm。

仿真版在 `q=0` 的中心截面采样未发现重叠，最小 X 向间隙约 **0.009 mm**，
接近用户要求的零位轻触；没有为追求数学上的零间隙额外修改指定的 0.5 mm 减量。
该值属于几何检查，尚未包含 PhysX 碰撞近似、接触容差或材料形变。

**打印与垫片厚度：** 当前打印面比仿真面低 0.3 mm；若后续每侧再加 1 mm 的垫片，
最终表面会比仿真多出约 0.7 mm/侧。此处按用户指定的 0.8 mm 生成，未擅自改成其他减量。

在用原始 STL 重建初版资产后，执行以下命令即可生成并激活本版本。
脚本要求 `meshes/bump_v2` 不存在，避免覆盖已有资产：

```bash
conda run -n dex_wire python tools/reduce_dexmate_bump.py --activate
```

当前 `manifest.json` 为 schema v2，补充当前几何版本与来源记录；历史 schema v1 在快照中保留。

## 入口与来源

- **定制入口：** `vega_1u_gripper_flexitac.urdf`。所有 mesh 使用目录内的相对路径。
- `upstream/`：`dexmate-urdf==0.8.4` 官方 URDF 及其引用网格的原样快照。
  保留路径层级及上游 `LICENSE`；不修改此基线。
- `meshes/custom/`：用户提供的原始 STL，以及生成的视觉 GLB、碰撞 OBJ。
- `manifest.json`：官方版本、输入和输出 SHA256、坐标变换、构建依赖版本、验证边界。
  哈希覆盖生成资产，不包含本说明。人工修改后应同步更新来源记录。

定制入口不依赖本机 Conda 安装路径。原版资产和定制入口均可独立解析。
目前环境配置仍使用 ALOHA；接入 Dexmate 还需要机器人控制、传感器和导入配置。

## 本轮定制

输入为 `gripper_finger_tpu_26mm_uniform_width_raise_2mm.stl`，来自相邻的
`dex_wire/assets/`。复制后的 STL 字节不变；按 26 mm 宽度及原模型尺寸判断其坐标单位为毫米，
生成时转换为米，不将模型居中。

STL 对应原 GLB 中的 **TPU 指体**，不是整个 finger 连杆：

- 新 STL 的原始包围盒为约 `[-13.667, -13.005, 27.593]` 至
  `[26.297, 12.995, 150.388]` mm，宽度 26 mm。
- 转为米后，Z 区间和最小 X 与 `gripper_l1` 原 TPU 指体吻合，支持沿用其坐标系。
- 原 `gripper_l1.glb` 的 TPU 节点为 `Mesh_1.002`，金属件为 `Mesh_1.002_1`；
  `gripper_l2.glb` 分别为 `Mesh_1.001`、`Mesh_1.001_1`。
- 保留金属件及其变换，只替换 TPU；新 STL 无颜色信息，临时赋深灰色。
- 根据原两个 GLB 节点的相对变换生成第二根指体，而非让两指引用同方向 STL。
  变换为 X 镜像，加约 `[0.002, 0.004, -0.023]` mm 平移，完整数值见 manifest。
- 左右手的 `L/R_gripper_l1/l2` 共四个连杆均采用此指体版本。
- 关节 origin、axis、limit、mimic 和连杆惯性参数全部保持官方值。

碰撞几何同步改为新 TPU 表面与保留金属件的凸包，避免使用旧指体的完整碰撞包络。
金属凸包是暂用的简化形状，会填平孔洞；TPU OBJ 保留原表面凹凸。
实际接触形状仍取决于 Isaac Sim 的碰撞近似设置，需要导入后核查，
尤其不能用整个 TPU 的单个凸包抹平需要研究的凹槽或凸起。

## 后续仍需完成

1. 验证实际安装、两指开合时的间隙、指面法向和碰撞几何；本轮只验证文件坐标关系。
   文件名中的“抬高 2 mm”沿用用户设计描述，本轮没有 CAD 参数化特征验证。
2. 根据定制指体及触觉片的真实质量、质心和惯量更新参数。当前官方值是占位，
   STL 外观不表达 TPU 柔性，也不自动产生材料接触响应。
3. 设置夹爪驱动和速度/力矩限值；官方夹爪 `effort/velocity` 为零。
   配置固定基座，以及 lift、torso、head 的锁定/控制方式和 mimic 导入行为。
4. 加入 FlexiTac 触觉片安装面、采样位置、法向及传感器配置。
5. 复核历史渲染使用的夹爪视觉安装角修正和底座重复网格过滤。
   本轮未把视觉补丁套用到官方关节树；应统一检查视觉、碰撞和传感器安装。
6. 配置头部双目和腕相机的光学坐标系、内参及实测安装外参。

## 重建

在仓库根目录，用已有 `dex_wire` 环境执行下面命令；参数必须指向原始安装包和 STL。
脚本只接受已审计的官方 URDF SHA256，输出目录必须不存在，避免覆盖手工定制。

```bash
conda run -n dex_wire python tools/build_dexmate_assets.py \
  --package-root /path/to/site-packages/dexmate_urdf \
  --finger-stl /path/to/gripper_finger_tpu_26mm_uniform_width_raise_2mm.stl \
  --output /tmp/dexmate-assets-review
```

构建脚本重建本轮几何版本；后续人工修改 URDF 时，不应直接覆盖重建结果，
应先比较差异并保存新增配置及来源。

## 2026-09-15 几何预览与闭合检查

使用以下命令生成 PNG、可离线旋转/缩放/开合的 HTML，以及截面检查 JSON：

```bash
conda run -n dex_wire python tools/preview_dexmate.py \
  --output Isaacsim_tactile_env/output/dexmate_review_new
```

输出目录必须不存在。产物保存在 Git 忽略的 `output/` 下，HTML 的网格数据内嵌，
不使用 CDN；浏览器需支持 WebGL。可切换官方原版、定制视觉、定制碰撞几何。
局部预览使用左夹爪的底座坐标系；双手安装关系另由整机 PNG 展示。
整机预览保留官方安装关系，头部 `[0.5, 0, 0]`，手臂各关节为零，夹爪为 `0.4 rad`。

本轮检查在夹爪底座坐标系 `Y=0` 中心截面内，沿 Z 每 0.025 mm 采样实体区间：

| 夹爪 q（rad） | 两指截面最大 X 向重叠（mm） |
| --- | --- |
| 0 | 0.991 |
| 0.005 | 0.100 |
| 0.01 / 0.02 / 0.4 / 0.7854 | 采样未发现重叠 |

**加高后的指体在原闭合零位会相交。** 这是需要对照实机确认的新发现。
未自动移动指体、改变关节零位或缩小范围。中心截面无重叠不等于完整三维模型无碰撞，
也不代表该角度是可用的闭合限位；未模拟 TPU 受压形变或传感器接触。

已生成实际 URDF/STL 的预览，并通过 Chrome 检查交互页面的初始三维渲染。
本机现有 Conda 环境和常见安装位置未找到 Isaac Sim 运行时；
URDF→USD、PhysX 开合与接触验证仍未执行。
