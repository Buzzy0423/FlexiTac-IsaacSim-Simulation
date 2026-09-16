# 运行指南

所有命令从仓库根目录执行。当前结果与下一步见 [STATUS](../STATUS.md)，测量证据见 [首轮报告](../reports/2026-09-15_first_validation.md)。

## 安装与环境

本机已配置 `dexmate_isaacsim`，可直接激活。新机器可从依赖声明创建环境：

```bash
conda env create -f environment_dexmate.yml
conda activate dexmate_isaacsim
python -m pip check
```

环境包含 Python 3.11、Isaac Sim 5.1.0 和 CAD 转换依赖；当前没有安装整套 Isaac Lab 扩展。
`wheel==0.45.1` 用于兼容 Isaac Sim 固定的 `packaging==23.0`。
不要直接使用上游 ALOHA 安装说明升级这个环境的依赖。
参考 [NVIDIA 5.1 Python 安装说明](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html)。

## 启动检查

每次换一个新的输出目录。以下命令对应本机已验证的 R535 驱动：

```bash
# 基础落体和 RTX 渲染
./tools/run_dexmate_sim.sh tools/check_dexmate_runtime.py \
  --workaround-r535-vulkan \
  --output Isaacsim_tactile_env/output/runtime_new

# 官方桌子、自由开合和四路名义相机；默认开启自碰撞，应用局部修正
./tools/run_dexmate_sim.sh tools/validate_dexmate_scene.py \
  --workaround-r535-vulkan --four-cameras \
  --output Isaacsim_tactile_env/output/dexmate_scene_new
```

`--disable-self-collision` 仅用于诊断对照；`--raw-collisions` 绕过局部修正以复现导入器原始行为。
`--steps-per-target 5` 可缩短诊断，但不能替代默认每段 240 步的完整验证。
`--contact-regression` 增加两手固定测试块的闭合、保持和释放检查，需要开启自碰撞。
采用两个相差 2 mm 的横向位置，分别验证各指面持续受力；固定块回归不等于自由物体抓取。
四指 FlexiTac 默认启用，完整接触与触觉验证命令：

```bash
./tools/run_dexmate_sim.sh tools/validate_dexmate_scene.py \
  --workaround-r535-vulkan --four-cameras --contact-regression \
  --output Isaacsim_tactile_env/output/dexmate_tactile_new
```

采样位置、计算方式和可复用接口见[触觉接入指南](tactile.md)。

单手可移动物体验证（左手侧向夹住、抬升 5 cm、保持 2 秒、放回并松开）：

```bash
./tools/run_dexmate_sim.sh tools/validate_dexmate_scene.py \
  --workaround-r535-vulkan --four-cameras --grasp-regression \
  --output Isaacsim_tactile_env/output/dexmate_grasp_new
```

保留开启自碰撞和 `tactile.enabled=true`。此选项建立一个受重力影响的 50 g 长方块，
通过左臂关节驱动和夹爪摩擦夹持，不把物体绑定到机器人。
具体参数、通过条件与输出字段见[抓取验证指南](grasp.md)。

连续多轮 reset/step 与四相机同步记录：

```bash
./tools/run_dexmate_sim.sh tools/run_dexmate_episodes.py \
  --workaround-r535-vulkan --episodes 3 \
  --output Isaacsim_tactile_env/output/dexmate_episodes_new
```

需要本机 `ffmpeg`／`ffprobe` 编码和检查视频；详细接口及索引说明见[环境与数据指南](environment.md)。
默认读取 [工作区 JSON](../../Isaacsim_tactile_env/dexmate_workspace.json) 中的官方桌子；
`--table-proxy` 仅用于明确需要简化桌面定位体的诊断，不是正式桌子资产。

## 本机兼容设置

- RTX 4080 SUPER，实际驱动 535.309.01；Vulkan 报成 535.53.01。
- `--workaround-r535-vulkan` 仅在 `nvidia-smi` 确认 R535.256+ 时使用
  [NVIDIA 已记录的处理方法](https://docs.omniverse.nvidia.com/utilities/latest/common/technical-requirements.html)。
  新驱动若不属于该范围，去掉参数，不将其用于任意不兼容驱动。
- 启动脚本只在进程内预加载 Conda `libstdc++.so.6`，满足 ICU 的 `CXXABI_1.3.15` 依赖。
- 启动脚本通过 `OMNI_KIT_ACCEPT_EULA=YES` 接受 Isaac Sim 标准许可。
- 保留 Kit 默认快速退出；完整逐扩展关闭曾出现挂起。脚本关闭前设置退出码，并保存报告。
- 不修改系统驱动或其他 Conda 环境。

## 场景参数和结果

机器人固定基座、双臂零位、`head_j1=0.5 rad`；夹爪测试 `0.4 → 0 → 0.4 rad`。
运行副本烘焙 GLB 节点变换、覆盖零驱动限值，源资产不回写。
碰撞近似及来源见 [资产指南](assets.md)。

工作区 JSON 的 `worktop_board` 控制近端桌板尺寸、平面中心和显示色；板底自动跟随官方桌面高度。
底座前缘与桌架近端 X=0 对齐。测试方块放在桌板中心，报告的 `cube_support_surface` 标明支撑面；
存在桌板时，`table_contact_passed` 检查的是板顶上的静止高度。

四路相机：`zed_left_camera`、`zed_right_camera`、`L_camera_link`、`R_camera_link`。
头部两路暂用 640×480、焦距 24 mm、水平成像面 36 mm；按 URDF +Z 向前转换到 USD 约定。
自装 C960 腕相机按 workspace 配置使用 640×360、90° 对角视场、2.88 mm 焦距、1 mm 近裁剪，
安装坐标取自 STL 镜头前端中心／法线；[参数来源与验证](../reports/2026-09-16_wrist_camera_simulation.md)。
这些是名义场景参数，尚未匹配实机内外参和数据裁剪。

| 输出 | 内容 |
| --- | --- |
| `runtime.json` / `runtime.png` | 基础物理与渲染检查 |
| `validation.json` | 验证范围、哈希、关节状态、接触对和相机输出信息 |
| `contact_hold.usda` | 启用接触回归时，固定测试块的保持阶段场景快照 |
| `grasp_report.json`、`grasp_sequence.npz` | 自由物体抓取断言、位姿／动作／触觉峰值与实际受力记录 |
| `grasp_*.png`、`grasp_replay.gif` | 各阶段结束画面和场景／四片触觉的同帧回放 |
| `tactile_sequence.npz`、`tactile_metadata.json` | 每物理步的四指触觉、参数／检查结果 |
| `tactile_contact.png`、`tactile_stages.png` | 同时刻的四片 12×32 接触热图，以及开始／接触／结束对比 |
| `scene.png` / `closed.png` | 实际仿真场景／闭合状态 |
| `scene_side.png` / `scene_top.png` | 场景摆放的侧视／俯视预览 |
| `head_*.png` / `wrist_*.png` | 四路名义相机 |
| `runtime.urdf`、`import_meshes/`、`robot.usd`、`scene.usda` | 本次导入副本与场景 |

先检查退出码，再读报告的 `status` 与 `validation_scope`。
关闭自碰撞的通过不等于两指接触或抓取通过；相机生成图像不等于标定通过。
输出目录不提交，长期证据摘要写入 `docs/reports/`。
历史输出会清理可重建的导入网格和 USD 快照；视频／数值／报告保留，`runtime.urdf` 仅保留来源证据。
需要 USD 场景时按本页命令在新目录重新导出；清理范围见 [2026-09-16 记录](../reports/2026-09-16_output_cleanup.json)。
`phase_contacts` 区分各阶段；`last_impulse_step` 只记录冲量大于 1e-7 N·s 的步骤，
因为接触偏移范围内的候选接触点可能没有受力，不能仅凭事件数量判断已经碰到物体。

## 轻量检查

```bash
ruff check tools/build_dexmate_assets.py tools/reduce_dexmate_bump.py \
  tools/preview_dexmate.py tools/convert_vention_step.py \
  tools/check_dexmate_runtime.py tools/validate_dexmate_scene.py \
  tools/dexmate_collision.py tools/dexmate_contact_regression.py tests/test_dexmate_collision.py
python -m unittest discover -s tests -p test_dexmate_collision.py
bash -n tools/run_dexmate_sim.sh
git diff --check
```

Ruff 可使用已有 `robot` 环境中的可执行文件。仅改文档／忽略规则时，不必重跑 GPU 仿真。

触觉单元检查需要 Isaac Sim 随附的 Warp。本机无需额外安装 Warp，在已激活环境中运行：

```bash
WARP_CACHE_PATH=/tmp/dexmate_warp_test_cache \
PYTHONPATH="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/extscache/omni.warp.core-1.8.2+lx64" \
python -m unittest discover -s tests -p 'test_dexmate_*.py'
```

此检查使用 CPU 内核；完整 GPU 接触验证仍使用上面的启动命令。

## 输出整理

`output/README.md` 索引本机保留的运行。历史报告、图像及预览移至 `output/archive/<原运行名>/`；
其中可重建的 USD／USDA／OBJ／运行 URDF 已删除，旧文档中的运行名称仍可用于定位归档报告。
当前通过的完整运行及几何／触觉预览保留在顶层；不要把这些输出当作源资产。
清理清单见 `docs/reports/2026-09-15_output_cleanup.json`。
