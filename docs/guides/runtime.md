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

# 官方桌子、自由开合和四路名义相机；显式隔离尚未修复的机器人自碰撞
./tools/run_dexmate_sim.sh tools/validate_dexmate_scene.py \
  --workaround-r535-vulkan --disable-self-collision --four-cameras \
  --output Isaacsim_tactile_env/output/dexmate_scene_new
```

调试自碰撞时移除 `--disable-self-collision`，仍写入新目录。
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

四路相机：`zed_left_camera`、`zed_right_camera`、`L_camera_link`、`R_camera_link`。
暂用 640×480、焦距 24 mm、水平成像面 36 mm；按 URDF +Z 向前转换到 USD 约定。
腕相机近裁剪面 20 mm 用于避开自身镜头壳体，原安装位置没有移动。
这些是场景验证参数，尚未匹配实机内外参和数据裁剪。

| 输出 | 内容 |
| --- | --- |
| `runtime.json` / `runtime.png` | 基础物理与渲染检查 |
| `validation.json` | 验证范围、哈希、关节状态、接触对和相机输出信息 |
| `scene.png` / `closed.png` | 实际仿真场景／闭合状态 |
| `head_*.png` / `wrist_*.png` | 四路名义相机 |
| `runtime.urdf`、`import_meshes/`、`robot.usd`、`scene.usda` | 本次导入副本与场景 |

先检查退出码，再读报告的 `status` 与 `validation_scope`。
关闭自碰撞的通过不等于两指接触或抓取通过；相机生成图像不等于标定通过。
输出目录不提交，长期证据摘要写入 `docs/reports/`。

## 轻量检查

```bash
ruff check tools/build_dexmate_assets.py tools/reduce_dexmate_bump.py \
  tools/preview_dexmate.py tools/convert_vention_step.py \
  tools/check_dexmate_runtime.py tools/validate_dexmate_scene.py
bash -n tools/run_dexmate_sim.sh
git diff --check
```

Ruff 可使用已有 `robot` 环境中的可执行文件。仅改文档／忽略规则时，不必重跑 GPU 仿真。
