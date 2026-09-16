# 资产维护与重建

先看 [已确认决策](../decisions.md)。源文件、受维护的派生资产和 manifest 一起提交；
运行副本、临时转换结果和预览留在 `output/`。不要覆盖既有运行目录。

## Dexmate 文件定位

以下路径相对 `Isaacsim_tactile_env/assets/dexmate/`：

| 路径 | 用途 |
| --- | --- |
| `vega_1u_gripper_flexitac.urdf` | 当前定制机器人入口，mesh 路径均相对本目录 |
| `upstream/` | `dexmate-urdf==0.8.4` 原始 URDF、网格与 Apache LICENSE |
| `meshes/custom/` | 最初用户 STL 和初版指体资源，作为不可覆盖的历史基线 |
| `meshes/pad_66x26_v3/simulation/` | 当前 66×26 mm 凸起、减少 0.5 mm 的 STL、视觉 GLB、TPU／金属碰撞 OBJ |
| `meshes/pad_66x26_v3/printing/gripper_finger_pad_66x26_bump_minus_0p8mm_print.stl` | 当前 66×26 mm 凸起、减少 0.8 mm 的打印件，单位毫米 |
| `meshes/bump_v2/` | 上一版 60.764 mm 长凸起及高度修改来源，保留供追溯 |
| `manifest.json`、`meshes/pad_66x26_v3/revision.json` | 官方版本、变换、几何版本、源文件和输出哈希 |
| `meshes/bump_v2/previous_*` | 激活前的 URDF／manifest 快照；其相对路径按资产根目录解释 |

原始用户 STL 为 `gripper_finger_tpu_26mm_uniform_width_raise_2mm.stl`。
只替换 TPU 部分并保留金属件；第二根手指通过原模型节点变换镜像生成，左右手共四根同步更新。
减高度只移动矩形凸起面的四个角点，保持宽度、孔位及其他顶点。

初次生成使用已有 `dex_wire` 环境（提供新版 trimesh `load_scene` API）。
`dexmate_isaacsim` 中 Isaac Sim 固定的旧 trimesh 不用于这些离线指体脚本，勿为此升级仿真依赖。

```bash
# 重建基线：output 必须不存在，package-root 指向已审计的 0.8.4 安装包
conda run -n dex_wire python tools/build_dexmate_assets.py \
  --package-root /path/to/site-packages/dexmate_urdf \
  --finger-stl Isaacsim_tactile_env/assets/dexmate/meshes/custom/gripper_finger_tpu_26mm_uniform_width_raise_2mm.stl \
  --output /tmp/dexmate_asset_review

# 在新基线上分别生成两个减量，并激活仿真版
conda run -n dex_wire python tools/reduce_dexmate_bump.py \
  --assets /tmp/dexmate_asset_review --activate

# 当前正式资产的离线几何预览
conda run -n dex_wire python tools/preview_dexmate.py \
  --output Isaacsim_tactile_env/output/dexmate_geometry_new
```

当前目录已有 `bump_v2`，不要对其直接重复运行减量脚本。
预览中的间隙是中心截面采样结果，不是完整三维最近距离或动力学接触证明。

当前长度版本由 `tools/extend_dexmate_finger.py` 从 `bump_v2` 派生：凸起近端之前的顶点全部保留，
接触段沿 Z 拉长到 66 mm，末端帽平移约 5.236 mm；宽度与 X 向高度不变。
只在尚无 `pad_66x26_v3` 的资产副本上运行构建：

```bash
conda run -n dex_wire python tools/extend_dexmate_finger.py --assets /tmp/dexmate_asset_review --activate
# 当前版本的新旧截面对比与 64×24 mm 传感器外形预览
conda run -n dex_wire python tools/extend_dexmate_finger.py --preview-only \
  --preview-output Isaacsim_tactile_env/output/pad66_review_new
```

源文件和历史版本不覆盖；URDF 惯量仍为暂用值，本次未按未知 TPU 密度重新估算。

## Vention 官方桌子

源文件：[VentionAssembly_312098_v6.STEP](../../Isaacsim_tactile_env/assets/VentionAssembly_312098_v6.STEP)。
转换结果在 `Isaacsim_tactile_env/assets/vention_312098_v6/`：
GLB 用于预览，NPZ 保存各组件的顶点／面供场景生成 USD；manifest 保存单位、变换和哈希。

```bash
conda activate dexmate_isaacsim
python tools/convert_vention_step.py \
  --input Isaacsim_tactile_env/assets/VentionAssembly_312098_v6.STEP \
  --output /tmp/vention_asset_review
```

读取 STEP 中 124 个实体及开放壳体，共 144 个组件、616112 个三角面；
开放壳体包含脚垫和端盖，不能遗漏。转换毫米为米并调整轴向，不做拟合缩放。
桌面坐标以近端短边中央为基准；尺寸和原点见 manifest 与 [决策](../decisions.md)。
颜色暂用统一灰色，几何来自用户提供的官方 CAD。

## 导入与碰撞的边界

`validate_dexmate_scene.py` 在每次运行目录中生成独立导入副本：
烘焙 GLB 节点变换；为零限值设置暂用驱动值；使用固定基座和原生 PhysX mimic；
机器人碰撞采用凸分解，接触偏移设置为 0.2 mm、静止偏移为零。
`tools/dexmate_collision.py` 在运行层解除碰撞子树的实例化，使参数确实写到全部碰撞体（数量见运行报告）；
视觉仍保留实例化。凸分解启用 shrink wrap，误差百分比 1、最多 64 个凸包、最小厚度 0.05 mm。
固定连接形成的同一刚体组、以及组间直接相邻关节使用 pairwise filtering；
左右手之间、同手两根手指之间及其他非相邻连杆仍启用碰撞。
初始化关节状态在第一次物理步之前写入，避免先在错误零位求解碰撞。
夹爪 mimic 约束使用自然频率 5000 rad/s、阻尼比 1；这是机械联动的数值近似，
替代导入器过软的 25 rad/s、0.005 默认值，不代表实机测得的刚度。
桌子为静态三角面碰撞，保留槽和支架。

金属碰撞网格原先是暂用凸包，TPU OBJ 保留表面；凸分解属于近似，后续任务仍需检查接触精度。
质量惯量沿用官方数据，几何外观不自动表达 TPU 柔性。
检查失败时在运行副本定位，明确确认后再更新正式资产并同步来源清单。

Dexmate manifest 的 `files_sha256` 不含 Markdown；文档调整不需要重新生成模型。
历史细节仅在需要时查阅 [资产准备记录](../reports/2026-09-15_asset_preparation.md)。

## 自装腕相机离线预览

用户相机与支架 STL 使用底座 CAD 坐标，不能直接替换旧 `camera_link` 的文件路径。
下面的工具保留正式资产，在新的输出目录移除旧安装组件并生成新旧对照、可开合的离线交互页和装配 GLB。

```bash
conda run --no-capture-output -n dex_wire python tools/preview_dexmate_wrist_camera.py \
  --output Isaacsim_tactile_env/output/wrist_camera_review_new
```

依赖 `dex_wire` 已有的 trimesh、scipy、matplotlib 和 coal。`review.json` 记录来源哈希、变换、
17 个开合姿态的三角面距离及相机／支架内部采样；不修改 URDF、碰撞配置或相机光学坐标。
检查范围与后续接入事项见 [腕相机报告](../reports/2026-09-16_wrist_camera.md)。

## 当前 C960 仿真资产

`meshes/wrist_c960_v1/` 为当前派生版本，保留修改前 URDF／manifest。原始 STL 和 upstream 不改写。
左右旧支架及旧底座碰撞 OBJ 已替换：保留底座的六个主要实体分别取凸包，新支架与相机使用表面网格供 PhysX 凸分解。
固定件安装接触沿用原固定簇／相邻关节过滤；凸包可能填平小凹槽，并非精确 CAD 碰撞。
相机沿用官方暂用质量，惯量按新外壳均匀密度计算；底座惯量仍继承，不视为真实 C960 总成测量。

生成隔离候选（已激活时会从版本内的旧 URDF 快照重建）：

```bash
conda run --no-capture-output -n dex_wire python tools/build_dexmate_wrist_camera.py \
  --output Isaacsim_tactile_env/output/wrist_camera_candidate_new
```

候选目录包含 `candidate.urdf`、供触觉模块使用的 `manifest.json`、网格和来源记录。
验证用 workspace 副本应把 `robot.urdf` 指向该文件，并将桌子资产路径写为绝对路径。
`run_dexmate_episodes.py --workspace /path/to/candidate_workspace.json ...` 可以直接验证候选。
`--activate` 用于激活已生成、已验证的候选，并核对源／输出哈希；当前版本已激活，重复操作会拒绝覆盖。
正式入口仍是原 `dexmate_workspace.json`，腕相机光学参数在其 `cameras` 中，见[仿真验证](../reports/2026-09-16_wrist_camera_simulation.md)。
