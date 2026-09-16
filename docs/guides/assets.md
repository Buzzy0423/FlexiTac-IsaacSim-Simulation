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
| `meshes/bump_v2/simulation/` | 凸起减少 0.5 mm 的 STL、视觉 GLB、TPU／金属碰撞 OBJ |
| `meshes/bump_v2/printing/gripper_finger_26mm_bump_minus_0p8mm_print.stl` | 凸起减少 0.8 mm 的打印件，坐标单位毫米 |
| `manifest.json`、`meshes/bump_v2/revision.json` | 官方版本、变换、几何版本、源文件和输出哈希 |
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
桌子为静态三角面碰撞，保留槽和支架。

金属碰撞网格原先是暂用凸包，TPU OBJ 保留表面；导入后的近似仍需验证。
质量惯量沿用官方数据，几何外观不自动表达 TPU 柔性。
检查失败时在运行副本定位，明确确认后再更新正式资产并同步来源清单。

Dexmate manifest 的 `files_sha256` 不含 Markdown；文档调整不需要重新生成模型。
历史细节仅在需要时查阅 [资产准备记录](../reports/2026-09-15_asset_preparation.md)。
