# PC 直连腕相机：设备身份与采集配置

两台 EMEET SmartCam C960 已读取成功。使用用户现有 udev 规则，无需改写系统规则：

| 观测名 | 设备别名 | 序列号 | 本次节点 |
| --- | --- | --- | --- |
| wrist_left | `/dev/dexmate-wrist-left` | A260401000123002 | `/dev/video0` |
| wrist_right | `/dev/dexmate-wrist-right` | A260421000122337 | `/dev/video2` |

来源 `/etc/udev/rules.d/99-dexmate-wrist-cams.rules`，与运行中 `udevadm info` 的序列号及 DEVLINKS 一致。
只匹配 index 0；另两个节点是 UVC metadata，不能当作另两台相机。USB VID/PID 为 `328f:0121`。

## 已配置和验证

- 配置文件：`Isaacsim_tactile_env/dexmate_wrist_cameras.json`。
- 1280×720、MJPG、请求 30 fps、四个 V4L2 缓冲区，不裁剪／镜像／旋转。
- 保留原自动曝光、自动白平衡、60 Hz 防闪烁及自动曝光动态帧率设置。
- 双台同时运行，每台预热 15 帧后测量 120 帧：左 **29.989 fps**，右 **29.933 fps**。
  主机读图最大间隔分别 36.57／36.83 ms；各 120 个不同解码帧，分辨率和像素格式与请求一致。
- 支持格式：MJPG 有 1920×1080、1280×960、1280×720、1024×576、960×720、800×600、640×480、640×360；
  YUYV 有 640×480、640×360。设备对这些模式均报告 30 fps，本次最终验证范围仅 720p MJPG。

初次 OpenCV 使用一个缓冲区时只有约 20 fps。关闭动态降帧、单台、640×480、手动 10 ms 曝光均未解决。
原生 V4L2 四缓冲查询显示能够接近 30 fps；OpenCV 改为四缓冲后恢复上述帧率，且恢复了全部原始曝光设置。
因此初次约 20 fps 是这条单缓冲采集路径的问题，不能作为相机能力上限。
最终短测未证明长时间稳定性、曝光时刻精确同步或低延迟上限；四缓冲需由采集循环持续读取，避免积压。

## 内参仍待标定

读到的标准 UVC 控件和能力没有 `fx/fy/cx/cy`、畸变、光学外参，也没有 focus／zoom 控件。
检查的相邻项目配置中未找到这两台序列号对应的内参；当前图像内没有标定板。
profile 的 `camera_matrix`、`distortion_coefficients`、`optical_pose_in_gripper` 保持 `null`，状态 `requires_calibration`。
不能把未知畸变写成零，也不能把仿真的 24 mm 焦距／36×27 mm aperture 当作这两台 C960 的实测参数。

每台相机需要已有标定文件，或对标定板采集不同位置、距离、倾角的多张画面，分别求解 K 和畸变，
检查独立图像重投影误差后再接入仿真。方法参见 [OpenCV 官方标定说明](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html)。
保持最终采集模式不变；当前实机 profile 是 16:9 的 1280×720，仿真仍为 4:3 的 640×480，
不能忽略裁剪／模式差异直接缩放 K。相机相对夹爪的外参是另外的标定问题。

## 证据与复现

最新输出：`Isaacsim_tactile_env/output/wrist_camera_profile_20260916_v1/`，
含左右 PNG、组合图、各设备能力／控制／udev 属性、逐帧主机时间及完整 JSON；不提交真实现场图像。
`tested_profile.json` 保留测试时的配置字节，之后仅更新了配置中的结果说明文字，采集参数相同。
摘要：[设备检查 JSON](2026-09-16_wrist_camera_devices_summary.json)。
早期对照位于 `output/wrist_camera_probe_20260916_v{1,2}/` 和 `output/wrist_camera_rates_20260916_v1/`。
初次终端原生 V4L2 查询 90 帧，滚动累计帧率 24.32→27.12→28.07 fps（包含启动），是诊断线索而非最终验收。

运行命令见[环境指南](../guides/environment.md#本机真实腕相机采集检查)。采集脚本通过 Ruff；
源 STL、udev 规则、机器人 URDF、仿真相机参数均未由本任务修改。设备读取与采集已完成，精确内外参仍待标定输入。
