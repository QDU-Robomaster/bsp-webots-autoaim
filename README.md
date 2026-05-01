# BSP Webots AutoAim

基于 `libxr` / `xrobot` 的 Webots 仿真自瞄主仓。当前主线把 Webots 视觉链路、瞄点生成、虚拟下位机转发、云台力矩控制和预览落盘接成一条闭环基线。

## 当前链路

```text
WebotsCamera -> CameraFrameSync -> ArmorDetector -> ArmorTracker -> Aimer
    -> SharedTopic -> WebotsGimbal / WebotsFireNotify
VisionPreview 旁路订阅图像、detector、tracker 和 aimer 话题并直接输出 overlay 视频。
```

- `WebotsCamera` 从 Webots 世界读取相机图像和真实传感器节点。
- Webots 世界 `basicTimeStep = 1 ms`。
- IMU 每个 world step 发布，图像按 `100 Hz` 发布。
- `CameraFrameSync` 消费同一相机的图像和 IMU，输出 `camera_imu`。
- `ArmorDetector` / `ArmorTracker` 均消费同步后的相机/IMU 数据。
- `Aimer` 消费 `tracker/target`、`referee/bullet_speed` 和 `gimbal/rotation`，发布 `tracker/target_eulr`、`tracker/fire_notify` 和 `tracker/send`。
- `SharedTopic` 通过 `User/main.cpp` 里的 pipe-backed 虚拟 UART 模拟上下位机低带宽话题转发。
- `WebotsGimbal` 订阅默认域 `target_eulr`，按 Webots IMU 姿态和 gyro 反馈输出电机力矩。
- `WebotsFireNotify` 订阅默认域 `fire_notify`，驱动 Webots `fire_led`。
- `VisionPreview` 默认写 `/tmp/webots_autoaim_preview/raw.avi`、`overlay.avi` 和各 topic TSV；`overlay.avi` 是模块直出的叠加视频，不依赖桌面录屏。

## 坐标系边界

`ArmorDetector` 的 PnP 位姿来自 OpenCV 光学相机坐标：

- `x` 向右
- `y` 向下
- `z` 向前

`WebotsCamera` / IMU 对外发布的姿态语义是：

- `x` 向右
- `y` 向前
- `z` 向上

因此 `ArmorTracker.cfg.frames.rotation` 必须保留为：

```text
wxyz = [0.5, -0.5, 0.5, -0.5]
```

这个旋转把 detector 输出转到 tracker/IMU 发布坐标。不能改成 identity；否则 tracker 会把光学坐标的 `x/y` 当作整车旋转平面，导致半径塌缩和装甲板固定偏移。

## 目录

```text
Modules/   模块依赖清单
User/      xrobot 装配配置、生成入口和验证辅助工具
webots/    world、proto、资源文件
libxr/     框架与底层组件
```

## 装配源

- `Modules/modules.yaml`
  - 列当前 Webots 闭环基线需要的模块。
- `User/xrobot.yaml`
  - Webots 主装配配置源。
- `User/xrobot_main.hpp`
  - 由 `xrobot_gen_main` 生成，不手改。

## 话题

- 图像：
  - `camera_image`
- 原始传感器：
  - `camera_gyro`
  - `camera_accl`
  - `camera_quat`
- 同步后 IMU：
  - `camera_imu`
- 瞄点和命令：
  - `tracker/target`
  - `tracker/target_eulr`
  - `tracker/fire_notify`
  - `tracker/send`
  - `aimer/metrics`
  - `aimer/trajectory`
- 虚拟下位机侧：
  - 默认域 `target_eulr`
  - 默认域 `fire_notify`
  - 默认域 `bullet_speed`

## 构建

```bash
git submodule update --init --recursive
xrobot_init_mod
xrobot_gen_main --output User/xrobot_main.hpp --config User/xrobot.yaml
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
```

在 `ubuntu24` 上配置时需要让 CMake 能找到 OpenVINO。当前可用做法：

```bash
export INTEL_OPENVINO_DIR=/home/xiao/toolchains/openvino_2025.4.0_archive
export OpenVINO_DIR=$INTEL_OPENVINO_DIR/runtime/cmake
export TBB_DIR=$INTEL_OPENVINO_DIR/runtime/3rdparty/tbb/lib/cmake/TBB
export LD_LIBRARY_PATH=$INTEL_OPENVINO_DIR/runtime/lib/intel64:$INTEL_OPENVINO_DIR/runtime/3rdparty/tbb/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

## 运行

有屏幕时直接运行 Webots world。extern controller 运行时还需要：

```bash
QT_QPA_PLATFORM=offscreen
```

## 验证边界

当前主线保留主运行链路和 `VisionPreview` 直出 overlay；truth compare 等验证工具仍作为临时 overlay 使用，不进主线。

已验证的默认频率关系：

- `camera_gyro / camera_accl / camera_quat = 1000 Hz`
- `camera_image = 100 Hz`
- `camera_imu = 100 Hz`
- `overlay.avi = 100 Hz`

## 注意

- 这个仓库的目标是 Webots BSP，不另起一套视觉算法。
- Webots 闭环用于仿真验证真实 topic / SharedTopic / 控制路径，不另起一套视觉算法。
- 如果调整 Webots world 的云台关节轴或相机安装姿态，必须重新标定 `WebotsGimbal` 的 pitch/yaw 力矩符号。
