# BSP Webots AutoAim

基于 `libxr` / `xrobot` 的 Webots 仿真自瞄主仓。当前收口目标是先把视觉链路稳定跑到 `ArmorTracker`，暂不接入 `Aimer` 和云台闭环控制。

## 当前链路

```text
WebotsCamera -> CameraFrameSync -> ArmorDetector -> ArmorTracker
```

- `WebotsCamera` 从 Webots 世界读取相机图像和真实传感器节点。
- Webots 世界 `basicTimeStep = 1 ms`。
- IMU 每个 world step 发布，图像按 `100 Hz` 发布。
- `CameraFrameSync` 消费同一相机的图像和 IMU，输出 `camera_imu`。
- `ArmorDetector` / `ArmorTracker` 均消费同步后的相机/IMU 数据。

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
  - 只列当前 tracker 验证链路需要的模块。
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

当前分支不接 `SharedTopic` / `Aimer` / `WebotsGimbal` 闭环。后续接入下游控制时，应在这个稳定的相机同步基线之上继续扩展，而不是改变相机/IMU 的时间与坐标语义。

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

当前分支只保留主运行链路；视频录制、truth compare 等验证工具作为临时 overlay 使用，不进主线。

已验证的默认频率关系：

- `camera_gyro / camera_accl / camera_quat = 1000 Hz`
- `camera_image = 100 Hz`
- `camera_imu = 100 Hz`

## 注意

- 这个仓库的目标是 Webots BSP，不另起一套视觉算法。
- 当前主线只声明到 tracker 为止的稳定基线。
- `Aimer` 和执行器闭环不在当前合并范围内。
