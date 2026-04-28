# BSP Webots AutoAim

基于 `libxr` / `xrobot` 的 Webots 仿真自瞄主仓。

当前主仓的职责是把共享视觉模块接到 Webots 世界，并尽量模拟真实硬件的发布/转发边界：

- `WebotsCamera`
- `CameraFrameSync`
- `ArmorDetector`
- `ArmorTracker`
- `WebotsReferee`
- `WebotsGimbal`
- `WebotsFireNotify`
- `SharedTopic`
- `SharedTopicClient`

## Role

- `rm_auto_aim`
  - Webots extern controller 入口

## Layout

```text
Modules/   模块依赖清单
User/      xrobot 装配配置与生成入口
webots/    world、controller、资源文件
libxr/     框架与底层组件
```

## Assembly

- `Modules/modules.yaml`
  - 决定需要拉取哪些模块仓库
- `User/xrobot.yaml`
  - Webots 主装配配置
- `User/xrobot_main.hpp`
  - 由 `xrobot_gen_main` 生成，不手改

## Core Topic Contract

当前链路按“图像大载荷走共享图像桥，低带宽状态走 SharedTopic”来拆：

1. `WebotsCamera`
   - 每个 world step 发布一组原始 imu 话题：
     - `camera_gyro`
     - `camera_accl`
     - `camera_quat`
   - 图像写入 `CameraBase::ImageFrame`，其中 `timestamp_us` 是传感器侧时间
   - 图像 payload 由 `CameraFrameSync` 的图像 sink 提交到 `camera_image`
   - `gimbal/rotation` 仍保留在 `gimbal` domain，兼容现有消费者
2. `CameraFrameSync`
   - 以 `gyro` 为主时间线组装 imu
   - 直接读取已提交图像结构体里的 `timestamp_us` 作为图像时间基线
   - 直接读取 `gyro/accl/quat` payload 里的 `sensor_timestamp_us`
   - 输出同步后的 `camera_imu`
3. `ArmorDetector` / `ArmorTracker`
   - 消费 `camera_image + camera_imu`
4. 其他仿真侧输入
   - `referee/bullet_speed`
   - `gimbal/rotation`

## Communication Model

Webots 主仓保留完整通信模拟链，不直接把结果硬塞给执行器。

- Host side
  - `SharedTopic_Host`
  - `SharedTopicClient_Host`
- MCU side
  - `SharedTopic_MCU`
  - `SharedTopicClient_MCU`

当前转发的核心话题是：

- Host -> MCU
  - `bullet_speed`
  - `gimbal/rotation`
  - `camera_sync_config`
- MCU -> Host
  - `camera_gyro`
  - `camera_accl`
  - `camera_quat`
  - `tracker/fire_notify`
  - `tracker/target_eulr`

## Build

```bash
git submodule update --init --recursive
xrobot_init_mod
xrobot_gen_main --output User/xrobot_main.hpp --config User/xrobot.yaml
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
```

在 `ubuntu24` 上配置时需要先让 CMake 能找到 OpenVINO。当前可用做法是显式导出：

```bash
export INTEL_OPENVINO_DIR=/home/xiao/toolchains/openvino_2025.4.0_archive
export OpenVINO_DIR=$INTEL_OPENVINO_DIR/runtime/cmake
export TBB_DIR=$INTEL_OPENVINO_DIR/runtime/3rdparty/tbb/lib/cmake/TBB
export LD_LIBRARY_PATH=$INTEL_OPENVINO_DIR/runtime/lib/intel64:$INTEL_OPENVINO_DIR/runtime/3rdparty/tbb/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

## Run

有屏幕时直接运行 Webots world 即可；无头环境通常需要：

```bash
xvfb-run -a webots --stdout --stderr --batch --mode=fast webots/worlds/auto_aim_test_field.wbt
```

extern controller 运行时还需要：

```bash
QT_QPA_PLATFORM=offscreen
```

## Preview

预览统一只由 YAML 控制，不走 CMake 开关。

- `armor_detector.cfg.debug.preview`
- `armor_tracker.cfg.debug.preview`

如果要快速检查当前发布频率，可在运行 controller 前设置：

```bash
export XR_FREQ_PROBE=1
```

当前已经验证过的 Webots 默认频率关系是：

- `camera_gyro / camera_accl / camera_quat = 1000 Hz`
- `camera_image / sync_frame = 100 Hz`
- `camera_imu = 100 Hz`

## Notes

- 这个仓库的目标是“仿真 BSP”，不是另起一套视觉算法
- 当前同步链路已经验证 `raw imu : image = 10 : 1`
- `User/xrobot_main.hpp` 是生成文件，不手改
