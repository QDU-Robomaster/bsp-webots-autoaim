# BSP Webots AutoAim

基于 `libxr` / `xrobot` 的 Webots 仿真自瞄主仓。

当前主线已经统一到共享视觉核心：

- `ArmorDetector`
- `ArmorTracker`
- `Aimer`

Webots 主仓只负责把这套共享核心接到仿真输入/执行/通信链：

- `WebotsCamera`
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

共享视觉主线的数据流是：

1. 输入侧发布
   - `image_raw`
   - `image_header`
   - `camera_pose`
   - `gimbal/rotation`
   - 相机标定由 `User/xrobot.yaml` 中的 `MainCameraInfo` 以编译期常量注入，不再发布 `camera_info` topic
2. `ArmorDetector` 输出
   - `armor_detector/armors_result`
   - `armor_detector/metrics`
3. `ArmorTracker` 输出
   - `tracker/info`
   - `tracker/metrics`
   - `tracker/target`
4. `Aimer` 输出
   - `tracker/target_eulr`
   - `tracker/send`
   - `aimer/metrics`

Webots 主仓额外接入：

- `referee/bullet_speed`
  - 由 `WebotsReferee` 提供
- `gimbal/rotation`
  - 由 `WebotsCamera` / `WebotsGimbal` 链路提供
- `tracker/send`
  - 通过 `SharedTopic` 模拟上位机与下位机通信

## Communication Model

Webots 主仓保留完整通信模拟链，不是直接把目标角喂给执行器。

- Host side
  - `SharedTopic_Host`
  - `SharedTopicClient_Host`
- MCU side
  - `SharedTopic_MCU`
  - `SharedTopicClient_MCU`

当前转发的核心话题是：

- `bullet_speed`
- `rotation`
- `target_eulr`
- `send`

## Build

```bash
git submodule update --init --recursive
xrobot_init_mod
xrobot_gen_main --output User/xrobot_main.hpp
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
make -C webots/controllers/taget_controller
```

CI / 基础镜像已经内置 OpenVINO，不需要在主仓 workflow 里手动安装。

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

## Notes

- 这个仓库的目标是“仿真 BSP”，不是另起一套视觉算法
- 当前共享视觉核心已经回到各模块仓库的 `master`
- 剩余运行期问题主要是 tracker 稳定性，不是装配层或 CI 问题
