# BSP Webots AutoAim

Webots 自瞄 BSP，使用与 Linux/树莓派相同的当前视觉模块接口：

```text
WebotsCamera / CameraSync
    → CameraFrameSync(TRIGGER)
    → ArmorDetector(OPENVINO_640X512)
    → ArmorTracker → Aimer
    → WebotsGimbal / WebotsFireNotify
```

## 配置和依赖

`User/xrobot.yaml` 是配置源。`MainFrameLayout` 只描述 800×600、BGR8、2400-byte step；
`MainCameraCalibration` 单独描述原生标定。默认 world 的水平视场角为 0.596886 rad，
对应 fx=fy=1300.258730617794、cx=400、cy=300。图像携带原生逐帧几何和 SharedFrame 所有权。

Webots 相机每 10 ms 更新渲染图像和每个仿真 step 的 IMU；固定 WIDE 档位与 CameraSync
使用 20000 us 触发周期，CameraFrameSync 使用当前 STOP/START/FRAME 协议。50 Hz 是仿真时间下的
触发配置，不是软件渲染环境的墙钟吞吐承诺。

配置显式选择：

```yaml
network:
  model: {expr: ArmorDetectorModel::OPENVINO_640X512}
```

因此 BSP 要求 OpenVINO Runtime；没有对应 SDK/设备/模型时明确失败，不自动换 Hailo 或其他模型。
`XR_ARMOR_OPENVINO_DEVICE=CPU` 可显式选择 CPU；省略时沿用模型后端的可见设备选择规则。
Tracker 的 `camera_mount_to_body` 外参和跟踪/弹道参数仍由 YAML 控制。

WebotsReferee 使用 `RefereeTypes::RobotGameRefereePack`，与当前 Aimer 类型完全一致。
仿真发射器的弹速、热量、射频和 shot event 仍在 `webots_launcher` 主题中；当前 Aimer 使用其
`default_bullet_speed`，所以默认 YAML 中 Aimer、WebotsReferee、WebotsFireNotify 均设置 23 m/s。

### 模块版本

本 BSP 使用 ArmorDetector、WebotsCamera、WebotsReferee、WebotsGimbal、WebotsFireNotify
的当前接口，依赖由 `Modules/modules.yaml` 指向各模块 master。更新历史工作目录时，应同时
核对这些模块的版本；旧模块快照不能与新 YAML 混用。

LibXR 验证版本为 `72e1774ab15f0d613eb403ee6491a752080c5c66`。构建入口仅初始化缺失的
submodule，不重置已有 checkout，不覆盖本地模块修改。`Modules/modules.yaml` 列出所需依赖，
包含 DurationStatistics、Referee 和 CMD。模块未准备好时给出缺失清单，不以旧模块替代。

## 生成与构建

环境：C++20、CMake/Ninja、Webots R2025a、OpenCV、OpenVINO、Python xrobot 0.3.1。
Windows 推荐在 Docker / Dev Container 内运行：

```bash
bash docker/entrypoints/build.sh
```

该入口先生成 `User/xrobot_main.hpp` 和 `User/xrobot_constexpr.hpp`，然后构建
`build/rm_auto_aim`。可通过 `XR_BUILD_DIR`、`XR_BUILD_TYPE`、`XR_BUILD_JOBS` 调整输出位置、
构建类型和并发。已安装的 OpenVINO 路径自动从常规 `/opt/intel` 目录发现，也可设置 `OpenVINO_DIR`。

手工命令等价于：

```bash
python3 -m xrobot.GenerateMain --config User/xrobot.yaml --output User/xrobot_main.hpp
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR=/opt/intel/openvino_2025.4.0/runtime/cmake
cmake --build build -j4 --target rm_auto_aim
```

生成头文件是受版本控制的输出；配置修改后应重新生成，不能只手工改头文件。

## 运行实际 world

```bash
XR_ARMOR_OPENVINO_DEVICE=CPU \
LIBGL_ALWAYS_SOFTWARE=1 \
python3 run_headless_preview.py --controller build/rm_auto_aim \
  --runtime-sec 40 --sim-flow-rate 0.1 --run-root .vscode-runs
```

或使用同一个入口：

```bash
XR_ARMOR_OPENVINO_DEVICE=CPU XR_RUNTIME_SEC=40 \
bash docker/entrypoints/headless_preview.sh
```

默认场景是已有的移动、旋转四装甲板车辆；没有替换成实拍贴图输入或直接注入检测结果。
无头 Webots 使用 Xvfb/XCB，controller 的预览环境可以使用 offscreen。
软件渲染可低于指定时间流速，日志中的仿真时间与命令的墙钟时长应分别理解。

`--sim-flow-rate` 通过 `WEBOTS_SIM_FLOW_RATE` 传给 BSP，启动时打印配置倍率。
直接运行程序且未设置此变量时默认为 `1.0`；例如 `0.04` 表示目标仿真／墙钟时间比为
0.04，实际仍受处理能力限制。非法数值会在连接 Webots 前报错；取得 world
的 `basicTimeStep` 后，还会在平台初始化前检查派生周期是否超出底层整数范围。

`--runtime-sec` 从 controller 启动开始计算，不包含 world 加载时间。有限时长运行必须有实际
pipeline 帧、无运行错误且进程未提前退出，才会写 `status=PASS`。崩溃、无帧和连接超时均为失败。
结束时停止本次启动的进程组；这是有界进程停止，不是模块析构或流水线 drain 测试。

Windows 的现有 `docker/windows-deploy.ps1` 和 Dev Container 仍可作为入口。Docker 镜像增加了
xauth/Xvfb 运行依赖，Dev Container 初始化不再强制切到历史 LibXR 提交。

## 三路 Web 预览

默认开启 Detector、Tracker、Aimer 的 Web 预览，三路共用容器内 `8080` 端口。
根路径 `/` 是汇总页；单路地址分别为 `/stream/armor_detector`、
`/stream/armor_tracker`、`/stream/aimer_preview`。预览缩放为0.5，不改变检测输入尺寸。
在对应模块的 `preview.enabled` 中关闭预览，修改后重新生成并编译。

HTTP 服务没有认证，配置的 `0.0.0.0` 会监听所在网络环境的全部网卡。
Docker 创建运行容器时可用 `-p 127.0.0.1:18080:8080` 仅向宿主本机开放，
浏览器访问 `http://127.0.0.1:18080/`。现有容器若未映射端口，需要创建带映射的运行
容器；不要把内部8080、宿主18080或其他BSP的端口配置混为一谈。
原生Linux运行时应根据访问范围设置绑定地址／防火墙，不要无保护地暴露到公网。

## 回归和带目标验收

BSP 配置与 launcher 的快速回归：

```bash
python3 tests/config_contract_test.py
python3 tests/launcher_test.py
python3 tests/startup_test.py build/rm_auto_aim
```

最后一项使用已构建的真实程序检查非法输入，不需要启动Webots。
若要同时检查极小倍率导致的周期越界，先启动一个匹配的空闲Webots world，再传
`--webots-url tcp://<host>:<port>/self`；这两项会连接world读取时间步，但不初始化流水线。

构建只读观测版本，并运行同一 BSP 的目标/空场验收：

```bash
XR_BUILD_ACCEPTANCE=ON bash docker/entrypoints/build.sh
python3 tests/run_acceptance.py --controller build/rm_auto_aim_acceptance \
  --run-root .vscode-runs/acceptance --case both --runtime-sec 40
```

每次使用新的 `--run-root`。观测版本不改变模块配置或算法，仅订阅实际 Topic。保存的数据包括
逐帧身份/时间戳、角点、PnP、tracker、Aimer 命令和裁判摘要，以及真实渲染图和角点叠图。
空场 fixture 只在独立测试目录中把目标移远、停止目标控制器，原 world 和资源保持不变。

验收区分启动前缀、运行期连续帧和停止时在途帧；检查 SharedFrame 身份、时间戳、几何、顺序、
有限值、角点凸性、PnP 正深度/重投影残差，以及正样本的跟踪与命令和空场的零误触发。
这些检查证明功能集成，不代替大规模识别精度、世界真值位姿误差、命中率或实机 Hailo 验收。
