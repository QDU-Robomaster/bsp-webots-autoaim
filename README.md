# BSP Webots AutoAim

Webots 仿真自瞄 BSP，基于 `libxr` / `xrobot` 组织工程。

## Layout

```text
Modules/                  模块目录
User/                     用户配置和生成入口
webots/                   Webots world / proto / resources
libxr/                    libxr submodule
CMakePresets.json         命令行 CMake preset
.vscode/                  VS Code 配置
.devcontainer/            Dev Container 配置
docker/                   Windows Docker 辅助脚本
```

## Prepare

模块和 submodule 由使用者按项目约定初始化。开始构建前确认这些目录已经存在：

```text
libxr/
Modules/
```

如果 OpenVINO 不在 CMake 默认搜索路径里，在本机环境中设置 `OpenVINO_DIR`
或 `CMAKE_PREFIX_PATH`。

## Generate

```bash
python3 -m xrobot.GenerateMain --output User/xrobot_main.hpp --config User/xrobot.yaml
```

`User/xrobot_main.hpp` 是生成文件。

## Build

```bash
cmake -S . -B build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DAUTO_AIM_PREVIEW_IMAGE=ON
cmake --build build/debug --target rm_auto_aim -j$(nproc)
```

## Run

有桌面环境时直接打开 Webots world。无头预览可用：

```bash
python3 run_headless_preview.py --repo . --controller build/debug/rm_auto_aim --run-root .vscode-runs --runtime-sec 10
```

## VS Code

Webots BSP 可在 Dev Container 中使用。

推荐扩展：

- `ms-vscode.cmake-tools`
- `llvm-vs-code-extensions.vscode-clangd`
- `webfreak.debug`
- `xrobot.xrobot`

常用入口：

- `CMake: Select a Kit`
- `Tasks: Run Task` -> `Build: Webots debug`
- `Tasks: Run Task` -> `Webots: headless preview 10s`
- `Tasks: Run Task` -> `Webots: manual extern world`
- `Run and Debug` -> `Webots: Debug controller (paste extern URL)`
- `Run and Debug` -> `Webots: Attach to running rm_auto_aim`

## Docker On Windows

首次使用：

```powershell
.\docker\windows-deploy.ps1
```

运行一次无头预览：

```powershell
.\docker\windows-deploy.ps1 -Preview -RuntimeSec 10
```

离线镜像路径：

```powershell
.\docker\windows-deploy.ps1 -SkipImageBuild -ImageTar C:\path\to\bsp-webots-autoaim-webots-local.tar
```

模块默认由用户在仓库里手动初始化。确实需要 Docker 入口代为初始化时，显式设置
`XR_FORCE_XROBOT_SETUP=1`。

常用 Docker 命令：

```powershell
docker compose build
docker compose run --rm --no-build autoaim-build
docker compose run --rm --no-build autoaim-preview
```
