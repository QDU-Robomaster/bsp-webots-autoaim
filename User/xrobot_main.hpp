#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsCamera.hpp"
#include "CameraFrameSync.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "xrobot_constexpr.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsCamera<ProjectConstexpr::MainCameraInfo> WebotsCamera_0(
      hw,
      appmgr,
      {"camera", 100, 0.8, 0.0, "camera", ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName}
  );
  static CameraFrameSync<
      ProjectConstexpr::MainCameraInfo
  > CameraFrameSync_0(
      hw,
      appmgr,
      WebotsCamera_0
  );
  static ArmorDetector<ProjectConstexpr::MainCameraInfo> ArmorDetector_0(
      hw,
      appmgr,
      {2, {85.0, 48.0, 1.5, 20.0, 8.0}, {false, 420, 50, 600, 600, true, 0.5, 0.3, 0.5}},
      CameraFrameSync_0
  );
  static ArmorTracker<ProjectConstexpr::MainCameraInfo> ArmorTracker_0(
      hw,
      appmgr,
      {{30.0, 30.0}, {0.15, 1.0}, {5, 0.3}, {0.092, 100, 0.19133, 0.21265, SolveTrajectory::NORMAL, {13.0, 0.0, 1.0, -1.0, 0.01, "table.bin"}}, {20.0, 100.0, 800.0}, {0.26, 0.12, 0.4}, {0.05, 0.02}, {{0.5, -0.5, 0.5, -0.5}, {0.0, 0.0, 0.0}}, {true, 0.05, true}},
      CameraFrameSync_0
  );

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
