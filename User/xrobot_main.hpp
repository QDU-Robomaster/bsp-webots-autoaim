#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsCamera.hpp"
#include "CameraSync.hpp"
#include "CameraFrameSync.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"
#include "xrobot_constexpr.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 23.0);
  static WebotsCamera<ProjectConstexpr::MainCameraInfo> WebotsCamera_0(
      hw,
      appmgr,
      {"camera", 100, 0.8, 0.0, "camera", ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName, "libxr_def_domain", "CAMERA", true}
  );
  static CameraSync CameraSync_0(
      hw,
      appmgr,
      "CAMERA",
      "camera_sync_result",
      "camera_gyro",
      3,
      "camera_sync_command"
  );
  static CameraFrameSync<
      ProjectConstexpr::MainCameraInfo
  > CameraFrameSync_0(
      hw,
      appmgr,
      WebotsCamera_0,
      {CameraFrameSync<ProjectConstexpr::MainCameraInfo>::SyncMode::RAW_PROBE, 0, "libxr_def_domain", "camera_sync_command", "camera_sync_result", 3, 1, 50.0F}
  );
  static ArmorDetector<ProjectConstexpr::MainCameraInfo> ArmorDetector_0(
      hw,
      appmgr,
      {2, {0.1, 0.1, true, 16.0, "AUTO_DETECT", "LATENCY"}, false, "host", "robot_game_ref", {false, "armor_detector_preview", 0.5, 1, 1, "window", "0.0.0.0", 8080, "armor_detector", 30.0}, {false, true, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.15, 1.0}, {true, 0.5, 0.9, true}},
      CameraFrameSync_0
  );
  static ArmorTracker<ProjectConstexpr::MainCameraInfo> ArmorTracker_0(
      hw,
      appmgr,
      {{30.0, 30.0}, {0.15, 1.0}, {5, 0.3}, {20.0, 100.0, 800.0}, {0.26, 0.12, 0.4}, {0.05, 0.02}, {{0.5, -0.5, 0.5, -0.5}, {0.0, 0.0, 0.0}}, {true, 0.25, true, false, false, true, false}, {false, "armor_tracker_preview", 0.5, 1, 1, "window", "0.0.0.0", 8080, "armor_tracker", 30.0}},
      CameraFrameSync_0
  );
  static SharedTopic SharedTopic_Host(hw, appmgr, "uart_host", 512, {"bullet_speed"});
  static SharedTopicClient SharedTopicClient_MCU(hw, appmgr, "uart_client", 512, {"bullet_speed"});

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
