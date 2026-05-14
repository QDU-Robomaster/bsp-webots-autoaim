#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsGimbal.hpp"
#include "WebotsFireNotify.hpp"
#include "WebotsCamera.hpp"
#include "CameraSync.hpp"
#include "CameraFrameSync.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "Aimer.hpp"
#include "xrobot_constexpr.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 23.0);
  static WebotsGimbal WebotsGimbal_0(hw, appmgr);
  static WebotsFireNotify WebotsFireNotify_0(hw, appmgr, 23.0, 10.0, 240.0, 40.0, 20.0, 30.0, 10);
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
      {2, {0.1, true, 16.0, "AUTO_DETECT", "LATENCY"}, false, "host", "robot_game_ref", {false, "armor_detector_preview", 0.5, 1, 1, "window", "0.0.0.0", 8080, "armor_detector", 30.0}, {true, 0.2, 0.9, true}},
      CameraFrameSync_0
  );
  static ArmorTracker<ProjectConstexpr::MainCameraInfo> ArmorTracker_0(
      hw,
      appmgr,
      {{false, -1, 2, 15, 75}, {{{0.7071067811865476, -0.7071067811865475, 0.0, 0.0}, {0.0, 0.0, 0.0}}}, {false, "armor_tracker_preview", 0.5, 1, 1, "window", "0.0.0.0", 8080, "armor_tracker", 30.0}},
      CameraFrameSync_0
  );
  static Aimer<ProjectConstexpr::MainCameraInfo> aimer(
      hw,
      appmgr,
      {-1.0, -1.4, 2.0, 23.0, 14.0, true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015, 0.03, 0.003, 0.05, true, 0.05, 50.0, 9000000.0, 0.0, 1.0, 100.0, 9000000.0, 0.0, 1.0, {false, "aimer_preview", 0.5, 1, 1, "window", "0.0.0.0", 8080, "aimer_preview", 30.0}}
  );

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
