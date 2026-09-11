#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsCamera.hpp"
#include "CameraSync.hpp"
#include "CameraFrameSync.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "Aimer.hpp"
#include "WebotsGimbal.hpp"
#include "WebotsFireNotify.hpp"
#include "xrobot_constexpr.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 23.0);
  static WebotsCamera<ProjectConstexpr::MainFrameLayout> WebotsCamera_0(
      hw,
      appmgr,
      ProjectConstexpr::MainCameraCalibration,
      {"camera", 100, 0.8, 0.0, "camera", ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName, "libxr_def_domain", "CAMERA", true, 20000}
  );
  static CameraSync CameraSync_0(
      hw,
      appmgr,
      "CAMERA",
      "camera_sync_result",
      "camera_gyro",
      20000,
      "camera_sync_command"
  );
  static CameraFrameSync<
      ProjectConstexpr::MainFrameLayout
  > CameraFrameSync_0(
      hw,
      appmgr,
      WebotsCamera_0,
      {CameraFrameSyncMode::TRIGGER, 0, "libxr_def_domain", "camera_sync_command", "camera_sync_result", 1, 10000, CameraFrameSyncRawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP, ProjectConstexpr::MainQuatTopicName}
  );
  static ArmorDetector<ProjectConstexpr::MainFrameLayout> ArmorDetector_0(
      hw,
      appmgr,
      {2, {ArmorDetectorModel::OPENVINO_640X512, 0.1, true, 16.0, 0.619, 0.45, 0.1, 128}, false, "host", "robot_game_ref", {true, "armor_detector_preview", 0.5, 1, 1, "web", "0.0.0.0", 8080, "armor_detector", 30.0}},
      CameraFrameSync_0
  );
  static ArmorTracker<ProjectConstexpr::MainFrameLayout> ArmorTracker_0(
      hw,
      appmgr,
      {{false, -1, 2, 15, 75, {1.6, 2.0, 1.2, 0.8, 2.0, 8.0, 7.5, 6000.0, 4.0, 8.0, 0.5, 0.55, 0.35, 0.25}}, {{{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}}, {true, "armor_tracker_preview", 0.5, 1, 1, "web", "0.0.0.0", 8080, "armor_tracker", 30.0}},
      CameraFrameSync_0
  );
  static Aimer<ProjectConstexpr::MainFrameLayout> aimer(
      hw,
      appmgr,
      {0.0, 0.0, 2.0, 23.0, 14.0, 0.02, 0.001, 16, -20.0, 35.0, true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015, 0.03, 0.003, 0.05, true, 0.05, 50.0, 9000000.0, 0.0, 1.0, 100.0, 9000000.0, 0.0, 1.0, {true, "aimer_preview", 0.5, 1, 1, "web", "0.0.0.0", 8080, "aimer_preview", 30.0}, true, 0.05, 1.0, false, "robot_game_ref"},
      ProjectConstexpr::MainCameraCalibration
  );
  static WebotsGimbal WebotsGimbal_0(hw, appmgr);
  static WebotsFireNotify WebotsFireNotify_0(hw, appmgr, 23.0, 10.0, 240.0, 40.0, 20.0, 30.0, 10);

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}