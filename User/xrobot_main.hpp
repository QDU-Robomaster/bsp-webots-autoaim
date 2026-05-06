#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsGimbal.hpp"
#include "WebotsFireNotify.hpp"
#include "WebotsCamera.hpp"
#include "CameraFrameSync.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "Aimer.hpp"
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"
#include "xrobot_constexpr.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 23.0);
  static WebotsGimbal WebotsGimbal_0(hw, appmgr);
  static WebotsFireNotify WebotsFireNotify_0(hw, appmgr);
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
      {2, {0.1, 0.1, true, 16.0}, false, "host", "robot_game_ref", {false, "armor_detector_preview", 0.5, 1, 1, "window", "0.0.0.0", 8080, 80, "armor_detector"}},
      CameraFrameSync_0
  );
  static ArmorTracker<ProjectConstexpr::MainCameraInfo> ArmorTracker_0(
      hw,
      appmgr,
      {{30.0, 30.0}, {0.15, 1.0}, {5, 0.3}, {0.092, 100, 0.19133, 0.21265, SolveTrajectory::NORMAL, {13.0, 0.0, 1.0, -1.0, 0.01, "table.bin"}}, {20.0, 100.0, 800.0}, {0.26, 0.12, 0.4}, {0.05, 0.02}, {{0.5, -0.5, 0.5, -0.5}, {0.0, 0.0, 0.0}}, {true, 0.05, true}},
      CameraFrameSync_0
  );
  static Aimer Aimer_0(
      hw,
      appmgr,
      {-1.0, -1.4, 60.0, 20.0, 2.0, 0.03, 0.015, 23.0, 14.0, 3.0, 2.0, 2.0, true, true, 0.0035, 50.0, 9000000.0, 0.0, 1.0, 100.0, 9000000.0, 0.0, 1.0, 7}
  );
  static SharedTopic SharedTopic_Host(hw, appmgr, "uart_host", 512, {{"bullet_speed", "referee"}});
  static SharedTopicClient SharedTopicClient_Host(
      hw,
      appmgr,
      "uart_host",
      512,
      {{"target_eulr", "tracker"}, {"fire_notify", "tracker"}, {"gimbal_plan", "tracker"}}
  );
  static SharedTopic SharedTopic_MCU(
      hw,
      appmgr,
      "uart_client",
      512,
      {"target_eulr", "fire_notify", "gimbal_plan"}
  );
  static SharedTopicClient SharedTopicClient_MCU(hw, appmgr, "uart_client", 512, {"bullet_speed"});

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
