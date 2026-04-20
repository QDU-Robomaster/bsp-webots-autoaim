#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsGimbal.hpp"
#include "WebotsFireNotify.hpp"
#include "WebotsCamera.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"
#include "xrobot_constexpr.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 30.0);
  static WebotsGimbal WebotsGimbal_0(hw, appmgr);
  static WebotsFireNotify WebotsFireNotify_0(hw, appmgr);
  static WebotsCamera<ProjectConstexpr::MainCameraInfo> WebotsCamera_0(
      hw,
      appmgr,
      {"camera", 100, 0.8, 0.0}
  );
  static ArmorDetector ArmorDetector_0(
      hw,
      appmgr,
      {2, {85.0, 48.0, 1.5, 20.0, 8.0, 1.0, 5.0, 1.5, 25.0}, {false, 420, 50, 600, 600, true, 0.5, 0.3, 0.5}, {false, false, 1, 0.75}},
      ProjectConstexpr::MainCameraInfo
  );
  static ArmorTracker ArmorTracker_0(
      hw,
      appmgr,
      {{10.0, 10.0}, {0.45, 1.3}, {2, 0.3}, {1e-08, 0, 0.0, 0.0, SolveTrajectory::CalculateMode::NORMAL, {13.0, 0.0, 1.0, -1.0, 0.01, "Modules/ArmorTracker/table.bin"}}, {20.0, 100.0, 800}, {0.05, 0.02}, {{0.5, -0.5, 0.5, -0.5}, {0.0, 0.0, 0.0}}},
      ProjectConstexpr::MainCameraInfo
  );
  static SharedTopic SharedTopic_Host(
      hw,
      appmgr,
      "uart_host",
      8192,
      256,
      {{"bullet_speed", "referee"}, {"rotation", "gimbal"}}
  );
  static SharedTopicClient SharedTopicClient_Host(
      hw,
      appmgr,
      "uart_host",
      8192,
      256,
      {{"fire_notify", "tracker"}, {"target_eulr", "tracker"}}
  );
  static SharedTopic SharedTopic_MCU(
      hw,
      appmgr,
      "uart_client",
      8192,
      256,
      {"fire_notify", "target_eulr"}
  );
  static SharedTopicClient SharedTopicClient_MCU(
      hw,
      appmgr,
      "uart_client",
      8192,
      256,
      {"bullet_speed", "rotation"}
  );

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}