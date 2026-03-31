#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsGimbal.hpp"
#include "WebotsFireNotify.hpp"
#include "WebotsCamera.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "Aimer.hpp"
#include "SharedTopic.hpp"
#include "SharedTopicClient.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 30.0);
  static WebotsGimbal WebotsGimbal_0(hw, appmgr);
  static WebotsFireNotify WebotsFireNotify_0(hw, appmgr);
  static WebotsCamera WebotsCamera_0(hw, appmgr, {800, 600, 2400, 0, CameraBase::Encoding::RGB8, {1300.258730617794, 0.0, 396.2904, 0.0, 1300.258730617794, 304.1172, 0.0, 0.0, 1.0}, CameraBase::DistortionModel::PLUMB_BOB, {-0.09558691800515781, 0.3013704144837407, -0.0008218465102445683, 0.00024582434306615617, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {2323.906982421875, 0.0, 712.9446224841959, 0.0, 0.0, 2324.767578125, 546.6426169058832, 0.0, 0.0, 0.0, 1.0, 0.0}}, {"camera", 30, 2.0, 0.0});
  static ArmorDetector ArmorDetector_0(hw, appmgr, {0, {85.0, 48.0, 0.05, 0.6, 8.0, 0.5, 5.8, 3.2, 35.0}, {false, 0, 0, 800, 600, true, 0.7, 0.3, 0.8}, {false, false, 1, 0.75}});
  static ArmorTracker ArmorTracker_0(hw, appmgr, {{10.0, 1.0}, {5, 15, 75}, {{0.5, -0.5, 0.5, -0.5}, {0.0, 0.0, 0.0}}, {false, 1, 0.75, true}});
  static Aimer Aimer_0(hw, appmgr, {0.0, 0.0, 60.0, 20.0, 2.0, 0.03, 0.015, 30.0, 14.0, 3.0, 2.0, 2.0, true});
  static SharedTopic SharedTopic_Host(hw, appmgr, "uart_host", 8192, 256, {{"bullet_speed", "referee"}, {"rotation", "gimbal"}});
  static SharedTopicClient SharedTopicClient_Host(hw, appmgr, "uart_host", 8192, 256, {{"send", "tracker"}, {"target_eulr", "tracker"}});
  static SharedTopic SharedTopic_MCU(hw, appmgr, "uart_client", 8192, 256, {"send", "target_eulr"});
  static SharedTopicClient SharedTopicClient_MCU(hw, appmgr, "uart_client", 8192, 256, {"bullet_speed", "rotation"});

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
