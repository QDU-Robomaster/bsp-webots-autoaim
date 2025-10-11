#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsReferee.hpp"
#include "WebotsCamera.hpp"
#include "BlinkLED.hpp"
#include "SharedTopicClient.hpp"
#include "CameraBase.hpp"
#include "WebotsGimbal.hpp"
#include "ArmorDetector.hpp"
#include "SharedTopic.hpp"
#include "ArmorTracker.hpp"
#include "WebotsFireNotify.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsReferee WebotsReferee_0(hw, appmgr, 30.0);
  static WebotsGimbal WebotsGimbal_0(hw, appmgr);
  static WebotsFireNotify WebotsFireNotify_0(hw, appmgr);
  static WebotsCamera WebotsCamera_0(hw, appmgr, {1440, 1080, 4320, 0, CameraBase::Encoding::RGB8, {2340.46464112537, 0.0, 713.3224120377864, 0.0, 2336.8745144649124, 547.4106752074272, 0.0, 0.0, 1.0}, CameraBase::DistortionModel::PLUMB_BOB, {-0.09558691800515781, 0.3013704144837407, -0.0008218465102445683, 0.00024582434306615617, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {2323.906982421875, 0.0, 712.9446224841959, 0.0, 0.0, 2324.767578125, 546.6426169058832, 0.0, 0.0, 0.0, 1.0, 0.0}}, {"camera", 30, 0.2, 0.0});
  static ArmorDetector ArmorDetector_0(hw, appmgr, {{{ArmorNumber::NEGATIVE}, 0.5}, 0, 85, {0.1, 0.4, 40.0}, {0.7, 0.8, 3.2, 3.2, 5.5, 35.0}});
  static ArmorTracker ArmorTracker_0(hw, appmgr, {{10.0}, {0.15, 1.0}, {5, 0.3}, {0.092, 100, 0.19133, 0.21265}, {20.0, 100.0, 800}, {0.05, 0.02}, {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});
  static SharedTopic SharedTopic_Host(hw, appmgr, "uart_host", 8192, 256, {{"bullet_speed", "referee"}, {"rotation", "gimbal"}});
  static SharedTopicClient SharedTopicClient_Host(hw, appmgr, "uart_host", 8192, 256, {{"fire_notify", "tracker"}, {"target_eulr", "tracker"}});
  static SharedTopic SharedTopic_MCU(hw, appmgr, "uart_client", 8192, 256, {"fire_notify", "target_eulr"});
  static SharedTopicClient SharedTopicClient_MCU(hw, appmgr, "uart_client", 8192, 256, {"bullet_speed", "rotation"});

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}