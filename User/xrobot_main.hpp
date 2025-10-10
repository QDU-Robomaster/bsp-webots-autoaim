#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "WebotsCamera.hpp"
#include "BlinkLED.hpp"
#include "CameraBase.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static WebotsCamera WebotsCamera_0(hw, appmgr, {1280, 720, 3840, 0, CameraBase::Encoding::RGB8, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, CameraBase::DistortionModel::PLUMB_BOB, {0.0, 0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, {"camera", 30, 0.15, 0.0});
  static ArmorDetector ArmorDetector_0(hw, appmgr, {{{ArmorNumber::NEGATIVE}, 0.7}, 1, 85, {0.1, 0.4, 40.0}, {0.7, 0.8, 3.2, 3.2, 5.5, 35.0}});
  static ArmorTracker ArmorTracker_0(hw, appmgr, {{10.0}, {0.15, 1.0}, {5, 0.3}, {0.092, 100, 0.19133, 0.21265}, {20.0, 100.0, 800}, {0.05, 0.02}, {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}