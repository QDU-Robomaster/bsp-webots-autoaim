#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include "thread.hpp"
#include "WebotsReferee.hpp"
#include "WebotsCamera.hpp"
#include "CameraSync.hpp"
#include "CameraFrameSync.hpp"
#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "Aimer.hpp"
#include "WebotsGimbal.hpp"
#include "WebotsFireNotify.hpp"

namespace xrobot_generated {
template <typename...> struct TypeList {};
template <typename Source, typename... Views>
struct RegistrationMatches
    : std::bool_constant<(!std::is_reference<Views>::value && ...) &&
                         (std::is_convertible<Source*, Views*>::value && ...)> {};

template <typename> struct MonitorSignature : std::false_type {};
template <typename T> struct MonitorSignature<void (T::*)()> : std::true_type {};
template <typename T> struct MonitorSignature<void (T::*)() noexcept> : std::true_type {};
template <typename T> struct MonitorSignature<void (T::*)() const> : std::true_type {};
template <typename T> struct MonitorSignature<void (T::*)() const noexcept> : std::true_type {};
template <typename T> struct MonitorSignature<void (T::*)() &> : std::true_type {};
template <typename T> struct MonitorSignature<void (T::*)() & noexcept> : std::true_type {};
template <typename T, typename = void> struct HasMonitor : std::false_type {};
template <typename T>
struct HasMonitor<T, std::void_t<decltype(&T::OnMonitor)>>
    : MonitorSignature<decltype(&T::OnMonitor)> {};

template <typename T> inline void Monitor(T& instance) {
  if constexpr (HasMonitor<T>::value) {
    instance.OnMonitor();
  }
}
}  // namespace xrobot_generated

// Force only this entry inline in optimized Clang builds.
#if defined(__clang__) && defined(__OPTIMIZE__) && !defined(LIBXR_DEBUG_BUILD) && \
    ((defined(XROBOT_OPTIMIZED_BUILD) && XROBOT_OPTIMIZED_BUILD) || \
     (!defined(XROBOT_OPTIMIZED_BUILD) && defined(NDEBUG)))
#define XR_XROBOT_MAIN_INLINE [[gnu::always_inline]] inline
#else
#define XR_XROBOT_MAIN_INLINE inline
#endif

[[noreturn]] XR_XROBOT_MAIN_INLINE void XRobotMain(
    LibXR::RamFS& ramfs) {
  static const WebotsReferee::Param& xr_arg_WebotsReferee_0_param =
      {
.bullet_speed = 23.0
, .shooter_heat_limit = 240.0f
, .shooter_cooling_value = 40.0f
, .robot_id = 7
, .robot_level = 1
, .max_hp = 200
, .chassis_power_limit = 45
, .publish_period_ms = 100
}
  ;
  // modules[0]: WebotsReferee_0
  static WebotsReferee WebotsReferee_0(
      xr_arg_WebotsReferee_0_param
  );
  // modules[1]: WebotsCamera_0
  static WebotsCamera<AutoAimRunConfig::Webots::MainFrameLayout> WebotsCamera_0(
      static_cast<LibXR::RamFS&>(ramfs)
      , AutoAimRunConfig::Webots::MainCameraCalibration
      , WebotsCamera<AutoAimRunConfig::Webots::MainFrameLayout>::RuntimeParam{
.device_name = "camera"
, .fps = 100
, .exposure = 0.8
, .gain = 0.0
, .pose_def_name = "camera"
, .image_topic_name = AutoAimRunConfig::Webots::MainImageTopicName
, .imu_topic_name = AutoAimRunConfig::Webots::MainImuTopicName
, .raw_topic_domain_name = "libxr_def_domain"
, .trigger_active_level = true
, .trigger_period_us = 20000
}
  );
  static const CameraSync::Param& xr_arg_CameraSync_0_param =
      {
.camera_sync_topic_name = "camera_sync_result"
, .imu_topic_name = "camera_gyro"
, .trigger_period_us = 20000
, .camera_sync_command_topic_name = "camera_sync_command"
}
  ;
  // modules[2]: CameraSync_0
  static CameraSync CameraSync_0(
      static_cast<LibXR::GPIO&>(WebotsCamera_0)
      , xr_arg_CameraSync_0_param
  );
  // modules[3]: CameraFrameSync_0
  static CameraFrameSync<AutoAimRunConfig::Webots::MainFrameLayout> CameraFrameSync_0(
      static_cast<CameraFrameSync<AutoAimRunConfig::Webots::MainFrameLayout>::Base&>(WebotsCamera_0)
      , CameraFrameSync<AutoAimRunConfig::Webots::MainFrameLayout>::RuntimeParam{
CameraFrameSyncMode::TRIGGER
, 0
, "libxr_def_domain"
, "camera_sync_command"
, "camera_sync_result"
, 1
, 10000
, CameraFrameSyncRawImuFrame::BODY_X_RIGHT_Y_FORWARD_Z_UP
, AutoAimRunConfig::Webots::MainQuatTopicName
}
  );
  // modules[4]: ArmorDetector_0
  static ArmorDetector<AutoAimRunConfig::Webots::MainFrameLayout> ArmorDetector_0(
      static_cast<ArmorDetector<AutoAimRunConfig::Webots::MainFrameLayout>::Sync&>(CameraFrameSync_0)
      , ArmorDetector<AutoAimRunConfig::Webots::MainFrameLayout>::Config{
.detect_color = 2
, .network = {
.model = ArmorDetectorModel::OPENVINO_640X512
, .min_confidence = 0.1
, .enable_quad_check = true
, .min_quad_area_px = 16.0
, .logit_threshold = 0.619
, .nms_threshold = 0.45
, .bbox_expand = 0.1
, .max_detections = 128
}
, .referee_auto_detect_color = false
, .referee_domain = "host"
, .referee_topic = "robot_game_ref"
, .preview = {
.enabled = true
, .preview_window_name = "armor_detector_preview"
, .preview_scale = 0.5
, .preview_wait_key_ms = 1
, .queue_capacity = 1
, .output_mode = "web"
, .web_bind_address = "0.0.0.0"
, .web_port = 8080
, .web_stream_name = "armor_detector"
, .max_fps = 30.0
}
}
  );
  // modules[5]: ArmorTracker_0
  static ArmorTracker<AutoAimRunConfig::Webots::MainFrameLayout> ArmorTracker_0(
      static_cast<LibXR::RamFS&>(ramfs)
      , static_cast<ArmorTracker<AutoAimRunConfig::Webots::MainFrameLayout>::FrameSync&>(CameraFrameSync_0)
      , ArmorTracker<AutoAimRunConfig::Webots::MainFrameLayout>::Config{
.tracker = {
.require_target_tag = false
, .target_tag_id = -1
, .min_detect_count = 2
, .max_temp_lost_count = 15
, .outpost_max_temp_lost_count = 75
, .target_select = {
.observed_count_weight = 1.6
, .distance_weight = 2.0
, .area_weight = 1.2
, .spin_weight = 0.8
, .angle_weight = 2.0
, .max_distance_m = 8.0
, .distance_span_m = 7.5
, .area_norm_px = 6000.0
, .observed_count_norm = 4.0
, .max_spin_rad_s = 8.0
, .max_angle_norm = 0.5
, .detecting_scale = 0.55
, .temp_lost_scale = 0.35
, .switch_margin = 0.25
}
}
, .extrinsic = {
.camera_mount_to_body = {
.rotation = {
1.0
, 0.0
, 0.0
, 0.0
}
, .translation = {
0.0
, 0.0
, 0.0
}
}
}
, .preview = {
.enabled = true
, .preview_window_name = "armor_tracker_preview"
, .preview_scale = 0.5
, .preview_wait_key_ms = 1
, .queue_capacity = 1
, .output_mode = "web"
, .web_bind_address = "0.0.0.0"
, .web_port = 8080
, .web_stream_name = "armor_tracker"
, .max_fps = 30.0
}
}
  );
  // modules[6]: aimer
  static Aimer<AutoAimRunConfig::Webots::MainFrameLayout> aimer(
      Aimer<AutoAimRunConfig::Webots::MainFrameLayout>::Config{
.yaw_offset = 0.0
, .roll_offset = 0.0
, .yaw_rate_threshold = 2.0
, .default_bullet_speed = 23.0
, .min_valid_bullet_speed = 14.0
, .ballistic_drag_k = 0.02
, .ballistic_integration_dt_s = 0.001
, .ballistic_max_iterations = 16
, .ballistic_min_elevation_deg = -20.0
, .ballistic_max_elevation_deg = 35.0
, .auto_fire = true
, .image_to_now_s = 0.0
, .vision_to_command_delay_s = 0.0
, .command_transport_delay_s = 0.0
, .gimbal_response_delay_s = 0.0
, .fire_delay_s = 0.0
, .low_speed_extra_predict_s = 0.015
, .high_speed_extra_predict_s = 0.03
, .min_fire_threshold = 0.003
, .max_fire_threshold = 0.05
, .enable_mpc_plan = true
, .mpc_fire_thresh = 0.05
, .max_yaw_acc = 50.0
, .q_yaw_pos = 9000000.0
, .q_yaw_vel = 0.0
, .r_yaw_acc = 1.0
, .max_roll_acc = 100.0
, .q_roll_pos = 9000000.0
, .q_roll_vel = 0.0
, .r_roll_acc = 1.0
, .preview = {
.enabled = true
, .preview_window_name = "aimer_preview"
, .preview_scale = 0.5
, .preview_wait_key_ms = 1
, .queue_capacity = 1
, .output_mode = "web"
, .web_bind_address = "0.0.0.0"
, .web_port = 8080
, .web_stream_name = "aimer_preview"
, .max_fps = 30.0
}
, .enable_runtime_log = true
, .bullet_speed_log_delta = 0.05
, .heat_log_delta = 1.0
, .convert_raw_gimbal_quat_to_body = false
, .referee_topic = "robot_game_ref"
}
      , AutoAimRunConfig::Webots::MainCameraCalibration
  );
  static const WebotsGimbal::Param& xr_arg_WebotsGimbal_0_param =
      {
.pid_pitch_angle = WebotsGimbal::DefaultPitchAnglePid()
, .pid_pitch_omega = WebotsGimbal::DefaultPitchOmegaPid()
, .pid_yaw_angle = WebotsGimbal::DefaultYawAnglePid()
, .pid_yaw_omega = WebotsGimbal::DefaultYawOmegaPid()
, .pitch_inertia = 0.00012f
, .yaw_inertia = 0.0002f
, .pitch_torque_limit = 0.035f
, .yaw_torque_limit = 0.04f
, .control_period_ms = 1
, .log_interval = 1000
}
  ;
  // modules[7]: WebotsGimbal_0
  static WebotsGimbal WebotsGimbal_0(
      xr_arg_WebotsGimbal_0_param
  );
  static const WebotsFireNotify::Param& xr_arg_WebotsFireNotify_0_param =
      {
.bullet_speed = 23.0
, .single_shot_heat = 10.0
, .shooter_heat_limit = 240.0
, .shooter_cooling_value = 40.0
, .max_fire_frequency_hz = 20.0
, .fire_delay_ms = 30.0
, .state_publish_period_ms = 10
}
  ;
  // modules[8]: WebotsFireNotify_0
  static WebotsFireNotify WebotsFireNotify_0(
      xr_arg_WebotsFireNotify_0_param
  );
  for (;;) {
    ::xrobot_generated::Monitor(WebotsReferee_0);
    ::xrobot_generated::Monitor(WebotsCamera_0);
    ::xrobot_generated::Monitor(CameraSync_0);
    ::xrobot_generated::Monitor(CameraFrameSync_0);
    ::xrobot_generated::Monitor(ArmorDetector_0);
    ::xrobot_generated::Monitor(ArmorTracker_0);
    ::xrobot_generated::Monitor(aimer);
    ::xrobot_generated::Monitor(WebotsGimbal_0);
    ::xrobot_generated::Monitor(WebotsFireNotify_0);
    LibXR::Thread::Sleep(1000);
  }
}

#undef XR_XROBOT_MAIN_INLINE

/* User Code Begin XRobotMain */
/* User Code End XRobotMain */
// clang-format off
// NOLINTBEGIN
#define XR_REGISTER_DETAIL_ramfs(...) \
  static_assert(std::is_same<::xrobot_generated::TypeList<__VA_ARGS__>, \
      ::xrobot_generated::TypeList<LibXR::RamFS>>::value && \
      ::xrobot_generated::RegistrationMatches< \
          std::remove_reference_t<decltype(ramfs)>, __VA_ARGS__>::value, \
      "XR_REGISTER changed; regenerate xrobot_main.hpp")
#define XR_REGISTER(name, ...) XR_REGISTER_DETAIL_##name(__VA_ARGS__)

#define XROBOT_MAIN() ::XRobotMain(ramfs)

// NOLINTEND
// clang-format on
