#pragma once

// Compile-time constants for each run configuration. Each YAML under
// User/ names its namespace explicitly; edit values here, not in generated code.

#include "CameraBase.hpp"

namespace AutoAimRunConfig {

namespace Webots {
inline constexpr CameraTypes::FrameLayout MainFrameLayout = {.width = 800, .height = 600, .step = 2400, .encoding = CameraTypes::Encoding::BGR8};
inline constexpr CameraTypes::CameraCalibration MainCameraCalibration = {.native_width = 800, .native_height = 600, .camera_matrix = {1300.258730617794, 0.0, 400.0, 0.0, 1300.258730617794, 300.0, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {1300.258730617794, 0.0, 400.0, 0.0, 0.0, 1300.258730617794, 300.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
inline constexpr const char* MainImageTopicName = "camera_image";
inline constexpr const char* MainImuTopicName = "camera_imu";
inline constexpr const char* MainQuatTopicName = "camera_quat";
}  // namespace Webots

}  // namespace AutoAimRunConfig
