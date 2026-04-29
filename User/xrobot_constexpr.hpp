#pragma once
#include "camera_types.hpp"
namespace ProjectConstexpr {
inline constexpr CameraTypes::CameraInfo MainCameraInfo = {800, 600, 2400, CameraTypes::Encoding::BGR8, {1300.258730617794, 0.0, 400.0, 0.0, 1300.258730617794, 300.0, 0.0, 0.0, 1.0}, CameraTypes::DistortionModel::PLUMB_BOB, {0.0, 0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {1300.258730617794, 0.0, 400.0, 0.0, 0.0, 1300.258730617794, 300.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
inline constexpr const char* MainImageTopicName = "camera_image";
inline constexpr const char* MainImuTopicName = "camera_imu";
inline constexpr const char* MainGyroTopicName = "camera_gyro";
inline constexpr const char* MainAcclTopicName = "camera_accl";
inline constexpr const char* MainQuatTopicName = "camera_quat";
}  // namespace ProjectConstexpr
