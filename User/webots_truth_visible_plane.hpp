#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraBase.hpp"

struct WebotsTruthVisibleFace
{
  bool valid{false};
  Eigen::Vector3d center_optical = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_optical = Eigen::Matrix3d::Identity();
  Eigen::Vector3d normal_optical = Eigen::Vector3d::Zero();
  std::array<cv::Point2f, 4> points{};
  cv::Point2f center{};
  cv::Rect box{};
  double frontality{0.0};
};

inline const Eigen::Matrix3d& WebotsCameraNodeToOpticalRotation()
{
  static const Eigen::Matrix3d kCameraNodeToOptical =
      (Eigen::Matrix3d() << 0.0, -1.0, 0.0,
       0.0, 0.0, -1.0,
       1.0, 0.0, 0.0)
          .finished();
  return kCameraNodeToOptical;
}

inline std::array<cv::Point2f, 4> SortTruthQuadPoints(std::array<cv::Point2f, 4> points)
{
  const cv::Point2f center =
      (points[0] + points[1] + points[2] + points[3]) * 0.25f;
  std::sort(points.begin(), points.end(),
            [&](const cv::Point2f& lhs, const cv::Point2f& rhs)
            {
              const double lhs_angle = std::atan2(lhs.y - center.y, lhs.x - center.x);
              const double rhs_angle = std::atan2(rhs.y - center.y, rhs.x - center.x);
              return lhs_angle < rhs_angle;
            });
  return points;
}

inline bool FiniteTruthPoint(const cv::Point2f& point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

inline bool ProjectWebotsTruthFaceBox(const CameraTypes::CameraInfo& camera_info,
                                     const Eigen::Matrix3d& r_optical_box,
                                     const Eigen::Vector3d& t_optical_box,
                                     double width,
                                     double height,
                                     double depth,
                                     WebotsTruthVisibleFace& face)
{
  const double half_width = width * 0.5;
  const double half_height = height * 0.5;
  const double half_depth = depth * 0.5;
  const std::array<Eigen::Vector3d, 4> face_corners_box = {
      Eigen::Vector3d(half_width, half_height, -half_depth),
      Eigen::Vector3d(half_width, -half_height, -half_depth),
      Eigen::Vector3d(-half_width, -half_height, -half_depth),
      Eigen::Vector3d(-half_width, half_height, -half_depth)};
  const Eigen::Vector3d face_center_box(0.0, 0.0, -half_depth);

  face = WebotsTruthVisibleFace{};
  if (camera_info.width == 0 || camera_info.height == 0)
  {
    return false;
  }

  cv::Mat camera_matrix(3, 3, CV_64F,
                        const_cast<double*>(camera_info.camera_matrix.data()));
  camera_matrix = camera_matrix.clone();

  const auto dist = CameraTypes::BuildPnPDistCoeffs(camera_info);
  cv::Mat dist_coeffs;
  if (!dist.empty())
  {
    dist_coeffs = cv::Mat(1, static_cast<int>(dist.size()), CV_64F,
                          const_cast<double*>(dist.data()))
                      .clone();
  }

  const cv::Rect image_rect(
      0, 0, static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
  const cv::Mat zero_rvec = cv::Mat::zeros(3, 1, CV_64F);
  const cv::Mat zero_tvec = cv::Mat::zeros(3, 1, CV_64F);

  std::vector<cv::Point3f> optical_points;
  optical_points.reserve(4);
  std::array<cv::Point2f, 4> projected_points{};
  bool valid = true;
  for (std::size_t i = 0; i < face_corners_box.size(); ++i)
  {
    const Eigen::Vector3d point_optical = t_optical_box + r_optical_box * face_corners_box[i];
    valid = valid && point_optical.allFinite() && point_optical.z() > 1e-6;
    optical_points.emplace_back(point_optical.x(), point_optical.y(), point_optical.z());
  }
  if (!valid)
  {
    return false;
  }

  const Eigen::Vector3d center_optical = t_optical_box + r_optical_box * face_center_box;
  if (!center_optical.allFinite() || !(center_optical.z() > 1e-6))
  {
    return false;
  }

  const Eigen::Vector3d width_axis = (r_optical_box * Eigen::Vector3d::UnitX()).normalized();
  const Eigen::Vector3d up_axis =
      (r_optical_box * (-Eigen::Vector3d::UnitY())).normalized();
  Eigen::Vector3d normal_axis = width_axis.cross(up_axis).normalized();
  if (!normal_axis.allFinite())
  {
    return false;
  }
  if (normal_axis.dot(-center_optical) < 0.0)
  {
    normal_axis = -normal_axis;
  }
  Eigen::Vector3d y_axis = width_axis.normalized();
  Eigen::Vector3d z_axis = normal_axis.cross(y_axis).normalized();
  y_axis = z_axis.cross(normal_axis).normalized();
  if (!(y_axis.allFinite() && z_axis.allFinite()))
  {
    return false;
  }

  const double frontality = normal_axis.normalized().dot((-center_optical).normalized());
  if (!(frontality > 0.05))
  {
    return false;
  }

  std::vector<cv::Point2f> projected;
  cv::projectPoints(optical_points, zero_rvec, zero_tvec, camera_matrix, dist_coeffs,
                    projected);
  if (projected.size() != 4)
  {
    return false;
  }

  valid = true;
  for (std::size_t i = 0; i < projected_points.size(); ++i)
  {
    projected_points[i] = projected[i];
    valid = valid && FiniteTruthPoint(projected_points[i]);
  }
  if (!valid)
  {
    return false;
  }

  projected_points = SortTruthQuadPoints(projected_points);
  const std::vector<cv::Point2f> contour(projected_points.begin(), projected_points.end());
  const double area = std::abs(cv::contourArea(contour));
  if (!(area > 1.0))
  {
    return false;
  }

  const cv::Rect box = cv::boundingRect(contour);
  if ((box & image_rect).area() <= 0)
  {
    return false;
  }

  face.valid = true;
  face.center_optical = center_optical;
  face.normal_optical = normal_axis;
  face.points = projected_points;
  face.center =
      (projected_points[0] + projected_points[1] + projected_points[2] +
       projected_points[3]) *
      0.25f;
  face.box = box;
  face.frontality = frontality;
  face.rotation_optical.col(0) = normal_axis;
  face.rotation_optical.col(1) = y_axis;
  face.rotation_optical.col(2) = z_axis;
  return true;
}

inline constexpr double kWebotsTruthSpArmorWidth = 0.135;
inline constexpr double kWebotsTruthSpArmorHeight = 0.056;
inline constexpr double kWebotsTruthSpArmorDepth = 0.0001;
inline constexpr double kWebotsTruthLightbarLength = 0.056;

inline double WebotsTruthVisiblePlaneEnvOffset(const char* name)
{
  if (name == nullptr)
  {
    return 0.0;
  }
  if (const char* env = std::getenv(name))
  {
    char* end = nullptr;
    const double value = std::strtod(env, &end);
    if (end != env && std::isfinite(value))
    {
      return value;
    }
  }
  return 0.0;
}

inline Eigen::Vector3d WebotsTruthVisiblePlaneDelta()
{
  return Eigen::Vector3d(WebotsTruthVisiblePlaneEnvOffset("XR_TRUTH_VISIBLE_DX"),
                         WebotsTruthVisiblePlaneEnvOffset("XR_TRUTH_VISIBLE_DY"),
                         WebotsTruthVisiblePlaneEnvOffset("XR_TRUTH_VISIBLE_DZ"));
}

inline double WebotsTruthVisiblePlaneEnvPositiveOrDefault(const char* name,
                                                          double fallback)
{
  const double value = WebotsTruthVisiblePlaneEnvOffset(name);
  if (std::isfinite(value) && value > 0.0)
  {
    return value;
  }
  return fallback;
}

inline double WebotsTruthSpArmorWidth()
{
  return WebotsTruthVisiblePlaneEnvPositiveOrDefault("XR_TRUTH_ARMOR_WIDTH",
                                                     kWebotsTruthSpArmorWidth);
}

inline double WebotsTruthSpArmorHeight()
{
  return WebotsTruthVisiblePlaneEnvPositiveOrDefault("XR_TRUTH_ARMOR_HEIGHT",
                                                     kWebotsTruthSpArmorHeight);
}

inline double WebotsTruthSpArmorDepth()
{
  return WebotsTruthVisiblePlaneEnvPositiveOrDefault("XR_TRUTH_ARMOR_DEPTH",
                                                     kWebotsTruthSpArmorDepth);
}

inline Eigen::Vector3d WebotsTruthVisiblePlaneTranslation()
{
  return Eigen::Vector3d(0.0, -0.009, -0.0008) + WebotsTruthVisiblePlaneDelta();
}

inline bool ProjectWebotsTruthVisibleFace(
    const CameraTypes::CameraInfo& camera_info,
    const Eigen::Matrix3d& r_optical_armor_root,
    const Eigen::Vector3d& t_optical_armor_root,
    WebotsTruthVisibleFace& face)
{
  // Match SP solvePnP semantics: 135 mm armor width and 56 mm lightbar length,
  // not the 120 x 110 mm white digit-board visible plane.
  static const Eigen::Matrix3d kVisiblePlaneRotation =
      Eigen::AngleAxisd(-CV_PI * 0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d kVisiblePlaneTranslation = WebotsTruthVisiblePlaneTranslation();

  const Eigen::Matrix3d r_optical_box = r_optical_armor_root * kVisiblePlaneRotation;
  const Eigen::Vector3d t_optical_box =
      t_optical_armor_root + r_optical_armor_root * kVisiblePlaneTranslation;
  return ProjectWebotsTruthFaceBox(camera_info, r_optical_box, t_optical_box,
                                   WebotsTruthSpArmorWidth(),
                                   WebotsTruthSpArmorHeight(),
                                   WebotsTruthSpArmorDepth(), face);
}

inline bool ProjectWebotsTruthLightbarCenterlines(
    const CameraTypes::CameraInfo& camera_info,
    const Eigen::Matrix3d& r_optical_left_lightbar,
    const Eigen::Vector3d& t_optical_left_lightbar,
    const Eigen::Matrix3d& r_optical_right_lightbar,
    const Eigen::Vector3d& t_optical_right_lightbar,
    WebotsTruthVisibleFace& face)
{
  face = WebotsTruthVisibleFace{};
  if (camera_info.width == 0 || camera_info.height == 0)
  {
    return false;
  }

  const double half_length = kWebotsTruthLightbarLength * 0.5;
  const Eigen::Vector3d left_axis =
      (r_optical_left_lightbar * Eigen::Vector3d::UnitY()).normalized();
  const Eigen::Vector3d right_axis =
      (r_optical_right_lightbar * Eigen::Vector3d::UnitY()).normalized();
  const Eigen::Vector3d height_axis = (left_axis + right_axis).normalized();
  if (!(left_axis.allFinite() && right_axis.allFinite() && height_axis.allFinite()))
  {
    return false;
  }

  std::array<Eigen::Vector3d, 4> optical_points = {
      t_optical_left_lightbar - left_axis * half_length,
      t_optical_right_lightbar - right_axis * half_length,
      t_optical_right_lightbar + right_axis * half_length,
      t_optical_left_lightbar + left_axis * half_length};
  std::array<cv::Point2f, 4> projected_points{};
  std::vector<cv::Point3f> cv_points;
  cv_points.reserve(optical_points.size());
  bool valid = true;
  for (std::size_t i = 0; i < optical_points.size(); ++i)
  {
    valid = valid && optical_points[i].allFinite() && optical_points[i].z() > 1e-6;
    cv_points.emplace_back(optical_points[i].x(), optical_points[i].y(),
                           optical_points[i].z());
  }
  if (!valid)
  {
    return false;
  }

  cv::Mat camera_matrix(3, 3, CV_64F,
                        const_cast<double*>(camera_info.camera_matrix.data()));
  camera_matrix = camera_matrix.clone();
  const auto dist = CameraTypes::BuildPnPDistCoeffs(camera_info);
  cv::Mat dist_coeffs;
  if (!dist.empty())
  {
    dist_coeffs = cv::Mat(1, static_cast<int>(dist.size()), CV_64F,
                          const_cast<double*>(dist.data()))
                      .clone();
  }
  const cv::Mat zero_rvec = cv::Mat::zeros(3, 1, CV_64F);
  const cv::Mat zero_tvec = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(cv_points, zero_rvec, zero_tvec, camera_matrix, dist_coeffs,
                    projected);
  if (projected.size() != projected_points.size())
  {
    return false;
  }
  for (std::size_t i = 0; i < projected_points.size(); ++i)
  {
    projected_points[i] = projected[i];
    valid = valid && FiniteTruthPoint(projected_points[i]);
  }
  if (!valid)
  {
    return false;
  }

  projected_points = SortTruthQuadPoints(projected_points);
  const std::vector<cv::Point2f> contour(projected_points.begin(), projected_points.end());
  const double area = std::abs(cv::contourArea(contour));
  if (!(area > 1.0))
  {
    return false;
  }
  const cv::Rect image_rect(
      0, 0, static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
  const cv::Rect box = cv::boundingRect(contour);
  if ((box & image_rect).area() <= 0)
  {
    return false;
  }

  const Eigen::Vector3d center_optical =
      (t_optical_left_lightbar + t_optical_right_lightbar) * 0.5;
  const Eigen::Vector3d width_axis =
      (t_optical_right_lightbar - t_optical_left_lightbar).normalized();
  Eigen::Vector3d normal_axis = width_axis.cross(height_axis).normalized();
  if (!normal_axis.allFinite())
  {
    return false;
  }
  if (normal_axis.dot(-center_optical) < 0.0)
  {
    normal_axis = -normal_axis;
  }
  Eigen::Vector3d y_axis = width_axis.normalized();
  Eigen::Vector3d z_axis = normal_axis.cross(y_axis).normalized();
  y_axis = z_axis.cross(normal_axis).normalized();
  if (!(center_optical.allFinite() && y_axis.allFinite() && z_axis.allFinite()))
  {
    return false;
  }

  const double frontality = normal_axis.normalized().dot((-center_optical).normalized());
  if (!(frontality > 0.05))
  {
    return false;
  }

  face.valid = true;
  face.center_optical = center_optical;
  face.normal_optical = normal_axis;
  face.points = projected_points;
  face.center =
      (projected_points[0] + projected_points[1] + projected_points[2] +
       projected_points[3]) *
      0.25f;
  face.box = box;
  face.frontality = frontality;
  face.rotation_optical.col(0) = normal_axis;
  face.rotation_optical.col(1) = y_axis;
  face.rotation_optical.col(2) = z_axis;
  return true;
}
