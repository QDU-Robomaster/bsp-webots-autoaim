#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include "CameraFrameSync.hpp"
#include "armor.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "webots_truth_visible_plane.hpp"
#include "xrobot_constexpr.hpp"

class DetectorTruthCompare
{
 public:
  using MainFrameSync = CameraFrameSync<ProjectConstexpr::MainCameraInfo>;
  using SyncedFrame = MainFrameSync::SyncedFrame;

  DetectorTruthCompare(const DetectorTruthCompare&) = delete;
  DetectorTruthCompare& operator=(const DetectorTruthCompare&) = delete;

  DetectorTruthCompare(DetectorTruthCompare&&) = delete;
  DetectorTruthCompare& operator=(DetectorTruthCompare&&) = delete;

  ~DetectorTruthCompare()
  {
    std::lock_guard<std::mutex> lock(file_lock_);
    CloseFilesLocked();
    WriteSummaryLocked(done_.load(std::memory_order_relaxed) ? "done" : "exit");
  }

  DetectorTruthCompare()
  {
    const char* pairs_env = std::getenv("XR_DETECTOR_TRUTH_COMPARE_PATH");
    if (pairs_env != nullptr && pairs_env[0] != '\0')
    {
      pairs_path_ = pairs_env;
    }
    else
    {
      pairs_path_ = "detector_truth_compare_pairs.tsv";
    }

    frames_path_ = pairs_path_ + ".frames.tsv";
    summary_path_ = pairs_path_ + ".summary.txt";

    if (const char* max_frames_env = std::getenv("XR_DETECTOR_TRUTH_COMPARE_MAX_FRAMES"))
    {
      char* end = nullptr;
      const unsigned long parsed = std::strtoul(max_frames_env, &end, 10);
      if (end != max_frames_env && parsed > 0UL)
      {
        max_frames_ = static_cast<uint32_t>(
            std::min<unsigned long>(parsed, std::numeric_limits<uint32_t>::max()));
      }
    }
  }

  void Init(webots::Supervisor* supervisor)
  {
    supervisor_ = supervisor;
    (void)ResolveNodes();
  }

  void InstallBlocking()
  {
    LibXR::Topic::Domain armor_domain("armor_detector");
    auto armors_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("armors_result", UINT32_MAX, &armor_domain));
    auto armors_cb = LibXR::Topic::Callback::Create(
        [](bool, DetectorTruthCompare* self, LibXR::RawData& data)
        {
          auto* msg = reinterpret_cast<ArmorDetectionsMessage*>(data.addr_);
          self->ArmorsCallback(msg);
        },
        this);
    armors_topic.RegisterCallback(armors_cb);

    XR_LOG_PASS(
        "DetectorTruthCompare subscribed: image=%s imu=%s armor_detector/armors_result -> %s",
        ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName,
        pairs_path_.c_str());

    while (!Done())
    {
      MainFrameSync::Subscriber subscriber(ProjectConstexpr::MainImageTopicName,
                                           ProjectConstexpr::MainImuTopicName);
      if (!subscriber.Valid())
      {
        LibXR::Thread::Sleep(200);
        continue;
      }

      SyncedFrame synced_frame;
      while (!Done())
      {
        const auto wait_ans = subscriber.Wait(synced_frame, kSyncFrameWaitTimeoutMs);
        if (wait_ans == LibXR::ErrorCode::TIMEOUT)
        {
          continue;
        }
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          break;
        }

        SyncFrameCallback(synced_frame);
      }
    }
  }

  bool Done() const { return done_.load(std::memory_order_relaxed); }

 private:
  static constexpr uint32_t kSyncFrameWaitTimeoutMs = 100;
  static constexpr std::size_t kTruthFrameCacheSize = 64;
  static constexpr const char* kTruthVisibleFaceProtoDef = "XR_VISIBLE_FACE_POSE";
  static constexpr const char* kTruthLeftLightbarProtoDef = "XR_LEFT_LIGHTBAR_POSE";
  static constexpr const char* kTruthRightLightbarProtoDef = "XR_RIGHT_LIGHTBAR_POSE";
  static constexpr std::array<const char*, 4> kTruthLabels = {
      "front", "right", "back", "left"};
  static constexpr std::array<const char*, 4> kTruthNodeLabels = {
      "armor_front", "armor_right", "armor_back", "armor_left"};
  static constexpr std::array<std::array<double, 3>, 4> kArmorLocalPosSpin = {
      std::array<double, 3>{0.0, 0.205, -0.06},
      std::array<double, 3>{0.205, 0.0, -0.11},
      std::array<double, 3>{0.0, -0.205, -0.06},
      std::array<double, 3>{-0.205, 0.0, -0.11}};

  struct TruthFrameSnapshot
  {
    uint64_t image_timestamp_us{0};
    double sim_time_s{0.0};
    LibXR::Transform<double> camera_pose_world{};
    std::array<Eigen::Vector3d, 4> gt_world{};
    std::array<Eigen::Vector3d, 4> gt_cam{};
    std::array<std::array<cv::Point2f, 4>, 4> gt_image_points{};
    std::array<bool, 4> gt_visible{};
  };

  struct AssignmentResult
  {
    std::vector<int> det_indices{};
    std::vector<int> gt_index_for_det{};
    std::vector<double> pair_error_m{};
    double total_error_m = std::numeric_limits<double>::infinity();
    bool valid = false;
  };

  struct FixedOrderPnpProbe
  {
    bool valid{false};
    int method{-1};
    int point_order{-1};
    double reprojection_rmse_px{0.0};
    Eigen::Vector3d cam = Eigen::Vector3d::Zero();
    Eigen::Vector3d world = Eigen::Vector3d::Zero();
  };

  struct Pose3d
  {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  };

  static bool IsFiniteVec(const Eigen::Vector3d& v)
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  static double DistanceMeters(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
  {
    return (a - b).norm();
  }

  static bool FinitePoint(const cv::Point2f& point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y);
  }

  static std::array<cv::Point2f, 4> SortQuadPoints(const std::array<cv::Point2f, 4>& points)
  {
    std::array<cv::Point2f, 4> sorted = points;
    std::sort(sorted.begin(), sorted.end(),
              [](const cv::Point2f& lhs, const cv::Point2f& rhs)
              {
                return lhs.y < rhs.y;
              });

    std::array<cv::Point2f, 2> top_points = {sorted[0], sorted[1]};
    std::array<cv::Point2f, 2> bottom_points = {sorted[2], sorted[3]};
    std::sort(top_points.begin(), top_points.end(),
              [](const cv::Point2f& lhs, const cv::Point2f& rhs)
              {
                return lhs.x < rhs.x;
              });
    std::sort(bottom_points.begin(), bottom_points.end(),
              [](const cv::Point2f& lhs, const cv::Point2f& rhs)
              {
                return lhs.x < rhs.x;
              });
    return {top_points[0], top_points[1], bottom_points[1], bottom_points[0]};
  }

  static double MeanCornerErrorPx(const std::array<cv::Point2f, 4>& det_points,
                                  const std::array<cv::Point2f, 4>& gt_points)
  {
    double sum = 0.0;
    for (std::size_t i = 0; i < det_points.size(); ++i)
    {
      sum += cv::norm(det_points[i] - gt_points[i]);
    }
    return sum / static_cast<double>(det_points.size());
  }

  static double MaxCornerErrorPx(const std::array<cv::Point2f, 4>& det_points,
                                 const std::array<cv::Point2f, 4>& gt_points)
  {
    double max_err = 0.0;
    for (std::size_t i = 0; i < det_points.size(); ++i)
    {
      max_err = std::max(max_err,
                         static_cast<double>(cv::norm(det_points[i] - gt_points[i])));
    }
    return max_err;
  }

  static Eigen::Vector3d MeanPoint(const std::vector<Eigen::Vector3d>& points,
                                   const std::vector<int>& indices)
  {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    if (indices.empty())
    {
      return mean;
    }
    for (const int index : indices)
    {
      mean += points[static_cast<std::size_t>(index)];
    }
    return mean / static_cast<double>(indices.size());
  }

  static Eigen::Vector3d MeanPoint(const std::vector<Eigen::Vector3d>& points)
  {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    if (points.empty())
    {
      return mean;
    }
    for (const auto& point : points)
    {
      mean += point;
    }
    return mean / static_cast<double>(points.size());
  }

  static LibXR::Quaternion<double> PackedCameraRotation(
      const std::array<float, 4>& rotation_wxyz)
  {
    return LibXR::Quaternion<double>(rotation_wxyz[0], rotation_wxyz[1],
                                     rotation_wxyz[2], rotation_wxyz[3]);
  }

  static const cv::Mat& CameraMatrix()
  {
    static const cv::Mat camera_matrix = []()
    {
      return cv::Mat(3, 3, CV_64F,
                     const_cast<double*>(
                         ProjectConstexpr::MainCameraInfo.camera_matrix.data()))
          .clone();
    }();
    return camera_matrix;
  }

  static const cv::Mat& DistCoeffs()
  {
    static const cv::Mat dist_coeffs = []()
    {
      const auto coeffs =
          CameraTypes::BuildPnPDistCoeffs(ProjectConstexpr::MainCameraInfo);
      if (coeffs.size == 0)
      {
        return cv::Mat();
      }
      return cv::Mat(1, static_cast<int>(coeffs.size), CV_64F,
                     const_cast<double*>(coeffs.values.data()))
          .clone();
    }();
    return dist_coeffs;
  }

  static bool ProjectOpticalPoint(const CameraTypes::CameraInfo& camera_info,
                                  const Eigen::Vector3d& point_optical,
                                  cv::Point2f& image_point)
  {
    if (!(point_optical.allFinite() && point_optical.z() > 1e-6))
    {
      return false;
    }

    const double x = point_optical.x() / point_optical.z();
    const double y = point_optical.y() / point_optical.z();
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const auto& d = camera_info.distortion_coefficients;
    const double k1 = d[0];
    const double k2 = d[1];
    const double p1 = d[2];
    const double p2 = d[3];
    const double k3 = d[4];
    const double k4 = d[5];
    const double k5 = d[6];
    const double k6 = d[7];

    const double radial_num = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double radial_den = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
    if (!(std::isfinite(radial_num) && std::isfinite(radial_den) &&
          std::abs(radial_den) > 1e-12))
    {
      return false;
    }
    const double radial = radial_num / radial_den;

    const double x_dist = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double y_dist = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

    const auto& k = camera_info.camera_matrix;
    const double u = k[0] * x_dist + k[2];
    const double v = k[4] * y_dist + k[5];
    if (!(std::isfinite(u) && std::isfinite(v)))
    {
      return false;
    }

    image_point = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    return true;
  }

  static bool ProjectTruthFaceBox(const CameraTypes::CameraInfo& camera_info,
                                  const Eigen::Matrix3d& r_optical_box,
                                  const Eigen::Vector3d& t_optical_box,
                                  double width, double height, double depth,
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

    const cv::Rect image_rect(
        0, 0, static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
    std::array<cv::Point2f, 4> projected_points{};
    for (std::size_t i = 0; i < face_corners_box.size(); ++i)
    {
      const Eigen::Vector3d point_optical =
          t_optical_box + r_optical_box * face_corners_box[i];
      if (!ProjectOpticalPoint(camera_info, point_optical, projected_points[i]))
      {
        return false;
      }
    }

    const Eigen::Vector3d center_optical = t_optical_box + r_optical_box * face_center_box;
    if (!(center_optical.allFinite() && center_optical.z() > 1e-6))
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

    projected_points = SortQuadPoints(projected_points);
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

  static bool ProjectTruthVisibleFace(const CameraTypes::CameraInfo& camera_info,
                                      const Eigen::Matrix3d& r_optical_armor_root,
                                      const Eigen::Vector3d& t_optical_armor_root,
                                      WebotsTruthVisibleFace& face)
  {
    static const Eigen::Matrix3d kVisiblePlaneRotation =
        Eigen::AngleAxisd(-CV_PI * 0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d kVisiblePlaneTranslation = WebotsTruthVisiblePlaneTranslation();
    const Eigen::Matrix3d r_optical_box = r_optical_armor_root * kVisiblePlaneRotation;
    const Eigen::Vector3d t_optical_box =
        t_optical_armor_root + r_optical_armor_root * kVisiblePlaneTranslation;
    return ProjectTruthFaceBox(camera_info, r_optical_box, t_optical_box,
                               WebotsTruthSpArmorWidth(), WebotsTruthSpArmorHeight(),
                               WebotsTruthSpArmorDepth(), face);
  }

  static const std::vector<cv::Point3f>& ArmorObjectPoints(ArmorType armor_type)
  {
    return ArmorObjectPoints(armor_type, 55.0);
  }

  static const std::vector<cv::Point3f>& ArmorObjectPoints(ArmorType armor_type,
                                                           double armor_height_mm)
  {
    static const auto build = [](double width_mm, double height_mm)
    {
      const double half_width_m = width_mm * 0.5 / 1000.0;
      const double half_height_m = height_mm * 0.5 / 1000.0;
      return std::vector<cv::Point3f>{
          {0.0f, static_cast<float>(half_width_m), static_cast<float>(-half_height_m)},
          {0.0f, static_cast<float>(half_width_m), static_cast<float>(half_height_m)},
          {0.0f, static_cast<float>(-half_width_m), static_cast<float>(half_height_m)},
          {0.0f, static_cast<float>(-half_width_m), static_cast<float>(-half_height_m)}};
    };

    static const std::vector<cv::Point3f> small = build(135.0, 55.0);
    static const std::vector<cv::Point3f> large = build(225.0, 55.0);
    static const std::vector<cv::Point3f> small_56 = build(135.0, 56.0);
    static const std::vector<cv::Point3f> large_56 = build(225.0, 56.0);
    if (std::abs(armor_height_mm - 56.0) < 1e-6)
    {
      return armor_type == ArmorType::SMALL ? small_56 : large_56;
    }
    return armor_type == ArmorType::SMALL ? small : large;
  }

  static double ReprojectionRmse(const std::vector<cv::Point3f>& object_points,
                                 const std::vector<cv::Point2f>& image_points,
                                 const cv::Mat& rvec, const cv::Mat& tvec)
  {
    if (object_points.size() != image_points.size() || object_points.empty())
    {
      return 0.0;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, CameraMatrix(), DistCoeffs(), projected);
    if (projected.size() != image_points.size())
    {
      return 0.0;
    }

    double squared_sum = 0.0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
      const cv::Point2f diff = projected[i] - image_points[i];
      squared_sum += static_cast<double>(diff.x) * static_cast<double>(diff.x) +
                     static_cast<double>(diff.y) * static_cast<double>(diff.y);
    }
    return std::sqrt(squared_sum / static_cast<double>(image_points.size()));
  }

  static FixedOrderPnpProbe SolveFixedOrderPnp(const ArmorDetectorResult& armor,
                                               const Eigen::Matrix3d& r_wc,
                                               const Eigen::Vector3d& twc)
  {
    FixedOrderPnpProbe probe;
    if (armor.type == ArmorType::INVALID)
    {
      return probe;
    }
    for (const auto& point : armor.points)
    {
      if (!FinitePoint(point))
      {
        return probe;
      }
    }

    const std::vector<cv::Point2f> image_points = {
        armor.points[3], armor.points[0], armor.points[1], armor.points[2]};
    return SolveFixedOrderPnp(armor.type, image_points, r_wc, twc);
  }

  static FixedOrderPnpProbe SolveFixedOrderPnp(
      ArmorType armor_type, const std::vector<cv::Point2f>& image_points,
      const Eigen::Matrix3d& r_wc, const Eigen::Vector3d& twc)
  {
    return SolveFixedOrderPnp(armor_type, image_points, r_wc, twc, 55.0);
  }

  static FixedOrderPnpProbe SolveFixedOrderPnp(
      ArmorType armor_type, const std::vector<cv::Point2f>& image_points,
      const Eigen::Matrix3d& r_wc, const Eigen::Vector3d& twc, double armor_height_mm)
  {
    FixedOrderPnpProbe probe;
    if (armor_type == ArmorType::INVALID || image_points.size() != 4)
    {
      return probe;
    }

    const auto& object_points = ArmorObjectPoints(armor_type, armor_height_mm);
    constexpr std::array<int, 3> kMethods = {
        cv::SOLVEPNP_IPPE,
        cv::SOLVEPNP_ITERATIVE,
        cv::SOLVEPNP_EPNP,
    };

    for (const int method : kMethods)
    {
      cv::Mat rvec;
      cv::Mat tvec;
      if (!cv::solvePnP(object_points, image_points, CameraMatrix(), DistCoeffs(), rvec, tvec,
                        false, method))
      {
        continue;
      }
      if (rvec.empty() || tvec.empty())
      {
        continue;
      }

      const double z = tvec.at<double>(2);
      if (!std::isfinite(z) || z <= 1e-6)
      {
        continue;
      }

      probe.valid = true;
      probe.method = method;
      probe.cam = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), z);
      probe.world = r_wc * probe.cam + twc;
      probe.reprojection_rmse_px =
          ReprojectionRmse(object_points, image_points, rvec, tvec);
      return probe;
    }

    return probe;
  }

  static FixedOrderPnpProbe SolveTruthCornerPnp(
      ArmorType armor_type, const std::array<cv::Point2f, 4>& truth_points,
      const Eigen::Vector3d& truth_cam, const Eigen::Matrix3d& r_wc,
      const Eigen::Vector3d& twc)
  {
    return SolveBestPermutationPnp(armor_type, truth_points, truth_cam, r_wc, twc, 55.0);
  }

  static FixedOrderPnpProbe SolveTruthCornerPnp56(
      ArmorType armor_type, const std::array<cv::Point2f, 4>& truth_points,
      const Eigen::Vector3d& truth_cam, const Eigen::Matrix3d& r_wc,
      const Eigen::Vector3d& twc)
  {
    return SolveBestPermutationPnp(armor_type, truth_points, truth_cam, r_wc, twc, 56.0);
  }

  static FixedOrderPnpProbe SolveBestPermutationPnp(
      ArmorType armor_type, const std::array<cv::Point2f, 4>& image_points,
      const Eigen::Vector3d& truth_cam, const Eigen::Matrix3d& r_wc,
      const Eigen::Vector3d& twc, double armor_height_mm)
  {
    for (const auto& point : image_points)
    {
      if (!FinitePoint(point))
      {
        return {};
      }
    }

    FixedOrderPnpProbe best{};
    double best_err_m = std::numeric_limits<double>::infinity();
    std::array<int, 4> order = {0, 1, 2, 3};
    int order_index = 0;
    do
    {
      std::vector<cv::Point2f> permuted_points;
      permuted_points.reserve(4);
      for (const int index : order)
      {
        permuted_points.push_back(image_points[static_cast<std::size_t>(index)]);
      }

      FixedOrderPnpProbe probe =
          SolveFixedOrderPnp(armor_type, permuted_points, r_wc, twc, armor_height_mm);
      if (!probe.valid)
      {
        order_index++;
        continue;
      }

      const double err_m = DistanceMeters(probe.cam, truth_cam);
      if (err_m < best_err_m)
      {
        best = probe;
        best.point_order = order_index;
        best_err_m = err_m;
      }
      order_index++;
    } while (std::next_permutation(order.begin(), order.end()));

    return best;
  }

  static Eigen::Matrix3d AxisAngleToRotation(const double* rot)
  {
    if (rot == nullptr)
    {
      return Eigen::Matrix3d::Identity();
    }

    const Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
    const double axis_norm = axis.norm();
    if (!(axis_norm > 1e-9) || !std::isfinite(rot[3]))
    {
      return Eigen::Matrix3d::Identity();
    }

    return Eigen::AngleAxisd(rot[3], axis / axis_norm).toRotationMatrix();
  }

  static bool ReadTopLevelRobotPose(webots::Node* node, Pose3d& pose)
  {
    if (node == nullptr)
    {
      return false;
    }

    webots::Field* translation_field = node->getField("translation");
    webots::Field* rotation_field = node->getField("rotation");
    if (translation_field == nullptr || rotation_field == nullptr)
    {
      return false;
    }

    const double* t = translation_field->getSFVec3f();
    const double* r = rotation_field->getSFRotation();
    if (t == nullptr || r == nullptr)
    {
      return false;
    }

    pose.translation = Eigen::Vector3d(t[0], t[1], t[2]);
    pose.rotation = AxisAngleToRotation(r);
    return true;
  }

  static bool ReadRelativePose(webots::Node* node, const webots::Node* from_node,
                               Pose3d& pose, const char* label, bool& logged)
  {
    (void)label;
    if (node == nullptr || from_node == nullptr)
    {
      return false;
    }

    const double* m = node->getPose(from_node);
    if (m == nullptr)
    {
      return false;
    }

    const Eigen::Vector3d t_row(m[3], m[7], m[11]);
    const Eigen::Vector3d t_col(m[12], m[13], m[14]);

    Eigen::Matrix3d r_row = Eigen::Matrix3d::Identity();
    r_row << m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10];

    Eigen::Matrix3d r_col = Eigen::Matrix3d::Identity();
    r_col << m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10];

    const bool use_col_major = t_col.norm() > t_row.norm();
    pose.translation = use_col_major ? t_col : t_row;
    pose.rotation = use_col_major ? r_col : r_row;

    if (!logged)
    {
      logged = true;
    }
    return true;
  }

  static webots::Node* FindNamedNodeRecursiveImpl(
      webots::Node* node, const char* wanted_name,
      std::unordered_set<const webots::Node*>& visited)
  {
    if (node == nullptr || wanted_name == nullptr)
    {
      return nullptr;
    }

    webots::Field* name_field = node->getField("name");
    if (name_field == nullptr && node->isProto())
    {
      name_field = node->getBaseNodeField("name");
    }
    if (name_field != nullptr && name_field->getSFString() == wanted_name)
    {
      return node;
    }

    if (!visited.insert(node).second)
    {
      return nullptr;
    }

    auto search_field = [&](webots::Field* field) -> webots::Node*
    {
      if (field == nullptr)
      {
        return nullptr;
      }
      if (field->getType() == webots::Field::SF_NODE)
      {
        if (webots::Node* found =
                FindNamedNodeRecursiveImpl(field->getSFNode(), wanted_name, visited))
        {
          return found;
        }
      }
      else if (field->getType() == webots::Field::MF_NODE)
      {
        for (int i = 0; i < field->getCount(); ++i)
        {
          if (webots::Node* found =
                  FindNamedNodeRecursiveImpl(field->getMFNode(i), wanted_name, visited))
          {
            return found;
          }
        }
      }
      return nullptr;
    };

    for (int i = 0; i < node->getNumberOfFields(); ++i)
    {
      if (webots::Node* found = search_field(node->getFieldByIndex(i)))
      {
        return found;
      }
    }
    if (node->isProto())
    {
      for (int i = 0; i < node->getNumberOfBaseNodeFields(); ++i)
      {
        if (webots::Node* found = search_field(node->getBaseNodeFieldByIndex(i)))
        {
          return found;
        }
      }
    }
    return nullptr;
  }

  static webots::Node* FindNamedNodeRecursive(webots::Node* node, const char* wanted_name)
  {
    std::unordered_set<const webots::Node*> visited;
    return FindNamedNodeRecursiveImpl(node, wanted_name, visited);
  }

  void SyncFrameCallback(const SyncedFrame& synced_frame)
  {
    const auto* image_frame = synced_frame.GetImageFrame();
    if (image_frame == nullptr)
    {
      return;
    }

    if (supervisor_ == nullptr)
    {
      skipped_no_supervisor_++;
      return;
    }

    if (!ResolveNodes())
    {
      skipped_no_nodes_++;
      return;
    }

    TruthFrameSnapshot snapshot;
    snapshot.image_timestamp_us = image_frame->timestamp_us;
    snapshot.sim_time_s = supervisor_->getTime();
    snapshot.camera_pose_world = LibXR::Transform<double>(
        PackedCameraRotation(synced_frame.imu.rotation_wxyz),
        LibXR::Position<double>(
            static_cast<double>(synced_frame.imu.translation_xyz[0]),
            static_cast<double>(synced_frame.imu.translation_xyz[1]),
            static_cast<double>(synced_frame.imu.translation_xyz[2])));

    std::array<WebotsTruthVisibleFace, 4> truth_faces{};
    if (!BuildVisibleTruthFaces(ProjectConstexpr::MainCameraInfo, truth_faces))
    {
      skipped_bad_truth_++;
      return;
    }

    const auto r_wc = snapshot.camera_pose_world.rotation.ToRotationMatrix();
    const Eigen::Vector3d twc(snapshot.camera_pose_world.translation.x(),
                              snapshot.camera_pose_world.translation.y(),
                              snapshot.camera_pose_world.translation.z());
    for (std::size_t i = 0; i < truth_faces.size(); ++i)
    {
      snapshot.gt_visible[i] = truth_faces[i].valid;
      snapshot.gt_cam[i] = Eigen::Vector3d::Zero();
      snapshot.gt_world[i] = Eigen::Vector3d::Zero();
      snapshot.gt_image_points[i] = {};
      if (!truth_faces[i].valid)
      {
        continue;
      }

      snapshot.gt_cam[i] = truth_faces[i].center_optical;
      snapshot.gt_world[i] = r_wc * truth_faces[i].center_optical + twc;
      snapshot.gt_image_points[i] = SortQuadPoints(truth_faces[i].points);
    }

    std::lock_guard<std::mutex> lock(state_lock_);
    truth_frame_cache_.push_back(snapshot);
    while (truth_frame_cache_.size() > kTruthFrameCacheSize)
    {
      truth_frame_cache_.pop_front();
    }
  }

  void ArmorsCallback(ArmorDetectionsMessage* msg)
  {
    if (msg == nullptr || Done())
    {
      return;
    }

    if (msg->image_timestamp_us == 0)
    {
      skipped_no_truth_frame_++;
      return;
    }

    TruthFrameSnapshot truth_frame;
    if (!LookupTruthFrame(msg->image_timestamp_us, truth_frame))
    {
      skipped_no_truth_frame_++;
      if (skipped_no_truth_frame_ == 1 || (skipped_no_truth_frame_ % 100U) == 0U)
      {
        XR_LOG_WARN(
            "DetectorTruthCompare missing cached truth for ts=%llu (miss=%u cache=%zu)",
            static_cast<unsigned long long>(msg->image_timestamp_us),
            skipped_no_truth_frame_, TruthFrameCacheSize());
      }
      return;
    }

    std::vector<Eigen::Vector3d> det_cam;
    std::vector<Eigen::Vector3d> det_world;
    std::vector<FixedOrderPnpProbe> fixed_probes;
    std::vector<int> det_indices;
    det_cam.reserve(std::min<std::size_t>(msg->results.size(), 4));
    det_world.reserve(std::min<std::size_t>(msg->results.size(), 4));
    fixed_probes.reserve(std::min<std::size_t>(msg->results.size(), 4));
    det_indices.reserve(std::min<std::size_t>(msg->results.size(), 4));

    const auto r_wc = truth_frame.camera_pose_world.rotation.ToRotationMatrix();
    const Eigen::Vector3d twc(truth_frame.camera_pose_world.translation.x(),
                              truth_frame.camera_pose_world.translation.y(),
                              truth_frame.camera_pose_world.translation.z());
    for (std::size_t armor_index = 0; armor_index < msg->results.size() && armor_index < 4;
         ++armor_index)
    {
      const auto& armor = msg->results[armor_index];
      const Eigen::Vector3d position(armor.pose.translation.x(), armor.pose.translation.y(),
                                     armor.pose.translation.z());
      if (!IsFiniteVec(position))
      {
        continue;
      }
      det_indices.push_back(static_cast<int>(armor_index));
      det_cam.push_back(position);
      det_world.push_back(r_wc * position + twc);
      fixed_probes.push_back(SolveFixedOrderPnp(armor, r_wc, twc));
    }

    if (det_cam.empty())
    {
      empty_detection_frames_++;
      return;
    }

    const AssignmentResult assignment = SolveBestAssignment(truth_frame, det_cam, det_indices);
    if (!assignment.valid)
    {
      skipped_bad_truth_++;
      return;
    }

    const double pair_count = static_cast<double>(assignment.det_indices.size());
    const double mean_error_m = assignment.total_error_m / pair_count;
    const double max_error_m =
        *std::max_element(assignment.pair_error_m.begin(), assignment.pair_error_m.end());
    const Eigen::Vector3d gt_center_cam =
        MeanPoint(VisibleTruthPoints(truth_frame.gt_cam), assignment.gt_index_for_det);
    const Eigen::Vector3d det_center_cam = MeanPoint(det_cam);
    const double center_error_m = DistanceMeters(gt_center_cam, det_center_cam);

    double shape_error_sum_m = 0.0;
    double frame_corner_error_sum_px = 0.0;
    double frame_corner_max_px = 0.0;
    for (std::size_t i = 0; i < assignment.det_indices.size(); ++i)
    {
      const int gt_index = assignment.gt_index_for_det[i];
      shape_error_sum_m += DistanceMeters(
          truth_frame.gt_cam[gt_index] - gt_center_cam,
          det_cam[i] - det_center_cam);
    }
    const double shape_mean_error_m = shape_error_sum_m / pair_count;
    const std::string assignment_str = AssignmentString(assignment);

    {
      std::lock_guard<std::mutex> lock(file_lock_);
      OpenFilesLocked();
      for (std::size_t i = 0; i < assignment.det_indices.size(); ++i)
      {
        const int det_index = assignment.det_indices[i];
        const int gt_index = assignment.gt_index_for_det[i];
        const auto& armor = msg->results[static_cast<std::size_t>(det_index)];
        const auto& det_pos = det_cam[i];
        const auto& det_pos_world = det_world[i];
        const auto& fixed_probe = fixed_probes[i];
        const auto sorted_det_points = SortQuadPoints(armor.points);
        const auto& gt_image_points = truth_frame.gt_image_points[gt_index];
        const double corner_mean_err_px =
            MeanCornerErrorPx(sorted_det_points, gt_image_points);
        const double corner_max_err_px =
            MaxCornerErrorPx(sorted_det_points, gt_image_points);
        const FixedOrderPnpProbe best_detector_perm_probe =
            SolveBestPermutationPnp(armor.type, armor.points, truth_frame.gt_cam[gt_index],
                                    r_wc, twc, 55.0);
        const double best_detector_perm_err_m =
            best_detector_perm_probe.valid
                ? DistanceMeters(best_detector_perm_probe.cam, truth_frame.gt_cam[gt_index])
                : 0.0;
        const FixedOrderPnpProbe truth_corner_probe =
            SolveTruthCornerPnp(armor.type, gt_image_points, truth_frame.gt_cam[gt_index],
                                r_wc, twc);
        const double truth_corner_pnp_err_m =
            truth_corner_probe.valid
                ? DistanceMeters(truth_corner_probe.cam, truth_frame.gt_cam[gt_index])
                : 0.0;
        const FixedOrderPnpProbe truth_corner_probe_56 =
            SolveTruthCornerPnp56(armor.type, gt_image_points, truth_frame.gt_cam[gt_index],
                                  r_wc, twc);
        const double truth_corner_pnp56_err_m =
            truth_corner_probe_56.valid
                ? DistanceMeters(truth_corner_probe_56.cam, truth_frame.gt_cam[gt_index])
                : 0.0;
        const double fixed_err_m =
            fixed_probe.valid ? DistanceMeters(fixed_probe.cam, truth_frame.gt_cam[gt_index])
                              : 0.0;
        const double current_vs_fixed_m =
            fixed_probe.valid ? DistanceMeters(det_pos, fixed_probe.cam) : 0.0;
        pairs_file_ << frame_count_ << '\t' << msg->image_timestamp_us << '\t'
                    << std::fixed << std::setprecision(6) << truth_frame.sim_time_s << '\t'
                    << det_index << '\t' << kTruthLabels[gt_index] << '\t'
                    << assignment.pair_error_m[i] << '\t'
                    << (truth_frame.gt_visible[gt_index] ? 1 : 0) << '\t'
                    << static_cast<int>(armor.number) << '\t'
                    << static_cast<int>(armor.type) << '\t'
                    << armor.confidence << '\t' << static_cast<int>(fixed_probe.valid) << '\t'
                    << fixed_probe.method << '\t' << fixed_err_m << '\t'
                    << fixed_probe.reprojection_rmse_px << '\t'
                    << static_cast<int>(best_detector_perm_probe.valid) << '\t'
                    << best_detector_perm_probe.point_order << '\t'
                    << best_detector_perm_err_m << '\t'
                    << static_cast<int>(truth_corner_probe.valid) << '\t'
                    << truth_corner_probe.point_order << '\t'
                    << truth_corner_pnp_err_m << '\t'
                    << truth_corner_probe.reprojection_rmse_px << '\t'
                    << static_cast<int>(truth_corner_probe_56.valid) << '\t'
                    << truth_corner_probe_56.point_order << '\t'
                    << truth_corner_pnp56_err_m << '\t'
                    << truth_corner_probe_56.reprojection_rmse_px << '\t'
                    << current_vs_fixed_m << '\t' << corner_mean_err_px << '\t'
                    << corner_max_err_px;
        for (std::size_t corner_index = 0; corner_index < 4; ++corner_index)
        {
          const auto& det_point = sorted_det_points[corner_index];
          const auto& gt_point = gt_image_points[corner_index];
          pairs_file_ << '\t' << det_point.x << '\t' << det_point.y << '\t' << gt_point.x
                      << '\t' << gt_point.y << '\t' << (det_point.x - gt_point.x) << '\t'
                      << (det_point.y - gt_point.y);
        }
        pairs_file_ << '\t' << det_pos.x() << '\t' << det_pos.y() << '\t'
                    << det_pos.z() << '\t' << truth_frame.gt_cam[gt_index].x() << '\t'
                    << truth_frame.gt_cam[gt_index].y() << '\t'
                    << truth_frame.gt_cam[gt_index].z() << '\t' << det_pos_world.x() << '\t'
                    << det_pos_world.y() << '\t' << det_pos_world.z() << '\t'
                    << fixed_probe.cam.x() << '\t' << fixed_probe.cam.y() << '\t'
                    << fixed_probe.cam.z() << '\t' << fixed_probe.world.x() << '\t'
                    << fixed_probe.world.y() << '\t' << fixed_probe.world.z() << '\t'
                    << truth_frame.gt_world[gt_index].x() << '\t'
                    << truth_frame.gt_world[gt_index].y() << '\t'
                    << truth_frame.gt_world[gt_index].z() << '\n';

        if (fixed_probe.valid)
        {
          fixed_pair_errors_.push_back(fixed_err_m);
          fixed_current_delta_errors_.push_back(assignment.pair_error_m[i] - fixed_err_m);
          if (fixed_err_m + 1e-6 < assignment.pair_error_m[i])
          {
            fixed_better_count_++;
          }
          else if (assignment.pair_error_m[i] + 1e-6 < fixed_err_m)
          {
            fixed_worse_count_++;
          }
          else
          {
            fixed_equal_count_++;
          }
        }
        else
        {
          fixed_missing_count_++;
        }

        corner_mean_errors_px_.push_back(corner_mean_err_px);
        corner_max_errors_px_.push_back(corner_max_err_px);
        if (best_detector_perm_probe.valid)
        {
          best_detector_perm_errors_.push_back(best_detector_perm_err_m);
        }
        else
        {
          best_detector_perm_missing_count_++;
        }
        if (truth_corner_probe.valid)
        {
          truth_corner_pnp_errors_.push_back(truth_corner_pnp_err_m);
        }
        else
        {
          truth_corner_pnp_missing_count_++;
        }
        if (truth_corner_probe_56.valid)
        {
          truth_corner_pnp56_errors_.push_back(truth_corner_pnp56_err_m);
        }
        else
        {
          truth_corner_pnp56_missing_count_++;
        }
        frame_corner_error_sum_px += corner_mean_err_px;
        frame_corner_max_px = std::max(frame_corner_max_px, corner_max_err_px);
      }

      const double frame_corner_mean_px = frame_corner_error_sum_px / pair_count;
      frames_file_ << frame_count_ << '\t' << msg->image_timestamp_us << '\t'
                   << std::fixed << std::setprecision(6) << truth_frame.sim_time_s << '\t'
                   << static_cast<int>(assignment.det_indices.size()) << '\t'
                   << VisibleTruthCount(truth_frame.gt_visible) << '\t' << mean_error_m << '\t'
                   << max_error_m << '\t' << center_error_m << '\t' << shape_mean_error_m
                   << '\t' << frame_corner_mean_px << '\t' << frame_corner_max_px << '\t'
                   << assignment_str << '\n';

      pairs_file_.flush();
      frames_file_.flush();
    }

    frame_mean_errors_.push_back(mean_error_m);
    frame_center_errors_.push_back(center_error_m);
    frame_shape_mean_errors_.push_back(shape_mean_error_m);
    for (double err : assignment.pair_error_m)
    {
      pair_errors_.push_back(err);
      if (err > 0.05)
      {
        pair_err_gt_5cm_++;
      }
      if (err > 0.10)
      {
        pair_err_gt_10cm_++;
      }
      if (err > 0.20)
      {
        pair_err_gt_20cm_++;
      }
    }

    frame_count_++;
    if (((frame_count_ % 50U) == 0U) || frame_count_ == 1U)
    {
      XR_LOG_PASS(
          "DetectorTruthCompare frames=%u sim_t=%.3f mean_err=%.4f center_err=%.4f shape_err=%.4f max_err=%.4f assign=%s",
          frame_count_, truth_frame.sim_time_s, mean_error_m, center_error_m,
          shape_mean_error_m, max_error_m, assignment_str.c_str());
    }

    if (frame_count_ >= max_frames_)
    {
      done_.store(true, std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(file_lock_);
      CloseFilesLocked();
      WriteSummaryLocked("done");
      XR_LOG_PASS("DetectorTruthCompare done: frames=%u path=%s", frame_count_,
                  pairs_path_.c_str());
    }
  }

  static int VisibleTruthCount(const std::array<bool, 4>& gt_visible)
  {
    int count = 0;
    for (const bool visible : gt_visible)
    {
      count += visible ? 1 : 0;
    }
    return count;
  }

  static std::vector<Eigen::Vector3d> VisibleTruthPoints(
      const std::array<Eigen::Vector3d, 4>& gt_cam)
  {
    std::vector<Eigen::Vector3d> out;
    out.reserve(gt_cam.size());
    for (const auto& point : gt_cam)
    {
      out.push_back(point);
    }
    return out;
  }

  bool LookupTruthFrame(uint64_t image_timestamp_us, TruthFrameSnapshot& snapshot_out)
  {
    std::lock_guard<std::mutex> lock(state_lock_);
    for (auto it = truth_frame_cache_.rbegin(); it != truth_frame_cache_.rend(); ++it)
    {
      if (it->image_timestamp_us == image_timestamp_us)
      {
        snapshot_out = *it;
        return true;
      }
    }
    return false;
  }

  std::size_t TruthFrameCacheSize()
  {
    std::lock_guard<std::mutex> lock(state_lock_);
    return truth_frame_cache_.size();
  }

  bool ResolveNamedRobotNode(const char* name, webots::Node*& out)
  {
    if (out != nullptr)
    {
      return true;
    }
    if (supervisor_ == nullptr)
    {
      return false;
    }

    webots::Node* root = supervisor_->getRoot();
    if (root == nullptr)
    {
      return false;
    }

    webots::Field* children = root->getField("children");
    if (children == nullptr)
    {
      return false;
    }

    for (int i = 0; i < children->getCount(); ++i)
    {
      webots::Node* node = children->getMFNode(i);
      if (node == nullptr)
      {
        continue;
      }
      webots::Field* name_field = node->getField("name");
      if (name_field == nullptr)
      {
        continue;
      }
      if (name_field->getSFString() == name)
      {
        out = node;
        return true;
      }
    }
    return false;
  }

  bool ResolveNodes()
  {
    bool all_armors_resolved = true;
    for (auto* node : armor_nodes_)
    {
      all_armors_resolved = all_armors_resolved && (node != nullptr);
    }
    if (target_spin_node_ != nullptr && camera_node_ != nullptr &&
        target_robot_node_ != nullptr && self_robot_node_ != nullptr &&
        all_armors_resolved)
    {
      return true;
    }
    if (supervisor_ == nullptr)
    {
      return false;
    }

    target_spin_node_ = supervisor_->getFromDef("TARGET_SPIN");
    camera_node_ = supervisor_->getFromDef("camera");
    if (target_spin_node_ == nullptr || camera_node_ == nullptr)
    {
      return false;
    }
    if (!ResolveNamedRobotNode("target", target_robot_node_))
    {
      return false;
    }
    if (!ResolveNamedRobotNode("self", self_robot_node_))
    {
      return false;
    }

    webots::Node* root = supervisor_->getRoot();
    if (root == nullptr)
    {
      return false;
    }
    for (std::size_t i = 0; i < armor_nodes_.size(); ++i)
    {
      if (armor_nodes_[i] == nullptr)
      {
        armor_nodes_[i] = FindNamedNodeRecursive(root, kTruthNodeLabels[i]);
      }
      if (armor_nodes_[i] == nullptr)
      {
        return false;
      }
      if (visible_face_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        visible_face_nodes_[i] = armor_nodes_[i]->getFromProtoDef(kTruthVisibleFaceProtoDef);
      }
      if (left_lightbar_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        left_lightbar_nodes_[i] = armor_nodes_[i]->getFromProtoDef(kTruthLeftLightbarProtoDef);
      }
      if (right_lightbar_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        right_lightbar_nodes_[i] =
            armor_nodes_[i]->getFromProtoDef(kTruthRightLightbarProtoDef);
      }
    }

    XR_LOG_PASS("DetectorTruthCompare resolved target/self/camera/armor nodes");
    return true;
  }

  bool BuildVisibleTruthFaces(const CameraTypes::CameraInfo& camera_info,
                              std::array<WebotsTruthVisibleFace, 4>& truth_faces)
  {
    if (!ResolveNodes())
    {
      return false;
    }

    for (auto& face : truth_faces)
    {
      face = WebotsTruthVisibleFace{};
    }

    for (int face_index = 0; face_index < 4; ++face_index)
    {
      Pose3d armor_in_camera_node;
      if (!ReadRelativePose(armor_nodes_[face_index], camera_node_, armor_in_camera_node,
                            kTruthNodeLabels[face_index], armor_pose_logged_[face_index]))
      {
        continue;
      }

      const Eigen::Matrix3d r_optical_armor_root =
          WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.rotation;
      const Eigen::Vector3d t_optical_armor_root =
          WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.translation;

      if (left_lightbar_nodes_[face_index] != nullptr &&
          right_lightbar_nodes_[face_index] != nullptr)
      {
        Pose3d left_lightbar_in_camera_node;
        Pose3d right_lightbar_in_camera_node;
        if (ReadRelativePose(left_lightbar_nodes_[face_index], camera_node_,
                             left_lightbar_in_camera_node, kTruthLeftLightbarProtoDef,
                             left_lightbar_pose_logged_[face_index]) &&
            ReadRelativePose(right_lightbar_nodes_[face_index], camera_node_,
                             right_lightbar_in_camera_node, kTruthRightLightbarProtoDef,
                             right_lightbar_pose_logged_[face_index]))
        {
          if (ProjectWebotsTruthLightbarCenterlines(
                  camera_info,
                  WebotsCameraNodeToOpticalRotation() *
                      left_lightbar_in_camera_node.rotation,
                  WebotsCameraNodeToOpticalRotation() *
                      left_lightbar_in_camera_node.translation,
                  WebotsCameraNodeToOpticalRotation() *
                      right_lightbar_in_camera_node.rotation,
                  WebotsCameraNodeToOpticalRotation() *
                      right_lightbar_in_camera_node.translation,
                  truth_faces[face_index]))
          {
            continue;
          }
        }
      }

      if (visible_face_nodes_[face_index] != nullptr)
      {
        Pose3d visible_face_in_camera_node;
        if (ReadRelativePose(visible_face_nodes_[face_index], camera_node_,
                             visible_face_in_camera_node, kTruthVisibleFaceProtoDef,
                             visible_face_pose_logged_[face_index]))
        {
          const Eigen::Matrix3d r_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.rotation;
          const Eigen::Vector3d t_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.translation +
              r_optical_armor_root * WebotsTruthVisiblePlaneDelta();
          if (ProjectTruthFaceBox(camera_info, r_optical_box, t_optical_box,
                                  WebotsTruthSpArmorWidth(),
                                  WebotsTruthSpArmorHeight(),
                                  WebotsTruthSpArmorDepth(), truth_faces[face_index]))
          {
            continue;
          }
        }
      }
      ProjectTruthVisibleFace(camera_info, r_optical_armor_root,
                             t_optical_armor_root, truth_faces[face_index]);
    }
    return true;
  }

  static AssignmentResult SolveBestAssignment(const TruthFrameSnapshot& truth_frame,
                                              const std::vector<Eigen::Vector3d>& det_cam,
                                              const std::vector<int>& det_indices)
  {
    AssignmentResult best{};
    if (det_cam.empty() || det_indices.empty())
    {
      return best;
    }

    const int visible_count = VisibleTruthCount(truth_frame.gt_visible);
    if (visible_count < static_cast<int>(det_cam.size()))
    {
      return best;
    }

    best.det_indices = det_indices;
    std::array<int, 4> gt_order = {0, 1, 2, 3};
    do
    {
      bool visible_prefix = true;
      for (std::size_t i = 0; i < det_cam.size(); ++i)
      {
        if (!truth_frame.gt_visible[static_cast<std::size_t>(gt_order[i])])
        {
          visible_prefix = false;
          break;
        }
      }
      if (!visible_prefix)
      {
        continue;
      }

      std::vector<int> current_gt(det_cam.size(), -1);
      std::vector<double> current_pair_errors(det_cam.size(), 0.0);
      double total_error_m = 0.0;
      bool valid = true;
      for (std::size_t det_slot = 0; det_slot < det_cam.size(); ++det_slot)
      {
        const int gt_index = gt_order[det_slot];
        const double err = DistanceMeters(det_cam[det_slot], truth_frame.gt_cam[gt_index]);
        current_gt[det_slot] = gt_index;
        current_pair_errors[det_slot] = err;
        total_error_m += err;
        if (best.valid && total_error_m >= best.total_error_m)
        {
          valid = false;
          break;
        }
      }
      if (!valid)
      {
        continue;
      }

      best.valid = true;
      best.total_error_m = total_error_m;
      best.gt_index_for_det = current_gt;
      best.pair_error_m = current_pair_errors;
    } while (std::next_permutation(gt_order.begin(), gt_order.end()));

    return best;
  }

  static std::string AssignmentString(const AssignmentResult& assignment)
  {
    if (!assignment.valid)
    {
      return "invalid";
    }
    std::ostringstream ss;
    for (std::size_t i = 0; i < assignment.det_indices.size(); ++i)
    {
      if (i > 0)
      {
        ss << ',';
      }
      ss << "D" << assignment.det_indices[i] << "->"
         << kTruthLabels[assignment.gt_index_for_det[i]];
    }
    return ss.str();
  }

  void OpenFilesLocked()
  {
    if (!pairs_file_.is_open())
    {
      pairs_file_.open(pairs_path_, std::ios::out | std::ios::trunc);
      if (pairs_file_)
      {
        pairs_file_
            << "frame\timage_ts_us\tsim_time_s\tdet_index\tgt_label\terr_m\tgt_visible\t"
               "number\ttype\tconfidence\t"
               "fixed_ok\tfixed_method\tfixed_err_m\tfixed_reproj_rmse_px\t"
               "best_detector_perm_ok\tbest_detector_perm_order\t"
               "best_detector_perm_err_m\t"
               "truth_corner_pnp_ok\ttruth_corner_pnp_order\ttruth_corner_pnp_err_m\t"
               "truth_corner_pnp_reproj_rmse_px\t"
               "truth_corner_pnp56_ok\ttruth_corner_pnp56_order\t"
               "truth_corner_pnp56_err_m\ttruth_corner_pnp56_reproj_rmse_px\t"
               "current_vs_fixed_m\tcorner_mean_err_px\tcorner_max_err_px\t"
               "det_p0_x\tdet_p0_y\tgt_p0_x\tgt_p0_y\tdelta_p0_x\tdelta_p0_y\t"
               "det_p1_x\tdet_p1_y\tgt_p1_x\tgt_p1_y\tdelta_p1_x\tdelta_p1_y\t"
               "det_p2_x\tdet_p2_y\tgt_p2_x\tgt_p2_y\tdelta_p2_x\tdelta_p2_y\t"
               "det_p3_x\tdet_p3_y\tgt_p3_x\tgt_p3_y\tdelta_p3_x\tdelta_p3_y\t"
               "det_cam_x\tdet_cam_y\tdet_cam_z\t"
               "gt_cam_x\tgt_cam_y\tgt_cam_z\t"
               "det_world_x\tdet_world_y\tdet_world_z\t"
               "fixed_cam_x\tfixed_cam_y\tfixed_cam_z\t"
               "fixed_world_x\tfixed_world_y\tfixed_world_z\t"
               "gt_world_x\tgt_world_y\tgt_world_z\n";
      }
    }

    if (!frames_file_.is_open())
    {
      frames_file_.open(frames_path_, std::ios::out | std::ios::trunc);
      if (frames_file_)
      {
        frames_file_
            << "frame\timage_ts_us\tsim_time_s\tdet_count\tvisible_gt_count\t"
               "mean_err_m\tmax_err_m\tcenter_err_m\tshape_mean_err_m\t"
               "corner_mean_err_px\tcorner_max_err_px\tassignment\n";
      }
    }
  }

  void CloseFilesLocked()
  {
    if (pairs_file_.is_open())
    {
      pairs_file_.flush();
      pairs_file_.close();
    }
    if (frames_file_.is_open())
    {
      frames_file_.flush();
      frames_file_.close();
    }
  }

  static double MeanOf(const std::vector<double>& values)
  {
    if (values.empty())
    {
      return 0.0;
    }
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
  }

  static double Percentile(std::vector<double> values, double p)
  {
    if (values.empty())
    {
      return 0.0;
    }
    const double clamped = std::clamp(p, 0.0, 1.0);
    const std::size_t index = static_cast<std::size_t>(
        std::llround(clamped * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(index),
                     values.end());
    return values[index];
  }

  void WriteSummaryLocked(const char* status)
  {
    if (summary_written_ && done_.load(std::memory_order_relaxed))
    {
      return;
    }

    std::ofstream summary(summary_path_, std::ios::out | std::ios::trunc);
    summary << "status=" << status << '\n';
    summary << "pairs=" << pairs_path_ << '\n';
    summary << "frames=" << frames_path_ << '\n';
    summary << "frame_count=" << frame_count_ << '\n';
    summary << "max_frames=" << max_frames_ << '\n';
    summary << "skipped_no_supervisor=" << skipped_no_supervisor_ << '\n';
    summary << "skipped_no_nodes=" << skipped_no_nodes_ << '\n';
    summary << "skipped_no_truth_frame=" << skipped_no_truth_frame_ << '\n';
    summary << "skipped_bad_truth=" << skipped_bad_truth_ << '\n';
    summary << "empty_detection_frames=" << empty_detection_frames_ << '\n';
    summary << std::fixed << std::setprecision(6);
    summary << "frame_mean_err_mean=" << MeanOf(frame_mean_errors_) << '\n';
    summary << "frame_mean_err_p50=" << Percentile(frame_mean_errors_, 0.50) << '\n';
    summary << "frame_mean_err_p95=" << Percentile(frame_mean_errors_, 0.95) << '\n';
    summary << "frame_mean_err_max="
            << (frame_mean_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_mean_errors_.begin(),
                                        frame_mean_errors_.end()))
            << '\n';
    summary << "frame_center_err_mean=" << MeanOf(frame_center_errors_) << '\n';
    summary << "frame_center_err_p50=" << Percentile(frame_center_errors_, 0.50) << '\n';
    summary << "frame_center_err_p95=" << Percentile(frame_center_errors_, 0.95) << '\n';
    summary << "frame_center_err_max="
            << (frame_center_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_center_errors_.begin(),
                                        frame_center_errors_.end()))
            << '\n';
    summary << "frame_shape_err_mean=" << MeanOf(frame_shape_mean_errors_) << '\n';
    summary << "frame_shape_err_p50=" << Percentile(frame_shape_mean_errors_, 0.50) << '\n';
    summary << "frame_shape_err_p95=" << Percentile(frame_shape_mean_errors_, 0.95) << '\n';
    summary << "frame_shape_err_max="
            << (frame_shape_mean_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_shape_mean_errors_.begin(),
                                        frame_shape_mean_errors_.end()))
            << '\n';
    summary << "pair_err_mean=" << MeanOf(pair_errors_) << '\n';
    summary << "pair_err_p50=" << Percentile(pair_errors_, 0.50) << '\n';
    summary << "pair_err_p95=" << Percentile(pair_errors_, 0.95) << '\n';
    summary << "pair_err_max="
            << (pair_errors_.empty()
                    ? 0.0
                    : *std::max_element(pair_errors_.begin(), pair_errors_.end()))
            << '\n';
    summary << "pair_err_gt_0.05=" << pair_err_gt_5cm_ << '\n';
    summary << "pair_err_gt_0.10=" << pair_err_gt_10cm_ << '\n';
    summary << "pair_err_gt_0.20=" << pair_err_gt_20cm_ << '\n';
    summary << "fixed_pair_count=" << fixed_pair_errors_.size() << '\n';
    summary << "fixed_missing_count=" << fixed_missing_count_ << '\n';
    summary << "fixed_better_count=" << fixed_better_count_ << '\n';
    summary << "fixed_worse_count=" << fixed_worse_count_ << '\n';
    summary << "fixed_equal_count=" << fixed_equal_count_ << '\n';
    summary << "fixed_pair_err_mean=" << MeanOf(fixed_pair_errors_) << '\n';
    summary << "fixed_pair_err_p50=" << Percentile(fixed_pair_errors_, 0.50) << '\n';
    summary << "fixed_pair_err_p95=" << Percentile(fixed_pair_errors_, 0.95) << '\n';
    summary << "fixed_pair_err_max="
            << (fixed_pair_errors_.empty()
                    ? 0.0
                    : *std::max_element(fixed_pair_errors_.begin(),
                                        fixed_pair_errors_.end()))
            << '\n';
    summary << "current_minus_fixed_err_mean="
            << MeanOf(fixed_current_delta_errors_) << '\n';
    summary << "current_minus_fixed_err_p50="
            << Percentile(fixed_current_delta_errors_, 0.50) << '\n';
    summary << "current_minus_fixed_err_p95="
            << Percentile(fixed_current_delta_errors_, 0.95) << '\n';
    summary << "best_detector_perm_pair_count=" << best_detector_perm_errors_.size()
            << '\n';
    summary << "best_detector_perm_missing_count=" << best_detector_perm_missing_count_
            << '\n';
    summary << "best_detector_perm_err_mean=" << MeanOf(best_detector_perm_errors_)
            << '\n';
    summary << "best_detector_perm_err_p50="
            << Percentile(best_detector_perm_errors_, 0.50) << '\n';
    summary << "best_detector_perm_err_p95="
            << Percentile(best_detector_perm_errors_, 0.95) << '\n';
    summary << "best_detector_perm_err_max="
            << (best_detector_perm_errors_.empty()
                    ? 0.0
                    : *std::max_element(best_detector_perm_errors_.begin(),
                                        best_detector_perm_errors_.end()))
            << '\n';
    summary << "truth_corner_pnp_pair_count=" << truth_corner_pnp_errors_.size() << '\n';
    summary << "truth_corner_pnp_missing_count=" << truth_corner_pnp_missing_count_ << '\n';
    summary << "truth_corner_pnp_err_mean=" << MeanOf(truth_corner_pnp_errors_) << '\n';
    summary << "truth_corner_pnp_err_p50="
            << Percentile(truth_corner_pnp_errors_, 0.50) << '\n';
    summary << "truth_corner_pnp_err_p95="
            << Percentile(truth_corner_pnp_errors_, 0.95) << '\n';
    summary << "truth_corner_pnp_err_max="
            << (truth_corner_pnp_errors_.empty()
                    ? 0.0
                    : *std::max_element(truth_corner_pnp_errors_.begin(),
                                        truth_corner_pnp_errors_.end()))
            << '\n';
    summary << "truth_corner_pnp56_pair_count=" << truth_corner_pnp56_errors_.size()
            << '\n';
    summary << "truth_corner_pnp56_missing_count=" << truth_corner_pnp56_missing_count_
            << '\n';
    summary << "truth_corner_pnp56_err_mean=" << MeanOf(truth_corner_pnp56_errors_)
            << '\n';
    summary << "truth_corner_pnp56_err_p50="
            << Percentile(truth_corner_pnp56_errors_, 0.50) << '\n';
    summary << "truth_corner_pnp56_err_p95="
            << Percentile(truth_corner_pnp56_errors_, 0.95) << '\n';
    summary << "truth_corner_pnp56_err_max="
            << (truth_corner_pnp56_errors_.empty()
                    ? 0.0
                    : *std::max_element(truth_corner_pnp56_errors_.begin(),
                                        truth_corner_pnp56_errors_.end()))
            << '\n';
    summary << "corner_mean_err_px_mean=" << MeanOf(corner_mean_errors_px_) << '\n';
    summary << "corner_mean_err_px_p50="
            << Percentile(corner_mean_errors_px_, 0.50) << '\n';
    summary << "corner_mean_err_px_p95="
            << Percentile(corner_mean_errors_px_, 0.95) << '\n';
    summary << "corner_mean_err_px_max="
            << (corner_mean_errors_px_.empty()
                    ? 0.0
                    : *std::max_element(corner_mean_errors_px_.begin(),
                                        corner_mean_errors_px_.end()))
            << '\n';
    summary << "corner_max_err_px_mean=" << MeanOf(corner_max_errors_px_) << '\n';
    summary << "corner_max_err_px_p50="
            << Percentile(corner_max_errors_px_, 0.50) << '\n';
    summary << "corner_max_err_px_p95="
            << Percentile(corner_max_errors_px_, 0.95) << '\n';
    summary << "corner_max_err_px_max="
            << (corner_max_errors_px_.empty()
                    ? 0.0
                    : *std::max_element(corner_max_errors_px_.begin(),
                                        corner_max_errors_px_.end()))
            << '\n';
    summary_written_ = true;
  }

  webots::Supervisor* supervisor_{nullptr};
  webots::Node* target_spin_node_{nullptr};
  webots::Node* camera_node_{nullptr};
  webots::Node* target_robot_node_{nullptr};
  webots::Node* self_robot_node_{nullptr};
  std::array<webots::Node*, 4> armor_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> visible_face_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> left_lightbar_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> right_lightbar_nodes_{{nullptr, nullptr, nullptr, nullptr}};

  bool target_spin_pose_logged_{false};
  std::array<bool, 4> armor_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> visible_face_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> left_lightbar_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> right_lightbar_pose_logged_{{false, false, false, false}};

  std::string pairs_path_{};
  std::string frames_path_{};
  std::string summary_path_{};

  uint32_t max_frames_{400};
  std::atomic<bool> done_{false};
  bool summary_written_{false};

  std::mutex state_lock_{};
  std::deque<TruthFrameSnapshot> truth_frame_cache_{};

  std::mutex file_lock_{};
  std::ofstream pairs_file_{};
  std::ofstream frames_file_{};

  uint32_t frame_count_{0};
  uint32_t skipped_no_supervisor_{0};
  uint32_t skipped_no_nodes_{0};
  uint32_t skipped_no_truth_frame_{0};
  uint32_t skipped_bad_truth_{0};
  uint32_t empty_detection_frames_{0};
  uint32_t pair_err_gt_5cm_{0};
  uint32_t pair_err_gt_10cm_{0};
  uint32_t pair_err_gt_20cm_{0};
  uint32_t fixed_missing_count_{0};
  uint32_t fixed_better_count_{0};
  uint32_t fixed_worse_count_{0};
  uint32_t fixed_equal_count_{0};
  uint32_t best_detector_perm_missing_count_{0};
  uint32_t truth_corner_pnp_missing_count_{0};
  uint32_t truth_corner_pnp56_missing_count_{0};

  std::vector<double> frame_mean_errors_{};
  std::vector<double> frame_center_errors_{};
  std::vector<double> frame_shape_mean_errors_{};
  std::vector<double> pair_errors_{};
  std::vector<double> fixed_pair_errors_{};
  std::vector<double> fixed_current_delta_errors_{};
  std::vector<double> best_detector_perm_errors_{};
  std::vector<double> truth_corner_pnp_errors_{};
  std::vector<double> truth_corner_pnp56_errors_{};
  std::vector<double> corner_mean_errors_px_{};
  std::vector<double> corner_max_errors_px_{};
};
