#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraFrameSync.hpp"
#include "armor.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "webots_truth_visible_plane.hpp"
#include "xrobot_constexpr.hpp"

class TruthArmorsPublisher
{
 public:
  using CameraInfo = CameraTypes::CameraInfo;
  using MainFrameSync = CameraFrameSync<ProjectConstexpr::MainCameraInfo>;
  using ImageFrame = MainFrameSync::ImageFrame;
  using SyncedFrame = MainFrameSync::SyncedFrame;
  static constexpr uint32_t kSyncFrameWaitTimeoutMs = 100;

  TruthArmorsPublisher()
      : topic_name_(ResolveTopicName()),
        armors_topic_(topic_name_.c_str(), sizeof(ArmorDetectionsMessage), &armor_domain_)
  {
  }

  void Init(webots::Supervisor* supervisor) { supervisor_ = supervisor; }

  void InstallBlocking()
  {
    XR_LOG_PASS("TruthArmorsPublisher subscribed: image=%s imu=%s -> armor_detector/%s",
                ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName,
                topic_name_.c_str());

    while (true)
    {
      MainFrameSync::Subscriber subscriber(ProjectConstexpr::MainImageTopicName,
                                           ProjectConstexpr::MainImuTopicName);
      if (!subscriber.Valid())
      {
        LibXR::Thread::Sleep(200);
        continue;
      }

      SyncedFrame synced_frame;
      while (true)
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

        const ImageFrame* frame = synced_frame.GetImageFrame();
        if (frame != nullptr)
        {
          SyncFrameCallback(*frame);
        }
      }
    }
  }

 private:
  struct Pose3d
  {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  };

  static constexpr std::array<const char*, 4> kTruthLabels = {
      "armor_front", "armor_right", "armor_back", "armor_left"};
  static constexpr const char* kTruthVisibleFaceProtoDef = "XR_VISIBLE_FACE_POSE";
  static constexpr const char* kTruthLeftLightbarProtoDef = "XR_LEFT_LIGHTBAR_POSE";
  static constexpr const char* kTruthRightLightbarProtoDef = "XR_RIGHT_LIGHTBAR_POSE";

  static constexpr std::array<std::array<double, 3>, 4> kArmorLocalPosSpin = {
      std::array<double, 3>{0.0, 0.205, -0.06},
      std::array<double, 3>{0.205, 0.0, -0.11},
      std::array<double, 3>{0.0, -0.205, -0.06},
      std::array<double, 3>{-0.205, 0.0, -0.11}};

  static const char* ResolveTopicName()
  {
    const char* env = std::getenv("XR_TRUTH_ARMORS_TOPIC_NAME");
    return (env != nullptr && env[0] != '\0') ? env : "truth_armors_result";
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

  static bool ReadRelativePose(webots::Node* node, const webots::Node* from_node,
                               Pose3d& pose, bool& logged, const char* label)
  {
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
    r_row << m[0], m[1], m[2],
             m[4], m[5], m[6],
             m[8], m[9], m[10];

    Eigen::Matrix3d r_col = Eigen::Matrix3d::Identity();
    r_col << m[0], m[4], m[8],
             m[1], m[5], m[9],
             m[2], m[6], m[10];

    const bool use_col_major = t_col.norm() > t_row.norm();
    pose.translation = use_col_major ? t_col : t_row;
    pose.rotation = use_col_major ? r_col : r_row;

    if (!logged)
    {
      XR_LOG_PASS(
          "TruthArmorsPublisher %s row_t=(%.3f, %.3f, %.3f) col_t=(%.3f, %.3f, %.3f) choose=%s",
          label, t_row.x(), t_row.y(), t_row.z(), t_col.x(), t_col.y(), t_col.z(),
          use_col_major ? "col" : "row");
      logged = true;
    }
    return true;
  }

  static std::array<cv::Point2f, 4> SortArmorPoints(std::array<cv::Point2f, 4> points)
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

  static std::array<cv::Point3f, 4> SmallArmorObjectPoints()
  {
    constexpr float kHalfY = 135.0f * 0.5f / 1000.0f;
    constexpr float kHalfZ = 55.0f * 0.5f / 1000.0f;
    return {cv::Point3f(0.0f, kHalfY, kHalfZ), cv::Point3f(0.0f, -kHalfY, kHalfZ),
            cv::Point3f(0.0f, -kHalfY, -kHalfZ),
            cv::Point3f(0.0f, kHalfY, -kHalfZ)};
  }

  static Eigen::Matrix3d FaceRotationSpin(int face_index)
  {
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitY();
    switch (face_index)
    {
      case 0:
        x_axis = Eigen::Vector3d::UnitY();
        break;
      case 1:
        x_axis = Eigen::Vector3d::UnitX();
        break;
      case 2:
        x_axis = -Eigen::Vector3d::UnitY();
        break;
      case 3:
        x_axis = -Eigen::Vector3d::UnitX();
        break;
      default:
        break;
    }

    const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d y_axis = (z_axis.cross(x_axis)).normalized();

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    return rotation;
  }

  static bool FinitePoint(const cv::Point2f& point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y);
  }

  static webots::Node* FindNamedNodeRecursiveImpl(webots::Node* node,
                                                  const char* wanted_name,
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

  bool ResolveNodes()
  {
    bool all_armors_resolved = true;
    for (auto* node : armor_nodes_)
    {
      all_armors_resolved = all_armors_resolved && (node != nullptr);
    }
    if (target_spin_node_ != nullptr && camera_node_ != nullptr && all_armors_resolved)
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

    webots::Node* root = supervisor_->getRoot();
    if (root == nullptr)
    {
      return false;
    }
    for (std::size_t i = 0; i < armor_nodes_.size(); ++i)
    {
      if (armor_nodes_[i] == nullptr)
      {
        armor_nodes_[i] = FindNamedNodeRecursive(root, kTruthLabels[i]);
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

    XR_LOG_PASS("TruthArmorsPublisher resolved target/camera/armor nodes");
    return true;
  }

  bool BuildTruthMessage(const CameraInfo& camera_info,
                         ArmorDetectionsMessage& msg)
  {
    if (!ResolveNodes())
    {
      return false;
    }
    if (camera_info.width == 0 || camera_info.height == 0)
    {
      return false;
    }

    const cv::Point2f image_center(static_cast<float>(camera_info.width) * 0.5f,
                                   static_cast<float>(camera_info.height) * 0.5f);

    msg.image_timestamp_us = latest_image_timestamp_us_;
    msg.results.clear();
    msg.results.reserve(4);

    for (int face_index = 0; face_index < 4; ++face_index)
    {
      Pose3d armor_in_camera_node;
      if (!ReadRelativePose(armor_nodes_[face_index], camera_node_, armor_in_camera_node,
                            armor_pose_logged_[face_index], kTruthLabels[face_index]))
      {
        continue;
      }

      WebotsTruthVisibleFace face;
      if (left_lightbar_nodes_[face_index] != nullptr &&
          right_lightbar_nodes_[face_index] != nullptr)
      {
        Pose3d left_lightbar_in_camera_node;
        Pose3d right_lightbar_in_camera_node;
        if (ReadRelativePose(left_lightbar_nodes_[face_index], camera_node_,
                             left_lightbar_in_camera_node,
                             left_lightbar_pose_logged_[face_index],
                             kTruthLeftLightbarProtoDef) &&
            ReadRelativePose(right_lightbar_nodes_[face_index], camera_node_,
                             right_lightbar_in_camera_node,
                             right_lightbar_pose_logged_[face_index],
                             kTruthRightLightbarProtoDef))
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
                  face))
          {
            goto face_ready;
          }
        }
      }

      if (visible_face_nodes_[face_index] != nullptr)
      {
        Pose3d visible_face_in_camera_node;
        if (ReadRelativePose(visible_face_nodes_[face_index], camera_node_,
                             visible_face_in_camera_node,
                             visible_face_pose_logged_[face_index],
                             kTruthVisibleFaceProtoDef))
        {
          const Eigen::Matrix3d r_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.rotation;
          const Eigen::Vector3d t_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.translation;
          if (ProjectWebotsTruthFaceBox(camera_info, r_optical_box, t_optical_box,
                                        kWebotsTruthSpArmorWidth,
                                        kWebotsTruthSpArmorHeight,
                                        kWebotsTruthSpArmorDepth, face))
          {
            goto face_ready;
          }
        }
      }

      {
        const Eigen::Matrix3d r_optical_armor_root =
            WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.rotation;
        const Eigen::Vector3d t_optical_armor_root =
            WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.translation;
        if (!ProjectWebotsTruthVisibleFace(camera_info, r_optical_armor_root,
                                           t_optical_armor_root, face))
        {
          continue;
        }
      }

face_ready:

      ArmorDetectorResult result;
      result.color = ArmorColor::BLUE;
      result.number = ArmorNumber::THREE;
      result.type = ArmorType::SMALL;
      result.priority = GetArmorPriority(result.number);
      result.confidence = 1.0f;
      result.box = face.box;
      result.points = face.points;
      result.center = face.center;
      result.center_norm = {
          result.center.x / static_cast<float>(std::max<uint32_t>(1U, camera_info.width)),
          result.center.y / static_cast<float>(std::max<uint32_t>(1U, camera_info.height))};
      result.distance_to_image_center = cv::norm(result.center - image_center);
      result.pose = LibXR::Transform<double>(
          LibXR::Quaternion<double>(face.rotation_optical),
          LibXR::Position<double>(face.center_optical.x(), face.center_optical.y(),
                                  face.center_optical.z()));
      msg.results.emplace_back(std::move(result));
    }

    return true;
  }

  void SyncFrameCallback(const ImageFrame& frame)
  {
    if (supervisor_ == nullptr)
    {
      return;
    }
    latest_image_timestamp_us_ = frame.timestamp_us;

    ArmorDetectionsMessage msg;
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      if (!BuildTruthMessage(ProjectConstexpr::MainCameraInfo, msg))
      {
        return;
      }
    }

    armors_topic_.Publish(msg);
    publish_count_++;
    if ((publish_count_ % 100U) == 0U || publish_count_ == 1U)
    {
      XR_LOG_PASS("TruthArmorsPublisher frames=%u published=%zu topic=%s ts=%llu",
                  publish_count_, msg.results.size(), topic_name_.c_str(),
                  static_cast<unsigned long long>(msg.image_timestamp_us));
    }
  }

 private:
  webots::Supervisor* supervisor_{nullptr};
  webots::Node* target_spin_node_{nullptr};
  webots::Node* camera_node_{nullptr};
  std::array<webots::Node*, 4> armor_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> visible_face_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> left_lightbar_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> right_lightbar_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  bool target_spin_pose_logged_{false};
  std::array<bool, 4> armor_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> visible_face_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> left_lightbar_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> right_lightbar_pose_logged_{{false, false, false, false}};
  std::mutex state_lock_{};
  std::string topic_name_;
  LibXR::Topic::Domain armor_domain_ = LibXR::Topic::Domain("armor_detector");
  LibXR::Topic armors_topic_;
  uint64_t latest_image_timestamp_us_{0};
  uint32_t publish_count_{0};
};
