#pragma once

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>

#include "armor.hpp"
#include "libxr.hpp"
#include "logger.hpp"

class DetectorPoseAudit
{
 public:
  DetectorPoseAudit()
  {
    const char* path_env = std::getenv("XR_DETECTOR_POSE_AUDIT_PATH");
    if (path_env != nullptr && path_env[0] != '\0')
    {
      path_ = path_env;
    }
    else
    {
      path_ = "detector_pose_audit.tsv";
    }

    const char* max_frames_env = std::getenv("XR_DETECTOR_POSE_AUDIT_MAX_FRAMES");
    if (max_frames_env != nullptr && max_frames_env[0] != '\0')
    {
      char* end = nullptr;
      const unsigned long parsed = std::strtoul(max_frames_env, &end, 10);
      if (end != max_frames_env && parsed > 0UL)
      {
        max_frames_ = static_cast<uint32_t>(parsed);
      }
    }
  }

  void InstallBlocking()
  {
    LibXR::Topic::Domain armor_domain("armor_detector");
    auto armors_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("armors_result", UINT32_MAX, &armor_domain));
    auto armors_cb = LibXR::Topic::Callback::Create(
        [](bool, DetectorPoseAudit* self, LibXR::RawData& data)
        {
          auto* msg = reinterpret_cast<ArmorDetectionsMessage*>(data.addr_);
          self->ArmorsCallback(msg);
        },
        this);
    armors_topic.RegisterCallback(armors_cb);

    XR_LOG_PASS("DetectorPoseAudit subscribed: armor_detector/armors_result -> %s",
                path_.c_str());

    while (!done_.load(std::memory_order_relaxed))
    {
      LibXR::Thread::Sleep(100);
    }
  }

  bool Done() const { return done_.load(std::memory_order_relaxed); }

 private:
  void EnsureOpenLocked()
  {
    if (file_.is_open())
    {
      return;
    }

    file_.open(path_, std::ios::out | std::ios::trunc);
    if (!file_)
    {
      XR_LOG_ERROR("DetectorPoseAudit failed to open %s", path_.c_str());
      done_.store(true, std::memory_order_relaxed);
      return;
    }

    file_ << "frame_index\timage_timestamp_us\tdetection_index\tnumber\ttype\tcolor\t"
             "confidence\tcenter_x\tcenter_y\tdistance_to_image_center\t"
             "pose_tx\tpose_ty\tpose_tz\tqw\tqx\tqy\tqz\n";
  }

  void ArmorsCallback(ArmorDetectionsMessage* msg)
  {
    if (msg == nullptr)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(file_lock_);
    EnsureOpenLocked();
    if (!file_)
    {
      return;
    }

    ++frame_count_;
    for (std::size_t detection_index = 0; detection_index < msg->results.size();
         ++detection_index)
    {
      const auto& armor = msg->results[detection_index];
      file_ << frame_count_ << '\t' << msg->image_timestamp_us << '\t'
            << detection_index << '\t' << static_cast<int>(armor.number) << '\t'
            << static_cast<int>(armor.type) << '\t' << static_cast<int>(armor.color) << '\t'
            << armor.confidence << '\t' << armor.center.x << '\t' << armor.center.y << '\t'
            << armor.distance_to_image_center << '\t'
            << armor.pose.translation.x() << '\t' << armor.pose.translation.y() << '\t'
            << armor.pose.translation.z() << '\t' << armor.pose.rotation.w() << '\t'
            << armor.pose.rotation.x() << '\t' << armor.pose.rotation.y() << '\t'
            << armor.pose.rotation.z() << '\n';
    }
    file_.flush();

    if (max_frames_ > 0 && frame_count_ >= max_frames_)
    {
      done_.store(true, std::memory_order_relaxed);
    }
  }

  std::string path_{};
  std::ofstream file_{};
  std::mutex file_lock_{};
  std::atomic<bool> done_{false};
  uint32_t max_frames_{0};
  uint32_t frame_count_{0};
};
