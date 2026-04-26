#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <webots/Supervisor.hpp>

#include "CameraFrameSync.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_pipe.hpp"
#include "libxr_rw.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "terminal.hpp"
#include "thread.hpp"
#include "uart.hpp"
#include "xrobot_main.hpp"
#include "detector_pose_audit.hpp"
#include "detector_truth_compare.hpp"
#include "tracker_truth_compare.hpp"
#include "detector_video_recorder.hpp"
#include "tracker_video_recorder.hpp"
#include "truth_armors_publisher.hpp"

void (*log_cb_fun)(bool in_isr, LibXR::Topic, LibXR::RawData &log_data) =
    [](bool, LibXR::Topic tp, LibXR::RawData &log_data)
{
  UNUSED(tp);

  auto log = reinterpret_cast<LibXR::LogData *>(log_data.addr_);

  if (LibXR::STDIO::write_ && LibXR::STDIO::write_->Writable())
  {
    using clock = std::chrono::system_clock;

    static std::ofstream f;
    if (!f.is_open())
    {
      auto now = clock::now();
      std::time_t t = clock::to_time_t(now);
      std::tm tm{};
      localtime_r(&t, &tm);

      std::ostringstream oss;
      // 首次打开时按启动时间命名：YYYYMMDD_HHMMSS.log
      oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".log";
      f.open(oss.str(), std::ios::out | std::ios::app);

      LibXR::STDIO::Printf("Log written to %s\n", oss.str().c_str());
    }

    if (f)
    {
      auto now = clock::now();
      auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
          1000;
      std::time_t t2 = clock::to_time_t(now);
      std::tm tm2{};
      localtime_r(&t2, &tm2);
      char ts[32];
      (void)std::strftime(ts, sizeof(ts), "%F %T", &tm2);  // "YYYY-MM-DD HH:MM:SS"

      f << '[' << ts << '.' << std::setw(3) << std::setfill('0') << ms.count()
        << std::setfill(' ') << "][" << static_cast<unsigned>(log->level) << "] "
        << (log->file ? log->file : "?") << ':' << log->line << ' ' << log->message
        << '\n';
      f.flush();
    }
  }
};

namespace
{
using MainFrameSync = CameraFrameSync<ProjectConstexpr::MainCameraInfo>;

class RuntimeFreqProbe
{
 public:
  static constexpr uint32_t kSyncFrameWaitTimeoutMs = 100;

  void InstallBlocking()
  {
    LibXR::Topic::Domain gimbal_domain("gimbal");
    LibXR::Topic::Domain tracker_domain("tracker");

    RegisterCounter("gimbal/rotation",
                    LibXR::Topic::WaitTopic("rotation", UINT32_MAX, &gimbal_domain),
                    rotation_count_);
    RegisterCounter("tracker/target",
                    LibXR::Topic::WaitTopic("target", UINT32_MAX, &tracker_domain),
                    tracker_target_count_);
    RegisterCounter("tracker/target_eulr",
                    LibXR::Topic::WaitTopic("target_eulr", UINT32_MAX, &tracker_domain),
                    target_eulr_count_);
    RegisterCounter("tracker/send",
                    LibXR::Topic::WaitTopic("send", UINT32_MAX, &tracker_domain),
                    send_count_);
    RegisterCounter("mcu/target_eulr",
                    LibXR::Topic::WaitTopic("target_eulr", UINT32_MAX),
                    mcu_target_eulr_count_);
    RegisterCounter("mcu/fire_notify",
                    LibXR::Topic::WaitTopic("fire_notify", UINT32_MAX),
                    mcu_fire_notify_count_);

    sync_frame_thread_.Create(this, SyncFrameCounterThreadFun, "freq_probe_sync",
                              static_cast<size_t>(1024 * 64),
                              LibXR::Thread::Priority::LOW);

    installed_ = true;
    XR_LOG_PASS("Runtime frequency probe enabled");
  }

  void Report()
  {
    if (!installed_)
    {
      return;
    }

    const auto now = LibXR::Timebase::GetMilliseconds();
    if (!started_)
    {
      started_ = true;
      last_report_ms_ = now;
      Snapshot(last_sync_frame_count_, sync_frame_count_);
      Snapshot(last_rotation_count_, rotation_count_);
      Snapshot(last_tracker_target_count_, tracker_target_count_);
      Snapshot(last_target_eulr_count_, target_eulr_count_);
      Snapshot(last_send_count_, send_count_);
      Snapshot(last_mcu_target_eulr_count_, mcu_target_eulr_count_);
      Snapshot(last_mcu_fire_notify_count_, mcu_fire_notify_count_);
      XR_LOG_PASS("FreqProbe armed at sim_t_ms=%u", static_cast<unsigned>(now));
      return;
    }

    const uint32_t dt_ms = (now - last_report_ms_).ToMillisecond();
    if (dt_ms == 0)
    {
      return;
    }

    const uint64_t sync_frame_now = sync_frame_count_.load(std::memory_order_relaxed);
    const uint64_t rotation_now = rotation_count_.load(std::memory_order_relaxed);
    const uint64_t tracker_target_now =
        tracker_target_count_.load(std::memory_order_relaxed);
    const uint64_t target_eulr_now = target_eulr_count_.load(std::memory_order_relaxed);
    const uint64_t send_now = send_count_.load(std::memory_order_relaxed);
    const uint64_t mcu_target_eulr_now =
        mcu_target_eulr_count_.load(std::memory_order_relaxed);
    const uint64_t mcu_fire_notify_now =
        mcu_fire_notify_count_.load(std::memory_order_relaxed);

    const uint64_t sync_frame_delta = sync_frame_now - last_sync_frame_count_;
    const uint64_t rotation_delta = rotation_now - last_rotation_count_;
    const uint64_t tracker_target_delta = tracker_target_now - last_tracker_target_count_;
    const uint64_t target_eulr_delta = target_eulr_now - last_target_eulr_count_;
    const uint64_t send_delta = send_now - last_send_count_;
    const uint64_t mcu_target_eulr_delta =
        mcu_target_eulr_now - last_mcu_target_eulr_count_;
    const uint64_t mcu_fire_notify_delta =
        mcu_fire_notify_now - last_mcu_fire_notify_count_;

    XR_LOG_PASS(
        "FreqProbe sim_t_ms=%u dt_ms=%u sync_frame=%llu(%.1fHz) rotation=%llu(%.1fHz) tracker_target=%llu(%.1fHz) target_eulr=%llu(%.1fHz) send=%llu(%.1fHz) mcu_target_eulr=%llu(%.1fHz) mcu_fire_notify=%llu(%.1fHz)",
        static_cast<unsigned>(now), dt_ms,
        static_cast<unsigned long long>(sync_frame_delta), Hertz(sync_frame_delta, dt_ms),
        static_cast<unsigned long long>(rotation_delta), Hertz(rotation_delta, dt_ms),
        static_cast<unsigned long long>(tracker_target_delta),
        Hertz(tracker_target_delta, dt_ms),
        static_cast<unsigned long long>(target_eulr_delta), Hertz(target_eulr_delta, dt_ms),
        static_cast<unsigned long long>(send_delta), Hertz(send_delta, dt_ms),
        static_cast<unsigned long long>(mcu_target_eulr_delta),
        Hertz(mcu_target_eulr_delta, dt_ms),
        static_cast<unsigned long long>(mcu_fire_notify_delta),
        Hertz(mcu_fire_notify_delta, dt_ms));

    last_report_ms_ = now;
    last_sync_frame_count_ = sync_frame_now;
    last_rotation_count_ = rotation_now;
    last_tracker_target_count_ = tracker_target_now;
    last_target_eulr_count_ = target_eulr_now;
    last_send_count_ = send_now;
    last_mcu_target_eulr_count_ = mcu_target_eulr_now;
    last_mcu_fire_notify_count_ = mcu_fire_notify_now;
  }

 private:
  static double Hertz(uint64_t count, uint32_t dt_ms)
  {
    return dt_ms > 0 ? static_cast<double>(count) * 1000.0 / static_cast<double>(dt_ms)
                     : 0.0;
  }

  static void Snapshot(uint64_t &dst, const std::atomic<uint64_t> &src)
  {
    dst = src.load(std::memory_order_relaxed);
  }

  static void RegisterCounter(const char *label, LibXR::Topic::TopicHandle handle,
                              std::atomic<uint64_t> &counter)
  {
    LibXR::Topic topic(handle);
    auto cb = LibXR::Topic::Callback::Create(
        [](bool, std::atomic<uint64_t> *self_counter, LibXR::RawData &)
        { self_counter->fetch_add(1, std::memory_order_relaxed); },
        &counter);
    topic.RegisterCallback(cb);
    XR_LOG_PASS("FreqProbe subscribed: %s", label);
  }

  static void SyncFrameCounterThreadFun(RuntimeFreqProbe *self)
  {
    XR_LOG_PASS("FreqProbe subscribed: image=%s imu=%s", ProjectConstexpr::MainImageTopicName,
                ProjectConstexpr::MainImuTopicName);

    while (true)
    {
      MainFrameSync::Subscriber subscriber(ProjectConstexpr::MainImageTopicName,
                                           ProjectConstexpr::MainImuTopicName);
      if (!subscriber.Valid())
      {
        LibXR::Thread::Sleep(200);
        continue;
      }

      MainFrameSync::SyncedFrame synced_frame;
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

        self->sync_frame_count_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

 private:
  bool installed_{false};
  bool started_{false};
  LibXR::MillisecondTimestamp last_report_ms_{};
  std::atomic<uint64_t> sync_frame_count_{0};
  std::atomic<uint64_t> rotation_count_{0};
  std::atomic<uint64_t> tracker_target_count_{0};
  std::atomic<uint64_t> target_eulr_count_{0};
  std::atomic<uint64_t> send_count_{0};
  std::atomic<uint64_t> mcu_target_eulr_count_{0};
  std::atomic<uint64_t> mcu_fire_notify_count_{0};
  LibXR::Thread sync_frame_thread_{};
  uint64_t last_sync_frame_count_{0};
  uint64_t last_rotation_count_{0};
  uint64_t last_tracker_target_count_{0};
  uint64_t last_target_eulr_count_{0};
  uint64_t last_send_count_{0};
  uint64_t last_mcu_target_eulr_count_{0};
  uint64_t last_mcu_fire_notify_count_{0};
};

RuntimeFreqProbe g_runtime_freq_probe;
DetectorPoseAudit g_detector_pose_audit;
DetectorTruthCompare g_detector_truth_compare;
TrackerTruthCompare g_tracker_truth_compare;
DetectorVideoRecorder g_detector_video_recorder;
TrackerVideoRecorder g_tracker_video_recorder;
TruthArmorsPublisher g_truth_armors_publisher;

bool RuntimeFreqProbeEnabled()
{
  const char *env = std::getenv("XR_FREQ_PROBE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

void RuntimeFreqProbeThreadFun(void *)
{
  g_runtime_freq_probe.InstallBlocking();
  while (true)
  {
    LibXR::Thread::Sleep(100);
    g_runtime_freq_probe.Report();
  }
}

bool TrackerVideoRecorderEnabled()
{
  const char *env = std::getenv("XR_TRACKER_VIDEO_RECORDER");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool DetectorVideoRecorderEnabled()
{
  const char *env = std::getenv("XR_DETECTOR_VIDEO_RECORDER");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool TrackerTruthCompareEnabled()
{
  const char *env = std::getenv("XR_TRACKER_TRUTH_COMPARE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool DetectorTruthCompareEnabled()
{
  const char *env = std::getenv("XR_DETECTOR_TRUTH_COMPARE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool DetectorPoseAuditEnabled()
{
  const char *env = std::getenv("XR_DETECTOR_POSE_AUDIT");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool TruthArmorsPublisherEnabled()
{
  const char *env = std::getenv("XR_TRUTH_ARMORS_PUBLISHER");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

void TrackerVideoRecorderThreadFun(void *)
{
  g_tracker_video_recorder.InstallBlocking();
}

void DetectorVideoRecorderThreadFun(void *)
{
  g_detector_video_recorder.InstallBlocking();
}

void TrackerTruthCompareThreadFun(void *)
{
  g_tracker_truth_compare.InstallBlocking();
  while (!g_tracker_truth_compare.Done())
  {
    LibXR::Thread::Sleep(100);
  }
}

void DetectorTruthCompareThreadFun(void *)
{
  g_detector_truth_compare.InstallBlocking();
  while (!g_detector_truth_compare.Done())
  {
    LibXR::Thread::Sleep(100);
  }
}

void DetectorPoseAuditThreadFun(void *)
{
  g_detector_pose_audit.InstallBlocking();
  while (!g_detector_pose_audit.Done())
  {
    LibXR::Thread::Sleep(100);
  }
}

void TruthArmorsPublisherThreadFun(void *)
{
  g_truth_armors_publisher.InstallBlocking();
}
}  // namespace

class UartBridge : public LibXR::UART
{
 public:
  UartBridge(LibXR::ReadPort *read_port, LibXR::WritePort *write_port)
      : UART(read_port, write_port)
  {
  }
  ErrorCode SetConfig(LibXR::UART::Configuration) override
  {
    return LibXR::ErrorCode::NOT_SUPPORT;
  }
};

int main(int, char **)
{
  webots::Supervisor supervisor;
  double sim_flow_rate = 1.0;
  if (const char *env = std::getenv("WEBOTS_SIM_FLOW_RATE"))
  {
    char *end = nullptr;
    const double parsed = std::strtod(env, &end);
    if (end != env && std::isfinite(parsed) && parsed > 0.0)
    {
      sim_flow_rate = parsed;
    }
  }
  LibXR::PlatformInit(&supervisor, sim_flow_rate);
  g_detector_truth_compare.Init(&supervisor);
  g_tracker_truth_compare.Init(&supervisor);
  g_detector_video_recorder.Init(&supervisor);
  g_tracker_video_recorder.Init(&supervisor);
  g_truth_armors_publisher.Init(&supervisor);

  XR_LOG_PASS("Platform initialized");

  LibXR::Pipe pipe_host_tx(1024), pipe_host_rx(1024);

  UartBridge uart_host(&pipe_host_rx.GetReadPort(), &pipe_host_tx.GetWritePort());

  UartBridge uart_client(&pipe_host_tx.GetReadPort(), &pipe_host_rx.GetWritePort());

  LibXR::RamFS ramfs;

  LibXR::Terminal<1024, 64, 16, 128> terminal(ramfs);

  LibXR::Thread term_thread;
  term_thread.Create(&terminal, LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                     512, LibXR::Thread::Priority::MEDIUM);

  auto log_topic = LibXR::Topic(LibXR::Topic::Find("/xr/log"));
  auto log_cb = LibXR::Topic::Callback::Create(log_cb_fun, log_topic);
  log_topic.RegisterCallback(log_cb);

  LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::UART>({uart_host, {"uart_host"}}),
      LibXR::Entry<LibXR::UART>({uart_client, {"uart_client"}}),
      LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
      LibXR::Entry<webots::Supervisor>({supervisor, {"supervisor"}})};

  LibXR::Thread freq_probe_thread;
  if (RuntimeFreqProbeEnabled())
  {
    freq_probe_thread.Create<void *>(nullptr, RuntimeFreqProbeThreadFun, "freq_probe",
                                     1024, LibXR::Thread::Priority::MEDIUM);
  }

  LibXR::Thread tracker_video_thread;
  if (TrackerVideoRecorderEnabled())
  {
    tracker_video_thread.Create<void *>(nullptr, TrackerVideoRecorderThreadFun,
                                        "tracker_video", 4096,
                                        LibXR::Thread::Priority::MEDIUM);
  }

  LibXR::Thread detector_video_thread;
  if (DetectorVideoRecorderEnabled())
  {
    detector_video_thread.Create<void *>(nullptr, DetectorVideoRecorderThreadFun,
                                         "detector_video", 4096,
                                         LibXR::Thread::Priority::MEDIUM);
  }

  LibXR::Thread tracker_truth_compare_thread;
  if (TrackerTruthCompareEnabled())
  {
    tracker_truth_compare_thread.Create<void *>(nullptr, TrackerTruthCompareThreadFun,
                                                "tracker_truth_compare", 4096,
                                                LibXR::Thread::Priority::MEDIUM);
  }

  LibXR::Thread detector_truth_compare_thread;
  if (DetectorTruthCompareEnabled())
  {
    detector_truth_compare_thread.Create<void *>(nullptr, DetectorTruthCompareThreadFun,
                                                 "detector_truth_compare", 4096,
                                                 LibXR::Thread::Priority::MEDIUM);
  }

  LibXR::Thread detector_pose_audit_thread;
  if (DetectorPoseAuditEnabled())
  {
    detector_pose_audit_thread.Create<void *>(nullptr, DetectorPoseAuditThreadFun,
                                              "detector_pose_audit", 4096,
                                              LibXR::Thread::Priority::MEDIUM);
  }

  LibXR::Thread truth_armors_publisher_thread;
  if (TruthArmorsPublisherEnabled())
  {
    truth_armors_publisher_thread.Create<void *>(nullptr, TruthArmorsPublisherThreadFun,
                                                 "truth_armors_pub", 4096,
                                                 LibXR::Thread::Priority::MEDIUM);
  }

  XRobotMain(peripherals);
  return 0;
}
