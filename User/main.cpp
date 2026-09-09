#include <chrono>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <webots/Supervisor.hpp>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_system.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "terminal.hpp"
#include "thread.hpp"
#include "xrobot_main.hpp"

#if defined(XR_WEBOTS_ACCEPTANCE)
#include "../tests/WebotsAcceptance.hpp"
#endif

namespace
{
const char *FileLogLevelName(LibXR::LogLevel level)
{
  switch (level)
  {
    case LibXR::LogLevel::XR_LOG_LEVEL_ERROR:
      return "E";
    case LibXR::LogLevel::XR_LOG_LEVEL_WARN:
      return "W";
    case LibXR::LogLevel::XR_LOG_LEVEL_PASS:
      return "P";
    case LibXR::LogLevel::XR_LOG_LEVEL_INFO:
      return "I";
    case LibXR::LogLevel::XR_LOG_LEVEL_DEBUG:
      return "D";
    default:
      return "?";
  }
}
}  // namespace

void (*log_cb_fun)(bool, LibXR::Topic,
                   const LibXR::Topic::MessageView<LibXR::LogData>&) =
    [](bool, LibXR::Topic tp,
       const LibXR::Topic::MessageView<LibXR::LogData>& message)
{
  UNUSED(tp);
  const auto timestamp = message.timestamp;
  const auto* log = message.data;
  if (log == nullptr)
  {
    return;
  }

  if (LibXR::STDIO::write_ && LibXR::STDIO::write_->Writable())
  {
    using clock = std::chrono::system_clock;

    static std::mutex file_mutex;
    std::lock_guard<std::mutex> lock(file_mutex);
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

      LibXR::STDIO::Printf<"Log written to %s\n">(oss.str().c_str());
    }

    if (f)
    {
      const uint32_t timestamp_ms =
          static_cast<uint32_t>(static_cast<uint64_t>(timestamp) / 1000U);
      f << FileLogLevelName(log->level) << " [" << timestamp_ms << "]("
        << (log->file ? log->file : "?") << ':' << log->line << ") "
        << log->message << '\n';
      f.flush();
    }
  }
};

static int AcquireBspLock()
{
  constexpr const char *lock_path = "/tmp/xrobot-autoaim-camera.lock";
  int fd = open(lock_path, O_CREAT | O_RDWR | O_CLOEXEC, 0666);
  if (fd < 0)
  {
    XR_LOG_ERROR("failed to open BSP lock %s: %s", lock_path, std::strerror(errno));
    return -1;
  }

  if (flock(fd, LOCK_EX | LOCK_NB) != 0)
  {
    XR_LOG_ERROR("another autoaim BSP is already running (%s)", lock_path);
    close(fd);
    return -1;
  }
  return fd;
}

int main(int, char **)
{
  webots::Supervisor supervisor;
  LibXR::PlatformInit(&supervisor);

  const int bsp_lock_fd = AcquireBspLock();
  if (bsp_lock_fd < 0)
  {
    return 1;
  }
  (void)bsp_lock_fd;

  XR_LOG_PASS("Platform initialized");

  LibXR::RamFS ramfs;
  LibXR::Terminal<1024, 64, 16, 128> terminal(ramfs);

  LibXR::Thread term_thread;
  term_thread.Create(&terminal, LibXR::Terminal<1024, 64, 16, 128>::ThreadFun, "terminal",
                     512, LibXR::Thread::Priority::MEDIUM);

  auto log_topic = LibXR::Topic(LibXR::Topic::Find("/xr/log"));
  auto log_cb = LibXR::Topic::Callback::Create(log_cb_fun, log_topic);
  log_topic.RegisterCallback(log_cb);

  LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
      LibXR::Entry<webots::Supervisor>({supervisor, {"supervisor"}})};

#if defined(XR_WEBOTS_ACCEPTANCE)
  WebotsAcceptance::Install();
#endif
  XRobotMain(peripherals);

  while (true)
  {
    LibXR::Thread::Sleep(1000);
  }
  return 0;
}
