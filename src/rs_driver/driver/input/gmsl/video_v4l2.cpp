/**
 * @file
 * @brief V4L2 video capture VideoImplementation
 */

#include "gmsl_capture.hpp"
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <pthread.h>
#include <cstring>
#include <thread>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

#define LOG_TAG "GMSL_CAPTURE"
#ifndef DEF_WRAP
#define DEF_WRAP "\n"
#endif

#define GMSL_LOGE(TAG, P, Q...) printf("E/: [" TAG "] " P DEF_WRAP, ##Q)
#define GMSL_LOGI(TAG, P, Q...) printf("I/: [" TAG "] " P DEF_WRAP, ##Q)

namespace robosense
{
namespace video
{

struct V4L2Buffer
{
  void* data = nullptr;
  size_t size = 0;
};

class GmslCapture::VideoImpl
{
public:
  VideoImpl(const V4L2Params& params) : params_(params), fd_(-1), is_running_(false), buffer_count_(5)
  {
    buffers_.resize(buffer_count_);
  }

  ~VideoImpl()
  {
    stop();
    cleanup();
  }

  bool init()
  {
    if (!openDevice())
      return false;

    if (!checkCapabilities())
      return false;

    if (!configureFormat())
      return false;

    if (!setControls())
      return false;

    if (!setupMmap())
      return false;

    return true;
  }

  bool start()
  {
    if (is_running_)
      return true;

    if (!startStream())
      return false;

    startCaptureThread();
    is_running_ = true;
    return true;
  }

  void stop()
  {
    is_running_ = false;
    if (capture_thread_.joinable())
    {
      capture_thread_.join();
    }
    stopStream();
  }

  void setFrameCallback(FrameCallback cb)
  {
    callback_ = std::move(cb);
  }

private:
  bool openDevice()
  {
    std::string dev_path = params_.device_path.empty() ? "/dev/video0" : params_.device_path;
    fd_ = open(dev_path.c_str(), O_RDWR | O_NONBLOCK);

    if (fd_ < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to open device: %s, errno: %d", dev_path.c_str(), errno);
      return false;
    }

    GMSL_LOGI(LOG_TAG, "Opened device: %s, fd: %d", dev_path.c_str(), fd_);
    return true;
  }

  void cleanup()
  {
    for (auto& buf : buffers_)
    {
      if (buf.data)
      {
        munmap(buf.data, buf.size);
        buf.data = nullptr;
      }
    }

    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  bool checkCapabilities()
  {
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));

    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to query capabilities: %d", errno);
      return false;
    }

    GMSL_LOGI(LOG_TAG, "Device capabilities:");
    GMSL_LOGI(LOG_TAG, "  Driver: %s", cap.driver);
    GMSL_LOGI(LOG_TAG, "  Card: %s", cap.card);
    GMSL_LOGI(LOG_TAG, "  Bus: %s", cap.bus_info);

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
      GMSL_LOGE(LOG_TAG, "Device does not support video capture");
      return false;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
      GMSL_LOGE(LOG_TAG, "Device does not support streaming");
      return false;
    }

    return true;
  }

  bool configureFormat()
  {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = params_.width;
    fmt.fmt.pix.height = params_.height;
    fmt.fmt.pix.pixelformat = params_.pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to set format: %d", errno);
      return false;
    }

    // Verify actual format
    GMSL_LOGI(LOG_TAG, "Configured format: %c%c%c%c %dx%d", (fmt.fmt.pix.pixelformat & 0xFF),
              (fmt.fmt.pix.pixelformat >> 8) & 0xFF, (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
              (fmt.fmt.pix.pixelformat >> 24) & 0xFF, fmt.fmt.pix.width, fmt.fmt.pix.height);

    return true;
  }

  int queryCtrlIdByName(int fd, const char* ctrl_name)
  {
    if (fd < 0 || ctrl_name == nullptr || strlen(ctrl_name) == 0)
    {
      GMSL_LOGE(LOG_TAG, "Invalid input: fd=%d, ctrl_name=%s", fd, ctrl_name ? ctrl_name : "null");
      return -1;
    }

    struct v4l2_queryctrl query_ctrl;
    memset(&query_ctrl, 0, sizeof(query_ctrl));
    const uint32_t MAX_ID = 0x0800FFFF;
    int target_id = -1;

    query_ctrl.id = V4L2_CTRL_CLASS_CAMERA;

    while (true)
    {
      if (query_ctrl.id > MAX_ID)
      {
        GMSL_LOGE(LOG_TAG, "Exceed max ctrl ID (0x%x), stop search", MAX_ID);
        break;
      }

      if (ioctl(fd_, VIDIOC_QUERYCTRL, &query_ctrl) < 0)
      {
        query_ctrl.id++;
        continue;
      }

      if (query_ctrl.flags & V4L2_CTRL_FLAG_HAS_PAYLOAD)
      {
        query_ctrl.id++;
        continue;
      }

      if (strcmp(reinterpret_cast<const char*>(query_ctrl.name), ctrl_name) == 0)
      {
        target_id = query_ctrl.id;
        GMSL_LOGI(LOG_TAG, "Found ctrl '%s': id=0x%x", ctrl_name, target_id);
        break;
      }

      query_ctrl.id++;
    }

    if (target_id == -1)
    {
      GMSL_LOGE(LOG_TAG, "Control '%s' not found", ctrl_name);
    }

    return target_id;
  }

  bool setControls()
  {
    int target_id = queryCtrlIdByName(fd_, "Preferred Stride");
    if (target_id == -1)
    {
      GMSL_LOGE(LOG_TAG, "Preferred stride control not found");
      return false;
    }

    struct v4l2_control ctrl_stride;
    memset(&ctrl_stride, 0, sizeof(ctrl_stride));
    ctrl_stride.id = target_id;
    ctrl_stride.value = params_.preferred_stride;
    if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl_stride) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to set preferred_stride: %d", errno);
      return false;
    }

    GMSL_LOGI(LOG_TAG, "Set controls: preferred_stride=%d", params_.preferred_stride);
    return true;
  }

  bool setupMmap()
  {
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = buffer_count_;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to request buffers: %d", errno);
      return false;
    }

    if (req.count < 2)
    {
      GMSL_LOGE(LOG_TAG, "Insufficient buffers: %u", req.count);
      return false;
    }

    buffer_count_ = req.count;
    buffers_.resize(buffer_count_);

    for (size_t i = 0; i < buffer_count_; ++i)
    {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.index = i;
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
      {
        GMSL_LOGE(LOG_TAG, "Failed to query buffer %zu: %d", i, errno);
        return false;
      }

      buffers_[i].size = buf.length;
      buffers_[i].data = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

      if (buffers_[i].data == MAP_FAILED)
      {
        GMSL_LOGE(LOG_TAG, "Failed to mmap buffer %zu: %d", i, errno);
        return false;
      }
    }

    return true;
  }

  bool startStream()
  {
    for (size_t i = 0; i < buffer_count_; ++i)
    {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.index = i;
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0)
      {
        GMSL_LOGE(LOG_TAG, "Failed to queue buffer %zu: %d", i, errno);
        return false;
      }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to start stream: %d", errno);
      return false;
    }

    return true;
  }

  void stopStream()
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
  }

  void startCaptureThread()
  {
    is_running_ = true;
    capture_thread_ = std::thread(&VideoImpl::captureLoop, this);
  }

  void captureLoop()
  {
    fd_set fds;
    struct timeval tv;
    int r;

    while (is_running_)
    {
      FD_ZERO(&fds);
      FD_SET(fd_, &fds);

      // Timeout: 20ms
      tv.tv_sec = 0;
      tv.tv_usec = 20000;

      r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
      if (r < 0)
      {
        GMSL_LOGE(LOG_TAG, "Select error: %d", errno);
        continue;
      }

      if (r == 0)
      {
        continue;  // Timeout
      }

      if (!FD_ISSET(fd_, &fds))
      {
        continue;
      }

      captureFrame();
    }
  }

  uint64_t v4l2_timeval_to_us(const struct timeval* tv)
  {
    return (__u64)tv->tv_sec * 1000000ULL + tv->tv_usec;
  }
  void captureFrame()
  {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to dequeue buffer: %d", errno);
      return;
    }

    // Call user callback with frame data
    if (callback_ && buf.index < buffers_.size())
    {
      // Convert timeval to nanoseconds
      double timestamp = v4l2_timeval_to_us(&buf.timestamp) * 1e-6;
      callback_(buffers_[buf.index].data, buffers_[buf.index].size, timestamp);
    }

    // Re-queue buffer
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0)
    {
      GMSL_LOGE(LOG_TAG, "Failed to requeue buffer: %d", errno);
    }
  }

  V4L2Params params_;
  int fd_;
  bool is_running_;
  size_t buffer_count_;
  std::vector<V4L2Buffer> buffers_;
  std::thread capture_thread_;
  FrameCallback callback_;
};

// GmslCapture class VideoImplementation
GmslCapture::GmslCapture(const V4L2Params& params) : VideoImpl_(std::make_unique<VideoImpl>(params))
{
}

GmslCapture::~GmslCapture() = default;

bool GmslCapture::init()
{
  return VideoImpl_->init();
}

void GmslCapture::stop()
{
  VideoImpl_->stop();
}

void GmslCapture::setFrameCallback(FrameCallback cb)
{
  VideoImpl_->setFrameCallback(std::move(cb));
}
bool GmslCapture::start()
{
  return VideoImpl_->start();
}

}  // namespace video
}  // namespace robosense
