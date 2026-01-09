#pragma once

#include <string>
#include <functional>
#include <cstdint>
#include <memory>
#include <linux/videodev2.h>

namespace robosense
{
namespace video
{

struct V4L2Params
{
  std::string device_path;  // e.g. "/dev/video0"
  uint32_t width = 6448;
  uint32_t height = 2592;
  uint32_t pixel_format = V4L2_PIX_FMT_GREY;
  uint32_t preferred_stride = 6464;
};

using FrameCallback = std::function<void(void*, size_t, double)>;

class GmslCapture
{
public:
  explicit GmslCapture(const V4L2Params& params);
  ~GmslCapture();

  GmslCapture(const GmslCapture&) = delete;
  GmslCapture& operator=(const GmslCapture&) = delete;

  bool init();
  bool start();
  void stop();
  void setFrameCallback(FrameCallback cb);

private:
  class VideoImpl;
  std::unique_ptr<VideoImpl> VideoImpl_;
};

}  // namespace video
}  // namespace robosense