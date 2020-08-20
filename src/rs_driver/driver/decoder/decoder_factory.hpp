/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <rs_driver/driver/decoder/decoder_RS16.hpp>
#include <rs_driver/driver/decoder/decoder_RS32.hpp>
#include <rs_driver/driver/decoder/decoder_RS80.hpp>
#include <rs_driver/driver/decoder/decoder_RS128.hpp>
#include <rs_driver/driver/decoder/decoder_RSBP.hpp>
#include <rs_driver/driver/input.hpp>
#include <rs_driver/msg/packet_msg.h>
namespace robosense
{
namespace lidar
{
template <typename T_Point>
class DecoderFactory
{
public:
  inline static std::shared_ptr<DecoderBase<T_Point>> createDecoder(const LidarType& param_lidar_type,
                                                                    const RSDriverParam& param,
                                                                    const PacketMsg& msop_pkt_msg,
                                                                    const std::shared_ptr<Input>& input_ptr)
  {
    LidarType lidar_type = getLidarType(param_lidar_type, msop_pkt_msg);
    input_ptr->setLidarType(lidar_type);
    return switchLidar(lidar_type, param);
  }
  inline static std::shared_ptr<DecoderBase<T_Point>> createDecoder(const LidarType& param_lidar_type,
                                                                    const RSDriverParam& param,
                                                                    const PacketMsg& msop_pkt_msg)
  {
    LidarType lidar_type = getLidarType(param_lidar_type, msop_pkt_msg);
    return switchLidar(lidar_type, param);
  }

  inline static LidarType getLidarType(const LidarType& param_lidar_type, const PacketMsg& msop_pkt_msg)
  {
    if (param_lidar_type == LidarType::RSAUTO)
    {
      RSMsopHeader* header_ptr = (RSMsopHeader*)msop_pkt_msg.packet.data();
      return (LidarType)header_ptr->lidar_type;
    }
    else
    {
      return param_lidar_type;
    }
  }
  inline static std::shared_ptr<DecoderBase<T_Point>> switchLidar(const LidarType& lidar_type,
                                                                  const RSDriverParam& param)
  {
    std::shared_ptr<DecoderBase<T_Point>> ret_ptr;
    switch (lidar_type)
    {
      case LidarType::RS16:
        ret_ptr = std::make_shared<DecoderRS16<T_Point>>(param.decoder_param);
        break;
      case LidarType::RS32:
        ret_ptr = std::make_shared<DecoderRS32<T_Point>>(param.decoder_param);
        break;
      case LidarType::RSBP:
        ret_ptr = std::make_shared<DecoderRSBP<T_Point>>(param.decoder_param);
        break;
      case LidarType::RS128:
        ret_ptr = std::make_shared<DecoderRS128<T_Point>>(param.decoder_param);
        break;
      case LidarType::RS80:
        ret_ptr = std::make_shared<DecoderRS80<T_Point>>(param.decoder_param);
        break;
      default:
        ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << REND;
        exit(-1);
    }
    ret_ptr->loadCalibrationFile(param.angle_path);
    return ret_ptr;
  }
};
}  // namespace lidar
}  // namespace robosense
