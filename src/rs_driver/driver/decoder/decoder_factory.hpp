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
  DecoderFactory() = default;
  ~DecoderFactory() = default;
  static std::shared_ptr<DecoderBase<T_Point>> createDecoder(const LidarType& param_lidar_type,
                                                             const RSDriverParam& param, const PacketMsg& msop_pkt_msg,
                                                             const std::shared_ptr<Input>& input_ptr);
  static std::shared_ptr<DecoderBase<T_Point>> createDecoder(const LidarType& param_lidar_type,
                                                             const RSDriverParam& param, const PacketMsg& msop_pkt_msg);

private:
  static std::shared_ptr<DecoderBase<T_Point>> switchLidar(const LidarType& lidar_type, const RSDriverParam& param);
  static LidarType getLidarType(const LidarType& param_lidar_type, const PacketMsg& msop_pkt_msg);
  static const LidarConstantParameter getRS16ConstantParam();
  static const LidarConstantParameter getRS32ConstantParam();
  static const LidarConstantParameter getRSBPConstantParam();
  static const LidarConstantParameter getRS80ConstantParam();
  static const LidarConstantParameter getRS128ConstantParam();
};

template <typename T_Point>
inline std::shared_ptr<DecoderBase<T_Point>>
DecoderFactory<T_Point>::createDecoder(const LidarType& param_lidar_type, const RSDriverParam& param,
                                       const PacketMsg& msop_pkt_msg, const std::shared_ptr<Input>& input_ptr)
{
  LidarType lidar_type = getLidarType(param_lidar_type, msop_pkt_msg);
  input_ptr->setLidarType(lidar_type);
  return switchLidar(lidar_type, param);
}

template <typename T_Point>
inline std::shared_ptr<DecoderBase<T_Point>> DecoderFactory<T_Point>::createDecoder(const LidarType& param_lidar_type,
                                                                                    const RSDriverParam& param,
                                                                                    const PacketMsg& msop_pkt_msg)
{
  LidarType lidar_type = getLidarType(param_lidar_type, msop_pkt_msg);
  return switchLidar(lidar_type, param);
}

template <typename T_Point>
inline LidarType DecoderFactory<T_Point>::getLidarType(const LidarType& param_lidar_type,
                                                             const PacketMsg& msop_pkt_msg)
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

template <typename T_Point>
inline std::shared_ptr<DecoderBase<T_Point>> DecoderFactory<T_Point>::switchLidar(const LidarType& lidar_type,
                                                                                  const RSDriverParam& param)
{
  std::shared_ptr<DecoderBase<T_Point>> ret_ptr;
  switch (lidar_type)
  {
    case LidarType::RS16:
      ret_ptr = std::make_shared<DecoderRS16<T_Point>>(param.decoder_param, getRS16ConstantParam());
      break;
    case LidarType::RS32:
      ret_ptr = std::make_shared<DecoderRS32<T_Point>>(param.decoder_param, getRS32ConstantParam());
      break;
    case LidarType::RSBP:
      ret_ptr = std::make_shared<DecoderRSBP<T_Point>>(param.decoder_param, getRSBPConstantParam());
      break;
    case LidarType::RS128:
      ret_ptr = std::make_shared<DecoderRS128<T_Point>>(param.decoder_param, getRS128ConstantParam());
      break;
    case LidarType::RS80:
      ret_ptr = std::make_shared<DecoderRS80<T_Point>>(param.decoder_param, getRS80ConstantParam());
      break;
    default:
      RS_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << RS_REND;
      exit(-1);
  }
  ret_ptr->loadCalibrationFile(param.angle_path);
  return ret_ptr;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getRS16ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050A55A0A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 750;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 16;
  ret_param.DSR_TOFFSET = 2.8;
  ret_param.FIRING_FREQUENCY = 0.009;
  ret_param.RX = 0.03825;
  ret_param.RY = -0.01088;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getRS32ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050A55A0A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1500;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 32;
  ret_param.DSR_TOFFSET = 1.44;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.03997;
  ret_param.RY = -0.01087;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getRSBPConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050A55A0A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1500;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 32;
  ret_param.DSR_TOFFSET = 1.28;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.01473;
  ret_param.RY = 0.0085;
  ret_param.RZ = 0.09427;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getRS80ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xFE;
  ret_param.PKT_RATE = 4500;
  ret_param.BLOCKS_PER_PKT = 4;
  ret_param.CHANNELS_PER_BLOCK = 80;
  ret_param.LASER_NUM = 80;
  ret_param.DSR_TOFFSET = 3.23;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.03615;
  ret_param.RY = -0.017;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getRS128ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xFE;
  ret_param.PKT_RATE = 6000;
  ret_param.BLOCKS_PER_PKT = 3;
  ret_param.CHANNELS_PER_BLOCK = 128;
  ret_param.LASER_NUM = 128;
  ret_param.DSR_TOFFSET = 3.23;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.RX = 0.03615;
  ret_param.RY = -0.017;
  ret_param.RZ = 0;
  return ret_param;
}
}  // namespace lidar
}  // namespace robosense
