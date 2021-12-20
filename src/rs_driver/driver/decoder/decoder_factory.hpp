/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <rs_driver/driver/decoder/decoder_RS32.hpp>
#include <rs_driver/driver/decoder/decoder_RSBP.hpp>
#include <rs_driver/driver/decoder/decoder_RSHELIOS.hpp>
#if 0
#include <rs_driver/driver/decoder/decoder_RS16.hpp>
#include <rs_driver/driver/decoder/decoder_RS80.hpp>
#include <rs_driver/driver/decoder/decoder_RS128.hpp>
#include <rs_driver/driver/decoder/decoder_RSM1.hpp>
#include <rs_driver/driver/decoder/decoder_RSROCK.hpp>
#endif

namespace robosense
{
namespace lidar
{
template <typename T_PointCloud>
class DecoderFactory
{
public:

  static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(LidarType type, 
      const RSDecoderParam& param, const std::function<void(const Error&)>& excb);
};

template <typename T_PointCloud>
inline std::shared_ptr<Decoder<T_PointCloud>>
DecoderFactory<T_PointCloud>::createDecoder(LidarType type, const RSDecoderParam& param,
    const std::function<void(const Error&)>& excb)
{
  std::shared_ptr<Decoder<T_PointCloud>> ret_ptr;

  switch (type)
  {
#if 0
    case LidarType::RS16:
      ret_ptr = std::make_shared<DecoderRS16<T_PointCloud>>(param.decoder_param);
      break;
#endif
    case LidarType::RS32:
      ret_ptr = std::make_shared<DecoderRS32<T_PointCloud>>(param, excb);
      break;
    case LidarType::RSBP:
      ret_ptr = std::make_shared<DecoderRSBP<T_PointCloud>>(param, excb);
      break;
#if 0
    case LidarType::RS128:
      ret_ptr = std::make_shared<DecoderRS128<T_PointCloud>>(param.decoder_param);
      break;
    case LidarType::RS80:
      ret_ptr = std::make_shared<DecoderRS80<T_PointCloud>>(param.decoder_param);
      break;
    case LidarType::RSM1:
      ret_ptr = std::make_shared<DecoderRSM1<T_PointCloud>>(param.decoder_param);
      break;
#endif
    case LidarType::RSHELIOS:
      ret_ptr = std::make_shared<DecoderRSHELIOS<T_PointCloud>>(param, excb);
      break;
#if 0
    case LidarType::RSROCK:
      ret_ptr = std::make_shared<DecoderRSROCK<T_PointCloud>>(param.decoder_param);
      break;
#endif
    default:
      RS_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << RS_REND;
      exit(-1);
  }

  return ret_ptr;
}

#if 0
template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRS16ConstantParam()
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
  ret_param.DIS_RESOLUTION = 0.005;
  ret_param.RX = 0.03825;
  ret_param.RY = -0.01088;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRS32ConstantParam()
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
  ret_param.DIS_RESOLUTION = 0.005;

  ret_param.RX = 0.03997;
  ret_param.RY = -0.01087;
  ret_param.RZ = 0;

  return ret_param;
}

template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRSBPConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA050 A55A 0A05 AA55;
  ret_param.DIFOP_ID = 0x5555 1111 5A00 FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1500;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 32;
  ret_param.DSR_TOFFSET = 1.28;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.DIS_RESOLUTION = 0.005;
  ret_param.RX = 0.01473;
  ret_param.RY = 0.0085;
  ret_param.RZ = 0.09427;
  return ret_param;
}

template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRS80ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xFE;
  ret_param.PKT_RATE = 4500;
  ret_param.BLOCKS_PER_PKT = 4;
  ret_param.CHANNELS_PER_BLOCK = 80;
  ret_param.LASER_NUM = 80;
  ret_param.DSR_TOFFSET = 3.236;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.DIS_RESOLUTION = 0.005;
  ret_param.RX = 0.03615;
  ret_param.RY = -0.017;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRS128ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xFE;
  ret_param.PKT_RATE = 6000;
  ret_param.BLOCKS_PER_PKT = 3;
  ret_param.CHANNELS_PER_BLOCK = 128;
  ret_param.LASER_NUM = 128;
  ret_param.DSR_TOFFSET = 3.236;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.DIS_RESOLUTION = 0.005;
  ret_param.RX = 0.03615;
  ret_param.RY = -0.017;
  ret_param.RZ = 0;
  return ret_param;
}

template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRSM1ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0xA55AAA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCKS_PER_PKT = 25;
  ret_param.CHANNELS_PER_BLOCK = 5;
  ret_param.LASER_NUM = 5;
  ret_param.DIS_RESOLUTION = 0.005;
  return ret_param;
}

template <typename T_PointCloud>
inline const LidarConstantParameter DecoderFactory<T_PointCloud>::getRSHELIOSConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x5A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1500;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.BLOCKS_PER_FRAME = 1800;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 32;
  ret_param.DSR_TOFFSET = 1.0;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.DIS_RESOLUTION = 0.0025;
  ret_param.RX = 0.03498;
  ret_param.RY = -0.015;
  ret_param.RZ = 0.0;
  return ret_param;
}

// TODO
template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getRSROCKConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0x000001005A05AA55;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 1071;  // TODO
  ret_param.BLOCKS_PER_PKT = 6;
  ret_param.CHANNELS_PER_BLOCK = 14 * 4;  // TODO
  ret_param.LASER_NUM = 4;
  ret_param.DSR_TOFFSET = 1.0;
  ret_param.FIRING_FREQUENCY = 0.018;
  ret_param.DIS_RESOLUTION = 0.0025;
  ret_param.RX = 0.07526;
  ret_param.RY = 0.00968;
  ret_param.RZ = 0.0;
  return ret_param;
}
#endif

}  // namespace lidar
}  // namespace robosense
