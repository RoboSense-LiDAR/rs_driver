/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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
#include "rs_common/debug/error_code.h"
#include "rs_common/debug/prompt.h"
#pragma once

namespace robosense
{
namespace common
{
/**
 * @brief Error printer Class
 * Use to print error code
 */
class ErrorPrinter
{
public:
  /**
     * @brief Error code print function
     * @detail
     *
     * 0x0 for Success
     * for each module range
     * 0x400~0x800 WARNING
     * 0x800~0xC00 ERROR
     *
     */
  inline void printErrorCode(const ErrCode &error)
  {
    std::string info;
    switch (error)
    {
    case ErrCode_Success:
      info = "ErrCode_Success";
      break;
    case ErrCode_WrongUsrConfigNameYaml:
      info = "ErrCode_WrongUsrConfigNameYaml";
      break;
    case ErrCode_WrongConfigYaml:
      info = "ErrCode_WrongConfigYaml";
      break;
    case ErrCode_LidarDriverInterrupt:
      info = "ErrCode_LidarDriverInterrupt";
      break;
    case ErrCode_LidarPointsProtoSendError:
      info = "ErrCode_LidarPointsProtoSendError";
      break;
    case ErrCode_LidarPointsProtoReceiveError:
      info = "ErrCode_LidarPointsProtoReceiveError";
      break;
    case ErrCode_LidarPacketsProtoSendError:
      info = "ErrCode_LidarPacketsProtoSendError";
      break;
    case ErrCode_LidarPacketsProtoReceiveError:
      info = "ErrCode_LidarPacketsProtoReceiveError";
      break;
    case ErrCode_ImuDriverConnectfail:
      info = "ErrCode_ImuDriverConnectfail";
      break;
    case ErrCode_ImuDriverGyroztoolarge:
      info = "ErrCode_ImuDriverGyroztoolarge";
      break;
    case ErrCode_ImuDriverInterrupt:
      info = "ErrCode_ImuDriverInterrupt";
      break;
    case ErrCode_ImuDriverDisconnect:
      info = "ErrCode_ImuDriverDisconnect";
      break;
    case ErrCode_GnssDriverConnectfail:
      info = "ErrCode_GnssDriverConnectfail";
      break;
    case ErrCode_GnssDriverGyroztoolarge:
      info = "ErrCode_GnssDriverGyroztoolarge";
      break;
    case ErrCode_GnssDriverInterrupt:
      info = "ErrCode_GnssDriverInterrupt";
      break;
    case ErrCode_GnssDriverLacksatellite:
      info = "ErrCode_GnssDriverLacksatellite";
      break;
    case ErrCode_GnssDriverDisconnect:
      info = "ErrCode_GnssDriverDisconnect";
      break;
    case ErrCode_PreprocessingImuTimeout:
      info = "ErrCode_PreprocessingImuTimeout";
      break;
    case ErrCode_PreprocessingOdomTimeout:
      info = "ErrCode_PreprocessingOdomTimeout";
      break;
    case ErrCode_PreprocessingGnssTimeout:
      info = "ErrCode_PreprocessingGnssTimeout";
      break;
    case ErrCode_PreprocessingLidarTimeout:
      info = "ErrCode_PreprocessingLidarTimeout";
      break;
    case ErrCode_PreprocessingLidarNumOverflow:
      info = "ErrCode_PreprocessingLidarNumOverflow";
      break;
    case ErrCode_PreprocessingUnknownLidarFrameId:
      info = "ErrCode_PreprocessingUnknownLidarFrameId";
      break;
    case ErrCode_PreProcessingCallbackTimeout:
      info = "ErrCode_PreProcessingCallbackTimeout";
      break;
    case ErrCode_PreProcessingTimestampError:
      info = "ErrCode_PreProcessingTimestampError";
      break;
    case ErrCode_PreProcessingConfigError:
      info = "ErrCode_PreProcessingConfigError";
      break;
    case ErrCode_ImuProtoSendError:
      info = "ErrCode_ImuProtoSendError";
      break;
    case ErrCode_ImuProtoReceiveError:
      info = "ErrCode_ImuProtoReceiveError";
      break;
    case ErrCode_GnssProtoSendError:
      info = "ErrCode_GnssProtoSendError";
      break;
    case ErrCode_GnssProtoReceiveError:
      info = "ErrCode_GnssProtoReceiveError";
      break;
    case ErrCode_OdomProtoSendError:
      info = "ErrCode_OdomProtoSendError";
      break;
    case ErrCode_OdomProtoReceiveError:
      info = "ErrCode_OdomProtoReceiveError";
      break;
    case ErrCode_FailLoadingGlobalConfig:
      info = "ErrCode_FailLoadingGlobalConfig";
      break;
    case ErrCode_MissingCoreConfig:
      info = "ErrCode_MissingCoreConfig";
      break;
    case ErrCode_MapServerLoadMap:
      info = "ErrCode_MissingCoreConfig";
      break;
    case ErrCode_MapServerEmptyOrigin:
      info = "ErrCode_MapServerEmptyOrigin";
      break;
    case ErrCode_MsfRunBeforeConfig:
      info = "ErrCode_MsfRunBeforeConfig";
      break;
    case ErrCode_MsfUndefObserverType:
      info = "ErrCode_MsfUndefObserverType";
      break;
    case ErrCode_MsfUndefObserverName:
      info = "ErrCode_MsfUndefObserverName";
      break;
    case ErrCode_MsfMultiplePredictor:
      info = "ErrCode_MsfMultiplePredictor";
      break;
    case ErrCode_MsfWrongObserverTpye:
      info = "ErrCode_MsfWrongObserverTpye";
      break;
    case ErrCode_MsfUndefType:
      info = "ErrCode_MsfUndefType";
      break;
    case ErrCode_MsfNoObserver:
      info = "ErrCode_MsfNoObserver";
      break;
    case ErrCode_MsfNoStateForUpdate:
      info = "ErrCode_MsfNoStateForUpdate";
      break;
    case ErrCode_LidarFailObserve:
      info = "ErrCode_LidarFailObserve";
      break;

    case ErrCode_IMU_OVERFLOW:
      info = "ErrCode_IMU_OVERFLOW";
      break;
    case ErrCode_LIDAR_OVERFLOW:
      info = "ErrCode_LIDAR_OVERFLOW";
      break;

    case ErrCode_ODOM_OVERFLOW:
      info = "ErrCode_ODOM_OVERFLOW";
      break;
    case ErrCode_GNSS_OVERFLOW:
      info = "ErrCode_GNSS_OVERFLOW";
      break;
    case ErrCode_MANY_GNSS_MSG_WAITING:
      info = "ErrCode_MANY_GNSS_MSG_WAITING";
      break;
    case ErrCode_EMPTY_DISP_RS_MAP:
      info = "ErrCode_EMPTY_DISP_RS_MAP";
      break;
    case ErrCode_LocalizationProtobufSendError:
      info = "ErrCode_LocalizationProtobufSendError";
      break;
    case ErrCode_PerceptionConfigError:
      info = "ErrCode_PerceptionConfigError";
      break;
    case ErrCode_PerceptionInitError:
      info = "ErrCode_PerceptionInitError";
      break;
    case ErrCode_PerceptionNullInput:
      info = "ErrCode_PerceptionNullInput";
      break;
    case ErrCode_PerceptionBadRotateCalibration:
      info = "ErrCode_PerceptionBadRotateCalibration";
      break;
    case ErrCode_PerceptionBadHeightCalibration:
      info = "ErrCode_PerceptionBadHeightCalibration";
      break;
    case ErrCode_PerceptionGroundInitFalse:
      info = "ErrCode_PerceptionGroundInitFalse";
      break;
    case ErrCode_PerceptionProtobufSendError:
      info = "ErrCode_PerceptionProtobufSendError";
      break;
    }
    if ((error & 0xF00) != 0 && (error & 0xF00) < 0x800)
    {
      WARNING << "WARNING: " << info << "  " << std::hex << "0x" << error << REND;
    }
    if ((error & 0xF00) >= 0x800)
    {
      ERROR << "ERROR: " << info << "  " << std::hex << "0x" << error << REND;
    }
  }
};
} // namespace common
} // namespace robosense
