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

#pragma once

namespace robosense
{
namespace common
{

/**
   * @brief Error Code for Robosense SDK.
   * @detail
   *
   * 0x0 for Success
   * for each module range
   * 0x001 ~ 0x400 for normal
   * 0x401 ~ 0x800 for Warning
   * 0x801 ~ 0xC00 for Critical Error
   *
   */

enum ErrCode
{
   /* Success */
   ErrCode_Success = 0x0,
   ErrCode_HeartBeat = 0x1,
   /* Initialization: 0x1~0xFFF*/
   ErrCode_WrongUsrConfigNameYaml = 0x801,
   ErrCode_WrongConfigYaml = 0x802,

   /* Lidar : 0x1000~0x1FFF*/
   ErrCode_LidarDriverInterrupt = 0x1401,
   ErrCode_LidarPointsProtoSendError = 0x1402,
   ErrCode_LidarPointsProtoReceiveError = 0x1403,
   ErrCode_LidarPacketsProtoSendError = 0x1404,
   ErrCode_LidarPacketsProtoReceiveError = 0x1405,

   /* Imu : 0x2000~0x2FFF*/
   ErrCode_ImuDriverConnectfail = 0x2401,
   ErrCode_ImuDriverGyroztoolarge = 0x2402,
   ErrCode_ImuDriverInterrupt = 0x2403,
   ErrCode_ImuDriverDisconnect = 0x2801,
   ErrCode_ImuProtoSendError = 0x2404,
   ErrCode_ImuProtoReceiveError = 0x2405,

   /* Gnss : 0x3000~0x3FFF*/
   ErrCode_GnssDriverConnectfail = 0x3401,
   ErrCode_GnssDriverGyroztoolarge = 0x3402,
   ErrCode_GnssDriverInterrupt = 0x3403,
   ErrCode_GnssDriverLacksatellite = 0x3404,
   ErrCode_GnssDriverDisconnect = 0x3801,
   ErrCode_GnssProtoSendError = 0x3405,
   ErrCode_GnssProtoReceiveError = 0x3406,

   /* Odom : 0x4000~0x4FFF*/
   ErrCode_OdomProtoSendError = 0x4401,
   ErrCode_OdomProtoReceiveError = 0x4402,

   /* Camera : 0x5000~0x5FFF*/

   /* Preprocessing : 0x10000~0x10FFF*/
   ErrCode_PreprocessingImuTimeout = 0x10401,
   ErrCode_PreprocessingOdomTimeout = 0x10402,
   ErrCode_PreprocessingGnssTimeout = 0x10403,
   ErrCode_PreprocessingLidarTimeout = 0x10404,
   ErrCode_PreprocessingLidarNumOverflow = 0x10405,
   ErrCode_PreprocessingUnknownLidarFrameId = 0x10406,
   ErrCode_PreProcessingCallbackTimeout = 0x10407,
   ErrCode_PreProcessingTimestampError = 0x10408,
   ErrCode_PreProcessingConfigError = 0x10801,

   /* Localization : 0x11000~0x11FFF*/
   ErrCode_LocalizationGeneralConfigWarning = 0x11401,

   ErrCode_LocalzationGeneralMapWarning = 0x11420,
   ErrCode_EMPTY_DISP_RS_MAP = 0x11409,

   ErrCode_LocalizationGeneralProgramWarning = 0x11440,

   ErrCode_LocalizationGeneralRunWarning = 0x11460,
   ErrCode_LocalizationProtobufSendError = 0x11461,
   ErrCode_IMU_OVERFLOW = 0x11463,
   ErrCode_LIDAR_OVERFLOW = 0x11464,
   ErrCode_ODOM_OVERFLOW = 0x11467,
   ErrCode_GNSS_OVERFLOW = 0x11468,
   ErrCode_MANY_GNSS_MSG_WAITING = 0x11469,
   ErrCode_GnssFailConnect = 0x1146A,
   ErrCode_LocalizationUnstable = 0x1146B,
   ErrCode_VehicleStateEmpty = 0x1146C,
   ErrCode_LidarFailObserve = 0x1146D,

   ErrCode_LocalizationGeneralConfigError = 0x11801,
   ErrCode_FailLoadingGlobalConfig = 0x11802,
   ErrCode_MissingCoreConfig = 0x11803,
   ErrCode_MapServerConfig = 0x11804,
   ErrCode_MsfUndefObserverType = 0x11807,
   ErrCode_MsfUndefObserverName = 0x11808,
   ErrCode_MsfMultiplePredictor = 0x11809,
   ErrCode_MsfWrongObserverTpye = 0x1180A,
   ErrCode_MsfUndefType = 0x1180B,
   ErrCode_MsfNoObserver = 0x1180C,
   ErrCode_ConfigurationError = 0x1180F,

   ErrCode_LocalzationGeneralMapError = 0x11820,
   ErrCode_MapServerLoadMap = 0x11821,
   ErrCode_MapServerEmptyOrigin = 0x11822,
   ErrCode_ObsvrMapServerNotMatch = 0x11823,

   ErrCode_LocalizationGeneralProgramError = 0x11840,
   ErrCode_MsfRunBeforeConfig = 0x11841,
   ErrCode_UsingUintRsloc = 0x11842,
   ErrCode_GridMapNotInit = 0x11843,
   ErrCode_RsmapNotInit = 0x11844,

   ErrCode_LocalizationGeneralRunError = 0x11860,
   ErrCode_MsfNoStateForUpdate = 0x11861,
   ErrCode_FailStartObserver = 0x11862,
   ErrCode_InitializationTimeout = 0x11863,
   ErrCode_InitializationFailed = 0x11864,
   ErrCode_LocalizationLost = 0x1186B,

   /* Perception : 0x12000~0x12FFF*/
   ErrCode_PerceptionConfigError = 0x12401,
   ErrCode_PerceptionInitError = 0x12402,
   ErrCode_PerceptionNullInput = 0x12403,
   ErrCode_PerceptionBadRotateCalibration = 0x12404,
   ErrCode_PerceptionBadHeightCalibration = 0x12405,
   ErrCode_PerceptionGroundInitFalse = 0x12406,
   ErrCode_PerceptionProtobufSendError = 0x12407,
   ErrCode_OdometryNoData = 0x12408,
   /*  Connection */
   ErrCode_ConnectionBroken = 0x13800,
   ErrCode_InitCommandPortError = 0x13801,
   ErrCode_InitExceptionPortError = 0x13802,
   ErrCode_InitStatusPortError = 0x13803,

   //IMU
   ErrCode_ImuSendReceiveConflict = 0x13831,
   ErrCode_InitImuReceiverError = 0x13832,
   ErrCode_InitImuSenderError = 0x13833,
   ErrCode_ImuReceiveError = 0x13834,
   ErrCode_ImuSendError = 0x13835,

   //GNSS
   ErrCode_GnssSendReceiveConflict = 0x13841,
   ErrCode_InitGnssReceiverError = 0x13842,
   ErrCode_InitGnssSenderError = 0x13843,
   ErrCode_GnssReceiveError = 0x13844,
   ErrCode_GnssSendError = 0x13845,

   //Points
   ErrCode_InitPointsReceiverError = 0x13851,
   ErrCode_PointsReceiveError = 0x13852,

   //Odom
   ErrCode_InitOdomSenderError = 0x13861,
   ErrCode_OdomSendError = 0x13862,

   //Pose
   ErrCode_InitPoseReceiverError = 0x13871,
   ErrCode_PoseReceiveError = 0x13872,

};

} // namespace common
} // namespace robosense
