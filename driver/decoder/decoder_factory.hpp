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
#include "driver/decoder/decoder_16.hpp"
#include "driver/decoder/decoder_32.hpp"
#include "driver/decoder/decoder_128.hpp"
#include "driver/decoder/decoder_bp.hpp"
namespace robosense
{
namespace sensor
{
template <typename vpoint>
class DecoderFactory
{

public:
    inline static std::shared_ptr<DecoderBase<vpoint>> createDecoder(const std::string &lidar_type, RSDecoder_Param param)
    {
        if (lidar_type == "RS16")
        {
            return std::make_shared<Decoder16<vpoint>>(param);
        }
        else if (lidar_type == "RS32")
        {
            return std::make_shared<Decoder32<vpoint>>(param);
        }
        else if (lidar_type == "RSBP")
        {
            return std::make_shared<DecoderBP<vpoint>>(param);
        }
        else if (lidar_type == "RS128")
        {
            return std::make_shared<Decoder128<vpoint>>(param);
        }
        else
        {
    //        rs_print(RS_ERROR, "[RSFCT] Wrong lidar type: %s, Please set RS16 or RS32 or RSBP or RS128 !", lidar_type.c_str());
            // ERROR<<"Wrong lidar type : "<<lidar_type<<REND;
            // ERROR<<"Please set RS16 or RS32 or RSBP or RS128 ! "<<REND;
            exit(-1);
        }
    }
};

} // namespace sensor
} // namespace robosense
