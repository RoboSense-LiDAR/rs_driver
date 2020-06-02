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
#include <rs_driver/driver/decoder/decoder_16.hpp>
#include <rs_driver/driver/decoder/decoder_32.hpp>
#include <rs_driver/driver/decoder/decoder_128.hpp>
#include <rs_driver/driver/decoder/decoder_bp.hpp>
namespace robosense
{
    namespace lidar
    {
        template <typename vpoint>
        class DecoderFactory
        {

        public:
            inline static std::shared_ptr<DecoderBase<vpoint>> createDecoder(const LidarType &_lidar_type, const RSDecoderParam &_param)
            {
                switch (_lidar_type)
                {
                case LidarType::RS16:
                    return std::make_shared<Decoder16<vpoint>>(_param);
                    break;
                case LidarType::RS32:
                    return std::make_shared<Decoder32<vpoint>>(_param);
                    break;
                case LidarType::RSBP:
                    return std::make_shared<DecoderBP<vpoint>>(_param);
                    break;
                case LidarType::RS128:
                    return std::make_shared<Decoder128<vpoint>>(_param);
                    break;
                default:
                    assert(false);
                }
            }
        };

    } // namespace lidar
} // namespace robosense
