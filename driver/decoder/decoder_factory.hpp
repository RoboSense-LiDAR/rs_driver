
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
