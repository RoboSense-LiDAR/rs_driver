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
#pragma once
#include <rs_driver/common/common_header.h>
namespace robosense
{
namespace lidar
{
using namespace boost::asio;
class SerialCommunicator
{
public:
  SerialCommunicator()=default;
  ~SerialCommunicator() = default;
  bool init(const RSSerialParam& param);
private:
  serial_port serial_;

};
bool SerialCommunicator::init(const RSSerialParam& param)
{
  serial_.open(param.port_name);
  serial_.set_option(serial_port_base::baud_rate(param.baudrate));
  serial_.set_option(serial_port_base::character_size(param.character_size));
  serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
  serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

  //serial_.write_some(buffer(&hexValue, sizeof(uint8_t)));
}

}  // namespace lidar
}  // namespace robosense