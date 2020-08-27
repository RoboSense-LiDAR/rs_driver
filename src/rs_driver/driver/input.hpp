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
#define RS16_PCAP_SLEEP_DURATION 1150
#define RS32_PCAP_SLEEP_DURATION 500
#define RSBP_PCAP_SLEEP_DURATION 500
#define RS128_PCAP_SLEEP_DURATION 100
#define RS80_PCAP_SLEEP_DURATION 120

#include <rs_driver/common/common_header.h>
#include <rs_driver/common/error_code.h>
#include <rs_driver/driver/driver_param.h>
#include <rs_driver/msg/packet_msg.h>
using boost::asio::deadline_timer;
using boost::asio::ip::udp;
namespace robosense
{
namespace lidar
{
class Input
{
public:
  Input(const RSInputParam& input_param, const std::function<void(const Error&)>& excb);
  ~Input();
  bool init();
  bool start();
  void stop();
  void regRecvMsopCallback(const std::function<void(const PacketMsg&)>& callback);
  void regRecvDifopCallback(const std::function<void(const PacketMsg&)>& callback);
  void setLidarType(const LidarType& type);

private:
  inline bool setSocket(const std::string& pkt_type, const uint16_t& port);
  inline void getMsopPacket();
  inline void getDifopPacket();
  inline void getPcapPacket();
  inline void checkDifopDeadline();
  inline void checkMsopDeadline();
  static void handleReceive(const boost::system::error_code& ec, std::size_t length, boost::system::error_code* out_ec,
                            std::size_t* out_length);

private:
  LidarType lidar_type_;
  RSInputParam input_param_;
  std::function<void(const Error&)> excb_;
  bool init_flag_;
  /* pcap file parse */
  pcap_t* pcap_;
  bpf_program pcap_msop_filter_;
  bpf_program pcap_difop_filter_;
  /* live socket */
  std::unique_ptr<udp::socket> msop_sock_ptr_;
  std::unique_ptr<udp::socket> difop_sock_ptr_;
  std::unique_ptr<deadline_timer> msop_deadline_;
  std::unique_ptr<deadline_timer> difop_deadline_;
  Thread msop_thread_;
  Thread difop_thread_;
  Thread pcap_thread_;
  boost::asio::io_service msop_io_service_;
  boost::asio::io_service difop_io_service_;
  std::vector<std::function<void(const PacketMsg&)>> difop_cb_;
  std::vector<std::function<void(const PacketMsg&)>> msop_cb_;
};

inline Input::Input(const RSInputParam& input_param, const std::function<void(const Error&)>& excb)
  : lidar_type_(LidarType::RS128), input_param_(input_param), excb_(excb), init_flag_(false), pcap_(nullptr)
{
}

inline Input::~Input()
{
  stop();
  if (pcap_ != NULL)
  {
    pcap_close(pcap_);
  }
}

inline bool Input::init()
{
  if (input_param_.read_pcap)
  {
    char errbuf[PCAP_ERRBUF_SIZE];
    if ((pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf)) == NULL)
    {
      excb_(Error(ErrCode_PcapWrongDirectory));
      return false;
    }
    else
    {
      std::stringstream msop_filter;
      std::stringstream difop_filter;
      msop_filter << "src host " << input_param_.device_ip << " && ";
      msop_filter << "udp dst port " << input_param_.msop_port;
      difop_filter << "src host " << input_param_.device_ip << " && ";
      difop_filter << "udp dst port " << input_param_.difop_port;
      pcap_compile(pcap_, &pcap_msop_filter_, msop_filter.str().c_str(), 1, 0xFFFFFFFF);
      pcap_compile(pcap_, &pcap_difop_filter_, difop_filter.str().c_str(), 1, 0xFFFFFFFF);
    }
  }
  else
  {
    if (!setSocket("msop", input_param_.msop_port) || !setSocket("difop", input_param_.difop_port))
    {
      return false;
    }
  }
  init_flag_ = true;
  return true;
}

inline bool Input::start()
{
  if (!init_flag_)
  {
    excb_(Error(ErrCode_StartBeforeInit));
    return false;
  }
  if (!input_param_.read_pcap)
  {
    msop_thread_.start_.store(true);
    difop_thread_.start_.store(true);
    msop_thread_.thread_.reset(new std::thread([this]() { getMsopPacket(); }));
    difop_thread_.thread_.reset(new std::thread([this]() { getDifopPacket(); }));
  }
  else
  {
    pcap_thread_.start_.store(true);
    pcap_thread_.thread_.reset(new std::thread([this]() { getPcapPacket(); }));
  }
  return true;
}

inline void Input::stop()
{
  if (!input_param_.read_pcap)
  {
    msop_thread_.start_.store(false);
    difop_thread_.start_.store(false);
    if (msop_thread_.thread_ != nullptr && msop_thread_.thread_->joinable())
    {
      msop_thread_.thread_->join();
    }
    if (difop_thread_.thread_ != nullptr && difop_thread_.thread_->joinable())
    {
      difop_thread_.thread_->join();
    }
  }
  else
  {
    pcap_thread_.start_.store(false);
    if (pcap_thread_.thread_ != nullptr && pcap_thread_.thread_->joinable())
    {
      pcap_thread_.thread_->join();
    }
  }
}

inline void Input::regRecvMsopCallback(const std::function<void(const PacketMsg&)>& callback)
{
  msop_cb_.emplace_back(callback);
}

inline void Input::regRecvDifopCallback(const std::function<void(const PacketMsg&)>& callback)
{
  difop_cb_.emplace_back(callback);
}

inline void Input::setLidarType(const LidarType& type)
{
  lidar_type_ = type;
}

inline bool Input::setSocket(const std::string& pkt_type, const uint16_t& port)
{
  if (pkt_type == "msop")
  {
    try
    {
      msop_sock_ptr_.reset(new udp::socket(msop_io_service_, udp::endpoint(udp::v4(), port)));
      msop_deadline_.reset(new deadline_timer(msop_io_service_));
    }
    catch (...)
    {
      excb_(Error(ErrCode_MsopPortBuzy));
      return false;
    }
    msop_deadline_->expires_at(boost::posix_time::pos_infin);
    checkMsopDeadline();
  }
  else if (pkt_type == "difop")
  {
    try
    {
      difop_sock_ptr_.reset(new udp::socket(difop_io_service_, udp::endpoint(udp::v4(), port)));
      difop_deadline_.reset(new deadline_timer(difop_io_service_));
    }
    catch (...)
    {
      excb_(Error(ErrCode_DifopPortBuzy));
      return false;
    }
    difop_deadline_->expires_at(boost::posix_time::pos_infin);
    checkDifopDeadline();
  }
  return true;
}

inline void Input::getMsopPacket()
{
  char* precv_buffer = (char*)malloc(RSLIDAR_PKT_LEN);
  while (msop_thread_.start_.load())
  {
    msop_deadline_->expires_from_now(boost::posix_time::seconds(1));
    boost::system::error_code ec = boost::asio::error::would_block;
    std::size_t ret = 0;

    msop_sock_ptr_->async_receive(boost::asio::buffer(precv_buffer, RSLIDAR_PKT_LEN),
                                  boost::bind(&Input::handleReceive, _1, _2, &ec, &ret));
    do
    {
      msop_io_service_.run_one();
    } while (ec == boost::asio::error::would_block);
    if (ec)
    {
      excb_(Error(ErrCode_MsopPktTimeout));
      continue;
    }
    if (ret < RSLIDAR_PKT_LEN)
    {
      excb_(Error(ErrCode_MsopPktIncomplete));
      continue;
    }
    PacketMsg msg;
    memcpy(msg.packet.data(), precv_buffer, RSLIDAR_PKT_LEN);
    for (auto& iter : msop_cb_)
    {
      iter(msg);
    }
  }
  free(precv_buffer);
}

inline void Input::getDifopPacket()
{
  while (difop_thread_.start_.load())
  {
    char* precv_buffer = (char*)malloc(RSLIDAR_PKT_LEN);

    difop_deadline_->expires_from_now(boost::posix_time::seconds(2));
    boost::system::error_code ec = boost::asio::error::would_block;
    std::size_t ret = 0;

    difop_sock_ptr_->async_receive(boost::asio::buffer(precv_buffer, RSLIDAR_PKT_LEN),
                                   boost::bind(&Input::handleReceive, _1, _2, &ec, &ret));
    do
    {
      difop_io_service_.run_one();
    } while (ec == boost::asio::error::would_block);
    if (ec)
    {
      free(precv_buffer);
      excb_(Error(ErrCode_DifopPktTimeout));
      continue;
    }
    if (ret < RSLIDAR_PKT_LEN)
    {
      free(precv_buffer);
      excb_(Error(ErrCode_DifopPktIncomplete));
      continue;
    }
    PacketMsg msg;
    memcpy(msg.packet.data(), precv_buffer, RSLIDAR_PKT_LEN);
    for (auto& iter : difop_cb_)
    {
      iter(msg);
    }
    free(precv_buffer);
  }
}

inline void Input::getPcapPacket()
{
  while (pcap_thread_.start_.load())
  {
    struct pcap_pkthdr* header;
    const u_char* pkt_data;
    switch (lidar_type_)
    {
      case LidarType::RS16:
        usleep(RS16_PCAP_SLEEP_DURATION / input_param_.pcap_rate);
        break;
      case LidarType::RS32:
        usleep(RS32_PCAP_SLEEP_DURATION / input_param_.pcap_rate);
        break;
      case LidarType::RSBP:
        usleep(RSBP_PCAP_SLEEP_DURATION / input_param_.pcap_rate);
        break;
      case LidarType::RS128:
        usleep(RS128_PCAP_SLEEP_DURATION / input_param_.pcap_rate);
        break;
      case LidarType::RS80:
        usleep(RS80_PCAP_SLEEP_DURATION / input_param_.pcap_rate);
        break;
      default:
        break;
    }
    if (!pcap_thread_.start_.load())
    {
      break;
    }
    if (pcap_next_ex(pcap_, &header, &pkt_data) >= 0)
    {
      if (!input_param_.device_ip.empty() && (0 != pcap_offline_filter(&pcap_msop_filter_, header, pkt_data)))
      {
        PacketMsg msg;
        memcpy(msg.packet.data(), pkt_data + 42, RSLIDAR_PKT_LEN);
        for (auto& iter : msop_cb_)
        {
          iter(msg);
        }
      }
      else if (!input_param_.device_ip.empty() && (0 != pcap_offline_filter(&pcap_difop_filter_, header, pkt_data)))
      {
        PacketMsg msg;
        memcpy(msg.packet.data(), pkt_data + 42, RSLIDAR_PKT_LEN);
        for (auto& iter : difop_cb_)
        {
          iter(msg);
        }
      }
      else
      {
        continue;
      }
    }
    else
    {
      if (input_param_.pcap_repeat)
      {
        excb_(Error(ErrCode_PcapRepeat));
        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
      }
      else
      {
        excb_(Error(ErrCode_PcapExit));
        break;
      }
    }
  }
}

inline void Input::checkDifopDeadline()
{
  if (difop_deadline_->expires_at() <= deadline_timer::traits_type::now())
  {
    difop_sock_ptr_->cancel();
    difop_deadline_->expires_at(boost::posix_time::pos_infin);
  }
  difop_deadline_->async_wait(boost::bind(&Input::checkDifopDeadline, this));
}

inline void Input::checkMsopDeadline()
{
  if (msop_deadline_->expires_at() <= deadline_timer::traits_type::now())
  {
    msop_sock_ptr_->cancel();
    msop_deadline_->expires_at(boost::posix_time::pos_infin);
  }
  msop_deadline_->async_wait(boost::bind(&Input::checkMsopDeadline, this));
}

inline void Input::handleReceive(const boost::system::error_code& ec, std::size_t length,
                                 boost::system::error_code* out_ec, std::size_t* out_length)
{
  *out_ec = ec;
  *out_length = length;
}

}  // namespace lidar
}  // namespace robosense
