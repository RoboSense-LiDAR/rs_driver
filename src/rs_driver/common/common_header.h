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
/*Common*/
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <math.h>
#include <memory>
#include <array>
#include <algorithm>
#include <functional>
#include <iterator>
#include <chrono>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <stdexcept>
#include <mutex>
#include <type_traits>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <rs_driver/macro/version.h>
/*Linux*/
#ifdef __GNUC__
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

/*Pcap*/
#include <pcap.h>

/*Camera*/
typedef std::pair<std::string, double> CameraTrigger;

/*Packet Length*/
#ifndef RSLIDAR_PKT_LEN
#define RSLIDAR_PKT_LEN 1248
#endif
/*Output style*/
#ifndef RS_INFOL
#define RS_INFOL (std::cout << "\033[32m")
#endif
#ifndef RS_INFO
#define RS_INFO (std::cout << "\033[1m\033[32m")
#endif
#ifndef RS_WARNING
#define RS_WARNING (std::cout << "\033[1m\033[33m")
#endif
#ifndef RS_ERROR
#define RS_ERROR (std::cout << "\033[1m\033[31m")
#endif
#ifndef RS_DEBUG
#define RS_DEBUG (std::cout << "\033[1m\033[36m")
#endif
#ifndef RS_TITLE
#define RS_TITLE (std::cout << "\033[1m\033[35m")
#endif
#ifndef RS_MSG
#define RS_MSG (std::cout << "\033[1m\033[37m")
#endif
#ifndef RS_REND
#define RS_REND "\033[0m" << std::endl
#endif

