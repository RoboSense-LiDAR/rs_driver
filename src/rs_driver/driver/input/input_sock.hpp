
#pragma once

#ifdef __linux__

#ifdef ENABLE_EPOLL_RECEIVE
#include <rs_driver/driver/input/unix/input_sock_epoll.hpp>
#else
#include <rs_driver/driver/input/unix/input_sock_select.hpp>
#endif

#elif _WIN32

#include <rs_driver/driver/input/win/input_sock_select.hpp>

#endif

