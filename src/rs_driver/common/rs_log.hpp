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

#pragma once

#include <iostream>

#ifdef _WIN32
    #include <WinSock2.h>
    #include <windows.h>

    void inline enableANSI() {
        static bool enable = false;
        if(!enable)
        {
            HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
            DWORD dwMode = 0;
            GetConsoleMode(hOut, &dwMode);
            dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
            SetConsoleMode(hOut, dwMode);
            enable = true;
        }
    }

    #define ENABLE_ANSI() enableANSI()

    #define RS_ERROR   ENABLE_ANSI(); std::cout << "\033[1m\033[31m"  // bold red
    #define RS_WARNING ENABLE_ANSI(); std::cout << "\033[1m\033[33m"  // bold yellow
    #define RS_INFO    ENABLE_ANSI(); std::cout << "\033[1m\033[32m"  // bold green
    #define RS_INFOL   ENABLE_ANSI(); std::cout << "\033[32m"         // green
    #define RS_DEBUG   ENABLE_ANSI(); std::cout << "\033[1m\033[36m"  // bold cyan
    #define RS_REND    "\033[0m" << std::endl

    #define RS_TITLE   ENABLE_ANSI(); std::cout << "\033[1m\033[35m"  // bold magenta
    #define RS_MSG     ENABLE_ANSI(); std::cout << "\033[1m\033[37m"  // bold white
#else
    #define RS_ERROR   std::cout << "\033[1m\033[31m"  // bold red
    #define RS_WARNING std::cout << "\033[1m\033[33m"  // bold yellow
    #define RS_INFO    std::cout << "\033[1m\033[32m"  // bold green
    #define RS_INFOL   std::cout << "\033[32m"         // green
    #define RS_DEBUG   std::cout << "\033[1m\033[36m"  // bold cyan
    #define RS_REND    "\033[0m" << std::endl

    #define RS_TITLE   std::cout << "\033[1m\033[35m"  // bold magenta
    #define RS_MSG     std::cout << "\033[1m\033[37m"  // bold white
#endif

