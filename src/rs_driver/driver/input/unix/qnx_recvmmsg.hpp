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

#ifdef __QNX__
#include <sys/time.h> 
#include <sys/socket.h> 
#include <cerrno>  
 
// QNX fallback definition for mmsghdr/recvmmsg.
//
// Keep this block disabled by default to avoid redefinition conflicts on
// platforms / SDK versions where mmsghdr or recvmmsg is already provided
// by the system headers or libc.
//
// Enable this block only when building on a target that does not provide
// recvmmsg-compatible declarations / implementations.   
#if 0
struct mmsghdr {
    struct msghdr msg_hdr;
    unsigned int  msg_len;
};

static int recvmmsg(int sockfd, struct mmsghdr *msgvec, unsigned int vlen, int flags, struct timespec *timeout)
{
    unsigned int i = 0;
    for (i = 0; i < vlen; ++i)
    {
        ssize_t ret = recvmsg(sockfd, &msgvec[i].msg_hdr, flags);
        if (ret < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                if (i > 0) {
                    return i; 
                }
                return -1; 
            }
            if (errno == EINTR) {
                if (i > 0) return i;
                i--; 
                continue; 
            }
            if (i == 0) return -1;
            return i; 
        }
        msgvec[i].msg_len = ret;
    }
    return i; 
}
#endif
#endif