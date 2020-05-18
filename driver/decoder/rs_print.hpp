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
#ifndef __RSLIDAR_PRINT_H__
#define __RSLIDAR_PRINT_H__
#include <ctime>
#include <cstdio>

namespace robosense
{

typedef enum
{
    RS_DEBUG,
    RS_INFO,
    RS_WARNING,
    RS_ERROR,
    RS_FATAL
}T_log_rank;
   
#define RS_NONE     "\033[m"
#define RS_GREEN    "\033[0;32;32m"
#define RS_YELLOW   "\033[0;33m"
#define RS_PURPLE   "\033[0;35m"
#define RS_RED      "\033[0;31m"


#define rs_print(level, fmt, arg...)     \
    do {    \
        struct timespec ts; \
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);    \
        switch (level)  \
        {   \
        case RS_DEBUG:  \
        printf("[%ld.%ld : %s(%d)] " fmt "\n", ts.tv_sec, ts.tv_nsec, \
                __FILE__, __LINE__, ##arg);   \
            break;  \
        case RS_INFO:  \
            printf(RS_GREEN "[%ld.%ld : %s(%d)] " fmt "\n" RS_NONE, ts.tv_sec, ts.tv_nsec,  \
                __FILE__, __LINE__, ##arg);   \
            break;  \
        case RS_WARNING:  \
            printf(RS_YELLOW "[%ld.%ld : %s(%d)] " fmt "\n" RS_NONE, ts.tv_sec, ts.tv_nsec,  \
                __FILE__, __LINE__, ##arg);   \
            break;  \
        case RS_ERROR:  \
            printf(RS_PURPLE "[%ld.%ld : %s(%d)] " fmt "\n" RS_NONE, ts.tv_sec, ts.tv_nsec, \
                __FILE__, __LINE__, ##arg);   \
            break;  \
        case RS_FATAL:  \
            printf(RS_RED "[%ld.%ld : %s(%d)] " fmt "\n" RS_NONE, ts.tv_sec, ts.tv_nsec, \
                __FILE__, __LINE__, ##arg);   \
            break;  \
        default:    \
            printf("[%ld.%ld : %s(%d)] " fmt "\n", ts.tv_sec, ts.tv_nsec, \
                __FILE__, __LINE__, ##arg);   \
            break;  \
        }   \
    } while(0)

}

#endif
