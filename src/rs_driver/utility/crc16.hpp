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

#include <cstddef>
#include <cstdint>
#include <vector>

#define CRC_POLY_16 0xA001

namespace robosense
{
namespace lidar
{
    bool crc_tab16_init = false;
    uint16_t crc_tab16[256] = {};

    void init_crc16_tab(void)
    {
        uint16_t i;
        uint16_t j;
        uint16_t crc;
        uint16_t c;

        for (i = 0; i < 256; i++)
        {

            crc = 0;
            c = i;

            for (j = 0; j < 8; j++)
            {

                if ((crc ^ c) & 0x0001)
                    crc = (crc >> 1) ^ CRC_POLY_16;
                else
                    crc = crc >> 1;

                c = c >> 1;
            }

            crc_tab16[i] = crc;
        }

        crc_tab16_init = true;
    }

    uint16_t crc_16(const uint8_t *input_str, int length)
    {

        if (!crc_tab16_init)
            init_crc16_tab();

        uint16_t crc = 0;
        const uint8_t *ptr = input_str;

        if (ptr != NULL)
            for (int i = 0; i < length; i++)
            {
                crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
            }

        return crc;
    }
}  // namespace lidar
}  // namespace robosense
