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
#include <cstdint>
#include <cstddef>
#include <cstring>

#if defined ENABLE_SSSE3 || defined ENABLE_AVX2
#include <immintrin.h>
#include <tmmintrin.h> // For SSSE3 intrinsics
#endif

#if defined ENABLE_ARM_NEON || defined ENABLE_ARM64
#include <arm_neon.h>
#endif
namespace robosense
{
namespace lidar
{
   /**
     * @brief Optimized generic implementation with loop unrolling
     * @param src Source byte array
     * @param dst Destination byte array
     * @param total_src_bytes Total number of bytes in the source(must be a multiple of 32)
     * @return Number of bytes copied to destination
     */
    inline size_t copy_3of4_bytes_generic(const uint8_t* src, uint8_t* dst, size_t total_src_bytes) {
        if (!src || !dst || total_src_bytes == 0 || total_src_bytes % 32 != 0) {
            return 0;
        }
        
        const size_t groups = total_src_bytes / 4;
        const size_t total_dst_bytes = groups * 3;

        size_t i = 0;
        // Process 8 groups at a time for better cache utilization
        for (; i <= groups - 8; i += 8) {
            // Group 1
            dst[i * 3 + 2] = src[i * 4];
            dst[i * 3 + 1] = src[i * 4 + 1];
            dst[i * 3 + 0] = src[i * 4 + 2];
            // Group 2
            dst[(i + 1) * 3 + 2] = src[(i + 1) * 4];
            dst[(i + 1) * 3 + 1] = src[(i + 1) * 4 + 1];
            dst[(i + 1) * 3 + 0] = src[(i + 1) * 4 + 2];
            // Group 3
            dst[(i + 2) * 3 + 2] = src[(i + 2) * 4];
            dst[(i + 2) * 3 + 1] = src[(i + 2) * 4 + 1];
            dst[(i + 2) * 3 + 0] = src[(i + 2) * 4 + 2];
            // Group 4
            dst[(i + 3) * 3 + 2] = src[(i + 3) * 4];
            dst[(i + 3) * 3 + 1] = src[(i + 3) * 4 + 1];
            dst[(i + 3) * 3 + 0] = src[(i + 3) * 4 + 2];
            // Group 5
            dst[(i + 4) * 3 + 2] = src[(i + 4) * 4];    
            dst[(i + 4) * 3 + 1] = src[(i + 4) * 4 + 1];
            dst[(i + 4) * 3 + 0] = src[(i + 4) * 4 + 2];
            // Group 6
            dst[(i + 5) * 3 + 2] = src[(i + 5) * 4];
            dst[(i + 5) * 3 + 1] = src[(i + 5) * 4 + 1];
            dst[(i + 5) * 3 + 0] = src[(i + 5) * 4 + 2];
            // Group 7
            dst[(i + 6) * 3 + 2] = src[(i + 6) * 4];
            dst[(i + 6) * 3 + 1] = src[(i + 6) * 4 + 1];
            dst[(i + 6) * 3 + 0] = src[(i + 6) * 4 + 2];
            // Group 8
            dst[(i + 7) * 3 + 2] = src[(i + 7) * 4];
            dst[(i + 7) * 3 + 1] = src[(i + 7) * 4 + 1];
            dst[(i + 7) * 3 + 0] = src[(i + 7) * 4 + 2];
        }

        return total_dst_bytes;
    }
        
    // Fallback to generic version if SIMD acceleration is disabled
    // x86 platform optimizations
#if defined ENABLE_SSSE3
    /**
     * @brief SSSE3-optimized implementation for x86 platforms, using 128-bit SIMD
     * @param src Source byte array
     * @param dst Destination byte array
     * @param total_src_bytes Total number of bytes in the source(must be a multiple of 16)
     * @return Number of bytes copied to destination
     */
    inline size_t copy_3of4_bytes_ssse3(const uint8_t* src, uint8_t* dst, size_t total_src_bytes) {
        if (!src || !dst || total_src_bytes == 0 || total_src_bytes % 16 != 0) {
            return 0;
        }
        const uint8_t* src_end = src + total_src_bytes;
        const size_t total_dst_bytes = total_src_bytes / 4 * 3;
        // Shuffle mask: Extracts [2,1,0] from each 4-byte segment, ignoring the 4th byte of each group
        const __m128i shuffle_mask = _mm_set_epi8(
            0xFF, 0xFF, 0xFF, 0xFF,  // Ignore bytes 12-15 (invalid)
            12, 13, 14,              // Bytes 9-11: src[14], src[13], src[12] (4th group)
            8, 9, 10,                // Bytes 6-8: src[10], src[9], src[8] (3rd group)
            4, 5, 6,                 // Bytes 3-5: src[6], src[5], src[4] (2nd group)
            0, 1, 2                  // Bytes 0-2: src[2], src[1], src[0] (1st group)
        );
        
        while (src + 16 <= src_end) {
            __m128i data = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src));
            __m128i shuffled = _mm_shuffle_epi8(data, shuffle_mask);  // Reorder bytes using the mask
            
            // Store the first 12 valid bytes, ignoring the last 4 padding bytes
            _mm_storeu_si128(reinterpret_cast<__m128i*>(dst), shuffled);
            
            src += 16;
            dst += 12;  // Each iteration outputs 12 bytes (3 bytes × 4 groups)
        }
        return total_dst_bytes;
    }
    
#endif // ENABLE_SSSE3

#ifdef ENABLE_AVX2
    /**
     * @brief AVX2-optimized implementation for x86 platforms, using 256-bit SIMD (processes 32 bytes at a time).
     * @param src Source byte array
     * @param dst Destination byte array
     * @param total_src_bytes Total number of bytes in the source (must be a multiple of 16).
     * @return Number of bytes copied to destination
     */
    inline size_t copy_3of4_bytes_avx2(const uint8_t* src, uint8_t* dst, size_t total_src_bytes) {
        if (!src || !dst || total_src_bytes == 0 || total_src_bytes % 16 != 0) {
            return 0;
        }
        
        const uint8_t* src_end = src + total_src_bytes;
        const size_t total_dst_bytes = total_src_bytes / 4 * 3;
        // Shuffle mask for AVX2: Extracts [2,1,0] from each 4-byte segment across 32 bytes
        const __m256i shuffle_mask = _mm256_set_epi8(
            // Upper 128 bits (processes bytes 16-31, structured for alignment)
            0xFF, 0xFF, 0xFF, 0xFF, 12, 13, 14, 8, 9, 10, 4, 5, 6, 0, 1, 2,
            // Lower 128 bits (processes bytes 0-15)
            0xFF, 0xFF, 0xFF, 0xFF, 12, 13, 14, 8, 9, 10, 4, 5, 6, 0, 1, 2
        );
        
        while (src + 32 <= src_end) {
            __m256i data = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(src));
            __m256i shuffled = _mm256_shuffle_epi8(data, shuffle_mask);
            
            // Extract and store the lower and upper 128-bit segments (each contains 12 valid bytes)
            __m128i low = _mm256_extracti128_si256(shuffled, 0);
            __m128i high = _mm256_extracti128_si256(shuffled, 1);
            
            _mm_storeu_si128(reinterpret_cast<__m128i*>(dst), low);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + 12), high);
            
            src += 32;
            dst += 24;
        }
        return total_dst_bytes;
    }
#endif // ENABLE_AVX2

// ARM platform optimizations
#ifdef ENABLE_ARM_NEON
    
    /**
     * @brief NEON-optimized implementation for ARM platforms, using 128-bit SIMD
     * @param src Source byte array
     * @param dst Destination byte array
     * @param total_src_bytes Total number of bytes in the source
     * @return Number of bytes copied to destination
     */
    inline size_t copy_3of4_bytes_neon(const uint8_t* src, uint8_t* dst, size_t total_src_bytes) {
        if (!src || !dst || total_src_bytes == 0 || total_src_bytes % 16 != 0) {
            return 0;
        }
        
        const uint8_t* src_end = src + total_src_bytes;
        const size_t total_dst_bytes = total_src_bytes / 4 * 3;
        // NEON shuffle mask: Extracts [2,1,0] from each 4-byte segment
        const uint8_t mask_array[16] = {2, 1, 0, 6, 5, 4, 10, 9, 8, 14, 13, 12, 0, 0, 0, 0};
        const uint8x16_t shuffle_mask = vld1q_u8(mask_array);
        
        while (src + 16 <= src_end) {
            uint8x16_t data = vld1q_u8(src);
            uint8x16_t shuffled = vqtbl1q_u8(data, shuffle_mask); // Reorder bytes using the table lookup
            vst1q_u8(dst, shuffled);
            
            src += 16;
            dst += 12;
        }
        return total_dst_bytes;
    }
#endif // ENABLE_ARM_NEON

#ifdef ENABLE_ARM64
  /**
     * @brief NEON-optimized implementation for ARM64 platforms, processing 32 bytes at a time
     * @param src Source byte array
     * @param dst Destination byte array
     * @param total_src_bytes Total number of bytes in the source
     * @return Number of bytes copied to destination
     */
    inline size_t copy_3of4_bytes_neon_arm64(const uint8_t* src, uint8_t* dst, size_t total_src_bytes) {
        if (!src || !dst || total_src_bytes == 0 || total_src_bytes % 16 != 0) {
            return 0;
        }
        const size_t total_dst_bytes = total_src_bytes / 4 * 3;
        // Process 32-byte blocks first
        size_t neon_bytes = (total_src_bytes / 32) * 32;
        const uint8_t* src_end_neon = src + neon_bytes;
        
        const uint8_t mask_array[16] = {2, 1, 0, 6, 5, 4, 10, 9, 8, 14, 13, 12, 0, 0, 0, 0};
        const uint8x16_t shuffle_mask = vld1q_u8(mask_array);
        
        // Process two 16-byte blocks in parallel
        while (src + 32 <= src_end_neon) {
            uint8x16_t data1 = vld1q_u8(src);
            uint8x16_t data2 = vld1q_u8(src + 16);
            
            uint8x16_t shuffled1 = vqtbl1q_u8(data1, shuffle_mask);
            uint8x16_t shuffled2 = vqtbl1q_u8(data2, shuffle_mask);
            
            vst1q_u8(dst, shuffled1);
            vst1q_u8(dst + 12, shuffled2);
            
            src += 32;
            dst += 24;
        }
        
        // Handle remaining bytes with the generic NEON implementation
        if (src < src + total_src_bytes) {
            copy_3of4_bytes_neon(src, dst, total_src_bytes - neon_bytes);
        }
        return total_dst_bytes;
    }
    #endif // ENABLE_ARM64

/**
 * @brief Dispatcher function that selects the best implementation based on compile-time flags
 * @param src Source byte array
 * @param dst Destination byte array
 * @param total_src_bytes Total number of bytes in the source
 * @return Number of bytes copied to destination
*/
inline size_t copy_3of4_bytes_optimized(const uint8_t* src, uint8_t* dst, size_t total_src_bytes) {

    #if defined(ENABLE_AVX2)
        return copy_3of4_bytes_avx2(src, dst, total_src_bytes);
    #elif defined(ENABLE_SSSE3)
        return copy_3of4_bytes_ssse3(src, dst, total_src_bytes);
    #elif defined(ENABLE_ARM64)
        return copy_3of4_bytes_neon_arm64(src, dst, total_src_bytes);
    #elif defined(ENABLE_ARM_NEON)
        return copy_3of4_bytes_neon(src, dst, total_src_bytes);
    #else
        // Fallback to the generic optimized version if no SIMD is available
        return copy_3of4_bytes_generic(src, dst, total_src_bytes);
    #endif
    }
}
}