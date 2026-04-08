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

#include <thread>
#include <atomic>

namespace robosense
{
namespace lidar
{
template <typename T, size_t Capacity = 102400>
class SyncQueue
{
public:
  SyncQueue() : head_(0), tail_(0)
  {
  }

  inline size_t push(const T& value)
  {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    size_t next_tail = (current_tail + 1) % Capacity;

    if (next_tail == head_.load(std::memory_order_acquire))
    {
      return -1;
    }

    buffer_[current_tail] = value;
    tail_.store(next_tail, std::memory_order_release);

    return size();
  }

  inline T pop()
  {
    T value;
    size_t current_head = head_.load(std::memory_order_relaxed);

    if (current_head == tail_.load(std::memory_order_acquire))
    {
      // RS_WARNING << "pop()::SyncQueue is empty, return default value" << RS_REND;
      return value;
    }

    value = buffer_[current_head];
    head_.store((current_head + 1) % Capacity, std::memory_order_release);

    return value;
  }

  inline T popWait(unsigned int usec = 1000000)
  {
    T value;
    auto start_time = std::chrono::steady_clock::now();

    while (true)
    {
      size_t current_head = head_.load(std::memory_order_relaxed);

      if (current_head != tail_.load(std::memory_order_acquire))
      {
        value = buffer_[current_head];
        head_.store((current_head + 1) % Capacity, std::memory_order_release);
        return value;
      }

      auto now = std::chrono::steady_clock::now();
      int64_t time_diff = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
      if (time_diff >= usec)
      {
        return value;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  inline void clear()
  {
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
  }

  inline size_t size() const
  {
    size_t head = head_.load(std::memory_order_acquire);
    size_t tail = tail_.load(std::memory_order_acquire);
    return (tail >= head) ? (tail - head) : (Capacity - head + tail);
  }
  inline size_t popBatch(std::vector<T>& out_batch, size_t max_count, unsigned int usec = 1000000)
  {
    auto start_time = std::chrono::steady_clock::now();
    while (true)
    {
      if (head_.load(std::memory_order_relaxed) != tail_.load(std::memory_order_acquire))
      {
        break; 
      }
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count() >= usec)
      {
        return 0; 
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    size_t current_head = head_.load(std::memory_order_relaxed);
    size_t current_tail = tail_.load(std::memory_order_acquire); 

    size_t available;
    if (current_tail >= current_head)
    {
      available = current_tail - current_head;
    }
    else
    {
      available = Capacity - current_head + current_tail;
    }
    size_t real_count = (max_count < available) ? max_count : available;
    if (out_batch.capacity() < out_batch.size() + real_count)
    {
      out_batch.reserve(out_batch.size() + real_count);
    }
    for (size_t i = 0; i < real_count; ++i)
    {
      size_t index = (current_head + i) % Capacity;
      out_batch.emplace_back(buffer_[index]);
    }
    head_.store((current_head + real_count) % Capacity, std::memory_order_release);

    return real_count;
  }
private:
  T buffer_[Capacity];
  alignas(64) std::atomic<size_t> head_;
  alignas(64) std::atomic<size_t> tail_;
};

}  // namespace lidar
}  // namespace robosense