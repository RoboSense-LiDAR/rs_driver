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
template <typename T>
class Queue
{
public:
  Queue()
  {
    is_task_finished_ = true;
  }

  T front()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_quque_.front();
  }

  void push(const T& value)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_quque_.push(value);
  }

  void pop()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_quque_.empty())
    {
      m_quque_.pop();
    }
  }

  T popFront()
  {
    T value;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_quque_.empty())
    {
      value = std::move(m_quque_.front());
      m_quque_.pop();
    }
    return value;
  }

  void clear()
  {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lock(m_mutex);
    swap(empty, m_quque_);
  }

  size_t size()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_quque_.size();
  }

public:
  std::queue<T> m_quque_;
  std::atomic<bool> is_task_finished_;

private:
  mutable std::mutex m_mutex;
};
}  // namespace lidar
}  // namespace robosense