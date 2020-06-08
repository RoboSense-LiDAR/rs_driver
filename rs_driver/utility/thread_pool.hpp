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
const int MAX_THREAD_NUM = 4;
struct Thread
{
  Thread()
  {
    start = false;
  }
  std::shared_ptr<std::thread> m_thread;
  std::atomic<bool> start;
};
class ThreadPool
{
public:
  typedef std::shared_ptr<ThreadPool> Ptr;
  ThreadPool() : stop_flag_{ false }
  {
    idl_thr_num_ = MAX_THREAD_NUM;
    for (int i = 0; i < idl_thr_num_; ++i)
    {
      pool_.emplace_back([this] {
        while (!this->stop_flag_)
        {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock{ this->m_lock_ };
            this->cv_task_.wait(lock, [this] { return this->stop_flag_.load() || !this->tasks_.empty(); });
            if (this->stop_flag_ && this->tasks_.empty())
              return;
            task = std::move(this->tasks_.front());
            this->tasks_.pop();
          }
          idl_thr_num_--;
          task();
          idl_thr_num_++;
        }
      });
    }
  }

  ~ThreadPool()
  {
    stop_flag_.store(true);
    cv_task_.notify_all();
    for (std::thread& thread : pool_)
    {
      thread.detach();
    }
  }

public:
  template <class F, class... Args>
  inline auto commit(F&& f, Args&&... args) -> std::future<decltype(f(args...))>
  {
    if (stop_flag_.load())
      throw std::runtime_error("Commit on LiDAR threadpool is stopped.");
    using RetType = decltype(f(args...));
    auto task =
        std::make_shared<std::packaged_task<RetType()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    std::future<RetType> future = task->get_future();
    {
      std::lock_guard<std::mutex> lock{ m_lock_ };
      tasks_.emplace([task]() { (*task)(); });
    }
    cv_task_.notify_one();
    return future;
  }

private:
  using Task = std::function<void()>;
  std::vector<std::thread> pool_;
  std::queue<Task> tasks_;
  std::mutex m_lock_;
  std::condition_variable cv_task_;
  std::atomic<bool> stop_flag_;
  std::atomic<int> idl_thr_num_;
};
}  // namespace lidar
}  // namespace robosense