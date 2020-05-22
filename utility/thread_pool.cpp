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
#ifndef __GNUC__
//#include "stdafx.h"
#endif
#include "utility/thread_pool.h"
namespace robosense
{

ThreadPool::Ptr ThreadPool::instance_ptr = nullptr;
std::mutex ThreadPool::instance_mutex;
ThreadPool::ThreadPool() : stoped{false}
{
    idl_thr_num = MAX_THREAD_NUM;
    for (int i = 0; i < idl_thr_num; ++i)
    {
        pool.emplace_back(
            [this] {
                while (!this->stoped)
                {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock{this->m_lock};
                        this->cv_task.wait(lock, [this] {
                            return this->stoped.load() || !this->tasks.empty();
                        });
                        if (this->stoped && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    idl_thr_num--;
                    task();
                    idl_thr_num++;
                }
            });
    }
}

ThreadPool::Ptr ThreadPool::getInstance()
{
    if (instance_ptr == nullptr)
    {
        std::lock_guard<std::mutex> lk(instance_mutex);
        if (instance_ptr == nullptr)
        {
            instance_ptr = std::shared_ptr<ThreadPool>(new ThreadPool);
        }
    }
    return instance_ptr;
}
ThreadPool::~ThreadPool()
{
    stoped.store(true);
    cv_task.notify_all();
    for (std::thread &thread : pool)
    {
        thread.detach();
    }
}


int ThreadPool::idlCount() { return idl_thr_num; }

} // namespace robosense