#pragma once

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>

class Timer{
public:
    Timer():
        running_(false)
    {

    }

    ~Timer(){
        stop();
    }

    void start(std::chrono::nanoseconds period, std::function<void()> callback){
        if(running_){
            return ;
        }

        running_ = true;
        period_ = period;
        callback_ = callback;
        timer_thread_ = std::thread(
            [this](){
                auto next_time = std::chrono::steady_clock::now();
                while(running_){
                    next_time += period_;
                    if(callback_){
                        callback_();
                    }
                    std::this_thread::sleep_until(next_time);
                }
            }
        );
    }

    void stop(){
        running_ = false;
        if(timer_thread_.joinable()){
            timer_thread_.join();
        }
    }

private:
    std::atomic<bool> running_;

    std::chrono::nanoseconds period_;

    std::function<void()> callback_;

    std::thread timer_thread_;
};