#pragma once

#include "chrono"
#include "thread"
#include "iostream"

namespace CB{

    class WallRate{
    public:
        WallRate(double frequency):
            period_(std::chrono::duration<double>(1.0 / frequency)),
            last_time_(std::chrono::steady_clock::now())
        {

        }

        void sleep(){
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_time_;
            if (elapsed < period_) {
                std::this_thread::sleep_for(period_ - elapsed);
            }
            else{
                std::cout << "WallRate has missed rate." << std::endl;
            }
            last_time_ = std::chrono::steady_clock::now();
        }

    private:
        double frequency_;

        std::chrono::duration<double> period_;

        std::chrono::steady_clock::time_point last_time_;
    };

}