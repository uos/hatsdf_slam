/**
 * @file led.cpp
 * @author Marcel Flottmann
 * @date 2021-02-11
 */

#include "led.h"

using namespace fastsense::ui;
using namespace std::chrono_literals;

Led::Led(const gpiod::line& line, double initial_freq) :
    line_(line), freq_(initial_freq), blink_thread_(&Led::blink, this), running_(true)
{
    line.request({"fastsense", gpiod::line_request::DIRECTION_OUTPUT, 0});
}

Led::~Led()
{
    running_ = false;
    blink_thread_.join();
}

void Led::blink()
{
    while (running_)
    {
        double freq = freq_; //thread safe copy

        if (freq == 0)
        {
            line_.set_value(0);
            std::this_thread::sleep_for(10ms);
        }
        else if (freq == std::numeric_limits<double>::infinity())
        {
            line_.set_value(1);
            std::this_thread::sleep_for(10ms);
        }
        else
        {
            line_.set_value(1);
            std::this_thread::sleep_for(std::chrono::duration<double>(1 / freq / 2)); // 50% duty cycle
            line_.set_value(0);
            std::this_thread::sleep_for(std::chrono::duration<double>(1 / freq / 2));
        }
    }
}

void Led::setFrequency(double freq)
{
    if (freq < 0)
    {
        throw std::invalid_argument("Frequency cannot be negative!");
    }
    freq_ = freq;
}

void Led::setOn()
{
    freq_ = std::numeric_limits<double>::infinity();
}

void Led::setOff()
{
    freq_ = 0;
}
