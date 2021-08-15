#pragma once

/**
 * @file led.h
 * @author Marcel Flottmann
 * @date 2021-02-11
 */

#include <gpiod.hpp>
#include <thread>
#include <atomic>

namespace fastsense::ui
{

/**
 * @brief Represents an LED on a GPIO line
 * 
 */
class Led
{
private:
    gpiod::line line_;
    std::atomic<double> freq_;
    std::thread blink_thread_;
    bool running_;

    /**
     * @brief Contains the blinking logic
     * 
     */
    void blink();
public:
    /**
     * @brief Construct a new Led object
     * 
     * @param line line the LED is connected to
     * @param initial_freq frequqncy the LED is started with in Hz. 0: LED is off, inf: LED is completly on
     */
    Led(const gpiod::line& line, double initial_freq = 0);
    ~Led();
    Led(const Led&) = delete;
    Led& operator=(const Led&) = delete;
    /// delete move assignment operator
    Led& operator=(Led&&) noexcept = delete;

    /// delete move constructor
    Led(Led&&) = delete;

    /**
     * @brief Set the frequency. 0: LED is off, inf: LED is completly on
     * 
     * @param freq new frequency in Hz
     */
    void setFrequency(double freq);

    /**
     * @brief Set the LED to on
     * 
     */
    void setOn();

    /**
     * @brief Set the LED to off
     * 
     */
    void setOff();
};

}