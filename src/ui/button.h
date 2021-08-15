#pragma once

/**
 * @file Button.h
 * @author Marcel Flottmann
 * @date 2021-02-11
 */

#include <gpiod.hpp>
#include <functional>

namespace fastsense::ui
{

/**
 * @brief Represents a button on a GPIO line
 * 
 */
class Button
{
private:
    gpiod::line line_;
    std::chrono::nanoseconds timeout_;
public:
/**
 * @brief Construct a new Button object
 * 
 * @param line gpiod line on that the button is connected
 * @param timeout timeout between checks of the condition
 */
    Button(const gpiod::line& line, std::chrono::nanoseconds timeout = std::chrono::nanoseconds{100'000'000});
    ~Button() = default;
    /// delete copy constructor
    Button(const Button&) = delete;
    /// delete copy assignment operator
    Button& operator=(const Button&) = delete;
    
    /// delete move assignment operator
    Button& operator=(Button&&) noexcept = delete;

    /// delete move constructor
    Button(Button&&) = delete;

    /**
     * @brief Wait for button press or until a condition is true
     *
     * @param condition condition to wait on
     * @return true button was pressed
     * @return false condition was met
     */
    bool wait_for_press_or_condition(std::function<bool()> condition);
};

}
