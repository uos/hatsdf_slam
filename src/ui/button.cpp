/**
 * @file Button.cpp
 * @author Marcel Flottmann
 * @date 2021-02-11
 */

#include "button.h"
#include "stdin.h"
#include <thread>

using namespace fastsense::ui;
using namespace std::chrono_literals;

Button::Button(const gpiod::line& line, std::chrono::nanoseconds timeout) :
    line_(line), timeout_(timeout)
{
    line_.request({"fastsense", gpiod::line_request::DIRECTION_INPUT, 0});
}

bool Button::wait_for_press_or_condition(std::function<bool()> condition)
{
    // Workaround:
    // Interrupts (gpiod events) causes the system to freeze.
    // So we just get the value of the line and explicitly sleep.

    // wait for press of button or enter...
    while (!line_.get_value() && !readStdIn())
    {
        // ...or condition
        if (condition())
        {
            return false;
        }
        std::this_thread::sleep_for(timeout_);
    }

    return true;
}