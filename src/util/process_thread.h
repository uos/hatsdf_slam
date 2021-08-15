#pragma once

/**
 * @file process_thread.h
 * @author Julian Gaal
 * @author Malte Hillmann
 */

#include <thread>

namespace fastsense::util
{

class ProcessThread
{
public:
    using UPtr = std::unique_ptr<ProcessThread>;

    ProcessThread() : worker{}, running{false} {}

    virtual ~ProcessThread()
    {
        stop();
    }
    
    ProcessThread(ProcessThread&) = delete;
    ProcessThread(ProcessThread&&) = delete;
    ProcessThread& operator=(const ProcessThread&) = delete;
    ProcessThread& operator=(ProcessThread&&) = delete;

    virtual void start()
    {
        if (!running)
        {
            running = true;
            worker = std::thread([&]()
            {
                this->thread_run();
            });
        }
    }

    virtual void stop()
    {
        if (running && worker.joinable())
        {
            running = false;
            worker.join();
        }
    }

protected:

    virtual void thread_run() = 0;

    /// Worker thread
    std::thread worker;
    /// Flag if the thread is running
    bool running;
};

} // namespace fastsense::util
