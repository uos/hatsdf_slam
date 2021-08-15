#pragma once

/**
 * @file base_kernel.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <hw/fpga_manager.h>

namespace fastsense::kernels
{

class BaseKernel
{
private:
    int narg_;
protected:
    /**
     * @brief Add argument at position narg to FPGA function call
     * 
     * @tparam T type of arg
     * @param arg argument to add to FPGA function call
     */
    template <typename T>
    inline void setArg(const T& arg)
    {
        kernel_.setArg(narg_++, arg);
    }

    template <typename T, typename ...Args>
    inline void setArgs(const T& arg, const Args& ...args)
    {
        setArg(arg);
        setArgs(args...);
    }

    template <typename T>
    inline void setArgs(const T& arg)
    {
        setArg(arg);
    }

    /// reset narg to 0
    void resetNArg()
    {
        narg_ = 0;
    }

    /// FPGA Kernel
    cl::Kernel kernel_;

    /// Pointer to Command Queue
    fastsense::CommandQueuePtr cmd_q_;

    /// TODO
    std::vector<cl::Event> pre_events_;

    /// TODO
    std::vector<cl::Event> execute_events_;

    /// eTODO
    std::vector<cl::Event> post_events_;
public:
    inline BaseKernel(const fastsense::CommandQueuePtr& queue, const char* name)
        :   narg_{0},
            kernel_{fastsense::hw::FPGAManager::get_program(), name},
            cmd_q_{queue},
            pre_events_(1),
            execute_events_(1),
            post_events_(1)
    {}

    /// default destructor
    virtual ~BaseKernel() = default;

    /// delete copy assignment operator
    BaseKernel& operator=(const BaseKernel& other) = delete;

    /// delete move assignment operator
    BaseKernel& operator=(BaseKernel&&) noexcept = delete;

    /// delete copy constructor
    BaseKernel(const BaseKernel&) = delete;

    /// delete move constructor
    BaseKernel(BaseKernel&&) = delete;

    /// Wait until Kernel completes
    virtual void waitComplete()
    {
        cl::Event::waitForEvents(post_events_);
    }
};

} // namespace fastsense::kernels