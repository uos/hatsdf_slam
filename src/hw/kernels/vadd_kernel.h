#pragma once

/**
 * @file vadd_kernel.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <iostream>

#include <hw/buffer/buffer.h>
#include "base_kernel.h"

namespace fastsense::kernels
{

class VaddKernel : public BaseKernel
{
public:
    VaddKernel(const CommandQueuePtr& queue)
        : BaseKernel(queue, "krnl_vadd")
    {}

    ~VaddKernel() override = default;

    void run(buffer::InputBuffer<int>& inbuffer_a, buffer::InputBuffer<int>& inbuffer_b, buffer::OutputBuffer<int>& outbuf, int data_size)
    {
        resetNArg();
        setArg(inbuffer_a.getBuffer());
        setArg(inbuffer_b.getBuffer());
        setArg(outbuf.getBuffer());
        setArg(data_size);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({inbuffer_a.getBuffer(), inbuffer_b.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({outbuf.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels