#pragma once

/**
 * @file reg_kernel.h
 * @author Patrick Hoffmann
 */

#include <hw/kernels/base_kernel.h>
#include <map/local_map.h>
#include <util/point_hw.h>
#include <util/logging/logger.h>
#include <util/filter.h>

namespace fastsense::kernels
{

class RegistrationKernel : public BaseKernel
{
public:
    RegistrationKernel(const CommandQueuePtr& queue)
        : BaseKernel{queue, "krnl_reg"}
    {}

    ~RegistrationKernel() override = default;

    /// delete copy assignment operator
    RegistrationKernel& operator=(const RegistrationKernel& other) = delete;

    /// delete move assignment operator
    RegistrationKernel& operator=(RegistrationKernel&&) noexcept = delete;

    /// delete copy constructor
    RegistrationKernel(const RegistrationKernel&) = delete;

    /// delete move constructor
    RegistrationKernel(RegistrationKernel&&) = delete;

    /**
     * @brief interface between the software and the hw, calls the run method of the kernel, writes all the data coming from the kernel
     *        into the datatypes used by the software
     *
     * @param map           current local map
     * @param scan_points   points from the current velodyne scan
     * @param local_h       reference for the local h matrix used by the software
     * @param local_g       reference for the local g matrix used by the software
     * @param local_error   local error ref
     * @param local_count   local count ref
     * @param transform     transform from last registration iteration (including imu one) - needs to be applied in the kernel
     */
    void synchronized_run(map::LocalMap& map,
                          buffer::InputBuffer<PointHW>& point_data,
                          int num_points,
                          int max_iterations,
                          float it_weight_gradient,
                          float epsilon,
                          Eigen::Matrix4f& transform)
    {
        // 4x4 Matrix and Number of iterations = 17
        buffer::OutputBuffer<float> out_transform(cmd_q_, 17);

        // 4x4 Matrix and epsilon = 17
        buffer::InputBuffer<float> in_transform(cmd_q_, 17);

        //write last transform to buffer
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                in_transform[row * 4 + col] = transform(row, col);
            }
        }
        in_transform[16] = epsilon;

        //run the encapsulated kernel
        run(map, point_data, num_points, max_iterations, it_weight_gradient, in_transform, out_transform);

        waitComplete();

        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                transform(row, col) = out_transform[row * 4 + col];
            }
        }

        static util::SlidingWindowFilter<float> filter(100);
        static int iter_count = 0;

        int i = out_transform[16];
        filter.update(i);
        iter_count++;
        if (iter_count > 100 && iter_count % 20 == 0)
        {
            fastsense::util::logging::Logger::info("Average Iterations: ", (int)filter.get_mean(), " / ", max_iterations);
        }
    }

    /**
     * @brief Called by the synchronized run method, uses the kernel to register
     *
     * @param map
     * @param scan_points
     * @param outbuf
     * @param queue
     */
    void run(map::LocalMap& map,
             buffer::InputBuffer<PointHW>& point_data,
             int num_points,
             int max_iterations,
             float it_weight_gradient,
             buffer::InputBuffer<float>& in_transform,
             buffer::OutputBuffer<float>& out_transform)
    {
        resetNArg();

        //std::cout << "Point data: size: " << static_cast<int>(point_data.size()) << std::endl << std::endl;

        auto m = map.get_hardware_representation();

        // MARKER: SPLIT
        constexpr int SPLIT_FACTOR = 3;

        for (int i = 0; i < SPLIT_FACTOR; i++)
        {
            setArg(point_data.getBuffer());
        }
        setArg(num_points);

        for (int i = 0; i < SPLIT_FACTOR; i++)
        {
            setArg(map.getBuffer().getBuffer());
            setArg(map.getBuffer().getBuffer());
            setArg(map.getBuffer().getBuffer());
        }

        setArg(m.sizeX);
        setArg(m.sizeY);
        setArg(m.sizeZ);
        setArg(m.posX);
        setArg(m.posY);
        setArg(m.posZ);
        setArg(m.offsetX);
        setArg(m.offsetY);
        setArg(m.offsetZ);
        setArg(max_iterations);
        setArg(it_weight_gradient);
        setArg(in_transform.getBuffer());
        setArg(out_transform.getBuffer());

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), point_data.getBuffer(), in_transform.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({out_transform.getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::kernels