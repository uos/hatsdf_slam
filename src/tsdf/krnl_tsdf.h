#pragma once

/**
 * @file krnl_tsdf.h
 * @author Marc Eisoldt
 * @author Malte Hillmann
 */

#include <hw/kernels/base_kernel.h>
#include <hw/buffer/buffer.h>
#include <map/local_map.h>
#include <util/point_hw.h>
#include <util/config/config_manager.h>
#include <hw/buffer/buffer.h>

#include <iostream>

namespace fastsense::tsdf
{

/**
 * @brief Wrapper around the TSDF Kernel
 */
class TSDFKernel : public kernels::BaseKernel
{
    /// Storage for the new TSDF Map before it is merged in the update process
    buffer::InputOutputBuffer<TSDFValue> new_entries;

public:

    /**
     * @brief Create a new TSDF kernel
     *
     * @param queue The CommandQueue for kernel operations
     * @param map_size The size of the 1D Array in the LocalMap
     */
    TSDFKernel(const CommandQueuePtr& queue, size_t map_size)
        : BaseKernel{queue, "krnl_tsdf"}, new_entries{cmd_q_, map_size}
    {

    }

    ~TSDFKernel() override = default;

    /// delete copy assignment operator
    TSDFKernel& operator=(const TSDFKernel& other) = delete;

    /// delete move assignment operator
    TSDFKernel& operator=(TSDFKernel&&) noexcept = delete;

    /// delete copy constructor
    TSDFKernel(const TSDFKernel&) = delete;

    /// delete move constructor
    TSDFKernel(TSDFKernel&&) = delete;

    /**
     * @brief Starts the Kernel and waits for completion
     * 
     * @param map The local map
     * @param scan_points The points to update with
     * @param num_points The number of Points in `scan_points`
     * @param up A Vector pointing in the up direction of the Scanner
     */
    void synchronized_run(map::LocalMap& map,
                          const buffer::InputBuffer<PointHW>& scan_points,
                          int num_points,
                          PointHW up = PointHW(0, 0, MATRIX_RESOLUTION))
    {
        auto& config = util::config::ConfigManager::config();

        int tau = config.slam.max_distance();
        int max_weight = config.slam.max_weight() * WEIGHT_RESOLUTION;
        float vertical_fov = config.lidar.vertical_fov_angle() / 180.0 * M_PI;
        int rings = config.lidar.rings();
        int dz_per_distance = std::tan(vertical_fov / (rings - 1.0) / 2.0) * MATRIX_RESOLUTION;

        run(map,
            scan_points,
            num_points,
            tau,
            max_weight,
            dz_per_distance,
            up);

        waitComplete();
    }

    /**
     * @brief Starts the Kernel without waiting. Requires waitComplete() to be called afterwards
     * 
     * @param map The local map
     * @param scan_points The points to update with
     * @param num_points The number of Points in `scan_points`
     * @param tau The truncation distance in mm
     * @param max_weight The max weight as an integer, with WEIGHT_RESOLUTION as the equivalent of 1.0f
     * @param dz_per_distance Number of interpolation steps as a function of the distance
     * @param up A Vector pointing in the up direction of the Scanner
     */
    void run(map::LocalMap& map,
             const buffer::InputBuffer<PointHW>& scan_points,
             int num_points,
             TSDFValue::ValueType tau,
             TSDFValue::WeightType max_weight,
             int dz_per_distance = 572, // default with 16 Rings and 30 degrees fov
             PointHW up = PointHW(0, 0, MATRIX_RESOLUTION))
    {
        for (auto& v : new_entries)
        {
            v = TSDFValue(0, 0);
        }

        auto m = map.get_hardware_representation();

        resetNArg();
        for (int i = 0; i < TSDF_SPLIT_FACTOR; i++)
        {
            setArg(scan_points.getBuffer());
        }
        setArg(num_points);
        for (int i = 0; i < TSDF_SPLIT_FACTOR; i++)
        {
            setArg(map.getBuffer().getBuffer());
        }

        setArgs(m.sizeX,   m.sizeY,   m.sizeZ);
        setArgs(m.posX,    m.posY,    m.posZ);
        setArgs(m.offsetX, m.offsetY, m.offsetZ);

        for (int i = 0; i < TSDF_SPLIT_FACTOR; i++)
        {
            setArg(new_entries.getBuffer());
        }
        setArg(tau);
        setArg(max_weight);
        setArg(dz_per_distance);
        setArgs(up.x, up.y, up.z);

        // Write buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer(), scan_points.getBuffer(), new_entries.getBuffer()}, CL_MIGRATE_MEM_OBJECT_DEVICE, nullptr, &pre_events_[0]);

        // Launch the Kernel
        cmd_q_->enqueueTask(kernel_, &pre_events_, &execute_events_[0]);

        // Read buffers
        cmd_q_->enqueueMigrateMemObjects({map.getBuffer().getBuffer()}, CL_MIGRATE_MEM_OBJECT_HOST, &execute_events_, &post_events_[0]);
    }
};

} // namespace fastsense::tsdf