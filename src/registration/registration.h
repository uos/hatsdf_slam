#pragma once

/**
 * @file registration.h
 * @author Patrick Hoffmann
 * @author Adrian Nitschmann
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include <mutex>
#include <algorithm>

#include "imu_accumulator.h"
#include <msg/imu.h>
#include <hw/kernels/reg_kernel.h>

namespace fastsense::registration
{

/**
 * @brief
 *
 */
class Registration
{
    using Matrix6i = Eigen::Matrix<long, 6, 6>;
    using Matrix6f = Eigen::Matrix<float, 6, 6>;
    using Vector6i = Eigen::Matrix<long, 6, 1>;
    using Vector6f = Eigen::Matrix<float, 6, 1>;

private:
    size_t max_iterations_;
    float it_weight_gradient_;
    float epsilon_;

    ImuAccumulator imu_accumulator_;


    fastsense::kernels::RegistrationKernel krnl;

    /**
     * @brief transforms xi vector 6x1 (angular and linear velocity) to transformation matrix 4x4
     *
     * @param xi vector
     */
    Matrix4f xi_to_transform(Vector6f xi);

public:

    /**
     * @brief Construct a new Registration object
     *
     * TODO
     *
     * @param q xilinx command queue
     * @param buffer imu buffer stamped shared ptr
     * @param max_iterations max convergence iterations
     * @param it_weight_gradient learning rate weight gradient
     */
    Registration(fastsense::CommandQueuePtr q, msg::ImuStampedBuffer::Ptr& buffer, unsigned int max_iterations = 50, float it_weight_gradient = 0.0, float epsilon = 0.01);

    /**
     * Destructor of the registration.
     */
    ~Registration() = default;

    /// delete copy assignment operator
    Registration& operator=(const Registration& other) = delete;

    /// delete move assignment operator
    Registration& operator=(Registration&&) noexcept = delete;

    /// delete copy constructor
    Registration(const Registration&) = delete;

    /// delete move constructor
    Registration(Registration&&) = delete;

    /**
     * @brief Registers the given pointcloud with the local ring buffer. Transforms the cloud
     *
     * @param cur_buffer
     * @param cloud
     * @return Matrix4f
     */
    void register_cloud(fastsense::map::LocalMap& localmap,
                        fastsense::buffer::InputBuffer<PointHW>& cloud,
                        int num_points,
                        const util::HighResTimePoint& cloud_timestamp,
                        Matrix4f& pose);

    /**
     * @brief Transforms a given pointcloud with the transform
     *
     * @param in_cloud
     * @param transform
     */
    static void transform_point_cloud(ScanPoints_t& in_cloud, const Matrix4f& transform);

    /**
     * @brief Transforms a given pointcloud with the transform
     *
     * @param in_cloud
     * @param transform
     */
    static void transform_point_cloud(fastsense::buffer::InputBuffer<PointHW>& in_cloud, const Matrix4f& transform);
};


} //namespace fastsense
