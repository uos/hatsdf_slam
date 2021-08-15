#pragma once

/**
 * @file imu.h
 * @author Julian Gaal
 */

#include <msg/imu.h>
#include <util/process_thread.h>
#include "api/phidget.h"
#include <util/filter.h>

namespace fastsense::driver
{

/**
 * @brief implements driver phidgets imu 1044, phidgets library version 2.1.9.20190409
 */
class Imu : public Phidget, public fastsense::util::ProcessThread
{
public:
    using UPtr = std::unique_ptr<Imu>;

    Imu() = delete;

    /**
     * Creates Imu instance
     * @param ringbuffer Buffer to write data into
     * @param filter_size Sice of sliding window for filtering
     */
    explicit Imu(const fastsense::msg::ImuStampedBuffer::Ptr& ringbuffer, size_t filter_size);

    ~Imu() override = default;

    /**
     * @brief delete copy assignment operator because of pointer member variable
     *
     * @return Imu& other imu
     */
    Imu& operator=(Imu&) = delete;

        /**
     * @brief delete move assignment operator because of pointer member variable
     *
     * @return Imu&& other imu (rvalue)
     */
    Imu& operator=(Imu&&) = delete;

    /**
     * @brief delete copy constructor because of pointer member variable
     * @param Imu& other imu
     */
    Imu(Imu&) = delete;

    /**
     * @brief delete move constructor 
     * @param Imu& other imu
     */
    Imu(Imu&&) = delete;

    /**
     * @brief Whether or not device has been calibrated
     *
     * @return true if device has been calibrated
     * @return false if device has NOT been calibrated
     */
    inline bool is_calibrated() const
    {
        return is_calibrated_;
    }

    /**
     * @brief Starts Imu thread
     * **NOTE** in contrast to velodyne driver, libphidget ALREADY listens to data coming from driver
     * from separate thread. This function only serves to provide the same api for all sensors
     */
    void start() override;

    /**
     * @brief Threadrun is empty, because
     * in contrast to velodyne driver, libphidget ALREADY listens to data coming from driver
     * from separate thread. This function only serves to provide the same api for all sensor
     */
    void thread_run() override {}

    /**
     * @brief Stops Imu
     * **NOTE** in contrast to velodyne driver, libphidget ALREADY listens to data coming from driver
     * from separate thread. This function only serves to provide the same api for all sensors and does not
     * ACTUALLY deattach device from linux
     *
     */
    void stop() override;

    /**
     * Getter for angular velocity covariance
     * @return angular velocity covariance
     */
    const std::array<double, 9>& get_angular_velocity_covariance() const;

    /**
     * Getter for linear acceleration covariance
     * @return linear acceleration covariance
     */
    const std::array<double, 9>& get_linear_acceleration_covariance() const;

    /**
     * Getter for magnetic field covariance
     * @return magnetic field covariance
     */
    const std::array<double, 9>& get_magnetic_field_covariance() const;

private:
    /// buffer, in which imu readings are saved
    fastsense::msg::ImuStampedBuffer::Ptr data_buffer_;

    /// Sliding Window averaging Filter
    util::SlidingWindowFilter<msg::Imu> filter_;

    /// whether or not imu is connected
    bool is_connected_;

    /// whether or not imu is calibrated
    bool is_calibrated_;

    /// whether or not to init compass with custom params defined in params.h
    bool init_compass_;

    /// angular velocity covariance
    std::array<double, 9> angular_velocity_covariance_;

    /// linear acceleration covariance
    std::array<double, 9> linear_acceleration_covariance_;

    /// magnetic field covariance
    std::array<double, 9> magnetic_field_covariance_;

    /// handle for phidgets "spatial" unit: imu
    CPhidgetSpatialHandle imu_handle_;

    /**
     * data_handler for raw data coming from libphidget
     * @param spatial *nused**
     * @param userptr pointer, with which data will be handled furt
     * @param data incoming data
     * @param count how many sensor readings have been made
     */
    static int spatial_data_handler(CPhidgetSpatialHandle spatial, void* userptr,
                                    CPhidgetSpatial_SpatialEventDataHandle* data,
                                    int count);

    /// calibrate gyro
    void zero();

    /**
     * Set data rate of imu
     * @param rate in ms
     */
    void set_data_rate(int rate);

    /// calibrate with zero() and wait, see hee: https://github.com/ros-drivers/phidgets_drivers/issues/40
    void calibrate();

    /**
     * @brief Initialize Imu
     * calls init_device(), init_api() and init_covariance()
     */
    void init();

    /// initialize phidget device
    void init_device();

    /// initiate libphigdet api
    void init_api();

    /// initialize covariance matrices
    void init_covariance();

    /**
     * data_handler, which gets raw data from libphidget
     * @param acceleration linear acceleration
     * @param angularRate angular velocity
     * @param magneticField magnetic field
     */
    void data_handler(const double acceleration[3], const double angularRate[3],
                      const double magneticField[3]);

    /// handles attachment of imu
    void attach_handler() override;

    /// handles detachment of imu
    void detach_handler() override;

    /// handles errors from imu
    void error_handler(int error) override;

    /**
     * Set compass correction parameters for magnetic field, see params.h
     * @param cc_mag_field
     * @param cc_offset0
     * @param cc_offset1
     * @param cc_offset2
     * @param cc_gain0
     * @param cc_gain1
     * @param cc_gain2
     * @param cc_T0
     * @param cc_T1
     * @param cc_T2
     * @param cc_T3
     * @param cc_T4
     * @param cc_T5
     */
    void set_compass_correction_parameters(double cc_mag_field, double cc_offset0,
                                           double cc_offset1, double cc_offset2,
                                           double cc_gain0, double cc_gain1,
                                           double cc_gain2, double cc_T0,
                                           double cc_T1, double cc_T2, double cc_T3,
                                           double cc_T4, double cc_T5);
};

} // namespace fastsense::driver;