/**
 * @file imu.cpp
 * @author Julian Gaal
 */

#include <thread>
#include <stdexcept>
#include <cmath>

#include "imu.h"
#include <util/time.h>
#include <util/params.h>
#include <msg/imu.h>

using namespace fastsense::driver;
using namespace fastsense::util;

using fastsense::msg::ImuStampedBuffer;

// TODO detach handler? How to handle connected/disconnectedness

Imu::Imu(const ImuStampedBuffer::Ptr& ringbuffer, size_t filter_size)
    :   Phidget(),
        ProcessThread(),
        data_buffer_{ringbuffer},
        filter_{filter_size},
        is_connected_{false},
        is_calibrated_{false},
        init_compass_{false},
        angular_velocity_covariance_{},
        linear_acceleration_covariance_{},
        magnetic_field_covariance_{},
        imu_handle_{nullptr}
{}

void fastsense::driver::Imu::start()
{
    if (not running)
    {
        running = true;
        if (not is_calibrated_) init();
    }
}

void Imu::stop()
{
    // cleanup happens in Phidgets destructor
    running = false;
}

void Imu::init()
{
    init_api();
    init_covariance();
    init_device();
}

void Imu::set_data_rate(int rate)
{
    if (not imu_handle_)
    {
        throw std::runtime_error("Can't set rate: Imu not initialized");
    }

    CPhidgetSpatial_setDataRate(imu_handle_, rate);
}

void Imu::zero()
{
    if (not imu_handle_)
    {
        throw std::runtime_error("Can't set zero: Imu not initialized");
    }

    // zero (calibrate) gyro
    CPhidgetSpatial_zeroGyro(imu_handle_);
}

int Imu::spatial_data_handler(CPhidgetSpatialHandle /* handle */, void* userptr,
                              CPhidgetSpatial_SpatialEventDataHandle* data,
                              int count)
{
    for (int i = 0; i < count; ++i)
    {
        ((Imu*)userptr)->data_handler(data[i]->acceleration, data[i]->angularRate,
                                      data[i]->magneticField);
    }
    return 0;
}

void Imu::set_compass_correction_parameters(double cc_mag_field, double cc_offset0,
        double cc_offset1, double cc_offset2,
        double cc_gain0, double cc_gain1,
        double cc_gain2, double cc_T0,
        double cc_T1, double cc_T2,
        double cc_T3, double cc_T4,
        double cc_T5)
{
    int ret = CPhidgetSpatial_setCompassCorrectionParameters(
                  imu_handle_, cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
                  cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);

    if (ret != EPHIDGET_OK)
    {
        throw ::std::runtime_error("Compass correction parameter error: " + Phidget::getErrorDescription(ret));
    }
}

void Imu::data_handler(const double* acceleration, const double* angularRate, const double* magneticField)
{
    // Even during calibration, the device reports back zeroes, so force driver to wait until
    // after calibration
    if (not is_calibrated_ || not running)
    {
        return;
    }

    const auto filtered_msg = filter_.update(msg::Imu{acceleration, angularRate, magneticField});
    data_buffer_->push_nb(msg::ImuStamped{std::move(filtered_msg), util::HighResTime::now()}, true);
}

void Imu::attach_handler()
{
    Phidget::attach_handler();
    // Set device params. This is in attach_handler(), since it has to be
    // repeated on reattachment.
    set_data_rate(params::period_);
}

void Imu::detach_handler()
{
    Phidget::detach_handler();
}

void Imu::error_handler(int error)
{
    Phidget::error_handler(error);
}

void Imu::calibrate()
{
    zero();
    // The API call returns directly, so we "enforce" the recommended 2 sec
    // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40
    using namespace ::std::chrono_literals;
    std::this_thread::sleep_for(2s);
    is_calibrated_ = true;
}

void Imu::init_device()
{
    openAndWaitForAttachment(-1, 10000);

    calibrate();

    if (init_compass_)
    {
        using namespace fastsense::util::params;
        Imu::set_compass_correction_parameters(cc_mag_field_, cc_offset0_, cc_offset1_, cc_offset2_, cc_gain0_, cc_gain1_,
                                               cc_gain2_, cc_T0_, cc_T1_, cc_T2_, cc_T3_, cc_T4_, cc_T5_);
    }
}

void Imu::init_api()
{
    // create the handle
    CPhidgetSpatial_create(&imu_handle_);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)imu_handle_);

    // register base class callbacks
    Phidget::registerHandlers();

    // register imu data callback
    CPhidgetSpatial_set_OnSpatialData_Handler(imu_handle_, spatial_data_handler,
            this);
}

void Imu::init_covariance()
{
    double ang_vel_var = params::angular_velocity_stdev_ * params::angular_velocity_stdev_;
    double lin_acc_var = params::linear_acceleration_stdev_ * params::linear_acceleration_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                angular_velocity_covariance_[idx]    = ang_vel_var;
                linear_acceleration_covariance_[idx] = lin_acc_var;
            }
            else
            {
                angular_velocity_covariance_[idx]    = 0.0;
                linear_acceleration_covariance_[idx] = 0.0;
            }
        }
    }
    // build covariance matrix

    double mag_field_var = params::magnetic_field_stdev_ * params::magnetic_field_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                magnetic_field_covariance_[idx] = mag_field_var;
            }
            else
            {
                magnetic_field_covariance_[idx] = 0.0;
            }
        }
    }
}

const std::array<double, 9>& Imu::get_angular_velocity_covariance() const
{
    return angular_velocity_covariance_;
}

const std::array<double, 9>& Imu::get_linear_acceleration_covariance() const
{
    return linear_acceleration_covariance_;
}

const std::array<double, 9>& Imu::get_magnetic_field_covariance() const
{
    return magnetic_field_covariance_;
}
