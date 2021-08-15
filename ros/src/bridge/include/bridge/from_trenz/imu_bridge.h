#pragma once

/**
 * @file imu_bridge.h
 * @author Julian Gaal
 * @date 2020-09-30
 */

#include <array>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "bridge_base.h"
#include <msg/imu.h>

namespace fastsense::bridge
{

/**
 * @brief ImuBridge converts msg::ImuMsg. received with the zeromq receiver,
 *        to sensor_msgs::Imu and publishes the message
 */
class ImuBridge :   public BridgeBase<msg::ImuStamped, sensor_msgs::Imu, 5555>, 
                    public util::ProcessThread
{
public:
    /**
     * @brief Construct a new Imu Bridge object
     * 
     * @param n nodehandle
     * @param board_addr ip addr of Trenz board
     * @param timeout how long to wait for message before trying again
     * @param discard_timestamp Whether or not to discard timestamps and replace with ros::Time::now()
     */
    ImuBridge(ros::NodeHandle& n, const std::string& board_addr, std::chrono::milliseconds timeout, bool discard_timestamp);

    /**
     * @brief Destroy the Imu Bridge object
     */
    ~ImuBridge() override = default;
private:
    /**
     * @brief Publishes an sensor_msgs::Imu (convert() FIRST for newest data)
     */
    void publish() final;

    /**
     * @brief Convert msg::ImuMsg into sensor_msgs::Imu
     */
    void convert() final;
    
    /**
     * @brief Run listens for data, converts into sensor_msgs::Imu
     * and publishes in an endless loop (running in its own thread)
     */
    void run() final;

    void thread_run() override
    {
        run();
    }

    /**
     * @brief Initialize covariance matrices (in accordance with imu driver)
     */
    void initCovariance();
    
    /// IMU ROS message
    sensor_msgs::Imu imu_ros_;
    
    /// MagneticField ROS message
    sensor_msgs::MagneticField mag_ros_;

    /// Magnetic Field publisher
    ros::Publisher mag_pub_;

    /// angular velocity covariance
    std::array<double, 9> angular_velocity_covariance_;

    /// linear acceleration covariance
    std::array<double, 9> linear_acceleration_covariance_;

    /// magnetic field covariance
    std::array<double, 9> magnetic_field_covariance_;

    /// Whether or not to discard timestamps and replace with ros::Time::now()
    const bool discard_timestamp_;
};

} // namespace fastsense::bridge