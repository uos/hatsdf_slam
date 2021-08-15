#pragma once

/**
 * @file config.h
 * @author Marcel Flottmann
 */

#include "config_types.h"

namespace fastsense::util::config
{

struct ImuConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize, "Size of the Buffer for incoming values");
    DECLARE_CONFIG_ENTRY(size_t, filterSize, "Size of the Window for the SlidingWindowFilter");
};

struct LidarConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, bufferSize, "Size of the Buffer for incoming values");
    DECLARE_CONFIG_ENTRY(uint16_t, port, "The Port to listen to");
    DECLARE_CONFIG_ENTRY(float, pointScale, "A Factor to apply to the entire Cloud");
    DECLARE_CONFIG_ENTRY(int, rings, "The number of rings that the lidar has");
    DECLARE_CONFIG_ENTRY(float, vertical_fov_angle, "The field of view in vertical direction in degrees");
};

struct BridgeConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(bool, use_from, "true: take input from PC, false: use Sensors");
    DECLARE_CONFIG_ENTRY(bool, use_to, "true: send output to PC");

    DECLARE_CONFIG_ENTRY(bool, send_original, "send PointCloud before Preprocessing. Only one send_* option can be active");
    DECLARE_CONFIG_ENTRY(bool, send_preprocessed, "send PointCloud after Preprocessing. Only one send_* option can be active");
    DECLARE_CONFIG_ENTRY(bool, send_after_registration, "send PointCloud after Registration. Only one send_* option can be active");

    DECLARE_CONFIG_ENTRY(std::string, host_from, "IP Address of the PC when 'use_from' is true");
    DECLARE_CONFIG_ENTRY(uint16_t, recv_timeout, "Timeout for the receiver");

    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_from, "Port of the from bridge for imu");
    DECLARE_CONFIG_ENTRY(uint16_t, imu_port_to, "Port of the to bridge for imu");

    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_from, "Port of the from bridge for pcl");
    DECLARE_CONFIG_ENTRY(uint16_t, pcl_port_to, "Port of the to bridge for pcl");

    DECLARE_CONFIG_ENTRY(uint16_t, transform_port_to, "Port of the to bridge for transform");
    DECLARE_CONFIG_ENTRY(uint16_t, tsdf_port_to, "Port of the to bridge for tsdf");
};

struct GPIOConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(std::string, button_chip, "GPIO chip of the button, e.g. gpiochip0");
    DECLARE_CONFIG_ENTRY(std::string, led_chip, "GPIO chip of the LED, e.g. gpiochip0");
    DECLARE_CONFIG_ENTRY(unsigned int, button_line, "GPIO line (pin) of the button");
    DECLARE_CONFIG_ENTRY(unsigned int, led_line, "GPIO line (pin) of the LED");
};


struct RegistrationConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(unsigned int, max_iterations, "Maximum number of iterations for the Registration");
    DECLARE_CONFIG_ENTRY(float, it_weight_gradient, "Factor to reduce Registration influence on later iterations");
    DECLARE_CONFIG_ENTRY(float, epsilon, "Minimum change between two iterations to stop Registration");
};

struct SlamConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(int, max_distance, "The truncation distance in mm");
    DECLARE_CONFIG_ENTRY(unsigned int, map_resolution, "The size of one TSDF cell in mm");
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_x, "The number of TSDF cells in x direction");
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_y, "The number of TSDF cells in y direction");
    DECLARE_CONFIG_ENTRY(unsigned int, map_size_z, "The number of TSDF cells in z direction");
    DECLARE_CONFIG_ENTRY(float, max_weight, "The maximum weight as a float where 1.0");
    DECLARE_CONFIG_ENTRY(float, initial_map_weight, "The initial weight as a float where 1.0");

    DECLARE_CONFIG_ENTRY(unsigned int, map_update_period, "Number of Scans before a TSDF Update happens");
    DECLARE_CONFIG_ENTRY(float, map_update_position_threshold, "Distance since the last TSDF Update before a new one happens");

    DECLARE_CONFIG_ENTRY(std::string, map_path, "Path where the global map should be saved");
};

struct Config : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_GROUP(ImuConfig, imu);
    DECLARE_CONFIG_GROUP(LidarConfig, lidar);
    DECLARE_CONFIG_GROUP(RegistrationConfig, registration);
    DECLARE_CONFIG_GROUP(GPIOConfig, gpio);
    DECLARE_CONFIG_GROUP(BridgeConfig, bridge);
    DECLARE_CONFIG_GROUP(SlamConfig, slam);
};

} // namespace fastsense::util::config
