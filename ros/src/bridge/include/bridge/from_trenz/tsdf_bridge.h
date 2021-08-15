/**
 * @file tsdf_bridge.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include "bridge_base.h"
#include <msg/tsdf.h>
#include <util/tsdf.h>

namespace fastsense::bridge
{

/**
 * @brief TSDFBridge converts TSDF data of type msg::TSDF into ROS Markers
 */
class TSDFBridge :  public BridgeBase<msg::TSDFStamped, visualization_msgs::Marker, 6666>, 
                    public util::ProcessThread
{
public:
    /**
     * @brief Construct a new TSDFBridge object
     * 
     * @param n nodehandle
     * @param board_addr ip addr of Trenz board
     * @param timeout how long to wait for message before trying again
     * @param discard_timestamp Whether or not to discard timestamps and replace with ros::Time::now()
     */
    TSDFBridge( ros::NodeHandle& n, 
                const std::string& board_addr, 
                std::chrono::milliseconds timeout, 
                bool discard_timestamp);

    /**
     * @brief Destroy the TSDFBridge object
     */
    ~TSDFBridge() override = default;
private:

    /**
     * @brief Publishes a visualization_msgs::Marker with TSDF values (convert() FIRST for newest data)
     */
    void publish() final;

    /**
     * @brief Converts msg::TSDF to visualization_msgs::Marker
     */
    void convert() final;

    /**
     * @brief Run listens for TSDFs, converts it to ROS Marker
     * and publishes in an endless loop (running in its own thread)
     */
    void run() final;

    /// one "iteration" of thread
    void thread_run() override
    {
        run();
    }

    /// returns true, if x y z is in bounds of map
    bool in_bounds(int x, int y, int z) const;

    /// gets tsdf value at x y z
    TSDFValue get_tsdf_value(int x, int y, int z) const;

    /// TSDF Point vector
    std::vector<geometry_msgs::Point> points_;

    /// Color vector
    std::vector<std_msgs::ColorRGBA> colors_;

    /// Whether or not to discard timestamps and replace with ros::Time::now()
    bool discard_timestamp_;

    /// scaling of pointcloud
    float scaling_;
};

} // namespace fastsense::bridge