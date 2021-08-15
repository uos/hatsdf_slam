/**
 * @file velodyne_bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <chrono>
#include <iterator>
#include <ros/ros.h>
#include <bridge/util.h>
#include <bridge/from_trenz/velodyne_bridge.h>

using namespace fastsense::bridge;

VelodyneBridge::VelodyneBridge(ros::NodeHandle& n, const std::string& board_addr, std::chrono::milliseconds timeout, bool discard_timestamp)
    :   BridgeBase{n, "velodyne/points", board_addr, 1000, timeout},
        ProcessThread{},
        points_{},
        discard_timestamp_{discard_timestamp}
{
}

void VelodyneBridge::run()
{
    while (running && ros::ok())
    {
        try
        {
            if (receive())
            {
                ROS_DEBUG_STREAM("Received " << msg_.data_.points_.size() << " points");
                convert();
                publish();
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("velo bridge error: " << e.what());
        }
    }
}

void VelodyneBridge::convert()
{
    points_.clear();

    timestamp_ = timestamp_to_rostime(msg_.timestamp_, discard_timestamp_);
    const auto& msg_points = msg_.data_.points_;
    const auto& scaling = msg_.data_.scaling_;

    std::transform(msg_points.begin(), msg_points.end(), std::back_inserter(points_), [&](const ScanPoint& p)
    {
        geometry_msgs::Point32 out;
        out.x = p.x() * 0.001f / scaling;
        out.y = p.y() * 0.001f / scaling;
        out.z = p.z() * 0.001f / scaling;
        return out;
    });

    ROS_DEBUG_STREAM("Converted points: " << msg_points.size() << "->" << points_.size() << " points\n");
}

void VelodyneBridge::publish()
{
    sensor_msgs::PointCloud pc;
    pc.header.stamp = timestamp_;
    pc.header.frame_id = "base_link";
    pc.points = points_;
    pub().publish(pc);

    ROS_DEBUG_STREAM("Published points values\n");
}