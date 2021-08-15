/**
 * @file bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-06
 */

#include <chrono>
#include <ros/ros.h>
#include <bridge/from_trenz/tsdf_bridge.h>
#include <bridge/from_trenz/imu_bridge.h>
#include <bridge/from_trenz/velodyne_bridge.h>
#include <bridge/from_trenz/transform_bridge.h>

namespace fs = fastsense;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from_trenz_bridge");
    ros::NodeHandle n("~");

    // Setup board address
    std::string board_addr;
    n.param<std::string>("board_addr", board_addr, "192.168.1.123");
    ROS_INFO_STREAM("Board address is " << board_addr);

    // setup timer for receiver
    int timeout_ros;
    n.param("timeout", timeout_ros, 10);
    ROS_INFO_STREAM("Timeout " << timeout_ros << "ms");
    auto timeout = std::chrono::milliseconds(static_cast<size_t>(timeout_ros)); 

    // Setup discard_timestamp
    bool discard_timestamp;
    n.param("discard_timestamp", discard_timestamp, false);
    ROS_INFO_STREAM("Discard timestamps: " << std::boolalpha << discard_timestamp);
    if (discard_timestamp)
    {
        ROS_WARN("Do not record bagfiles with 'discard_timestamp' enabled!\n");
    }

    // Setup save_poses
    bool save_poses;
    n.param("save_poses", save_poses, false);
    ROS_INFO_STREAM("Save poses: " << std::boolalpha << save_poses);

    fs::bridge::TSDFBridge tsdf_bridge{n, board_addr, timeout, discard_timestamp};
    fs::bridge::ImuBridge imu_bridge{n, board_addr, timeout, discard_timestamp};
    fs::bridge::VelodyneBridge velodyne_bridge{n, board_addr, timeout, discard_timestamp};
    fs::bridge::TransformBridge transform_bridge{n, board_addr, timeout, discard_timestamp, save_poses};
    
    tsdf_bridge.start();
    imu_bridge.start();
    velodyne_bridge.start();
    transform_bridge.start();
    
    ROS_INFO_STREAM("Timeout for bridge (ms): " << timeout_ros);
    ROS_INFO_STREAM("Board address is " << board_addr);
    ROS_INFO_STREAM("from_trenz bridge started");

    ros::Rate rate(0.1);
    while(ros::ok())
    {
        rate.sleep();
    }

    tsdf_bridge.stop();
    imu_bridge.stop();
    velodyne_bridge.stop();
    transform_bridge.stop();

    ROS_INFO_STREAM("from_trenz bridge stopped");

    return 0;
}
