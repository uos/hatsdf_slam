/**
 * @file velodyne_bridge.h
 * @author Marcel Flottmann
 * @author Julian Gaal
 * @author Marc Eisoldt
 * @date 2020-09-29
 */

#pragma once

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <msg/transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include "bridge_base.h"

#include <mutex>
#include <fstream>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace fastsense::bridge
{

/**
 * @brief TransformBridge converts transform data from the application to ROS messages
 * 
 */
class TransformBridge :  public BridgeBase<msg::TransformStamped, nav_msgs::Path, 8888>,
    public util::ProcessThread
{
public:
    /**
     * @brief Construct a new Transform Bridge object
     *
     * @param n ros::NodeHandle
     * @param board_addr ip addr of Trenz board
     * @param timeout how long to wait for new messages before trying again
     * @param discard_timestamp Whether or not to discard timestamps and replace with ros::Time::now()
     * @param save_poses Whether or not to save_poses by publishing to evaluation/SavePoseStamped
     */
    TransformBridge(ros::NodeHandle& n,
                    const std::string& board_addr,
                    std::chrono::milliseconds timeout,
                    bool discard_timestamp,
                    bool save_poses);

    /**
     * @brief Destroy the Transform Bridge object
     */
    ~TransformBridge() = default;

    /**
     * @brief Starts the Transform Bridge in its own thread
     */
    void start() override;

    /**
     * @brief Stops the Transform Bridge thread
     */
    void stop() override;
private:

    /**
     * @brief Publishes a geometry_msgs::TransformStamped (convert() FIRST for newest data)
     */
    void publish() override;

    /**
     * @brief Converts msg::Transform into geometry_msgs::TransformStamped
     */
    void convert() override;
   
    /**
     * @brief Run listens for transform data, converts it to ROS geometry_msgs::TransformStamped message
     * and publishes in an endless loop (running in its own thread)
     */
    void run() override;

    /// Run one "iteration" of thread
    void thread_run() override
    {
        run();
    }

    /// Broadcast transformation
    void broadcast();

    /// Transform broadcaster
    tf2_ros::TransformBroadcaster broadcaster;
    
    /// thread of broadcaster
    std::thread broadcaster_thread;

    /// TODO mutex why though
    std::mutex mtx;

    /// Local vector of lidar points that are published
    geometry_msgs::TransformStamped transform_data;

    /// first message received: used to detect first timestamp
    bool first_msg;

    /// Path message
    nav_msgs::Path pose_path;

    /// Single PoseStamped to append to path
    geometry_msgs::PoseStamped pose_stamped;

    /// Whether or not to discard timestamps and replace with ros::Time::now()
    bool discard_timestamp_;

    /// whether or not to save poses
    bool save_poses_;

    tf2::Vector3 rot_drift_;

    tf2::Vector3 last_position_corrected_;
    tf2::Matrix3x3 last_rotation_corrected_;

    tf2::Vector3 last_position_;
    tf2::Matrix3x3 last_rotation_;

    bool first_pose_;
};

} // namespace fastsense::bridge