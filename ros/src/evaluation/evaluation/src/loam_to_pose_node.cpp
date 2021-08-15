#include <ros/ros.h>
#include <evaluation/SavePoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

// Taken from Lego LOAM
using PointType = pcl::PointXYZI;

ros::Publisher pose_path_pub;
ros::Publisher save_path_pub;

/**
 * Update nav_msgs/Path with geometry_msgs::PoseStamped and publish
 * @param pose_stamped Pose
 */
void updatePosePath(const geometry_msgs::PoseStamped& pose_stamped)
{
    static nav_msgs::Path pose_path;
    static bool first_msg = true;

    pose_path.header = pose_stamped.header;
    pose_path.poses.push_back(pose_stamped);

    // RViz crashes if path is longer than 16384 (~13 minutes at 20 Scans/sec)
    // see https://github.com/ros-visualization/rviz/issues/1107
    if (pose_path.poses.size() >= 16380)
    {
        static std::default_random_engine rng((std::random_device()) ());

        // remove a random pose, but not the start nor a recent pose
        std::uniform_int_distribution dist(1, (int) pose_path.poses.size() - 1000);
        int index = dist(rng);
        pose_path.poses.erase(pose_path.poses.begin() + index);
    }

    pose_path_pub.publish(pose_path);
    ROS_DEBUG_STREAM("Publish nav_msgs/Path from LOAM");
}

/**
 * Lego Loam publishes its path as a pointcloud, therefore convert to pose
 * @param pcl_path PointCloud containing path data
 */
void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_path)
{
    pcl::PointCloud<PointType>::Ptr pcl(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*pcl_path, *pcl);
    auto point = pcl->points.back();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = pcl_path->header;
    pose_stamped.pose.orientation.w = 1;
    pose_stamped.pose.position.x = point.x;
    pose_stamped.pose.position.y = point.y;
    pose_stamped.pose.position.z = point.z;

    ROS_DEBUG_STREAM("Converted LOAM PCL to Pose");

    evaluation::SavePoseStamped save_pose;
    save_pose.id = "loam";
    save_pose.pose_stamped = pose_stamped;

    updatePosePath(pose_stamped);

    save_path_pub.publish(save_pose);
}

/**
 * This node converts LOAMs PointCloud that contains the current Pose (3d only!)
 * into a nav_msgs/Path and publishes as evaluation/SavePose to the save_pose_node as well
 *
 * @param argc number of args
 * @param argv args
 * @return 0 if success
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loam_pose_saver");
    ros::NodeHandle n("~");
    pose_path_pub = n.advertise<nav_msgs::Path>("/evaluation/loam_path", 1000);
    save_path_pub = n.advertise<evaluation::SavePoseStamped>("/evaluation/save_pose", 1000);
    ros::Subscriber sub = n.subscribe("/key_pose_origin", 1000, pointCloud2Callback);

    ROS_INFO_STREAM("Node listening to LOAM data at /key_pose_origin");

    ros::spin();

    return 0;
}
