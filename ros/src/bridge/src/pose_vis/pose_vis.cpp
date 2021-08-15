#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <msg/transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <random>
#include <fstream>
#include <memory>

auto pose_counter = 0u;
nav_msgs::Path original_pose_path;
std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;

void cloud_callback(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
    if (broadcaster == nullptr)
    {
        ROS_ERROR_STREAM("TF broadcaster not initialized!");
        return;
    }

    if (pose_counter < original_pose_path.poses.size())
    {
        auto& pose_stamped = original_pose_path.poses[pose_counter];

        geometry_msgs::TransformStamped transform_data;
        transform_data.header.frame_id = "map";
        transform_data.child_frame_id = "ground_truth";

        transform_data.transform.rotation.x = pose_stamped.pose.orientation.x;
        transform_data.transform.rotation.y = pose_stamped.pose.orientation.y;
        transform_data.transform.rotation.z = pose_stamped.pose.orientation.z;
        transform_data.transform.rotation.w = pose_stamped.pose.orientation.w;
        transform_data.transform.translation.x = pose_stamped.pose.position.x;
        transform_data.transform.translation.y = pose_stamped.pose.position.y;
        transform_data.transform.translation.z = pose_stamped.pose.position.z;

        transform_data.header.stamp = ros::Time::now();
        broadcaster->sendTransform(transform_data);
    }


    ++pose_counter;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_vis");
    ros::NodeHandle n("~");

    std::string pose_file;
    double scale = 1.0;
    bool matrix_mode = false;

    if (!n.getParam("pose_file", pose_file))
    {
        std::cout << "Pose file is required!" << std::endl;
        return 0;
    }

    n.getParam("scale", scale);
    n.getParam("matrix_mode", matrix_mode);

    std::ifstream pose_stream(pose_file);
    if(!pose_stream)
    {
        std::cerr << "Cannot open " << pose_file << '!' << std::endl;
        return 1;
    }

    /// Path message
    nav_msgs::Path pose_path;
    pose_path.header.frame_id = "map";

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();

    // bool first = true;

    // geometry_msgs::TransformStamped transform_data;
    // transform_data.header.frame_id = "map";
    // transform_data.child_frame_id = "ground_truth";

    std::cout << "Loading poses..." << std::endl;

    auto pose_count = 0u;

    while (!pose_stream.eof())
    {
        std::string line;
        std::getline(pose_stream, line);

        std::istringstream ss(line);

        if (!pose_stream)
        {
            continue;
        }

        if (matrix_mode)
        {
            double r00, r01, r02, 
                   r10, r11, r12,
                   r20, r21, r22,
                    t0,  t1,  t2;

            ss >> r00 >> r01 >> r02 >> t0
               >> r10 >> r11 >> r12 >> t1
               >> r20 >> r21 >> r22 >> t2;

            tf2::Matrix3x3 mat(r00, r01, r02, 
                               r10, r11, r12,
                               r20, r21, r22);

            tf2::Transform transform(mat, tf2::Vector3(t0, t1, t2));
            tf2::toMsg(transform, pose_stamped.pose);

            auto tmp = pose_stamped.pose;

            pose_stamped.pose.position.x = tmp.position.z;
            pose_stamped.pose.position.y = -tmp.position.x;
            pose_stamped.pose.position.z = tmp.position.y;

            pose_stamped.pose.orientation.x = -tmp.orientation.z;
            pose_stamped.pose.orientation.y = -tmp.orientation.x;
            pose_stamped.pose.orientation.z = -tmp.orientation.y;
        }
        else
        {
            ss >> pose_stamped.pose.position.x >> pose_stamped.pose.position.y >> pose_stamped.pose.position.z
               >> pose_stamped.pose.orientation.x >> pose_stamped.pose.orientation.y >> pose_stamped.pose.orientation.z 
               >> pose_stamped.pose.orientation.w;
        } 

        if (!ss)
        {
            ROS_ERROR_STREAM("Pose information corrupted!");
            pose_stream.close();
            return 1;
        }

        pose_stamped.pose.position.x *= scale;
        pose_stamped.pose.position.y *= scale;
        pose_stamped.pose.position.z *= scale;

        // if (first)
        // {
        //     transform_data.transform.rotation.x = pose_stamped.pose.orientation.x;
        //     transform_data.transform.rotation.y = pose_stamped.pose.orientation.y;
        //     transform_data.transform.rotation.z = pose_stamped.pose.orientation.z;
        //     transform_data.transform.rotation.w = pose_stamped.pose.orientation.w;
        //     transform_data.transform.translation.x = pose_stamped.pose.position.x;
        //     transform_data.transform.translation.y = pose_stamped.pose.position.y;
        //     transform_data.transform.translation.z = pose_stamped.pose.position.z;

        //     first = false;
        // }

        pose_path.poses.push_back(pose_stamped);
        ++pose_counter;
    }


    pose_stream.close();

    ROS_INFO_STREAM("Read " << pose_counter << " poses");

    original_pose_path = pose_path;

    ros::Publisher pub = n.advertise<nav_msgs::Path>("pose", 10);

    //tf2_ros::TransformBroadcaster broadcaster;
    broadcaster.reset(new tf2_ros::TransformBroadcaster());

    auto cloud_sub = n.subscribe<sensor_msgs::PointCloud>("/from_trenz_bridge/velodyne/points", 1, cloud_callback);

    std::cout << "Prepare for visualization..." << std::endl;

    while (pose_path.poses.size() >= 16380)
    {
        static std::default_random_engine rng((std::random_device())());

        // remove a random pose, but not the start nor a recent pose
        std::uniform_int_distribution dist(1, (int)pose_path.poses.size() - 1000);
        int index = dist(rng);
        pose_path.poses.erase(pose_path.poses.begin() + index);
    }

    ros::Duration duration(0.01);

    std::cout << "Visualize poses..." << std::endl;

    while (ros::ok())
    {
        pose_path.header.stamp = ros::Time::now();
        // transform_data.header.stamp = pose_path.header.stamp;
        // broadcaster->sendTransform(transform_data);
        pub.publish(pose_path);
        ros::spinOnce();
        duration.sleep();
    }

    return 0;
}