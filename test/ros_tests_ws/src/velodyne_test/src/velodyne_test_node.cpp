#include <driver/lidar/velodyne.h>
#include <msg/msgs_stamped.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <data/sensor_sync.h>
#include <msg/point_cloud.h>
#include <util/concurrent_ring_buffer.h>

#include <sstream>

using namespace fastsense;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1000);

    auto buffer = std::make_shared<util::ConcurrentRingBuffer<fastsense::msg::PointCloudStamped>>(16);
    fastsense::driver::VelodyneDriver v(2368, buffer);
    v.start();

    fastsense::msg::PointCloudStamped scan_stamped;
    while (ros::ok())
    {
        if (!buffer->pop_nb(&scan_stamped, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        auto& [scan, time] = scan_stamped;

        sensor_msgs::PointCloud pc;
        pc.header.frame_id = "world";

        std::transform(scan->points_.begin(), scan->points_.end(), std::back_inserter(pc.points), [] (const fastsense::ScanPoint & in)
        {
            geometry_msgs::Point32 out;
            out.x = in.x() * 0.001f;
            out.y = in.y() * 0.001f;
            out.z = in.z() * 0.001f;
            return out;
        });

        ROS_INFO("Scan: %zu Points", scan->points_.size());
        chatter_pub.publish(pc);

        ros::spinOnce();
    }

    v.stop();

    return 0;
}
