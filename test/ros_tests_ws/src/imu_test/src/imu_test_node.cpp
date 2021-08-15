//
// Created by julian on 8/18/20.
//

#include <iostream>
#include <imu.h>
#include <msg/imu.h>
#include <util/time.h>
#include <util/concurrent_ring_buffer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

namespace fs = fastsense;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data_raw", 5);
    ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 5);

    fs::msg::ImuStampedBuffer::Ptr imu_buffer = std::make_shared<fs::msg::ImuStampedBuffer>(100);
    fs::driver::Imu imu(imu_buffer, 25);
    imu.start();

    auto last = fs::util::HighResTime::now();

    fs::msg::ImuStamped data_stamped;
    while (ros::ok())
    {
        if (!imu_buffer->pop_nb(&data_stamped, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        auto& [data, time] = data_stamped;

//        std::cout << data << "\n--\n";

        auto time_now = ros::Time();
        std::cout << "Time since last msg: " << std::chrono::duration_cast<std::chrono::milliseconds>(time - last).count() << "\n";
        last = time;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu";
        imu_msg.header.stamp = time_now;
        imu_msg.linear_acceleration.x = data.acc.x();
        imu_msg.linear_acceleration.y = data.acc.y();
        imu_msg.linear_acceleration.z = data.acc.z();

        // set angular velocities
        imu_msg.angular_velocity.x = data.ang.x();
        imu_msg.angular_velocity.y = data.ang.y();
        imu_msg.angular_velocity.z = data.ang.z();

        auto& cov1 = imu.get_linear_acceleration_covariance();
        std::copy(cov1.begin(), cov1.end(), imu_msg.linear_acceleration_covariance.begin());

        auto& cov2 = imu.get_angular_velocity_covariance();
        std::copy(cov2.begin(), cov2.end(), imu_msg.angular_velocity_covariance.begin());

        imu_pub.publish(imu_msg);

        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.frame_id = "imu";
        mag_msg.header.stamp = time_now;
        std::array<double, 9> a;
        auto& cov = imu.get_magnetic_field_covariance();
        std::copy(cov.begin(), cov.end(), mag_msg.magnetic_field_covariance.begin());
        mag_msg.magnetic_field.x = data.mag.x();
        mag_msg.magnetic_field.y = data.mag.y();
        mag_msg.magnetic_field.z = data.mag.z();

        mag_pub.publish(mag_msg);
    }

    return 0;
}
