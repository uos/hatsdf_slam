#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

constexpr auto TOTAL_RINGS = 16u;

struct PCL2PCL2
{   
    /**
     * @brief Construct a new PCL2PCL2 object
     * 
     * @param n Nodehandle
     * @param topic topic to publish to
     */
    PCL2PCL2(ros::Publisher& pub) : pcl2pub{pub} {}
    
    /// default destructor
    ~PCL2PCL2() = default;

    double opening_angle(const geometry_msgs::Point32& point)
    {
        double norm = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        return std::acos(point.z / norm);
    }



    /**
     * @brief Convert PointCloud to PointCloud2
     * 
     * @param pcl incoming PointCloud
     */
    void pclCallback(const sensor_msgs::PointCloud::ConstPtr& pc_ptr)
    {
        sensor_msgs::PointCloud2 pc2;

        pc2.header = pc_ptr->header;
        pc2.height = 1;
        pc2.width = pc_ptr->points.size();
        
        pc2.fields.resize(5);

        pc2.fields[0].name = "x";
        pc2.fields[1].name = "y";
        pc2.fields[2].name = "z";
        pc2.fields[3].name = "intensity";
        pc2.fields[4].name = "ring";

        auto offset = 0u;

        for (auto index = 0u; index < 3; ++index, offset += sizeof(float))
        {
            pc2.fields[index].offset = offset;
            pc2.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
            pc2.fields[index].count = 1;
        }

        pc2.fields[3].offset = 16;
        pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        pc2.fields[4].count = 1;

        pc2.fields[4].offset = 20;
        pc2.fields[4].datatype = sensor_msgs::PointField::UINT16;
        pc2.fields[4].count = 1;

        pc2.is_bigendian = false;
        pc2.point_step = 22;
        pc2.row_step = pc2.width * pc2.point_step;
        pc2.data.resize(pc2.row_step * pc2.height);
        pc2.is_dense = true;

        sensor_msgs::PointCloud2Iterator<float> iter_x(pc2, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc2, "intensity");
        sensor_msgs::PointCloud2Iterator<int> iter_ring(pc2, "ring");

        const auto& pc_points = pc_ptr->points;

        std::vector<std::vector<std::vector<float>>> points(TOTAL_RINGS);
        std::map<double, int> ring_order;
        
        std::array<bool, TOTAL_RINGS> ring_valid;
        ring_valid.fill(false);

        for (auto index = 0u; index < pc_points.size(); ++index)
        {
            std::vector<float> point(3);

            point[0] = pc_points[index].x;
            point[1] = pc_points[index].y;
            point[2] = pc_points[index].z;

            auto ring = index % TOTAL_RINGS;

            if (!ring_valid[ring] && (point[0] != 0.0 || point[1] != 0.0 || point[2] != 0.0))
            {
                double angle = opening_angle(pc_points[index]);

                ring_order.insert(std::make_pair(angle, ring));
                ring_valid[ring] = true;
            }

            points[ring].push_back(point);
        }

        std::vector<std::vector<std::vector<float>>> ring_order_points(TOTAL_RINGS);

        auto index = 0u;

        if (ring_order.size() == 0)
        {
            std::cerr << "Could not find valid points in the cloud!" << std::endl;
            return;
        }

        for (const auto& entry : ring_order)
        {
            auto ring = entry.second;

            if ( ring_valid[ring])
            {
                ring_order_points[index] = std::move(points[ring]);
            }

            ++index;
        }

        for(; index < TOTAL_RINGS; ++index)
        {
            ring_order_points[index] = std::vector<std::vector<float>>(ring_order_points[0].size(), std::vector<float>(3, 0.0));
        }



        for (auto ring = 0u; ring < ring_order_points.size(); ++ring)
        {
            for (const auto& point : ring_order_points[ring])
            {
                iter_x[0] = point[0];
                iter_x[1] = point[1];
                iter_x[2] =  1.0 / (ring + 1.0);
                iter_intensity[0] = 1.0 / (ring + 1.0);
                iter_ring[0] = ring;

                ++iter_x;
                ++iter_intensity;
                ++iter_ring;
            }
        }

        pc2.header.stamp = ros::Time::now();

        // publish the generated pcl
        pcl2pub.publish(pc2);
    }

    // publishes PointCloud2
    ros::Publisher& pcl2pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl2pcl2");
    ros::NodeHandle n("~");

    std::string intopic;
    n.param("intopic", intopic, std::string("pcl"));

    std::string outtopic;
    n.param("outtopic", outtopic, std::string("pcl2"));

    ROS_INFO_STREAM("Publishing pcl from '" << intopic << "' as pcl2 on '" << outtopic << "'.");

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(outtopic, 1000);
    PCL2PCL2 pcl2pcl2(pub);
    ros::Subscriber sub = n.subscribe(intopic, 1000, &PCL2PCL2::pclCallback, &pcl2pcl2);

    ros::spin();

    return 0;
}