#include <ros/ros.h>
//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>

ros::Publisher cloud_pub;

struct MyPoint {
    float x;
    float y;
    float z;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_to_ros");
    ros::NodeHandle n("~");

    std::string dir;
    double time_rate;

    if (!n.getParam("dir", dir))
    {
        std::cout << "Please enter a directory as argument" << std::endl;
        return 0;
    }

    if (!n.getParam("rate", time_rate))
    {
        time_rate = 1.0;
    }

    std::cout << "Starting with rate " << time_rate << std::endl;

    std::ifstream time_stream(dir + std::string("/times.txt"));

    if(!time_stream)
    {
        std::cout << "Please enter a valid KITTI dataset folder with a \"time.txt\" in it!" << std::endl;
        return 0;
    }

    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);

    sensor_msgs::PointCloud2 cloud;

    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    
    // Define point structure

    // Describe your custom MyPoint object
    cloud.fields.resize(3);
    // MyPoint.x
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    // MyPoint.y
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    // MyPoint.z
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    int count = 0;

    double last_stamp = 0;
    double stamp = 0;

    while(ros::ok() && count < 10000)
    {
        // load point cloud
        FILE *stream;

        std::stringstream ss;

        //ss << std::string("/home/fastsense/ros_ws/kitti_velodyne/");
        ss << dir;
        ss << std::setw(6) << std::setfill('0') << count;
        ss << std::string(".bin");

        std::string file_name =  ss.str();

        if(!(time_stream >> stamp))
        {
            break;
        }

        std::cout << "Loading: " << file_name << std::endl;

        stream = fopen (file_name.c_str(),"rb");

        if (stream == NULL)
        {
            break;
        }

        // allocate 4 MB buffer (only ~130*4*4 KB are needed)
        int32_t num = 1000000;

        float *data = (float*)malloc(num*sizeof(float));

        float *px = data+0;
        float *py = data+1;
        float *pz = data+2;
        float *pr = data+3;

        num = fread(data,sizeof(float),num,stream)/4;

        cloud.data.clear();

        cloud.width = num;
        cloud.point_step = sizeof(MyPoint);
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.data.resize(cloud.row_step * cloud.height);

        // interpret the data as MyPoint vector
        MyPoint* pc2_points = reinterpret_cast<MyPoint*>(cloud.data.data());


        for (int32_t i=0; i<num; i++) 
        {   
            MyPoint p;
            p.x = *px;
            p.y = *py;
            p.z = *pz;

            pc2_points[i] = p;

            px+=4; py+=4; pz+=4; pr+=4;
        }

        fclose(stream);
        delete[] data;

        auto time_diff = (stamp - last_stamp) / time_rate;
        auto duration = ros::Duration(time_diff);
        duration.sleep();

        cloud.header.stamp = ros::Time(stamp);
        cloud_pub.publish(cloud);

        double hz;

        if (time_diff != 0.0)
        {
            hz = 1.0 / time_diff;
        }
        else
        {
            hz = 0.0;
        }

        std::cout << hz << " hz" << std::endl;


        last_stamp = stamp;
        ++count;
    }

    std::cout << "KITTI dataset has ended" << std::endl;

    return 0;
}
