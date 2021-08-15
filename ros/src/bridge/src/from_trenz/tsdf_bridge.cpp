/**
 * @file tsdf_bridge.cpp
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include <omp.h>
#include <iterator>
#include <ros/ros.h>
#include <bridge/from_trenz/tsdf_bridge.h>
#include <util/constants.h>
#include <bridge/util.h>

using namespace fastsense::bridge;

TSDFBridge::TSDFBridge( ros::NodeHandle& n, 
                        const std::string& board_addr, 
                        std::chrono::milliseconds timeout, 
                        bool discard_timestamp)
:   BridgeBase{n, "tsdf", board_addr, 1000, timeout},
    ProcessThread{},
    points_{},
    colors_{},
    discard_timestamp_{discard_timestamp}
{
    points_.reserve(30'000);
    colors_.reserve(30'000);
}

void TSDFBridge::run()
{   
    while (running && ros::ok())
    {
        try
        {
            if (receive())
            {
                ROS_DEBUG_STREAM("Received tsdf values\n");
                convert();
                publish();
            }
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("tsdf bridge error: " << e.what());
        }
    }
}

// TODO in util, so steffen and my version match
bool TSDFBridge::in_bounds(int x, int y, int z) const
{
    const auto& data = msg().data_;
    return abs(x - data.pos_[0] <= data.size_[0] / 2 && abs(y - data.pos_[1]) <= data.size_[1] / 2 && abs(z - data.pos_[2]) <= data.size_[2] / 2);
}

TSDFValue TSDFBridge::get_tsdf_value(int x, int y, int z) const
{
    const auto& data = msg().data_;

    if (!in_bounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }

    if (data.tsdf_data_.empty())
    {
        throw std::out_of_range("Index out of bounds: data empty");
    }
    
    return data.tsdf_data_[((x - data.pos_[0] + data.offset_[0] + data.size_[0]) % data.size_[0]) * data.size_[1] * data.size_[2] +
                     ((y - data.pos_[1] + data.offset_[1] + data.size_[1]) % data.size_[1]) * data.size_[2] +
                     (z - data.pos_[2] + data.offset_[2] + data.size_[2]) % data.size_[2]];
}

void TSDFBridge::convert()
{   
    constexpr size_t thread_count = 8;
    std::vector<std::pair<geometry_msgs::Point, std_msgs::ColorRGBA>> results[thread_count];

    int left[3], right[3];

    const auto& data = msg().data_;
    const float& scaling = data.scaling_;

    for (size_t i = 0; i < 3; i++)
    {
        left[i] = data.pos_[i] - data.size_[i] / 2;
        right[i] = data.pos_[i] + data.size_[i] / 2;
    }

    #pragma omp parallel num_threads(thread_count)
    {
        auto& result = results[omp_get_thread_num()];
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.b = 0;

        #pragma omp for collapse(3) schedule(static)
        for (int x = left[0]; x <= right[0]; x++)
        {
            for (int y = left[1]; y <= right[1]; y++)
            {
                for (int z = left[2]; z <= right[2]; z++)
                {
                    auto val = get_tsdf_value(x, y, z);
                    if (val.weight() == 0 || fabsf(val.value()) >= data.tau_)
                    {
                        continue;
                    }

                    geometry_msgs::Point point;
                    point.x = x * MAP_RESOLUTION * 0.001 / scaling;
                    point.y = y * MAP_RESOLUTION * 0.001 / scaling;
                    point.z = z * MAP_RESOLUTION * 0.001 / scaling;

                    // color.a = std::min(val.weight(), 1.0f);
                    if (val.value() >= 0)
                    {
                        color.r = val.value() / data.tau_;
                        color.g = 0;
                    }
                    else
                    {
                        color.r = 0;
                        color.g = -val.value() / data.tau_;
                    }

                    result.push_back(std::make_pair(point, color));
                }
            }
        }
    }

    std::vector<int> offsets(thread_count, 0);
    size_t total_results = 0;
    for (size_t i = 0; i < thread_count; i++)
    {
        offsets[i] = total_results;
        total_results += results[i].size();
    }
    
    if (total_results == 0)
    {
        return;
    }

    points_.resize(total_results);
    colors_.resize(total_results);

    #pragma omp parallel num_threads(thread_count)
    {   
        auto& result = results[omp_get_thread_num()];
        int offset = offsets[omp_get_thread_num()];
        for (size_t i = 0; i < result.size(); i++)
        {
            auto& p = result[i];
            points_[i + offset] = p.first;
            colors_[i + offset] = p.second;
        }
    }
}

void TSDFBridge::publish()
{
    const auto& timestamp = msg().timestamp_;
    const float& scaling = msg().data_.scaling_;

    visualization_msgs::Marker marker;
    marker.header = std_msgs::Header{};
    marker.header.stamp = timestamp_to_rostime(timestamp, discard_timestamp_);
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "window";
    marker.id = 0;
    marker.scale.x = marker.scale.y = MAP_RESOLUTION * 0.6 * 0.001 / scaling;
    marker.points = points_;
    marker.colors = colors_;
    pub().publish(marker);

    points_.clear();
    colors_.clear();
}