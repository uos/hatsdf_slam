/**
 * @file preprocessing.cpp
 * @author Pascal Buschermöhle
 * @author Malte Hillmann
 */

#include "preprocessing.h"
#include <util/logging/logger.h>
#include <util/config/config_manager.h>

#include <unordered_set>

using namespace fastsense::preprocessing;
using fastsense::buffer::InputBuffer;
using fastsense::util::logging::Logger;

namespace std
{
template<> struct hash<fastsense::ScanPoint>
{
    std::size_t operator()(fastsense::ScanPoint const& p) const noexcept
    {
        long long v = ((long long)p.x() << 32) ^ ((long long)p.y() << 16) ^ (long long)p.z();
        return std::hash<long long>()(v);
    }
};
}


Preprocessing::Preprocessing(const std::shared_ptr<PointCloudBuffer>& in_buffer,
                             const std::shared_ptr<PointCloudBuffer>& out_buffer,
                             const std::shared_ptr<PointCloudBuffer>& send_buffer,
                             bool send_original,
                             bool send_preprocessed,
                             float scale)
    : QueueBridge{in_buffer, out_buffer, 0, false}, send_original(send_original), send_preprocessed(send_preprocessed), send_buffer(send_buffer), scale(scale), 
      map_bounds(util::config::ConfigManager::config().slam.map_size_x() / 2 * MAP_RESOLUTION, 
                 util::config::ConfigManager::config().slam.map_size_y() / 2 * MAP_RESOLUTION, 
                 util::config::ConfigManager::config().slam.map_size_z() / 2 * MAP_RESOLUTION)
{

}

void Preprocessing::thread_run()
{
    fastsense::msg::PointCloudPtrStamped in_cloud;
    while (this->running)
    {
        if (!this->in_->pop_nb(&in_cloud, DEFAULT_POP_TIMEOUT))
        {
            continue;
        }

        fastsense::msg::PointCloudPtrStamped out_cloud;
        out_cloud.data_ = in_cloud.data_;
        out_cloud.timestamp_ = in_cloud.timestamp_;

        if (scale != 1.0f)
        {
            for (auto& point : out_cloud.data_->points_)
            {
                point = (point.cast<float>() * scale).cast<ScanPointType>();
            }
        }

        if (send_original)
        {
            in_cloud.data_->scaling_ = scale;
            send_buffer->push_nb(in_cloud);
        }


        /*if (util::config::ConfigManager::config().lidar.rings() == out_cloud.data_->rings_)
        {
            median_filter(out_cloud, 5);
        }
        else
        {
            Logger::warning("Expected rings not equal to received rings! Skipping median filter");
        }*/
        
        reduction_filter_closest(out_cloud);
        this->out_->push_nb(out_cloud, true);

        if (send_preprocessed)
        {
            out_cloud.data_->scaling_ = scale;
            send_buffer->push_nb(out_cloud);
        }
    }
}

void Preprocessing::reduction_filter_average(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_map<ScanPoint, std::pair<Vector3i, int>> point_map;
    std::pair<Vector3i, int> default_value = std::make_pair(Vector3i::Zero(), 0);

    point_map.reserve(cloud_points.size());

    for (const auto& point : cloud_points)
    {
        if ((point.x() == 0 && point.y() == 0 && point.z() == 0) || map_bounds.x() < std::abs(point.x()) || map_bounds.y() < std::abs(point.y()) || map_bounds.z() < std::abs(point.z()))
        {
            continue;
        }

        ScanPoint voxel(
            std::floor((float)point.x() / MAP_RESOLUTION),
            std::floor((float)point.y() / MAP_RESOLUTION),
            std::floor((float)point.z() / MAP_RESOLUTION)
        );

        auto& avg_point = point_map.try_emplace(voxel, default_value).first->second;
        avg_point.first += point.cast<int>();
        avg_point.second++;
    }

    cloud_points.resize(point_map.size());
    int counter = 0;
    for (auto& avg_point : point_map)
    {
        cloud_points[counter] = (avg_point.second.first / avg_point.second.second).cast<ScanPointType>();
        counter++;
    }
}

void Preprocessing::reduction_filter_closest(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_map<ScanPoint, std::pair<ScanPoint, int>> point_map;
    std::pair<ScanPoint, int> default_value = std::make_pair(ScanPoint::Zero(), MAP_RESOLUTION * 2);

    point_map.reserve(cloud_points.size());

    for (const auto& point : cloud_points)
    {
        if ((point.x() == 0 && point.y() == 0 && point.z() == 0) || map_bounds.x() < std::abs(point.x()) || map_bounds.y() < std::abs(point.y()) || map_bounds.z() < std::abs(point.z()))
        {
            continue;
        }

        ScanPoint voxel_center(
            std::floor((float)point.x() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.y() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.z() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2
        );
        int distance = (point - voxel_center).norm();

        auto& closest = point_map.try_emplace(voxel_center, default_value).first->second;
        if (distance < closest.second)
        {
            closest.first = point;
            closest.second = distance;
        }
    }

    cloud_points.resize(point_map.size());
    int counter = 0;
    for (auto& avg_point : point_map)
    {
        cloud_points[counter] = avg_point.second.first;
        counter++;
    }
}

void Preprocessing::reduction_filter_voxel_center(fastsense::msg::PointCloudPtrStamped& cloud)
{
    auto& cloud_points = cloud.data_->points_;

    std::unordered_set<ScanPoint> point_set;

    for (const auto& point : cloud_points)
    {
        if ((point.x() == 0 && point.y() == 0 && point.z() == 0) || map_bounds.x() < std::abs(point.x()) || map_bounds.y() < std::abs(point.y()) || map_bounds.z() < std::abs(point.z()))
        {
            continue;
        }

        ScanPoint voxel_center(
            std::floor((float)point.x() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.y() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2,
            std::floor((float)point.z() / MAP_RESOLUTION) * MAP_RESOLUTION + MAP_RESOLUTION / 2
        );

        point_set.insert(voxel_center);
    }

    cloud_points.resize(point_set.size());
    std::copy(point_set.begin(), point_set.end(), cloud_points.begin());
}


void Preprocessing::reduction_filter_random_point(fastsense::msg::PointCloudPtrStamped& cloud)
{

    auto& cloud_points = cloud.data_->points_;

    std::unordered_map<ScanPoint, ScanPoint> point_map;

    std::random_shuffle(cloud_points.begin(), cloud_points.end());

    for (const auto& point : cloud_points)
    {
        if (point.x() == 0 && point.y() == 0 && point.z() == 0)
        {
            continue;
        }

        ScanPoint voxel(
            std::floor((float)point.x() / MAP_RESOLUTION),
            std::floor((float)point.y() / MAP_RESOLUTION),
            std::floor((float)point.z() / MAP_RESOLUTION)
        );

        point_map.try_emplace(voxel, point);
    }

    cloud_points.resize(point_map.size());
    int counter = 0;
    for (auto& c_point : point_map)
    {
        cloud_points[counter] = c_point.second;
        counter++;
    }
}


uint8_t Preprocessing::median_from_array(std::vector<ScanPoint*> medians)
{
    std::vector<std::pair<int, double>> distances(medians.size());

    for (uint8_t i = 0; i < distances.size(); i++)
    {
        distances[i].first = i;
        distances[i].second = medians[i]->norm();
    }

    std::nth_element(distances.begin(), distances.begin() + distances.size() / 2, distances.end(), [](auto & left, auto & right)
    {
        return left.second < right.second;
    });

    return distances[distances.size() / 2].first;
}

void Preprocessing::median_filter(fastsense::msg::PointCloudPtrStamped& cloud, uint8_t window_size)
{
    if (window_size % 2 == 0)
    {
        Logger::warning("Median filter window must be % 2 == 1, but isn't. Skipping.");
        return;
    }

    std::vector<ScanPoint> result(cloud.data_->points_.size());

    int half_window_size = window_size / 2;

    #pragma omp parallel for schedule(static) shared(result)
    for (uint8_t ring = 0; ring < cloud.data_->rings_; ring++)
    {
        std::vector<ScanPoint*> window(window_size);
        for (uint32_t point = 0; point < (cloud.data_->points_.size() / cloud.data_->rings_); point++)
        {
            int i = (point * cloud.data_->rings_) + ring;
            int first_element = (i - (half_window_size * cloud.data_->rings_));
            for (uint8_t j = 0; j < window_size; j++)
            {
                int index = ((first_element + (j * cloud.data_->rings_)) + cloud.data_->points_.size()) % cloud.data_->points_.size();
                window[j] = (ScanPoint*)&cloud.data_->points_[index];
            }

            result[i] = *window[median_from_array(window)];

        }
    }

    cloud.data_->points_ = result;
}