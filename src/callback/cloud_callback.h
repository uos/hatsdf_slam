#pragma once

/**
 * @file cloud_callback.h
 * @author Pascal Buschermöhle
 */

#include <util/point_hw.h>
#include <msg/transform.h>
#include <map/local_map.h>
#include <eigen3/Eigen/Dense>
#include <util/process_thread.h>
#include <msg/tsdf.h>
#include <tsdf/krnl_tsdf.h>
#include <registration/registration.h>
#include <util/config/config_manager.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/point_cloud.h>
#include <callback/map_thread.h>

namespace fastsense::callback
{

using Registration = fastsense::registration::Registration;
using PointCloudBuffer = fastsense::util::ConcurrentRingBuffer<fastsense::msg::PointCloudPtrStamped>;
using fastsense::map::LocalMap;
using fastsense::map::GlobalMap;
using Eigen::Matrix4f;
using fastsense::util::config::ConfigManager;
using fastsense::buffer::InputBuffer;

class CloudCallback : public fastsense::util::ProcessThread
{
public:
    CloudCallback(Registration& registration,
                  const std::shared_ptr<PointCloudBuffer>& cloud_buffer,
                  const std::shared_ptr<LocalMap>& local_map,
                  const std::shared_ptr<GlobalMap>& global_map,
                  const msg::TransformStampedBuffer::Ptr& transform_buffer,
                  const msg::PointCloudPtrStampedBuffer::Ptr& pointcloud_buffer,
                  bool send_after_registration,
                  const fastsense::CommandQueuePtr& q,
                  MapThread& map_thread,
                  std::mutex& map_mutex,
                  float point_scale);

    /**
     * @brief Set the global map
     * 
     * @param global_map the new global map
     */
    void set_global_map(const std::shared_ptr<GlobalMap>& global_map);

    /**
     * @brief Set the local map
     * 
     * @param local_map the new local map
     */
    void set_local_map(const std::shared_ptr<LocalMap>& local_map);

protected:
    void thread_run() override;

private:
    Registration& registration;
    std::shared_ptr<PointCloudBuffer> cloud_buffer;
    std::shared_ptr<LocalMap> local_map;
    std::shared_ptr<GlobalMap> global_map;
    Matrix4f pose;
    msg::TransformStampedBuffer::Ptr transform_buffer;
    std::shared_ptr<PointCloudBuffer> pointcloud_buffer;
    bool send_after_registration;
    bool first_iteration;
    fastsense::CommandQueuePtr q;
    MapThread& map_thread;
    std::mutex& map_mutex;
    float point_scale;
};

}
