/**
 * @file map_thread.cpp
 * @author Steffen Hinderink, Marc Eisoldt
 */

#include <callback/map_thread.h>
#include <util/logging/logger.h>
#include <util/config/config_manager.h>
#include <util/runtime_evaluator.h>

namespace fastsense::callback
{

using fastsense::util::config::ConfigManager;
using fastsense::util::logging::Logger;

MapThread::MapThread(const std::shared_ptr<fastsense::map::LocalMap>& local_map,
                     std::mutex& map_mutex,
                     unsigned int period,
                     float position_threshold,
                     uint16_t port,
                     float scaling,
                     fastsense::CommandQueuePtr& q)
    : ProcessThread(),
      local_map_(local_map),
      tsdf_krnl_(q, local_map->getBuffer().size()),
      map_mutex_(map_mutex),
      active_(false),
      period_(period),
      position_threshold_(position_threshold),
      reg_cnt_(0),
      tsdf_msg_(),
      sender_(port),
      scaling_(scaling)
{
    /*
    Use the mutex as a 1-semaphore.
    wait = lock, signal = unlock.
    The mutex starts in the wrong state.
    */
    start_mutex_.lock();

    tsdf_msg_.data_.tsdf_data_.resize(local_map->getBuffer().size());
}

void MapThread::go(const Vector3i& pos, const Eigen::Matrix4f& pose, const fastsense::buffer::InputBuffer<PointHW>& points, int num_points)
{
    reg_cnt_++;
    const Vector3i& old_pos = local_map_->get_pos();
    float distance = ((pos.cast<float>() - old_pos.cast<float>()) * MAP_RESOLUTION).norm();

    bool position_condition = distance > position_threshold_;
    bool reg_cnt_condition = period_ > 0 && reg_cnt_ >= period_;
    if (!active_ && (position_condition || reg_cnt_condition))
    {
        pos_ = pos;
        pose_ = pose;
        if (points_ptr_ == nullptr || points_ptr_->size() != points.size())
        {
            points_ptr_.reset();
            points_ptr_.reset(new fastsense::buffer::InputBuffer<PointHW>(points));
        }
        else
        {
            points_ptr_->fill_from(points);
        }
        num_points_ = num_points;
        active_ = true;
        start_mutex_.unlock(); // signal
        reg_cnt_ = 0;
    }
}

void MapThread::thread_run()
{
    util::RuntimeEvaluator eval;
    map::LocalMap tmp_map(*local_map_);

    while (running)
    {
        start_mutex_.lock();
        if (!running)
        {
            break;
        }
        Logger::info("Starting SUV");

        // shift
        eval.start("shift");
        tmp_map.shift(pos_);
        eval.stop("shift");

        Matrix4i rotation_mat = Matrix4i::Identity();
        rotation_mat.block<3, 3>(0, 0) = ((pose_ * MATRIX_RESOLUTION).cast<int>()).block<3, 3>(0, 0);

        Eigen::Vector4i v;
        v << Vector3i(0, 0, MATRIX_RESOLUTION), 1;
        Vector3i up = (rotation_mat * v).block<3, 1>(0, 0) / MATRIX_RESOLUTION;

        PointHW up_hw(up.x(), up.y(), up.z());

        // tsdf update
        eval.start("tsdf");
        tsdf_krnl_.synchronized_run(tmp_map, *points_ptr_, num_points_, up_hw);
        eval.stop("tsdf");

        map_mutex_.lock();
        local_map_->swap(tmp_map);
        map_mutex_.unlock();

        eval.start("copy");
        tmp_map.fill_from(*local_map_);
        eval.stop("copy");

        // visualize
        eval.start("vis");
        tsdf_msg_.update_time();
        tsdf_msg_.data_.tau_ = ConfigManager::config().slam.max_distance();
        tsdf_msg_.data_.size_ = local_map_->get_size();
        tsdf_msg_.data_.pos_ = local_map_->get_pos();
        tsdf_msg_.data_.offset_ = local_map_->get_offset();
        tsdf_msg_.data_.scaling_ = scaling_;
        std::copy(local_map_->getBuffer().cbegin(), local_map_->getBuffer().cend(), tsdf_msg_.data_.tsdf_data_.data());
        sender_.send(tsdf_msg_);
        eval.stop("vis");

        Logger::info("Map Thread:\n", eval.to_string(), "\nStopping SUV");
        active_ = false;
    }
}

void MapThread::stop()
{
    if (running && worker.joinable())
    {
        running = false;
        start_mutex_.unlock();
        worker.join();
    }
}

void MapThread::set_local_map(const std::shared_ptr<fastsense::map::LocalMap>& local_map)
{
    local_map_ = local_map;
}

} // namespace fastsense::callback
