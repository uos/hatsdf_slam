#pragma once

/**
 * @file point_cloud.h
 * @author Marcel Flottmann
 */

#include <vector>
#include <memory>

#include "stamped.h"
#include <util/time.h>
#include <util/point.h>
#include <msg/zmq_converter.h>
#include <util/constants.h>

#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

/**
 * @brief A point cloud with fixed rings
 *
 * All columns are stored sequentially. A column consists of a point from every ring.
 * So the data is stored as following (C3R4 = column 3 ring 4)
 * [ C1R1 C1R2 C1R3 C1R4 C2R1 C2R2 C2R3 C2R4 ... ]
 */
class PointCloud : public ZMQConverter
{
public:
    using Ptr = std::shared_ptr<PointCloud>;
    
    /// default constructor
    PointCloud() 
    : points_{}
    , rings_{}
    , scaling_{1.0f}
    {
    }

    /// constructor with scaling
    PointCloud(float scaling) 
    : points_{}
    , rings_{}
    , scaling_{scaling}
    {
    }
    
    /// move constructor
    PointCloud(PointCloud&& pcl) noexcept
    : points_{std::move(pcl.points_)}
    , rings_{pcl.rings_}
    , scaling_{pcl.scaling_}
    {
    }

    /// move assignment
    PointCloud& operator=(PointCloud&& other) noexcept
    {
        points_ = std::move(other.points_);
        rings_ = other.rings_;
        scaling_ = other.scaling_;
        return *this;
    }

    /// /copy assignment 
    PointCloud& operator=(const PointCloud& other)
    {
        points_ = other.points_;
        rings_ = other.rings_;
        scaling_ = other.scaling_;
        return *this;
    }

    /// Copy constructor
    PointCloud(const PointCloud &p)
    : points_{p.points_}
    , rings_{p.rings_}
    , scaling_{p.scaling_}
    {
    }

    /// Default Destructor
    ~PointCloud() override = default;

    /// Points
    std::vector<fastsense::ScanPoint> points_;

    /// Rings
    uint16_t rings_;

    /// Scaling factor
    float scaling_;

    /**
     * @brief Turn incoming zmq message into PCL
     * 
     * @param msg zmq multipart msg
     */
    void from_zmq_msg(zmq::multipart_t& msg) override
    {
        rings_ = msg.poptyp<uint16_t>();

        zmq::message_t point_msg = msg.pop();
        size_t n_points = point_msg.size() / sizeof(fastsense::ScanPoint);
        points_.clear();
        points_.reserve(n_points);
        std::copy_n(static_cast<fastsense::ScanPoint*>(point_msg.data()), n_points, std::back_inserter(points_));

        scaling_ = msg.poptyp<float>();
    }

    /**
     * @brief Turn pointcloud into zmq mulitpart message
     * 
     * @return zmq::multipart_t zmq mulitpart message
     */
    [[nodiscard]]
    zmq::multipart_t to_zmq_msg() const override
    {
        zmq::multipart_t multi;
        multi.addtyp(rings_);
        multi.add(zmq::message_t(points_.begin(), points_.end()));
        multi.addtyp(scaling_);
        return multi;
    }
};

using PointCloudStamped = Stamped<PointCloud>;
using PointCloudPtrStamped = Stamped<PointCloud::Ptr>;
using PointCloudPtrStampedBuffer = util::ConcurrentRingBuffer<PointCloudPtrStamped>;

} // namespace fastsense::msg;
