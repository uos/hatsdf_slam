#pragma once

/**
 * @file tsdf.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#include "zmq_converter.h"
#include <vector>
#include <algorithm>

#include <util/point.h>
#include <util/tsdf.h>
#include <msg/stamped.h>

namespace fastsense::msg
{

/**
 * @brief Represents TSDF Bridge Message
 * 
 * Inherits from ZMQConverter, because it contains a dynamic data type (std::vector)
 */
struct TSDF : public ZMQConverter
{
    /// default constructor
    TSDF()
    : tau_{}
    , size_{}
    , pos_{}
    , offset_{}
    , scaling_{1.f}
    , tsdf_data_{}
    {
    }

    /// default destructor
    ~TSDF() override = default;

    /// copy assignment operator
    TSDF& operator=(const TSDF& other) = default;

    /// default move assignment operator
    TSDF& operator=(TSDF&&) noexcept = default;

    /// default copy constructor
    TSDF(const TSDF&) = default;

    /// default move constructor
    TSDF(TSDF&&) noexcept = default;

    /// truncation distance
    float tau_;

    /// size of map 
    Vector3i size_;

    /// global position
    Vector3i pos_;
    
    /// shift offset
    Vector3i offset_;

    /// scaling
    float scaling_;

    /// actual tsdf data
    std::vector<TSDFValue> tsdf_data_;

    /**
     * @brief Convert zmq_multipart to TSDF
     * 
     * @param msg multipart message
     */
    void from_zmq_msg(zmq::multipart_t& msg)
    {
        tau_ = msg.poptyp<float>();
        size_ = msg.poptyp<Vector3i>();
        pos_ = msg.poptyp<Vector3i>();
        offset_ = msg.poptyp<Vector3i>();
        scaling_ = msg.poptyp<float>();

        zmq::message_t tsdf_data_msg = msg.pop();
        size_t n_tsdf_values = tsdf_data_msg.size() / sizeof(TSDFValue);
        tsdf_data_.clear();
        tsdf_data_.reserve(n_tsdf_values);
        std::copy_n(static_cast<TSDFValue*>(tsdf_data_msg.data()), n_tsdf_values, std::back_inserter(tsdf_data_));
    }

    /**
     * @brief Convert TSDF to multipart
     * 
     * @return zmq::multipart_t zmw multipart message
     */
    zmq::multipart_t to_zmq_msg() const
    {
        zmq::multipart_t multi;
        multi.addtyp(tau_);
        multi.addtyp(size_);
        multi.addtyp(pos_);
        multi.addtyp(offset_);
        multi.addtyp(scaling_);
        multi.add(zmq::message_t(tsdf_data_.begin(), tsdf_data_.end()));
        return multi;
    }
};

using TSDFStamped = Stamped<TSDF>;

} // namespace fastsense::msg