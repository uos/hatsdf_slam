#pragma once

/**
 * @file stamped.h
 * @author Julian Gaal
 */

#include <util/time.h>
#include <msg/zmq_converter.h>

namespace fastsense::msg 
{

/**
 * @brief Stamped class: Struct of data and its timestamp
 * 
 * @tparam DATA_T type of data at time timestamp
 */
template<typename DATA_T>
struct Stamped : public ZMQConverter
{
    /**
     * @brief Construct a new Stamped object
     * 
     * Default: default initialized data, timestamp NOW
     */
    Stamped()
    : data_{}
    , timestamp_{util::HighResTime::now()}
    {
    }

    /**
     * @brief Construct a new Stamped object
     * 
     * Constructing with pointer type is forbidden
     * 
     * @param data Data to store with given timestamp
     * @param timepoint timestamp of data creation/recording
     */
    explicit Stamped(DATA_T data, util::HighResTimePoint timepoint = util::HighResTime::now())
    : data_(std::move(data))
    , timestamp_(timepoint)
    {
          static_assert(!std::is_pointer<DATA_T>::value, "The data type of Stamped<T> must not be a pointer.");
    }

    /// default copy assignment operator
    Stamped& operator=(const Stamped& other) = default;

    /// dedault move assignment operator
    Stamped& operator=(Stamped&&) noexcept = default;

    /// dedault copy constructor
    Stamped(const Stamped&) = default;

    /// dedault move constructor
    Stamped(Stamped&&) noexcept = default;

    /**
     * @brief Destroy the Stamped object
     */
    virtual ~Stamped() = default;

    /**
     * @brief Override from_zmq_msg 
     * 
     * If data type T inherits ZMQConverter data and timestamp is built from a multipart message
     * if data type T does not inherit vom ZMQConverter, data and timestamp is built from a standard zmq message
     * 
     * Check the template specializations!
     * 
     * @param msg 
     */
    void from_zmq_msg(zmq::multipart_t &msg) override
    {
        convert_from_zmq(msg);
    }

    /**
     * @brief Override to_zmq_msg 
     * 
     * If data type T inherits ZMQConverter a multipart message is built
     * if data type T does not inherit vom ZMQConverter, a standard zmq message is built
     * 
     * @param msg 
     */
    zmq::multipart_t to_zmq_msg() const override
    {
        return convert_to_zmq();
    }
    
    /**
     * @brief Convert zmq message to stamped message, where T of Stamped<T> inherits ZMQConverter
     * and is build from a custom multipart message
     * 
     * @tparam DATA_T Type of data in Stamped<T>
     * @param msg message to deserialize
     */
    template < typename TT = DATA_T, std::enable_if_t < std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    void convert_from_zmq(zmq::multipart_t &msg) 
    {
        timestamp_ = msg.poptyp<util::HighResTimePoint>();
        data_.from_zmq_msg(msg);
    }

    /**
     * @brief Convert zmq message to stamped message, where T of Stamped<T> does not inherit ZMQConverter
     * and is build from a standard zmq multipart message
     * 
     * @tparam DATA_T 
     * @param msg message to deserialize
     */
    template < typename TT = DATA_T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    void convert_from_zmq(zmq::multipart_t &msg) 
    {
        timestamp_ = msg.poptyp<util::HighResTimePoint>();
        auto leftover_msg = msg.pop();
        data_ = *static_cast<DATA_T*>(leftover_msg.data());
    }

    /**
     * @brief Convert stamped struct to zmq message, where T of Stamped<T> inherits ZMQConverter and builds
     * a custom zmq multipart message
     * 
     * @tparam DATA_T Type of data in Stamped<T>
     * @return zmq::multipart_t message that's built
     */
    template < typename TT = DATA_T, std::enable_if_t < std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    zmq::multipart_t convert_to_zmq() const 
    {
        zmq::multipart_t multi;
        multi.addtyp(timestamp_);
        multi.append(data_.to_zmq_msg());
        return multi;
    }

    /**
     * @brief Convert stamped struct to zmq message, where T of Stamped<T> does not inherit ZMQConverter 
     * and builds a standard zmq multipart message
     * 
     * @tparam DATA_T Type of data in Stamped<T>
     * @return zmq::multipart_t message that's built
     */
    template < typename TT = DATA_T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    zmq::multipart_t convert_to_zmq() const 
    {
        zmq::multipart_t multi;
        multi.addtyp(timestamp_);
        multi.addtyp(data_);
        return multi;
    }

    /**
     * @brief Update Time to NOW
     */
    inline void update_time()
    {
        timestamp_ = util::HighResTime::now();
    }

    /// Data to refer timestamp to
    DATA_T data_;

    /// Time of creation/recording of data_
    util::HighResTimePoint timestamp_;
};

}