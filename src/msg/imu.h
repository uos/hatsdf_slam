#pragma once

/**
 * @file imu.h
 * @author Julian Gaal
 */

#include <msg/stamped.h>
#include <msg/linear_acceleration.h>
#include <msg/angular_velocity.h>
#include <msg/magnetic_field.h>
#include <msg/stamped.h>

#include <iostream>
#include <memory>
#include <util/concurrent_ring_buffer.h>

namespace fastsense::msg
{

/**
 * @brief Represents data from imu, similar to ROS sensor_msgs/Imu
 */
struct Imu
{
    /**
     * @brief Construct a new Imu object (default)
     */
    Imu()
    : acc{}
    , ang{}
    , mag{}
    {}

    /**
     * @brief Construct a new Imu object
     * 
     * @param acceleration linear acceleration, c style array for compatibility with phidget driver
     * @param angular_rate rotational change rate, c style array for compatibility with phidget driver
     * @param magField magnetic field, c style array for compatibility with phidget driver
     */
    Imu(const double* acceleration, const double* angular_rate, const double* magField)
    : acc{acceleration}
    , ang{angular_rate}
    , mag{magField}
    {}

    /**
     * @brief Construct a new Imu object
     * 
     * @param acc linear acceleration
     * @param ang rotational change rate
     * @param mag magnetic field
     */
    Imu(LinearAcceleration acc, AngularVelocity ang, MagneticField mag)
    : acc{std::move(acc)}
    , ang{std::move(ang)}
    , mag{std::move(mag)}
    {}

    /// linear acceleration
    LinearAcceleration acc;

    /// angular velocity
    AngularVelocity ang;

    /// magnetic field
    MagneticField mag;

    /// shared pointer typedef
    using Ptr = std::shared_ptr<Imu>;

    /**
     * Add another Imu message to existing
     * @param other Imu message
     */
    void operator+=(const Imu& other)
    {
        acc += other.acc;
        ang += other.ang;
        mag += other.mag;
    }

    /**
     * Subtract another Imu message from existing
     * @param other Imu message
     */
    void operator-=(const Imu& other)
    {
        acc -= other.acc;
        ang -= other.ang;
        mag -= other.mag;
    }

    /**
     * Divide existing by another Imu message
     * @param other Imu message
     */
    void operator/=(const Imu& other)
    {
        acc /= other.acc;
        ang /= other.ang;
        mag /= other.mag;
    }
};

/// Imu Message with Timestamp
using ImuStamped = msg::Stamped<Imu>;

/// Buffer of Imu Messages with Timestamp
using ImuStampedBuffer = util::ConcurrentRingBuffer<ImuStamped>;

} // namespace fastsense::msg


/**
 * Using [this method](https://eigen.tuxfamily.org/dox/TopicCustomizing_InheritingMatrix.html) of inheriting
 * from eigen constructs leaves some operators unimplemented, e.g. the one below
 *
 * This method is only compiled, if T inherits from Vector3f
 *
 * @tparam T type that inherits from Vector3f
 * @param a type that inherits from Vector3f
 * @param b type that inherits from Vector3f
 * @return a / b
 */
template <typename T, std::enable_if_t<std::is_base_of_v<fastsense::Vector3f, T>, int> = 0>
inline T operator/(const T& a, const T& b)
{
    return T{a[0] / b[0], a[1] / b[1], a[2] / b[2]};
}

/**
 * Using [this method](https://eigen.tuxfamily.org/dox/TopicCustomizing_InheritingMatrix.html) of inheriting
 * from eigen constructs leaves some operators unimplemented, e.g. the one below
 *
 * This method is only compiled, if T inherits from Vector3f
 *
 * @tparam T type that inherits from Vector3f
 * @param a type that inherits from Vector3f
 * @param b double
 * @return a / b
 */
template <typename F, typename T, std::enable_if_t<std::is_base_of_v<fastsense::Vector3f, T>, int> = 0>
inline T operator/(const T& a, double b)
{
    return T{a[0] / static_cast<float>(b), a[1] / static_cast<float>(b), a[2] / static_cast<float>(b)};
}

/**
 * Using [this method](https://eigen.tuxfamily.org/dox/TopicCustomizing_InheritingMatrix.html) of inheriting
 * from eigen constructs leaves some operators unimplemented, e.g. the one below
 *
 * This method is only compiled, if T inherits from Vector3f
 *
 * @tparam T type that inherits from Vector3f
 * @param a type that inherits from Vector3f
 * @param b type that inherits from Vector3f
 * @return a - b
 */
template <typename T, std::enable_if_t<std::is_base_of_v<fastsense::Vector3f, T>, int> = 0>
inline T operator-(const T& a, const T& b)
{
    return T{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

/**
 * Using [this method](https://eigen.tuxfamily.org/dox/TopicCustomizing_InheritingMatrix.html) of inheriting
 * from eigen constructs leaves some operators unimplemented, e.g. the one below
 *
 * This method is only compiled, if T inherits from Vector3f
 *
 * @tparam T type that inherits from Vector3f
 * @param a type that inherits from Vector3f
 * @param b type that inherits from Vector3f
 * @return a + b
 */
template <typename T, std::enable_if_t<std::is_base_of_v<fastsense::Vector3f, T>, int> = 0>
inline T operator+(const T& a, const T& b)
{
    return T{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

/**
 * Divides two Imu messages
 * @param a imu message
 * @param b imu message
 * @return a / b
 */
inline fastsense::msg::Imu operator/(const fastsense::msg::Imu& a, const fastsense::msg::Imu& b)
{
    return fastsense::msg::Imu{a.acc / b.acc, a.ang / b.ang, a.mag / b.mag};
}

/**
 * Divides Imu message by factor
 * @param a imu message
 * @param b factor
 * @return a / b
 */
inline fastsense::msg::Imu operator/(const fastsense::msg::Imu& a, double b)
{
    return fastsense::msg::Imu{a.acc / b, a.ang / b, a.mag / b};
}

/**
 * Adds two Imu messages
 * @param a imu message
 * @param b imu message
 * @return a + b
 */
inline fastsense::msg::Imu operator+(const fastsense::msg::Imu& a, const fastsense::msg::Imu& b)
{
    return fastsense::msg::Imu{a.acc + b.acc, a.ang + b.ang, a.mag + b.mag};
}

/**
 * Subtract two Imu messages
 * @param a imu message
 * @param b imu message
 * @return a - b
 */
inline fastsense::msg::Imu operator-(const fastsense::msg::Imu& a, const fastsense::msg::Imu& b)
{
    return fastsense::msg::Imu{a.acc - b.acc, a.ang - b.ang, a.mag - b.mag};
}

/**
 * Compare two Imu messages
 * @param a imu message
 * @param b imu message
 * @return a == b
 */
inline bool operator==(const fastsense::msg::Imu& a, const fastsense::msg::Imu& b)
{
    return a.acc == b.acc && a.ang == b.ang && a.mag == b.mag;
}


/**
 * @brief pretty print imu data
 * 
 * @param os ostream to print into 
 * @param data imu data
 * @return std::ostream& ostream to return 
 */
inline std::ostream& operator<<(std::ostream& os, const fastsense::msg::Imu& data)
{
   os << "-- acc --\n";
   os << data.acc.x() << "\n";
   os << data.acc.y() << "\n";
   os << data.acc.z() << "\n";

   os << "-- ang --\n";
   os << data.ang.x() << "\n";
   os << data.ang.y() << "\n";
   os << data.ang.z() << "\n";

   os << "-- mag --\n";
   os << data.mag.x() << "\n";
   os << data.mag.y() << "\n";
   os << data.mag.z() << "\n";

   return os;
}
