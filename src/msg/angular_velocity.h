#pragma once

/**
 * @file angular_velocity.h
 * @author Julian Gaal
 */

#include <util/params.h>
#include <util/point.h>

constexpr double DEEGREES_TO_RADIANS = M_PI / 180.0;

namespace fastsense::msg
{

/**
 * @brief Represents angular velocity data from imu
 */
struct AngularVelocity : public Vector3f
{
    /// Default constructor 0 initialises all values
    AngularVelocity()
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {}

    /**
     * Constructor with
     * @param x angular velocity x axis
     * @param y angular velocity y axis
     * @param z angular velocity z axis
     */
    AngularVelocity(float x, float y, float z)
    :   Vector3f{x, y, z}
    {}

    /**
     * Phidgets driver specific constructor: accepts raw c style array
     * @param angular_rate angular rate c style arrey
     */
    AngularVelocity(const double* angular_rate)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        (*this)[0] = angular_rate[0] * DEEGREES_TO_RADIANS;
        (*this)[1] = angular_rate[1] * DEEGREES_TO_RADIANS;
        (*this)[2] = angular_rate[2] * DEEGREES_TO_RADIANS;
    }

    /**
     * Divide ang velocity by another
     * @param other angular velocity
     */
    void operator/=(const AngularVelocity& other)
    {
        (*this)[0] /= other[0];
        (*this)[1] /= other[1];
        (*this)[2] /= other[2];
    }

    /// This constructor allows you to construct AngularVelocity from Eigen expressions
    template<typename OtherDerived>
    AngularVelocity(const Eigen::MatrixBase<OtherDerived>& other)
            : Eigen::Vector3f(other)
    { }

    /// This method allows you to assign Eigen expressions to AngularVelocity
    template<typename OtherDerived>
    AngularVelocity& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Vector3f::operator=(other);
        return *this;
    }
};

} // namespace fastsense::msg
