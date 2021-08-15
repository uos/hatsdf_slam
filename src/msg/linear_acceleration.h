#pragma once

/**
 * @file linear_acceleration.h
 * @author Julian Gaal
 */

#include <util/params.h>
#include <util/point.h>

namespace fastsense::msg
{

/**
 * @brief Represents linear acceleration data from imu
 */
struct LinearAcceleration : public Vector3f
{
    /// Default Initialize to 0
    LinearAcceleration()
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {}

    /**
     * Linear acceleration in x, y and z direction
     * @param x Linear acceleration in x direction
     * @param y Linear acceleration in y direction
     * @param z Linear acceleration in z direction
     */
    LinearAcceleration(float x, float y, float z)
    :   Vector3f{x, y, z}
    {}

    /**
     * Phidgets driver specific constructor: accepts raw c style array
     * @param acceleration linear acceleration c style arrey
     */
    explicit LinearAcceleration(const double* acceleration)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        (*this)[0] = -acceleration[0] * fastsense::util::params::G;
        (*this)[1] = -acceleration[1] * fastsense::util::params::G;
        (*this)[2] = -acceleration[2] * fastsense::util::params::G;
    }

    /**
     * Divide existing Linear Acceleration by another
     * @param other Linear Acceleration
     */
    void operator/=(const LinearAcceleration& other)
    {
        (*this)[0] /= other[0];
        (*this)[1] /= other[1];
        (*this)[2] /= other[2];
    }

    /// This constructor allows you to construct LinearAcceleration from Eigen expressions
    template<typename OtherDerived>
    LinearAcceleration(const Eigen::MatrixBase<OtherDerived>& other)
            : Eigen::Vector3f(other)
    { }

    /// This method allows you to assign Eigen expressions to LinearAcceleration
    template<typename OtherDerived>
    LinearAcceleration& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Vector3f::operator=(other);
        return *this;
    }
};

} // namespace fastsense::msg
