#pragma once

/**
 * @file magnetic_field.h
 * @author Julian Gaal
 */

#include <util/point.h>
#include <limits>

// Taken from phidget21.h header, for portability
#define PUNK_DBL    1e300                   /**< Unknown Double */

namespace fastsense::msg
{

/**
 * @brief Represents magnetic field data from imu
 */
struct MagneticField : public Vector3f
{
    /// Zero initalize MagneticField
    MagneticField()
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {}

    /**
     * Magnetic Field in x, y and z directions
     * @param x
     * @param y
     * @param z
     */
    MagneticField(float x, float y, float z)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = z;
    }

    /**
     * Phidgets driver specific constructor: accepts raw c style array
     * @param magneticField magnetic field c style arrey
     */
    explicit MagneticField(const double* magneticField)
    :   Vector3f{0.0f, 0.0f, 0.0f}
    {
        if (magneticField[0] != PUNK_DBL)
        {
            // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
            (*this)[0] = magneticField[0] * 1e-4;
            (*this)[1] = magneticField[1] * 1e-4;
            (*this)[2] = magneticField[2] * 1e-4;
        }
        else
        {
            double nan = std::numeric_limits<double>::quiet_NaN();

            (*this)[0] = nan;
            (*this)[1] = nan;
            (*this)[2] = nan;
        }
    }

    /**
     * Divide magnetic field by another one
     * @param other magnetic field
     */
    void operator/=(const MagneticField& other)
    {
        (*this)[0] /= other[0];
        (*this)[1] /= other[1];
        (*this)[2] /= other[2];
    }

    /// This constructor allows you to construct MagneticField from Eigen expressions
    template<typename OtherDerived>
    MagneticField(const Eigen::MatrixBase<OtherDerived>& other)
            : Eigen::Vector3f(other)
    { }

    /// This method allows you to assign Eigen expressions to MagneticField
    template<typename OtherDerived>
    MagneticField& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::Vector3f::operator=(other);
        return *this;
    }
};

} // namespace fastsense::msg