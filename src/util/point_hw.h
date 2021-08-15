#pragma once

/**
 * @file point_hw.h
 * @author Marcel Flottmann
 * @author Malte Hillmann
 * @author Marc Eisoldt
 */

#include "hls_functions.h"
#include <util/constants.h>

struct PointHW
{
    int x;
    int y;
    int z;
    int dummy; //128 bit padding

    PointHW() : x(0), y(0), z(0)
    {}

    PointHW(int x, int y, int z)
        : x(x), y(y), z(z)
    {}

    PointHW(const PointHW& rhs)
    {
        *this = rhs;
    }

    /// default destructor
    ~PointHW() = default;

    PointHW& operator=(const PointHW& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    PointHW& operator=(int rhs)
    {
        x = rhs;
        y = rhs;
        z = rhs;
        return *this;
    }

    /// default move assignment operator
    PointHW& operator=(PointHW&&) = default;

    /// default move constructor
    PointHW(PointHW&&) = default;

    PointHW operator+(const PointHW& rhs) const
    {
        PointHW p;
        p.x = x + rhs.x;
        p.y = y + rhs.y;
        p.z = z + rhs.z;
        return p;
    }

    PointHW operator-(const PointHW& rhs) const
    {
        PointHW p;
        p.x = x - rhs.x;
        p.y = y - rhs.y;
        p.z = z - rhs.z;
        return p;
    }

    PointHW operator*(int rhs) const
    {
        PointHW p;
        p.x = x * rhs;
        p.y = y * rhs;
        p.z = z * rhs;
        return p;
    }

    PointHW operator/(int rhs) const
    {
        PointHW p;
        p.x = x / rhs;
        p.y = y / rhs;
        p.z = z / rhs;
        return p;
    }

    bool operator==(const PointHW& p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }
    bool operator!=(const PointHW& p) const
    {
        return !(*this == p);
    }

    int norm2() const
    {
        return x * x + y * y + z * z;
    }

    int norm() const
    {
        return hls_sqrt_approx(norm2());
    }

    PointHW abs() const
    {
        PointHW p;
        p.x = hls_abs(x);
        p.y = hls_abs(y);
        p.z = hls_abs(z);
        return p;
    }

    PointHW sign() const
    {
        PointHW p;
        p.x = x < 0 ? -1 : 1;
        p.y = y < 0 ? -1 : 1;
        p.z = z < 0 ? -1 : 1;
        return p;
    }

    PointHW to_map() const
    {
        PointHW p;
        p.x = x / MAP_RESOLUTION;
        p.y = y / MAP_RESOLUTION;
        p.z = z / MAP_RESOLUTION;
        return p;
    }

    PointHW to_mm() const
    {
        PointHW p;
        p.x = (x * MAP_RESOLUTION) + MAP_RESOLUTION / 2;
        p.y = (y * MAP_RESOLUTION) + MAP_RESOLUTION / 2;
        p.z = (z * MAP_RESOLUTION) + MAP_RESOLUTION / 2;
        return p;
    }
};

struct PointArith
{
    long x;
    long y;
    long z;

    PointArith() : x(0), y(0), z(0)
    {}

    PointArith(long x, long y, long z)
        : x(x), y(y), z(z)
    {}

    PointArith operator+(const PointArith& rhs) const
    {
        PointArith p;
        p.x = x + rhs.x;
        p.y = y + rhs.y;
        p.z = z + rhs.z;
        return p;
    }

    PointArith operator-(const PointArith& rhs) const
    {
        PointArith p;
        p.x = x - rhs.x;
        p.y = y - rhs.y;
        p.z = z - rhs.z;
        return p;
    }

    PointArith operator*(long rhs) const
    {
        PointArith p;
        p.x = x * rhs;
        p.y = y * rhs;
        p.z = z * rhs;
        return p;
    }

    PointArith operator/(long rhs) const
    {
        PointArith p;
        p.x = x / rhs;
        p.y = y / rhs;
        p.z = z / rhs;
        return p;
    }

    PointArith& operator=(long rhs)
    {
        x = rhs;
        y = rhs;
        z = rhs;
        return *this;
    }

    long norm2() const
    {
        return x * x + y * y + z * z;
    }

    long norm() const
    {
        return hls_sqrt_approx_arith(norm2());
    }

    PointArith cross(const PointArith& rhs) const 
    {
        PointArith p;
        p.x = y * rhs.z - z * rhs.y;
        p.y = z * rhs.x - x * rhs.z;
        p.z = x * rhs.y - y * rhs.x;
        return p;
    }
};
