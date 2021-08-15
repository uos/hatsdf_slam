#pragma once

/**
 * @file tsdf_hw.h
 * @author Marcel Flottmann
 * @date 2021-01-14
 */

#include <cinttypes>
#include <util/constants.h>

struct TSDFValueHW
{
    using ValueType = int16_t;
    using WeightType = int16_t;

    ValueType value;
    WeightType weight;
};
