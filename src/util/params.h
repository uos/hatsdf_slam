#pragma once

/**
 * @file params.h
 * @author Julian Gaal
 */

#include <cmath>

namespace fastsense::util::params
{

static constexpr float G = 9.80665;
static constexpr float MAX_TIMEDIFF_SECONDS = 0.1;

static constexpr int serial_number_ = -1;
static constexpr int period_ = 4;  // rate in ms

static constexpr float angular_velocity_stdev_ = 0.02 * (M_PI / 180.0); // 0.02 deg/s resolution, as per manual
static constexpr float linear_acceleration_stdev_ = 300.0 * 1e-6 * G; // 300 ug as per manual
static constexpr float magnetic_field_stdev_ = 0.095 * (M_PI / 180.0); // 0.095°/s as per manual

// compass correction params (see
// http://www.phidgets.com/docs/1044_User_Guide)
static constexpr double cc_mag_field_ = 0.52859;
static constexpr double cc_offset0_ = 0.03921;
static constexpr double cc_offset1_ = 0.19441;
static constexpr double cc_offset2_ = -0.03493;
static constexpr double cc_gain0_ = 1.81704;
static constexpr double cc_gain1_ = 1.81028;
static constexpr double cc_gain2_ = 2.04819;
static constexpr double cc_T0_ = 0.00142;
static constexpr double cc_T1_ = -0.03591;
static constexpr double cc_T2_ = 0.00160;
static constexpr double cc_T3_ = -0.05038;
static constexpr double cc_T4_ = -0.03942;
static constexpr double cc_T5_ = -0.05673;

} // namespace fastsense::util::params;