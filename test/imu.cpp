#include "catch2_config.h"
#include <msg/imu.h>
#include <util/filter.h>

using namespace fastsense;
using namespace fastsense::util;

#define REQUIRE_IMU(msg, val) \
    REQUIRE(msg.acc(0) == Approx(val)); \
    REQUIRE(msg.acc(1) == Approx(val)); \
    REQUIRE(msg.acc(2) == Approx(val)); \
    REQUIRE(msg.ang(0) == Approx(val)); \
    REQUIRE(msg.ang(1) == Approx(val)); \
    REQUIRE(msg.ang(2) == Approx(val)); \
    REQUIRE(msg.mag(0) == Approx(val)); \
    REQUIRE(msg.mag(1) == Approx(val)); \
    REQUIRE(msg.mag(2) == Approx(val)); \

#define IMU_WITH_VAL(val) msg::Imu(msg::LinearAcceleration(val, val, val), msg::AngularVelocity(val, val, val), msg::MagneticField(val, val, val))

TEST_CASE("Moving Average Filter Built In Type (window=0)", "[MovAvgFilter]")
{
    size_t window_size = 0;
    double mean = 0;

    SlidingWindowFilter<double> filter(window_size);

    mean = filter.update(1.);
    REQUIRE(mean == 1.);

    mean = filter.update(2.);
    REQUIRE(mean == 2.);

    mean = filter.update(3.);
    REQUIRE(mean == 3.);
}

TEST_CASE("Moving Average Filter Built In Type", "[MovAvgFilter]")
{
    double window_size = 3;
    double mean = 0;

    SlidingWindowFilter<double> filter(window_size);
    mean = filter.update(1.);
    REQUIRE(mean == 1.);

    mean = filter.update(2.);
    REQUIRE(mean == 2.);

    mean = filter.update(2.);
    REQUIRE(mean == 2.);

    // Buffer is filled for first time
    REQUIRE(filter.get_buffer().size() == window_size);

    // Mean from first three steps is (internally)
    // (((0 + 1/3) + 2/3) + 2/3) ~ 1.66667
    REQUIRE(filter.get_mean() == (((0. + 1./3.) + 2./3.) + 2./3.));

    // Now actual sliding window averaging begins: more than window_size measurements

    // front(): 1.
    REQUIRE(filter.get_buffer().front() == 1.);
    // Update with 3: mean += back (now 3) - front (1) / window_size => 2.33333
    auto old_mean = filter.get_mean();
    mean = filter.update(3.);
    REQUIRE(mean == old_mean + (3. - 1.)/window_size);
    // back(): 3, as just declared
    REQUIRE(filter.get_buffer().back() == 3.);

    // front(): 2.
    REQUIRE(filter.get_buffer().front() == 2.);
    // Update with 2: mean (2.33333) += (back (now 2) - front(now 2)/3)
    old_mean = filter.get_mean();
    mean = filter.update(2.);
    REQUIRE(mean == old_mean);
    // back(): 2.
    REQUIRE(filter.get_buffer().back() == 2.);
}

TEST_CASE("Moving Average Filter Imu", "[MovAvgFilter]")
{
    // Same logic as test above, but on all Imu Data

    double window_size = 3;
    msg::Imu mean{};

    SlidingWindowFilter<msg::Imu> filter(window_size);

    mean = filter.update(IMU_WITH_VAL(1.));
    REQUIRE_IMU(mean, 1.);

    mean = filter.update(IMU_WITH_VAL(2.));
    REQUIRE_IMU(mean, 2.);

    mean = filter.update(IMU_WITH_VAL(2.));
    REQUIRE_IMU(mean, 2.);

    // Buffer is filled for first time
    REQUIRE(filter.get_buffer().size() == window_size);

    // Mean from first three steps is (internally)
    // (((0 + 1/3) + 2/3) + 2/3) ~ 1.66667
    REQUIRE_IMU(filter.get_mean(), (((0. + 1./3.) + 2./3.) + 2./3.));

    // Now actual sliding window averaging begins: more than window_size measurements

    // front(): 1.
    REQUIRE_IMU(filter.get_buffer().front(), 1.);
    // Update with 3: mean += back (now 3) - front (1) / window_size => 2.33333
    auto old_mean = static_cast<double>(filter.get_mean().acc(0));
    mean = filter.update(IMU_WITH_VAL(3.));
    REQUIRE_IMU(mean, old_mean + (3. - 1.)/window_size);
    // back(): 3, as just declared
    REQUIRE_IMU(filter.get_buffer().back(), 3.);

    // front(): 2.
    REQUIRE_IMU(filter.get_buffer().front(), 2.);
    // Update with 2: mean (2.33333) += (back (now 2) - front(now 2)/3)
    auto old_imu_mean = filter.get_mean();
    mean = filter.update(IMU_WITH_VAL(2.));
    REQUIRE(mean == old_imu_mean);
    // back(): 2.
    REQUIRE_IMU(filter.get_buffer().back(), 2.);
}
