#include "catch2_config.h"
#include <msg/imu.h>
#include <msg/angular_velocity.h>
#include <msg/linear_acceleration.h>
#include <msg/magnetic_field.h>
#include <math.h>
#include <registration/imu_accumulator.h>
#include <util/point.h>
#include <iostream>
#include <util/time.h>

using namespace fastsense;
using namespace std::chrono_literals;


static void transform_point_cloud(std::vector<Vector3f>& in_cloud, const Matrix4f& mat)
{
    // #pragma omp parallel for schedule(static)
    for (auto i = 0u; i < in_cloud.size(); ++i)
    {
        Eigen::Vector4f v;
        v << in_cloud[i], 1;
        in_cloud[i] = (mat * v).block<3,1>(0, 0);

    }
}

TEST_CASE("AccumulatorPos", "[Accumulator]")
{

    //msg::ImuStampedBuffer imu_buffer(10);
    auto imu_buffer = std::make_shared<msg::ImuStampedBuffer>(10);

    auto stamp = util::HighResTime::now();
    auto diff = 250ms;

    msg::Imu imu{msg::LinearAcceleration{0,0,0}, msg::AngularVelocity{0,0,M_PI}, msg::MagneticField{0,0,0}};

    for (const auto& i: {1, 2, 3, 4, 5, 6, 7})
    {
        msg::ImuStamped imu_msg{imu, util::HighResTimePoint(stamp + i * diff)};
        imu_buffer->push(imu_msg);
    }

    registration::ImuAccumulator imu_acc{imu_buffer};

    Eigen::Matrix4f acc = imu_acc.acc_transform(util::HighResTimePoint(stamp + 5 * diff));

    // target rotation in deg
    auto target = 180; 
    // target rotation r in radians
    float r = target * (M_PI / 180); 
    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.001));

    REQUIRE(imu_buffer->size() ==  2);
    
    acc = imu_acc.acc_transform(util::HighResTimePoint(stamp + 7 * diff));

    // 2 imu messages left in buffer -> -90 grad rotation around z axis
    // target rotation in deg
    target = 90; 
    // target rotation r in radians
    r = target * (M_PI / 180); 
    rotation_mat.setIdentity();
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.000001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.000001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.000001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.000001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.000001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.000001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.000001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.000001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.000001));

    REQUIRE(imu_buffer->empty());
}

TEST_CASE("AccumulatorNeg", "[Accumulator]")
{

    auto imu_buffer = std::make_shared<msg::ImuStampedBuffer>(10);

    auto stamp = util::HighResTime::now();
    auto diff = 250ms;

    msg::Imu imu{msg::LinearAcceleration{0,0,0}, msg::AngularVelocity{0,0,-M_PI}, msg::MagneticField{0,0,0}};

    for (const auto& i: {1, 2, 3, 4, 5, 6, 7})
    {
        msg::ImuStamped imu_msg{imu, util::HighResTimePoint(stamp + i * diff)};
        imu_buffer->push(imu_msg);
    }

    registration::ImuAccumulator imu_acc{imu_buffer};

    Eigen::Matrix4f acc = imu_acc.acc_transform(util::HighResTimePoint(stamp + 5 * diff));

    // target rotation in deg
    auto target = -180; 
    // target rotation r in radians
    float r = target * (M_PI / 180); 
    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.001));

    REQUIRE(imu_buffer->size() ==  2);
    
    acc = imu_acc.acc_transform(util::HighResTimePoint(stamp + 7 * diff));

    // 2 imu messages left in buffer -> -90 grad rotation around z axis

    // target rotation in deg
    target = -90; 
    // target rotation r in radians
    r = target * (M_PI / 180); 
    rotation_mat.setIdentity();
    rotation_mat <<  cos(r), -sin(r), 0, 0,
                     sin(r),  cos(r), 0, 0,
                     0,       0,      1, 0,
                     0,       0,      0, 1;

    REQUIRE(acc(0,0) == Approx(rotation_mat(0,0)).margin(0.000001));
    REQUIRE(acc(0,1) == Approx(rotation_mat(0,1)).margin(0.000001));
    REQUIRE(acc(0,2) == Approx(rotation_mat(0,2)).margin(0.000001));
    REQUIRE(acc(1,0) == Approx(rotation_mat(1,0)).margin(0.000001));
    REQUIRE(acc(1,1) == Approx(rotation_mat(1,1)).margin(0.000001));
    REQUIRE(acc(1,2) == Approx(rotation_mat(1,2)).margin(0.000001));
    REQUIRE(acc(2,0) == Approx(rotation_mat(2,0)).margin(0.000001));
    REQUIRE(acc(2,1) == Approx(rotation_mat(2,1)).margin(0.000001));
    REQUIRE(acc(2,2) == Approx(rotation_mat(2,2)).margin(0.000001));

    REQUIRE(imu_buffer->empty());
}

TEST_CASE("AccumulatorCloudRotation", "[Accumulator]")
{
    auto imu_buffer = std::make_shared<msg::ImuStampedBuffer>(10);

    auto stamp = util::HighResTime::now();
    auto diff = 250ms;

    msg::Imu imu{msg::LinearAcceleration{0,0,0}, msg::AngularVelocity{0,0,-M_PI}, msg::MagneticField{0,0,0}};

    for (const auto& i: {1, 2, 3, 4, 5, 6, 7})
    {
        msg::ImuStamped imu_msg{imu, util::HighResTimePoint(stamp + i * diff)};
        imu_buffer->push(imu_msg);
    }

    registration::ImuAccumulator imu_acc{imu_buffer};

    Eigen::Matrix4f acc = imu_acc.acc_transform(util::HighResTimePoint(stamp + 5 * diff));

    std::vector<Vector3f> cloud(5);
    std::vector<Vector3f> result(5);

    cloud[0] = Vector3f{5, 0, 0};
    cloud[1] = Vector3f{2, 2, 2};
    cloud[2] = Vector3f{1, 2, 100};
    cloud[3] = Vector3f{540, 244, 124};
    cloud[4] = Vector3f{-3, -2, 2};

    transform_point_cloud(cloud, acc);

    result[0] = Vector3f{-5, 0, 0};
    result[1] = Vector3f{-2, -2, 2};
    result[2] = Vector3f{-1, -2, 100};
    result[3] = Vector3f{-540, -244, 124};
    result[4] = Vector3f{3, 2, 2};

    for(size_t i=0; i < cloud.size(); i++)
    {
        REQUIRE(cloud[i].x() == Approx(result[i].x()).margin(0.00001));
        REQUIRE(cloud[i].y() == Approx(result[i].y()).margin(0.00001));
        REQUIRE(cloud[i].z() == Approx(result[i].z()).margin(0.00001));
    }

    acc = imu_acc.acc_transform(util::HighResTimePoint(stamp + 7 * diff));

    transform_point_cloud(cloud, acc);

    result[0] = Vector3f{0, 5, 0};
    result[1] = Vector3f{-2, 2, 2};
    result[2] = Vector3f{-2, 1, 100};
    result[3] = Vector3f{-244, 540, 124};
    result[4] = Vector3f{2, -3, 2};

    for(size_t i=0; i < cloud.size(); i++)
    {
        REQUIRE(cloud[i].x() == Approx(result[i].x()).margin(0.00001));
        REQUIRE(cloud[i].y() == Approx(result[i].y()).margin(0.00001));
        REQUIRE(cloud[i].z() == Approx(result[i].z()).margin(0.00001));
    }
}