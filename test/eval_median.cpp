/**
 * @author Pascal Buschermöhle Marc Eisoldt
 *
 * Tests the combination of the TSDF Kernel with the registartion kernel
 * with a real point cloud and a simulated translation and rotation
 */


#include <stdlib.h>
#include <hw/kernels/vadd_kernel.h>
#include <registration/registration.h>
#include <util/pcd/pcd_file.h>
#include <tsdf/krnl_tsdf.h>
#include <preprocessing/preprocessing.h>

#include "catch2_config.h"

using fastsense::util::PCDFile;
using namespace fastsense::msg;
using namespace fastsense::preprocessing;

namespace fastsense::registration
{

//static const int DATA_SIZE = 4096;

constexpr unsigned int SCALE = 1000;

constexpr float MAX_OFFSET = 100; // TODO: this is too much

// Test Translation
constexpr float TX = 0.3 * SCALE;
constexpr float TY = 0.3 * SCALE;
constexpr float TZ = 0.0 * SCALE;
// Test Rotation
constexpr float RY = 5 * (M_PI / 180); //radiants

constexpr float TAU = 1 * SCALE;
constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;
constexpr int DZ_PER_DISTANCE = 572;

constexpr int MAX_ITERATIONS = 200;

constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION;
constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION;

// max difference between errors with and without median
constexpr int MAX_DIFFERENCE = 10;

struct EvalStats
{
    int minimum;
    int maximum;
    int average;
    int median;
};

/**
 * @brief Compares two sets of points one-on-one and does some statistics
 *
 * @param points_posttransform Transformed point cloud
 * @param points_pretransform Original point cloud
 */
static EvalStats get_transform_error(const ScanPoints_t& points_posttransform, const ScanPoints_t& points_pretransform)
{
    int minimum = std::numeric_limits<int>::infinity();
    int maximum = -std::numeric_limits<int>::infinity();
    int average = 0;

    int average_x = 0;
    int average_y = 0;
    int average_z = 0;

    std::vector<int> dists(points_pretransform.size());

    for (size_t i = 0; i < points_pretransform.size(); i++)
    {
        Eigen::Vector3i sub = points_pretransform[i] - points_posttransform[i];
        auto norm = sub.norm();

        if (norm < minimum)
        {
            minimum = norm;
        }

        if (norm > maximum)
        {
            maximum = norm;
        }

        average += norm;
        average_x += std::abs(sub.x());
        average_y += std::abs(sub.y());
        average_z += std::abs(sub.z());

        dists[i] = norm;
    }

    std::sort(dists.begin(), dists.end());

    return {minimum, maximum, static_cast<int>(average / points_pretransform.size()), dists[dists.size() / 2 + 1]};
}

static std::shared_ptr<fastsense::buffer::InputBuffer<PointHW>> scan_points_to_input_buffer(ScanPoints_t& cloud, const fastsense::CommandQueuePtr q)
{
    auto buffer_ptr = std::make_shared<fastsense::buffer::InputBuffer<PointHW>>(q, cloud.size());
    for (size_t i = 0; i < cloud.size(); i++)
    {
        auto point = cloud[i];
        PointHW tmp(point.x(), point.y(), point.z());
        (*buffer_ptr)[i] = tmp;
    }
    return buffer_ptr;
}

static EvalStats eval_registration(fastsense::map::LocalMap& local_map, fastsense::registration::Registration& reg, ScanPoints_t pretransformed, const ScanPoints_t& original, fastsense::CommandQueuePtr& q)
{
    auto buffer_ptr = scan_points_to_input_buffer(pretransformed, q);
    auto& buffer = *buffer_ptr;
    Matrix4f result_matrix = Matrix4f::Identity();
    reg.register_cloud(local_map, buffer, buffer.size(), util::HighResTime::now(), result_matrix);

    reg.transform_point_cloud(pretransformed, result_matrix);
    return get_transform_error(pretransformed, original);

}

static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

TEST_CASE("Eval_Median", "[eval_median][slow]")
{
    std::cout << "Testing 'Eval Median'" << std::endl;

    auto pointcloud_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(1);
    auto pointcloud_bridge_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(1);

    Preprocessing preprocessor{pointcloud_buffer, pointcloud_bridge_buffer, 0, false, false};

    fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();

    auto buffer = std::make_shared<msg::ImuStampedBuffer>(0);
    //test registration
    fastsense::registration::Registration reg(q, buffer, MAX_ITERATIONS);

    std::vector<std::vector<Vector3f>> float_points;
    unsigned int num_points;

    fastsense::util::PCDFile file("robo_lab.pcd");
    file.readPoints(float_points, num_points);

    auto count = 0u;

    ScanPoints_t scan_points(num_points);
    ScanPoints_t scan_points_preprocessed(num_points);

    auto q2 = fastsense::hw::FPGAManager::create_command_queue();
    fastsense::buffer::InputBuffer<PointHW> kernel_points(q2, num_points);
    fastsense::buffer::InputBuffer<PointHW> kernel_points_preprocessed(q2, num_points);

    Stamped<PointCloud::Ptr> cloud_stamped;
    cloud_stamped.data_ = std::make_shared<PointCloud>();
    cloud_stamped.data_->points_.resize(num_points);
    cloud_stamped.data_->rings_ = float_points.size();


    // Prepare normal registartion points
    for (const auto& ring : float_points)
    {
        for (const auto& point : ring)
        {
            scan_points[count].x() = point.x() * SCALE;
            scan_points[count].y() = point.y() * SCALE;
            scan_points[count].z() = point.z() * SCALE;

            kernel_points[count].x = scan_points[count].x();
            kernel_points[count].y = scan_points[count].y();
            kernel_points[count].z = scan_points[count].z();

            cloud_stamped.data_->points_[count] = scan_points[count].cast<ScanPointType>();            

            ++count;
        }
    }

    // Preprocessed points
    preprocessor.median_filter(cloud_stamped, 5);

    for (auto index = 0u; index < num_points; ++index)
    {
        scan_points_preprocessed[index] = cloud_stamped.data_->points_[index].cast<int>();
        kernel_points_preprocessed[index].x = cloud_stamped.data_->points_[index].x();
        kernel_points_preprocessed[index].y = cloud_stamped.data_->points_[index].y();
        kernel_points_preprocessed[index].z = cloud_stamped.data_->points_[index].z();
    }

    ScanPoints_t points_pretransformed_trans(scan_points);
    ScanPoints_t points_pretransformed_rot(scan_points);

    ScanPoints_t points_pretransformed_trans_preprocessed(scan_points_preprocessed);
    ScanPoints_t points_pretransformed_rot_preprocessed(scan_points_preprocessed);

    std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map.h5", 0.0, 0.0));
    fastsense::map::LocalMap local_map(SIZE_Y, SIZE_Y, SIZE_Z, global_map_ptr, q);

    std::shared_ptr<fastsense::map::GlobalMap> global_map_preprocessed_ptr(new fastsense::map::GlobalMap("test_global_map2.h5", 0.0, 0.0));
    fastsense::map::LocalMap local_map_preprocessed(SIZE_Y, SIZE_Y, SIZE_Z, global_map_preprocessed_ptr, q);
    

    // Initialize temporary testing variables

    Eigen::Matrix4f translation_mat;
    translation_mat << 1, 0, 0, TX,
                    0, 1, 0, TY,
                    0, 0, 1, TZ,
                    0, 0, 0,  1;

    Eigen::Matrix4f rotation_mat;
    rotation_mat <<  cos(RY), -sin(RY),      0, 0,
                 sin(RY),  cos(RY),      0, 0,
                 0,             0,       1, 0,
                 0,             0,       0, 1;

    //calc tsdf values for the points from the pcd and store them in the local map

    auto q3 = fastsense::hw::FPGAManager::create_command_queue();
    fastsense::tsdf::TSDFKernel krnl(q3, local_map.getBuffer().size());

    krnl.run(local_map, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
    krnl.waitComplete();

    krnl.run(local_map_preprocessed, kernel_points_preprocessed, kernel_points_preprocessed.size(), TAU, MAX_WEIGHT);
    krnl.waitComplete();

    SECTION("Test Registration No Transform")
    {
        std::cout << "    Section 'Test Registration No Transform'" << std::endl;
        auto without = eval_registration(local_map, reg, points_pretransformed_trans, scan_points, q);
        auto with = eval_registration(local_map_preprocessed, reg, points_pretransformed_trans_preprocessed, scan_points_preprocessed, q);
    
        std::cout << "Average error without preprocessing: " << without.average << std::endl;
        std::cout << "Average error with preprocessing: " << with.average << std::endl;
        REQUIRE(with.average - without.average < MAX_DIFFERENCE);
    }

    SECTION("Test Registration Translation")
    {
        std::cout << "    Section 'Test Registration Translation'" << std::endl;
        reg.transform_point_cloud(points_pretransformed_trans, translation_mat);
        reg.transform_point_cloud(points_pretransformed_trans_preprocessed, translation_mat);
        auto without = eval_registration(local_map, reg, points_pretransformed_trans, scan_points, q);
        auto with = eval_registration(local_map_preprocessed, reg, points_pretransformed_trans_preprocessed, scan_points_preprocessed, q);
    
        std::cout << "Average error without preprocessing: " << without.average << std::endl;
        std::cout << "Average error with preprocessing: " << with.average << std::endl;
        REQUIRE(with.average - without.average < MAX_DIFFERENCE);
    }

    SECTION("Registration test Rotation")
    {
        std::cout << "    Section 'Registration test Rotation'" << std::endl;
        reg.transform_point_cloud(points_pretransformed_rot, rotation_mat);
        reg.transform_point_cloud(points_pretransformed_rot_preprocessed, rotation_mat);
        auto without = eval_registration(local_map, reg, points_pretransformed_rot, scan_points, q); 
        auto with = eval_registration(local_map_preprocessed, reg, points_pretransformed_rot_preprocessed, scan_points_preprocessed, q);
    
        std::cout << "Average error without preprocessing: " << without.average << std::endl;
        std::cout << "Average error with preprocessing: " << with.average << std::endl;
        REQUIRE(with.average - without.average < MAX_DIFFERENCE);
    }
}

} //namespace fastsense::registration