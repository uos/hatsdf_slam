/**
 * @author Marc Eisoldt
 * @author Malte Hillmann
 * @author Marcel Flottmann
 *
 * Visualize the results of the hardware implementation of the TSDF generation and update
 * based on a real point cloud via the TSDF bridge
 */
#include "catch2_config.h"

#include <util/pcd/pcd_file.h>
#include <comm/queue_bridge.h>
#include <msg/tsdf.h>
#include <util/config/config_manager.h>
#include <tsdf/krnl_tsdf.h>

using namespace fastsense;

TEST_CASE("TSDF_Kernel_Vis", "[tsdf_kernel_vis]")
{
    std::cout << "Testing 'TSDF_Kernel_Vis'" << std::endl;
    SECTION("Visualize TSDF Data")
    {
        std::cout << "    Section 'Visualize TSDF Data'" << std::endl;
        constexpr unsigned int SCALE = 1000;
        constexpr float TAU = 1 * SCALE;
        constexpr float MAX_WEIGHT = 10 * WEIGHT_RESOLUTION;

        constexpr int SIZE_X = 20 * SCALE / MAP_RESOLUTION + 1;
        constexpr int SIZE_Y = 20 * SCALE / MAP_RESOLUTION + 1;
        constexpr int SIZE_Z = 5 * SCALE / MAP_RESOLUTION + 1;

        std::vector<std::vector<Vector3f>> float_points;
        unsigned int num_points;

        fastsense::util::PCDFile file("sim_cloud.pcd");
        file.readPoints(float_points, num_points);

        auto count = 0u;

        ScanPoints_t scan_points(num_points);

        auto queue = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::buffer::InputBuffer<PointHW> kernel_points(queue, num_points);

        std::vector<PointHW> kernel_points_sw(num_points);

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

                kernel_points_sw[count].x = kernel_points[count].x;
                kernel_points_sw[count].y = kernel_points[count].y;
                kernel_points_sw[count].z = kernel_points[count].z;

                ++count;
            }
        }

        std::cout << "num points: " << count << std::endl;

        fastsense::CommandQueuePtr q = fastsense::hw::FPGAManager::create_command_queue();
        std::shared_ptr<fastsense::map::GlobalMap> global_map_ptr(new fastsense::map::GlobalMap("test_global_map", 0.0, 0.0));
        fastsense::map::LocalMap local_map(SIZE_X, SIZE_Y, SIZE_Z, global_map_ptr, q);
        auto& size = local_map.get_size();
        auto& pos = local_map.get_pos();
        auto& offset = local_map.get_offset();

        auto tsdf_buffer = std::make_shared<fastsense::util::ConcurrentRingBuffer<fastsense::msg::TSDF>>(2);
        fastsense::comm::QueueBridge<fastsense::msg::TSDF, true> tsdf_bridge{tsdf_buffer, nullptr, 6666};

        tsdf_bridge.start();

        fastsense::buffer::InputOutputBuffer<TSDFValue> new_entries(q, local_map.getBuffer().size());

        for (auto& entry : new_entries)
        {
            entry.value(0);
            entry.weight(0);
        }

        auto q3 = fastsense::hw::FPGAManager::create_command_queue();
        fastsense::tsdf::TSDFKernel krnl(q3, local_map.getBuffer().size());

        krnl.run(local_map, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
        krnl.waitComplete();

        fastsense::msg::TSDF tsdf_msg;

        tsdf_msg.tau_ = TAU;
        tsdf_msg.size_ = size;
        tsdf_msg.pos_ = pos;
        tsdf_msg.offset_ = offset;
        tsdf_msg.tsdf_data_.reserve(local_map.getBuffer().size());
        std::copy(local_map.getBuffer().cbegin(), local_map.getBuffer().cend(), std::back_inserter(tsdf_msg.tsdf_data_));

        tsdf_buffer->push_nb(tsdf_msg, true);

        tsdf_bridge.stop();
    }
}