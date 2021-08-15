/**
 * @author Marc Eisoldt
 *
 * Test the hardware implementation of the TSDF generation and map update with simple scenarios
 */

#include <tsdf/krnl_tsdf.h>

#include "catch2_config.h"

using namespace fastsense;

TEST_CASE("Kernel_TSDF", "[kernel]")
{
    std::cout << "Testing 'Kernel_TSDF'" << std::endl;

    CommandQueuePtr q = hw::FPGAManager::create_command_queue();

    constexpr int TAU = 3 * MAP_RESOLUTION;
    constexpr int MAX_WEIGHT = 5 * WEIGHT_RESOLUTION;

    constexpr int SIZE_X = 50;
    constexpr int SIZE_Y = 50;
    constexpr int SIZE_Z = 10;

    SECTION("Generation")
    {
        std::cout << "    Section 'Generation'" << std::endl;

        auto gm_ptr = std::make_shared<map::GlobalMap>("MapTest.h5", 0, 0);
        map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, q};

        buffer::InputBuffer<PointHW> kernel_points(q, 1);
        kernel_points[0] = PointHW(6, 0, 0).to_mm();

        tsdf::TSDFKernel krnl(q, localMap.getBuffer().size());
        krnl.run(localMap, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
        krnl.waitComplete();

        // Front values
        CHECK(localMap.value(6, 0, 0).value() == 0 * MAP_RESOLUTION);
        CHECK(localMap.value(5, 0, 0).value() == 1 * MAP_RESOLUTION);
        CHECK(localMap.value(4, 0, 0).value() == 2 * MAP_RESOLUTION);
        CHECK(localMap.value(3, 0, 0).value() == TAU); // == 3 * MAP_RESOLUTION
        CHECK(localMap.value(2, 0, 0).value() == TAU);
        CHECK(localMap.value(1, 0, 0).value() == TAU);

        // Front weights
        CHECK(localMap.value(6, 0, 0).weight() == WEIGHT_RESOLUTION);
        CHECK(localMap.value(5, 0, 0).weight() == WEIGHT_RESOLUTION);
        CHECK(localMap.value(4, 0, 0).weight() == WEIGHT_RESOLUTION);
        CHECK(localMap.value(3, 0, 0).weight() == WEIGHT_RESOLUTION);
        CHECK(localMap.value(2, 0, 0).weight() == WEIGHT_RESOLUTION);
        CHECK(localMap.value(1, 0, 0).weight() == WEIGHT_RESOLUTION);

        // back values
        CHECK(localMap.value( 7, 0, 0).value() == -1 * MAP_RESOLUTION);
        CHECK(localMap.value( 8, 0, 0).value() == -2 * MAP_RESOLUTION);
        CHECK(localMap.value( 9, 0, 0).value() ==  0 * MAP_RESOLUTION);
        CHECK(localMap.value(10, 0, 0).value() ==  0 * MAP_RESOLUTION);
        CHECK(localMap.value(11, 0, 0).value() ==  0 * MAP_RESOLUTION);
        CHECK(localMap.value(12, 0, 0).value() ==  0 * MAP_RESOLUTION);

        // back weights
        CHECK(localMap.value( 7, 0, 0).weight() < WEIGHT_RESOLUTION);
        CHECK(localMap.value( 8, 0, 0).weight() < WEIGHT_RESOLUTION);
        CHECK(localMap.value( 9, 0, 0).weight() == 0);
        CHECK(localMap.value(10, 0, 0).weight() == 0);
        CHECK(localMap.value(11, 0, 0).weight() == 0);
        CHECK(localMap.value(12, 0, 0).weight() == 0);
    }

    SECTION("Update")
    {
        std::cout << "    Section 'Update'" << std::endl;

        constexpr int DEFAULT_WEIGHT = 8;

        auto gm_ptr = std::make_shared<map::GlobalMap>("MapTest.h5", 0, DEFAULT_WEIGHT);
        map::LocalMap localMap{SIZE_X, SIZE_Y, SIZE_Z, gm_ptr, q};

        buffer::InputBuffer<PointHW> kernel_points(q, 1);
        kernel_points[0] = PointHW(6, 0, 0).to_mm();

        tsdf::TSDFKernel krnl(q, localMap.getBuffer().size());
        krnl.run(localMap, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
        krnl.waitComplete();

        int new_weight = WEIGHT_RESOLUTION + DEFAULT_WEIGHT;

        // Front values
        CHECK(localMap.value(6, 0, 0).value() == 0);
        CHECK(localMap.value(5, 0, 0).value() == 1 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
        CHECK(localMap.value(4, 0, 0).value() == 2 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
        CHECK(localMap.value(3, 0, 0).value() == 3 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
        CHECK(localMap.value(2, 0, 0).value() == 3 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);
        CHECK(localMap.value(1, 0, 0).value() == 3 * MAP_RESOLUTION * WEIGHT_RESOLUTION / new_weight);

        // Front weights
        CHECK(localMap.value(6, 0, 0).weight() == new_weight);
        CHECK(localMap.value(5, 0, 0).weight() == new_weight);
        CHECK(localMap.value(4, 0, 0).weight() == new_weight);
        CHECK(localMap.value(3, 0, 0).weight() == new_weight);
        CHECK(localMap.value(2, 0, 0).weight() == new_weight);
        CHECK(localMap.value(1, 0, 0).weight() == new_weight);

        // test max weight
        for (int i = 0; i < MAX_WEIGHT / WEIGHT_RESOLUTION + 1; i++)
        {
            krnl.run(localMap, kernel_points, kernel_points.size(), TAU, MAX_WEIGHT);
            krnl.waitComplete();
        }

        CHECK(localMap.value(6, 0, 0).weight() == MAX_WEIGHT);
        CHECK(localMap.value(5, 0, 0).weight() == MAX_WEIGHT);
        CHECK(localMap.value(4, 0, 0).weight() == MAX_WEIGHT);
        CHECK(localMap.value(3, 0, 0).weight() == MAX_WEIGHT);
        CHECK(localMap.value(2, 0, 0).weight() == MAX_WEIGHT);
        CHECK(localMap.value(1, 0, 0).weight() == MAX_WEIGHT);
    }
}
