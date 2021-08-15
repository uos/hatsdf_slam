/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include "catch2_config.h"
#include "kernels/local_map_test_kernel.h"

using namespace fastsense::map;
using namespace fastsense::hw;
using namespace fastsense::kernels;
using Eigen::Vector3i;

constexpr int DEFAULT_VALUE = 4;
constexpr int DEFAULT_WEIGHT = 6;

TEST_CASE("Map", "[Map]")
{
    std::cout << "Testing 'Map'" << std::endl;
    std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", DEFAULT_VALUE, DEFAULT_WEIGHT);
    auto commandQueue = FPGAManager::create_command_queue();
    LocalMap localMap{5, 5, 5, gm_ptr, commandQueue};

    // write some tsdf values and weights into one corner of the ring buffer,
    // that will be written to the file as one chunk
    TSDFValue p0(0, 0);
    TSDFValue p1(1, 1);
    TSDFValue p2(2, 1);
    TSDFValue p3(3, 2);
    TSDFValue p4(4, 3);
    TSDFValue p5(5, 5);
    localMap.value(-2, 2, 0) = p0;
    localMap.value(-1, 2, 0) = p1;
    localMap.value(-2, 1, 0) = p2;
    localMap.value(-1, 1, 0) = p3;
    localMap.value(-2, 0, 0) = p4;
    localMap.value(-1, 0, 0) = p5;

    // test getter
    CHECK(localMap.get_pos() == Vector3i(0, 0, 0));
    CHECK(localMap.get_size() == Vector3i(5, 5, 5));
    CHECK(localMap.get_offset() == Vector3i(2, 2, 2));

    // test in_bounds
    CHECK(localMap.in_bounds(0, 2, -2));
    CHECK(!localMap.in_bounds(22, 0, 0));
    // test default values
    CHECK(localMap.value(0, 0, 0).value() == DEFAULT_VALUE);
    CHECK(localMap.value(0, 0, 0).weight() == DEFAULT_WEIGHT);
    // test value access
    CHECK(localMap.value(-1, 2, 0).value() == 1);
    CHECK(localMap.value(-1, 2, 0).weight() == 1);

    // ==================== shift ====================
    // shift so that the chunk gets unloaded
    // Each shift can only cover an area of one Map size
    localMap.shift(Vector3i( 5, 0, 0));
    localMap.shift(Vector3i(10, 0, 0));
    localMap.shift(Vector3i(15, 0, 0));
    localMap.shift(Vector3i(20, 0, 0));
    localMap.shift(Vector3i(24, 0, 0));

    CHECK(localMap.get_pos() == Vector3i(24, 0, 0));
    CHECK(localMap.get_size() == Vector3i(5, 5, 5));
    CHECK(localMap.get_offset() == Vector3i(26 % 5, 2, 2));

    // test in_bounds
    CHECK(!localMap.in_bounds(0, 2, -2));
    CHECK(localMap.in_bounds(22, 0, 0));
    // test values
    CHECK(localMap.value(24, 0, 0).value() == DEFAULT_VALUE);
    CHECK(localMap.value(24, 0, 0).weight() == DEFAULT_WEIGHT);

    // ==================== shift directions ====================
    localMap.value(24, 0, 0) = TSDFValue(24, 0);

    localMap.shift(Vector3i(24, 5, 0));
    localMap.value(24, 5, 0) = TSDFValue(24, 5);

    localMap.shift(Vector3i(19, 5, 0));
    localMap.value(19, 5, 0) = TSDFValue(19, 5);

    localMap.shift(Vector3i(19, 0, 0));
    localMap.value(19, 0, 0) = TSDFValue(19, 0);

    localMap.shift(Vector3i(24, 0, 0));
    CHECK(localMap.value(24, 0, 0).value() == 24);
    CHECK(localMap.value(24, 0, 0).weight() == 0);

    localMap.shift(Vector3i(19, 0, 0));
    CHECK(localMap.value(19, 0, 0).value() == 19);
    CHECK(localMap.value(19, 0, 0).weight() == 0);
    localMap.shift(Vector3i(24, 5, 0));
    CHECK(localMap.value(24, 5, 0).value() == 24);
    CHECK(localMap.value(24, 5, 0).weight() == 5);
    localMap.shift(Vector3i(19, 5, 0));
    CHECK(localMap.value(19, 5, 0).value() == 19);
    CHECK(localMap.value(19, 5, 0).weight() == 5);
    localMap.shift(Vector3i(24, 0, 0));
    CHECK(localMap.value(24, 0, 0).value() == 24);
    CHECK(localMap.value(24, 0, 0).weight() == 0);

    // ==================== shift back ====================
    localMap.shift(Vector3i(19, 0, 0));
    localMap.shift(Vector3i(14, 0, 0));
    localMap.shift(Vector3i( 9, 0, 0));
    localMap.shift(Vector3i( 4, 0, 0));
    localMap.shift(Vector3i( 0, 0, 0));

    // test return correct
    CHECK(localMap.get_pos() == Vector3i(0, 0, 0));
    CHECK(localMap.get_size() == Vector3i(5, 5, 5));
    CHECK(localMap.get_offset() == Vector3i(2, 2, 2));

    // test in_bounds
    CHECK(localMap.in_bounds(0, 2, -2));
    CHECK(!localMap.in_bounds(22, 0, 0));

    CHECK(localMap.value(0, 0, 0).value() == DEFAULT_VALUE);
    CHECK(localMap.value(0, 0, 0).weight() == DEFAULT_WEIGHT);
    CHECK(localMap.value(-1, 2, 0).value() == 1);
    CHECK(localMap.value(-1, 2, 0).weight() == 1);

    // ==================== kernel ====================
    auto q = FPGAManager::create_command_queue();
    LocalMapTestKernel krnl{q};
    krnl.run(localMap);
    krnl.waitComplete();

    CHECK(localMap.value(0, 0, 0).value() == DEFAULT_VALUE * 2);
    CHECK(localMap.value(0, 0, 0).weight() == DEFAULT_WEIGHT / 2);
    CHECK(localMap.value(-1, 2, 0).value() == 2);
    CHECK(localMap.value(-1, 2, 0).weight() == 0);

    localMap.write_back();

    // check file for the numbers
    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
    HighFive::Group g = f.getGroup("/map");
    HighFive::DataSet d = g.getDataSet("-1_0_0");
    std::vector<TSDFValue::RawType> chunk;
    d.read(chunk);

    // test pose
    gm_ptr->save_pose(8, 13, 21, 34, 55, 89, 1);
    gm_ptr->save_pose(144, 233, 377, 610, 987, 1597, 1);
    g = f.getGroup("/poses");
    d = g.getDataSet("1");

    std::vector<float> pose;
    d.read(pose);

    constexpr int CHUNK_SIZE = GlobalMap::CHUNK_SIZE;

    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 2)]).value() == 0 * 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 2)]).value() == 1 * 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 1)]).value() == 2 * 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 1)]).value() == 3 * 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 0)]).value() == 4 * 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 0)]).value() == 5 * 2);

    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 2)]).weight() == 0 / 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 2)]).weight() == 1 / 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 1)]).weight() == 1 / 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 1)]).weight() == 2 / 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 0)]).weight() == 3 / 2);
    CHECK(TSDFValue(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 0)]).weight() == 5 / 2);

    int i = 0;
    CHECK(pose[i++] == 144);
    CHECK(pose[i++] == 233);
    CHECK(pose[i++] == 377);
    CHECK(pose[i++] == 610);
    CHECK(pose[i++] == 987);
    CHECK(pose[i++] == 1597);
}
