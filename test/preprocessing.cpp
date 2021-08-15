/**
 * @file test/preprocessing.cpp
 * @author Pascal Buschermöhle
 */

#include "catch2_config.h"
#include <preprocessing/preprocessing.h>
#include <util/point.h>
#include <msg/point_cloud.h>

using namespace fastsense;
using namespace fastsense::msg;
using namespace fastsense::preprocessing;
using fastsense::ScanPoint;

TEST_CASE("Preprocessing", "[Preprocessing]")
{
    REQUIRE(1 == 1);

    auto pointcloud_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(1);
    auto pointcloud_bridge_buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(1);

    Preprocessing preprocessor{pointcloud_buffer, pointcloud_bridge_buffer, 0, false, false};
    PointCloud::Ptr cloud = std::make_shared<PointCloud>();
    Stamped<PointCloud::Ptr> cloud_stamped;
    cloud_stamped.data_ = cloud;

    std::vector<ScanPoint> points(24);
    //RING 1
    points[0] = {0, 0, 10}; //d = 10
    points[2] = {20, 20, 30}; //d = 41.231056
    points[4] = {0, 0, 30}; //d = 30
    points[6] = {30, 30, 10}; //d = 43.588989
    points[8] = {30, 30, 30}; //d = 51.961524
    points[10] = {10, 10, 10}; //d = 17.320508
    points[12] = {10, 10, 30}; //d = 33.166248
    points[14] = {20, 20, 10}; //d = 30
    points[16] = {40, 40, 10}; //d = 57.445626
    points[18] = {50, 50, 10}; //d = 71.414284
    points[20] = {40, 40, 30}; //d = 64.031242
    points[22] = {50, 50, 30}; //d = 76.811457

    //RING 2
    points[1] = {0, 0, 20}; //d = 20
    points[3] = {20, 20, 20}; //d = 34.641016
    points[5] = {20, 20, 40}; //d = 48.989795
    points[7] = {30, 30, 20}; //d = 46.904158
    points[9] = {30, 30, 40}; //d = 58.309519
    points[11] = {40, 40, 20}; //d = 60
    points[13] = {40, 40, 40}; //d = 69.282032
    points[15] = {0, 0, 40}; //d = 40
    points[17] = {10, 10, 20}; //d = 24.494897
    points[19] = {10, 10, 40}; //d = 42.426407
    points[21] = {50, 50, 40}; //d = 81.240384
    points[23] = {50, 50, 20}; //d = 73.484692
    
    cloud_stamped.data_->points_ = points;
    cloud_stamped.data_->rings_ = 2;
    cloud_stamped.data_->scaling_ = 1.0f;

    SECTION("Test median filter"){

        preprocessor.median_filter(cloud_stamped, 5);

        std::vector<ScanPoint> result(24);
        result[0] = {20, 20, 30}; //d = 41.231056
        result[2] = {20, 20, 30}; //d = 41.231056
        result[4] = {20, 20, 30}; //d = 41.231056
        result[6] = {20, 20, 30}; //d = 43.588989
        result[8] = {10, 10, 30}; //d = 33.166248
        result[10] = {10, 10, 30}; //d = 33.166248
        result[12] = {10, 10, 30}; //d = 33.166248
        result[14] = {10, 10, 30}; //d = 33.166248
        result[16] = {40, 40, 10}; //d = 57.445626
        result[18] = {40, 40, 30}; //d = 64.031242
        result[20] = {40, 40, 30}; //d = 64.031242
        result[22] = {40, 40, 30}; //d = 64.031242

        result[1] = {20, 20, 40}; //d = 48.989795
        result[3] = {30, 30, 20}; //d = 46.904158
        result[5] = {30, 30, 20}; //d = 46.904158
        result[7] = {20, 20, 40}; //d = 48.989795
        result[9] = {30, 30, 40}; //d = 58.309519
        result[11] = {30, 30, 40}; //d = 58.309519
        result[13] = {30, 30, 40}; //d = 58.309519
        result[15] = {10, 10, 40}; //d = 42.426407
        result[17] = {10, 10, 40}; //d = 42.426407
        result[19] = {10, 10, 40}; //d = 42.426407
        result[21] = {10, 10, 40}; //d = 42.426407
        result[23] = {10, 10, 40}; //d = 42.426407

        for(uint32_t i = 0; i < result.size(); i++)
        {
            REQUIRE(cloud_stamped.data_->points_[i] == result[i]);
        }
    }

    SECTION("Test reduction filter average"){

        //n = 5
        //x = 35,8
        //y = 33,4
        //z = 27,4
        //(1, 1, 1)
        points[0] = {10, 10, 10}; 
        points[1] = {58, 54, 25}; 
        points[2] = {20, 57, 37}; 
        points[3] = {38, 4, 2}; 
        points[4] = {53, 42, 63};

        //n = 2
        //x = -14
        //y = 10
        //z = 20
        //(0, 1, 1)
        points[5] = {-22, 10, 10}; 
        points[6] = {-6, 10, 30}; 

        //n = 1
        //x = 20
        //y = 20
        //z = -37
        //(1, 1, 0)
        points[7] = {20, 20, -37}; 

        //n = 3
        //x = 115
        //y = 93,6
        //z = 103,3
        //(2, 2, 2)
        points[8] = {116, 127, 86}; 
        points[9] = {112, 78, 98}; 
        points[10] = {117, 76, 126};

        //n = 1
        //x = 50
        //y = -50
        //z = 30
        //(1, 0, 1)
        points[11] = {50, -50, 30};

        //n = 3
        //x = -77,3
        //y = -90,6
        //z = 103,6
        //(-1, -1, 2)
        points[12] = {-65, -67, 69};
        points[13] = {-73, -92, 127};
        points[14] = {-94, -113, 115};

        //n = 1
        //x = 30
        //y = -128
        //z = 20
        //(1, -2, 1)
        points[15] = {30, -128, 20};

        //n = 2
        //x = 6,5
        //y = 18
        //z = -103
        //(1, 1, -1)
        points[16] = {9, 24, -116};
        points[17] = {4, 12, -90};

        //n = 1
        //x = 150
        //y = 150
        //z = 150
        //(3, 3, 3)
        points[18] = {150, 150, 150};

        //n = 5
        //x = -23,4
        //y = 33,8
        //z = -28,2
        //(0, 1, 0)
        points[19] = {-16, 1, -44};
        points[20] = {-3, 51, -20};
        points[21] = {-35, 39, -35};
        points[22] = {-3, 22, -15};
        points[23] = {-60, 56, -27};

        cloud_stamped.data_->points_ = points;
        cloud_stamped.data_->rings_ = 2;

        preprocessor.reduction_filter_average(cloud_stamped);

        std::vector<ScanPoint> result_reduction(10);

        result_reduction[0] = {35, 33, 27}; 
        result_reduction[1] = {-14, 10, 20}; 
        result_reduction[2] = {20, 20, -37};
        result_reduction[3] = {115, 93, 103}; //
        result_reduction[4] = {50, -50, 30};
        result_reduction[5] = {-77, -90, 103}; //
        result_reduction[6] = {30, -128, 20}; //
        result_reduction[7] = {6, 18, -103}; //
        result_reduction[8] = {150, 150, 150}; //
        result_reduction[9] = {-23, 33, -28};

        for(uint32_t i = 0; i < result_reduction.size(); i++)
        {
            auto f = std::find(result_reduction.begin(), result_reduction.end(), cloud_stamped.data_->points_[i]);
            REQUIRE(f != result_reduction.end());
        }
    }

    SECTION("Test reduction filter voxel center"){
        points.resize(8);

        points[0] = {50, 50, 50};
        points[1] = {30, 30, 30};
        points[2] = {-32, -53, 1};
        points[3] = {-4, 40, 63};
        points[4] = {-60, -60, -60};
        points[5] = {70, 70, -70};
        points[6] = {120, 78, 45};
        points[7] = {140, 1, -1};

        cloud_stamped.data_->points_ = points;
        cloud_stamped.data_->rings_ = 2;

        preprocessor.reduction_filter_voxel_center(cloud_stamped);

        std::vector<ScanPoint> result_reduction(7);
        result_reduction[0] = {32, 32, 32};
        result_reduction[1] = {-32, -32, 32};
        result_reduction[2] = {-32, 32, 32}; 
        result_reduction[3] = {-32, -32, -32};
        result_reduction[4] = {96, 96, -96}; 
        result_reduction[5] = {96, 96, 32};
        result_reduction[6] = {160, 32, -32};

        for(uint32_t i = 0; i < result_reduction.size(); i++)
        {
            auto f = std::find(result_reduction.begin(), result_reduction.end(), cloud_stamped.data_->points_[i]);
            REQUIRE(f != result_reduction.end());
        }
    }

    SECTION("Test reduction filter voxel center"){

        points.resize(8);

        points[0] = {50, 50, 50};
        points[1] = {30, 30, 30};
        points[2] = {20, 20, 20};

        points[3] = {-4, 40, 63};
        points[4] = {-60, 60, 60};

        points[5] = {70, 70, -70};

        points[6] = {120, 78, 45};

        points[7] = {140, 1, -1};

        cloud_stamped.data_->points_ = points;
        cloud_stamped.data_->rings_ = 2;

        preprocessor.reduction_filter_random_point(cloud_stamped);

        for(auto& point : cloud_stamped.data_->points_){
            std::cout << "x: " << point.x() << " y: " << point.y() << "z: " << point.z() << std::endl;
        }
    }
}