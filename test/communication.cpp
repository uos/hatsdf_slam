/**
 * @file communication.cpp
 * @author Marcel Flottmann
 * @date 2020-10-6
 */

#include "catch2_config.h"
#include <comm/receiver.h>
#include <comm/sender.h>
#include <comm/buffered_receiver.h>
#include <util/concurrent_ring_buffer.h>
#include <util/point.h>
#include <util/time.h>
#include <util/runner.h>
#include <msg/imu.h>
#include <msg/transform.h>
#include <msg/point_cloud.h>
#include <msg/tsdf.h>
#include <iostream>
#include <thread>

using namespace fastsense;
using namespace fastsense::comm;
using namespace fastsense::msg;
using namespace std::chrono_literals;

constexpr size_t iterations = 2;

#define SLEEP(x) std::this_thread::sleep_for(x)

TEST_CASE("Simple Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Simple Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        int value_received{};
        int value_to_send = 42;
        bool stop = false;

        std::thread receive_thread{[&]()
        {
            Receiver<int> receiver{"127.0.0.1", 2200, 20ms};

            while (!stop) {
                if (receiver.receive(value_received))
                {
                    break;
                }
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<int> sender{2200};
            SLEEP(500ms);
            sender.send(value_to_send);
            SLEEP(500ms);
            stop = true;
        }};

        receive_thread.join();
        send_thread.join();
        REQUIRE(value_to_send == value_received);
    }
}

TEST_CASE("PointCloud Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'PointCloud Sender Receiver Test'" << std::endl;

    for (size_t i = 0; i < iterations; ++i)
    {
        bool stop = false;

        PointCloud pc_to_send;
        pc_to_send.rings_ = 2;
        pc_to_send.points_.push_back({1, 2, 3});
        pc_to_send.points_.push_back({2, 3, 4});
        pc_to_send.points_.push_back({3, 4, 5});
        PointCloud pc_received;

        std::thread receive_thread{[&]()
        {
            Receiver<PointCloud> receiver{"127.0.0.1", 3235, 20ms};
            
            while (!stop)
            {
                if (receiver.receive(pc_received))
                {
                    break;
                }
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<PointCloud> sender{3235};
            SLEEP(500ms);
            sender.send(pc_to_send);
            SLEEP(500ms);
            stop = true;
        }};

        receive_thread.join();
        send_thread.join();
        REQUIRE(pc_to_send.rings_ == pc_received.rings_);
        REQUIRE(pc_to_send.points_ == pc_received.points_);
    }
}

TEST_CASE("PointCloudStamped Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'PointCloudStamped Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        bool stop = false;

        PointCloud pc_to_send;
        pc_to_send.rings_ = 2;
        pc_to_send.points_.push_back({1, 2, 3});
        pc_to_send.points_.push_back({2, 3, 4});
        pc_to_send.points_.push_back({3, 4, 5});

        auto tp_to_send = util::HighResTime::now();

        PointCloudStamped pcl_stamped{std::move(pc_to_send), tp_to_send};

        REQUIRE(pcl_stamped.data_.rings_ == 2);

        PointCloudStamped pcl_stamped_received;

        std::thread receive_thread{[&]()
        {
            Receiver<PointCloudStamped> receiver{"127.0.0.1", 1536, 20ms};

            while (!stop)
            {
                if (receiver.receive(pcl_stamped_received))
                {
                    break;
                }
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<PointCloudStamped> sender{1536};
            SLEEP(500ms);
            sender.send(pcl_stamped);
            SLEEP(500ms);
            stop = true;
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(pcl_stamped.data_.rings_ == pcl_stamped_received.data_.rings_);
        REQUIRE(pcl_stamped.data_.points_ == pcl_stamped_received.data_.points_);
        REQUIRE(pcl_stamped.timestamp_ == pcl_stamped_received.timestamp_);
    }
}

TEST_CASE("Stamped<PointCloud> Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Stamped<PointCloud> Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {   
        bool stop = false;

        PointCloud pc_to_send;
        pc_to_send.rings_ = 2;
        pc_to_send.points_.push_back({1, 2, 3});
        pc_to_send.points_.push_back({2, 3, 4});
        pc_to_send.points_.push_back({3, 4, 5});

        auto tp_to_send = util::HighResTime::now();

        Stamped<PointCloud> pcl_stamped{std::move(pc_to_send), tp_to_send};
        Stamped<PointCloud> pcl_stamped_received;

        std::thread receive_thread{[&]()
        {
            Receiver<Stamped<PointCloud>> receiver{"127.0.0.1", 1737, 20ms};

            while (!stop)
            {
                if (receiver.receive(pcl_stamped_received))
                {
                    break;
                }
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<Stamped<PointCloud>> sender{1737};
            SLEEP(500ms);
            sender.send(pcl_stamped);
            SLEEP(500ms);
            stop = true;
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(pcl_stamped.data_.rings_ == pcl_stamped_received.data_.rings_);
        REQUIRE(pcl_stamped.data_.points_ == pcl_stamped_received.data_.points_);
        REQUIRE(pcl_stamped.timestamp_ == pcl_stamped_received.timestamp_);
    }
}

TEST_CASE("TSDF Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'TSDF Sender Receiver Test'" << std::endl;

    for (size_t i = 0; i < iterations; ++i)
    {
        bool received = false;
        
        TSDF tsdf_msg;
        tsdf_msg.tau_ = 2;
        tsdf_msg.size_ = {10, 10, 10};
        tsdf_msg.pos_ = {0, 0, 0};
        tsdf_msg.offset_ = {0, 0, 0};
        tsdf_msg.tsdf_data_.resize(10 * 10 * 10);
        for (size_t i = 0; i < 10 * 10 * 10; ++i)
        {
            tsdf_msg.tsdf_data_[i].value(0);
            tsdf_msg.tsdf_data_[i].weight(0);
        }

        tsdf_msg.tsdf_data_[4 + 5 * 10 + 5 * 10 * 10].value(1);
        tsdf_msg.tsdf_data_[4 + 5 * 10 + 5 * 10 * 10].weight(1);

        tsdf_msg.tsdf_data_[3 + 5 * 10 + 5 * 10 * 10].value(2);
        tsdf_msg.tsdf_data_[3 + 5 * 10 + 5 * 10 * 10].weight(1);

        tsdf_msg.tsdf_data_[6 + 5 * 10 + 5 * 10 * 10].value(-1);
        tsdf_msg.tsdf_data_[6 + 5 * 10 + 5 * 10 * 10].weight(1);

        tsdf_msg.tsdf_data_[7 + 5 * 10 + 5 * 10 * 10].value(-2);
        tsdf_msg.tsdf_data_[7 + 5 * 10 + 5 * 10 * 10].weight(1);

        TSDF tsdf_received;

        std::thread receive_thread{[&]()
        {
            Receiver<TSDF> receiver{"127.0.0.1", 1288, 5ms};

            while (!received)
            {
                received = receiver.receive(tsdf_received);
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<TSDF> sender{1288};
            SLEEP(500ms);
            sender.send(tsdf_msg);
            SLEEP(500ms);
        }};

        receive_thread.join();
        send_thread.join();
        REQUIRE(tsdf_msg.tau_ == tsdf_received.tau_);
        REQUIRE(tsdf_msg.size_ == tsdf_received.size_);
        REQUIRE(tsdf_msg.pos_ == tsdf_received.pos_);
        REQUIRE(tsdf_msg.offset_ == tsdf_received.offset_);
        REQUIRE(tsdf_msg.tsdf_data_ == tsdf_received.tsdf_data_);
    }
}

TEST_CASE("TSDFStamped Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'TSDFStamped Sender Receiver Test'" << std::endl;

    for (size_t i = 0; i < iterations; ++i)
    {
        bool received = false;
        
        auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};
        TSDF tsdf_msg;
        tsdf_msg.tau_ = 2;
        tsdf_msg.size_ = {10, 10, 10};
        tsdf_msg.pos_ = {0, 0, 0};
        tsdf_msg.offset_ = {0, 0, 0};
        tsdf_msg.tsdf_data_.resize(10 * 10 * 10);
        for (size_t i = 0; i < 10 * 10 * 10; ++i)
        {
            tsdf_msg.tsdf_data_[i].value(0);
            tsdf_msg.tsdf_data_[i].weight(0);
        }

        tsdf_msg.tsdf_data_[4 + 5 * 10 + 5 * 10 * 10].value(1);
        tsdf_msg.tsdf_data_[4 + 5 * 10 + 5 * 10 * 10].weight(1);

        tsdf_msg.tsdf_data_[3 + 5 * 10 + 5 * 10 * 10].value(2);
        tsdf_msg.tsdf_data_[3 + 5 * 10 + 5 * 10 * 10].weight(1);

        tsdf_msg.tsdf_data_[6 + 5 * 10 + 5 * 10 * 10].value(-1);
        tsdf_msg.tsdf_data_[6 + 5 * 10 + 5 * 10 * 10].weight(1);

        tsdf_msg.tsdf_data_[7 + 5 * 10 + 5 * 10 * 10].value(-2);
        tsdf_msg.tsdf_data_[7 + 5 * 10 + 5 * 10 * 10].weight(1);

        TSDFStamped tsdf_stamped_sent{std::move(tsdf_msg), tp};
        TSDFStamped tsdf_stamped_received;

        std::thread receive_thread{[&]()
        {
            Receiver<TSDFStamped> receiver{"127.0.0.1", 1288, 5ms};

            while (!received)
            {
                received = receiver.receive(tsdf_stamped_received);
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<TSDFStamped> sender{1288};
            SLEEP(500ms);
            sender.send(tsdf_stamped_sent);
            SLEEP(500ms);
        }};

        receive_thread.join();
        send_thread.join();

        const auto& [tsdf_sent, timestamp_sent] = tsdf_stamped_sent;
        const auto& [tsdf_received, timestamp_received] = tsdf_stamped_received;

        REQUIRE(tsdf_sent.tau_ == tsdf_received.tau_);
        REQUIRE(tsdf_sent.size_ == tsdf_received.size_);
        REQUIRE(tsdf_sent.pos_ == tsdf_received.pos_);
        REQUIRE(tsdf_sent.offset_ == tsdf_received.offset_);
        REQUIRE(tsdf_sent.tsdf_data_ == tsdf_received.tsdf_data_);
        REQUIRE(timestamp_sent == timestamp_received);
    }
}

TEST_CASE("ImuStamped Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'ImuStamped Sender Receiver Test'" << std::endl;
    auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};

    LinearAcceleration acc{1, 2, 3};
    AngularVelocity ang{4, 5, 6};
    MagneticField mag{7, 8, 9};
    Imu imu{acc, ang, mag};

    ImuStamped imu_stamped{imu, tp};
    ImuStamped value_received{};

    for (size_t i = 0; i < iterations; ++i)
    {
        bool received = false;

        auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};

        LinearAcceleration acc{1, 2, 3};
        AngularVelocity ang{4, 5, 6};
        MagneticField mag{7, 8, 9};
        Imu imu{acc, ang, mag};

        ImuStamped imu_stamped{imu, tp};
        ImuStamped value_received{};

        std::thread receive_thread{[&]()
        {
            Receiver<ImuStamped> receiver{"127.0.0.1", 1339, 5ms};

            while (!received)
            {
                received = receiver.receive(value_received);
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<ImuStamped> sender{1339};
            SLEEP(500ms);
            sender.send(imu_stamped);
            SLEEP(500ms);
        }};

        receive_thread.join();
        send_thread.join();

        auto& [ imu_received, tp_received ] = value_received;

        REQUIRE(imu_received.acc.x() == 1);
        REQUIRE(imu_received.acc.y() == 2);
        REQUIRE(imu_received.acc.z() == 3);
        REQUIRE(imu_received.ang.x() == 4);
        REQUIRE(imu_received.ang.y() == 5);
        REQUIRE(imu_received.ang.z() == 6);
        REQUIRE(imu_received.mag.x() == 7);
        REQUIRE(imu_received.mag.y() == 8);
        REQUIRE(imu_received.mag.z() == 9);
        REQUIRE(tp_received == tp);
    }
}

 TEST_CASE("BufferedImuStampedReceiver Test", "[communication]")
 {
     std::cout << "Testing 'BufferedImuStampedReceiver Test'" << std::endl;

     auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};

     LinearAcceleration acc{1, 2, 3};
     AngularVelocity ang{4, 5, 6};
     MagneticField mag{7, 8, 9};
     Imu imu{acc, ang, mag};

     ImuStamped imu_stamped{imu, tp};
     ImuStamped value_received{};

     size_t n_msgs = 2;
     auto buffer = std::make_shared<msg::ImuStampedBuffer>(n_msgs);
     REQUIRE(buffer->empty());

     std::thread receive_thread{[&]()
     {
         BufferedImuStampedReceiver receiver{"127.0.0.1", 1544, 20ms, buffer};

         while (n_msgs != buffer->size())
         {
             receiver.receive();
         }
     }};

     std::thread send_thread{[&]()
     {
         Sender<ImuStamped> sender{1544};
         SLEEP(500ms);
         sender.send(imu_stamped);
         SLEEP(500ms);
         sender.send(imu_stamped);
         SLEEP(500ms);
     }};

     receive_thread.join();
     send_thread.join();

     REQUIRE(buffer->size() == 2);

     buffer->pop(&value_received);
     {
         auto& [imu_received, tp_received ] = value_received;

         REQUIRE(buffer->size() == 1);

         REQUIRE(imu_received.acc.x() == 1);
         REQUIRE(imu_received.acc.y() == 2);
         REQUIRE(imu_received.acc.z() == 3);
         REQUIRE(imu_received.ang.x() == 4);
         REQUIRE(imu_received.ang.y() == 5);
         REQUIRE(imu_received.ang.z() == 6);
         REQUIRE(imu_received.mag.x() == 7);
         REQUIRE(imu_received.mag.y() == 8);
         REQUIRE(imu_received.mag.z() == 9);
         REQUIRE(tp_received == tp);
    }


     buffer->pop(&value_received);
     {
         auto& [imu_received, tp_received ] = value_received;

         REQUIRE(buffer->empty());

         REQUIRE(imu_received.acc.x() == 1);
         REQUIRE(imu_received.acc.y() == 2);
         REQUIRE(imu_received.acc.z() == 3);
         REQUIRE(imu_received.ang.x() == 4);
         REQUIRE(imu_received.ang.y() == 5);
         REQUIRE(imu_received.ang.z() == 6);
         REQUIRE(imu_received.mag.x() == 7);
         REQUIRE(imu_received.mag.y() == 8);
         REQUIRE(imu_received.mag.z() == 9);
         REQUIRE(tp_received == tp);
     }
 }

  TEST_CASE("BufferedPclStampedReceiver Test", "[communication]")
  {
      std::cout << "Testing 'BufferedPclStampedReceiver Test'" << std::endl;

      static constexpr float SCALE = 0.5f;

      PointCloud pcl(1.f);
      REQUIRE(pcl.scaling_ == 1.f);

      pcl.rings_ = 2;
      pcl.scaling_ = SCALE;
      pcl.points_.push_back({1, 2, 3});
      pcl.points_.push_back({2, 3, 4});
      pcl.points_.push_back({3, 4, 5});

      PointCloudPtrStamped data_recv{};

      auto tp_to_send = util::HighResTime::now();

      PointCloudStamped pcl_stamped{std::move(pcl), tp_to_send};
      REQUIRE(pcl_stamped.data_.scaling_ == SCALE);

      size_t n_msgs = 2;
      auto buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(n_msgs);
      REQUIRE(buffer->empty());

      // test pcl custom constructor
      PointCloud test{0.1f};
      REQUIRE(test.points_.empty());
      REQUIRE(test.rings_ == 0);
      REQUIRE(test.scaling_ == 0.1f);

      std::thread receive_thread{[&]()
      {
          BufferedPclStampedReceiver receiver{"127.0.0.1", 1257, 20ms, buffer};

          while (n_msgs != buffer->size())
          {
              receiver.receive();
          }
      }};

      std::thread send_thread{[&]()
      {
          Sender<PointCloudStamped> sender{1257};
          SLEEP(500ms);
          sender.send(pcl_stamped);
          SLEEP(500ms);
          sender.send(pcl_stamped);
          SLEEP(500ms);
      }};

      receive_thread.join();
      send_thread.join();

      REQUIRE(buffer->size() == 2);

      buffer->pop(&data_recv);
      {
        auto&[pcl_ptr_stamped_recv, tp_received] = data_recv;

        REQUIRE(buffer->size() == 1);
        REQUIRE(pcl_stamped.data_.rings_ == pcl_ptr_stamped_recv->rings_);
        REQUIRE(pcl_stamped.data_.points_ == pcl_ptr_stamped_recv->points_);
        REQUIRE(pcl_stamped.data_.scaling_ == pcl_ptr_stamped_recv->scaling_);
        REQUIRE(pcl_ptr_stamped_recv->scaling_ == SCALE);
        REQUIRE(pcl_stamped.timestamp_ == tp_received);
      }

      buffer->pop(&data_recv);
      {
        auto&[pcl_ptr_stamped_recv, tp_received] = data_recv;

        REQUIRE(buffer->empty());
        REQUIRE(pcl_stamped.data_.rings_ == pcl_ptr_stamped_recv->rings_);
        REQUIRE(pcl_stamped.data_.points_ == pcl_ptr_stamped_recv->points_);
        REQUIRE(pcl_stamped.data_.scaling_ == pcl_ptr_stamped_recv->scaling_);
        REQUIRE(pcl_ptr_stamped_recv->scaling_ == SCALE);
        REQUIRE(pcl_stamped.timestamp_ == tp_received);
      }
  }

TEST_CASE("Stamped<int> Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Stamped<int> Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        bool received = false;

        auto tp = util::HighResTime::now();
        msg::Stamped<int> value_received{};
        msg::Stamped<int> value_to_send(42, tp);

        std::thread receive_thread{[&]()
        {
            Receiver<msg::Stamped<int>> receiver{"127.0.0.1", 1264, 20ms};

            while (!received)
            {
                received = receiver.receive(value_received);
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<msg::Stamped<int>> sender{1264};
            SLEEP(500ms);
            sender.send(value_to_send);
            SLEEP(500ms);
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(value_to_send.data_ == value_received.data_);
        REQUIRE(value_to_send.timestamp_ == value_received.timestamp_);
    }
}

TEST_CASE("Stamped<msg::Transform> Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Stamped<msg::Transform> Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        bool received = false;

        auto tp = util::HighResTime::now();
        msg::Stamped<msg::Transform> value_received;
        msg::Stamped<msg::Transform> value_to_send(std::move(msg::Transform{Quaternionf{1, 2, 3, 4}, Vector3f{5, 6, 7}, 0.5}), tp);

        std::thread receive_thread{[&]()
        {
            Receiver<msg::Stamped<msg::Transform>> receiver{"127.0.0.1", 2274, 20ms};

            while (!received)
            {
                received = receiver.receive(value_received);
            }
        }};

        std::thread send_thread{[&]()
        {
            Sender<msg::Stamped<msg::Transform>> sender{2274};
            SLEEP(500ms);
            sender.send(value_to_send);
            SLEEP(500ms);
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(value_to_send.data_.rotation.x() == Approx(value_received.data_.rotation.x()).margin(0.00001));
        REQUIRE(value_to_send.data_.rotation.y() == Approx(value_received.data_.rotation.y()).margin(0.00001));
        REQUIRE(value_to_send.data_.rotation.z() == Approx(value_received.data_.rotation.z()).margin(0.00001));
        REQUIRE(value_to_send.data_.rotation.w() == Approx(value_received.data_.rotation.w()).margin(0.00001));
        REQUIRE(value_to_send.data_.translation == value_received.data_.translation);
        REQUIRE(value_to_send.data_.scaling == value_received.data_.scaling);
        REQUIRE(value_received.data_.scaling == 0.5f);
        REQUIRE(value_to_send.timestamp_ == value_received.timestamp_);
    }
}