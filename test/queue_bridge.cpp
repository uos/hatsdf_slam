/**
 * @file queue_bridge.cpp
 * @author Marcel Flottmann
 * @date 2020-10-6
 */

#include "catch2_config.h"
#include <comm/receiver.h>
#include <msg/stamped.h>
#include <msg/point_cloud.h>
#include <comm/buffered_receiver.h>
#include <comm/queue_bridge.h>
#include <iostream>

using namespace fastsense::comm;
using namespace fastsense::msg;
using namespace fastsense::util;
using namespace std::chrono_literals;

#define SLEEP(x) std::this_thread::sleep_for(x)

TEST_CASE("QueueBridge", "[communication_queue_bridge]")
{
    std::cout << "Testing 'QueueBridge'" << std::endl;
    int value_received = 0;
    int value_to_send = 42;

    bool received = false;
    bool sending = true;

    auto in = std::make_shared<ConcurrentRingBuffer<int>>(16);
    auto out = std::make_shared<ConcurrentRingBuffer<int>>(16);
    QueueBridge<int, true> bridge{in, out, 5234};

    std::thread receive_thread{[&]()
    {
        Receiver<int> receiver{"127.0.0.1", 5234, 20ms};
        
        while (!received)
        {
            if (receiver.receive(value_received))
            {
                received = true;
            }
        }
    }};

    std::thread send_thread{[&]()
    {
        while (sending)
        {
            in->push(value_to_send);
            SLEEP(100ms);
        }
    }};

    bridge.start();
    while (!received)
    {
        std::this_thread::yield();
    }
    bridge.stop();

    sending = false;

    receive_thread.join();
    send_thread.join();
    REQUIRE(value_to_send == value_received);
    REQUIRE(out->size() != 0);
}

TEST_CASE("QueueBridge shared_ptr", "[communication_queue_bridge]")
{
    std::cout << "Testing 'QueueBridge shared_ptr'" << std::endl;
    int value_received = 0;
    int value_to_send = 42;

    bool received = false;
    bool sending = true;

    auto in = std::make_shared<ConcurrentRingBuffer<std::shared_ptr<int>>>(16);
    auto out = std::make_shared<ConcurrentRingBuffer<std::shared_ptr<int>>>(16);
    QueueBridge<std::shared_ptr<int>, true> bridge{in, out, 4234};

    std::thread receive_thread{[&]()
    {
        Receiver<int> receiver{"127.0.0.1", 4234, 20ms};

        while (!received)
        {
            if (receiver.receive(value_received))
            {
                received = true;
            }
        }

    }};

    std::thread send_thread{[&]()
    {
        while (sending)
        {
            in->push(std::make_shared<int>(value_to_send));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }};

    bridge.start();
    while (!received)
    {
        std::this_thread::yield();
    }
    bridge.stop();

    sending = false;

    receive_thread.join();
    send_thread.join();
    REQUIRE(value_to_send == value_received);
    REQUIRE(out->size() != 0);
}

TEST_CASE("QueueBridge Stamped<Imu>", "[communication_queue_bridge]")
{
    std::cout << "Testing 'QueueBridge Stamped<Imu>'" << std::endl;
    Imu imu{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    auto ts_sent = HighResTime::now();
    ImuStamped value_to_send{std::move(imu), ts_sent};
    ImuStamped value_received;

    bool received = false;
    bool sending = true;

    auto in = std::make_shared<ConcurrentRingBuffer<ImuStamped>>(16);
    auto out = std::make_shared<ConcurrentRingBuffer<ImuStamped>>(16);
    QueueBridge<ImuStamped, true> bridge{in, out, 3234};

    std::thread receive_thread{[&]()
    {
        Receiver<ImuStamped> receiver{"127.0.0.1", 3234, 20ms};

        while (!received)
        {
            if (receiver.receive(value_received))
            {
                received = true;
            }
        }
    }};

    std::thread send_thread{[&]()
    {
        while (sending)
        {
            in->push(value_to_send);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }};

    bridge.start();
    while (!received)
    {
        std::this_thread::yield();
    }
    bridge.stop();

    sending = false;

    receive_thread.join();
    send_thread.join();

    const auto& [ imu_received, ts ] = value_received;
    REQUIRE(imu_received.acc.x() == 1);
    REQUIRE(imu_received.acc.y() == 2);
    REQUIRE(imu_received.acc.z() == 3);
    REQUIRE(imu_received.ang.x() == 4);
    REQUIRE(imu_received.ang.y() == 5);
    REQUIRE(imu_received.ang.z() == 6);
    REQUIRE(imu_received.mag.x() == 7);
    REQUIRE(imu_received.mag.y() == 8);
    REQUIRE(imu_received.mag.z() == 9);
    REQUIRE(ts == ts_sent);
    REQUIRE(out->size() != 0);
}

TEST_CASE("QueueBridge Stamped<PointCloud>", "[communication_queue_bridge]")
{
    std::cout << "Testing 'QueueBridge Stamped<PointCloud>'" << std::endl;
    Imu imu{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    auto ts_sent = HighResTime::now();
    PointCloud pc_to_send;
    pc_to_send.rings_ = 2;
    pc_to_send.points_.push_back({1, 2, 3});
    pc_to_send.points_.push_back({2, 3, 4});
    pc_to_send.points_.push_back({3, 4, 5});

    Stamped<PointCloud> pcl_sent{std::move(pc_to_send), ts_sent};
    Stamped<PointCloud> pcl_received;

    bool received = false;
    bool sending = true;

    auto in = std::make_shared<ConcurrentRingBuffer<Stamped<PointCloud>>>(16);
    auto out = std::make_shared<ConcurrentRingBuffer<Stamped<PointCloud>>>(16);
    QueueBridge<Stamped<PointCloud>, true> bridge{in, out, 2234};

    std::thread receive_thread{[&]()
    {
        Receiver<Stamped<PointCloud>> receiver{"127.0.0.1", 2234, 20ms};
        while (!received)
        {
            if (receiver.receive(pcl_received))
            {
                received = true;
            }
        }
    }};

    std::thread send_thread{[&]()
    {
        while (sending)
        {
            in->push(pcl_sent);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }};

    bridge.start();
    while (!received)
    {
        std::this_thread::yield();
    }
    bridge.stop();

    sending = false;

    receive_thread.join();
    send_thread.join();

    const auto& [ pcl_data, ts ] = pcl_received;
    REQUIRE(pcl_data.points_ == pcl_sent.data_.points_);
    REQUIRE(ts == ts_sent);
    REQUIRE(out->size() != 0);
}