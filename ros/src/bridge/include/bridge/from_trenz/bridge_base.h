/**
 * @file bridge_base.h
 * @author Julian Gaal
 * @date 2020-09-29
 */

#pragma once

#include <string>
#include <iostream>
#include <ros/node_handle.h>
#include <comm/receiver.h>
#include <util/process_thread.h>
#include <chrono>

namespace fastsense::bridge
{   

/**
 * @brief The BridgeBase class is the base of all communication between Trenz board and an Ubuntu host
 * 
 * Each class that inherits from BridgeBase implements each a receiver, and a publisher
 * - The receiver receives data from type T and PORT via zeromq
 * - The publisher transforms the data received via zeromq into a ROS message of type P and publishes as soon as it becomes available
 * 
 * @tparam T data type that's sent via zeromq
 * @tparam P type of ROS publisher
 * @tparam PORT port that zeromq should listen
 */
template <typename T, typename P, int PORT>
class BridgeBase
{
private:
    /// Receiver of data from board
    comm::Receiver<T> receiver_;

    /// ros publisher of matching ROS message
    ros::Publisher pub_;

public:

    /**
     * @brief Destroy the Bridge Base object
     */
    virtual ~BridgeBase() = default;
    
    /**
     * @brief Perform on bridge "iteration": wait for data, convert it to a ROS message, publish
     */
    virtual void run() = 0;

    /**
     * @brief Return received msg
     * 
     * @return const T& received msg
     */
    inline const T& msg() const 
    {
        return msg_;
    }

    /**
     * @brief Return publisher
     * 
     * @return ros::Publisher& reference to publisher
     */
    inline const ros::Publisher& pub() const
    {
        return pub_;
    }

protected:
    /**
     * @brief Construct a new Bridge Base object
     * 
     * @param n ROS node handle
     * @param topic ROS topic that will be published to
     * @param board_addr board addr that will be listened to by zeromq
     * @param msg_buffer_size ROS msg buffer size
     * @param timeout (ms) length of time to wait for message before continuing
     */
    inline BridgeBase(ros::NodeHandle& n,
                      const std::string& topic,
                      const std::string& board_addr,
                      size_t msg_buffer_size = 1000,
                      std::chrono::milliseconds timeout = 100)
    :   receiver_{board_addr, PORT, timeout},
        pub_{n.advertise<P>(topic, msg_buffer_size)},
        msg_{},
        n{n}
    {
    }

    /// msg of type T that is received via zeromq
    T msg_;

    /// NodeHandle
    ros::NodeHandle& n;

    /**
     * Receive a message (non blocking)
     * @return true if message was returned within timeout
     * @return false if there was no message within timeout period
     */
    virtual bool receive() final
    {
        return receiver_.receive(msg_);
    }

    /**
     * @brief Pure virtual convert function: convert T to ROS message
     */
    virtual void convert() = 0;

    /**
     * @brief Pure virtual publish function: publish result from convert()
     */
    virtual void publish() = 0;
};
    
}