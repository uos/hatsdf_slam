#pragma once

/**
 * @file buffered_receiver.h
 * @author Julian Gaal, Pascal Buschermoehle
 */

#include <util/time.h>
#include <util/process_thread.h>
#include <util/concurrent_ring_buffer.h>
#include <comm/receiver.h>
#include <msg/imu.h>
#include <msg/point_cloud.h>
#include <iostream>

namespace fastsense::comm
{

/**
 * @brief BufferedReceiver: Receives messages via Receiver and writes data into buffer
 * 
 * @tparam BUFF_T Buffer Type
 * @tparam RECV_T Receive Type
 */
template <typename BUFF_T, typename RECV_T>
class BufferedReceiver : public util::ProcessThread
{
public:
    /**
     * @brief Delete copy constructor
     */
    BufferedReceiver(const BufferedReceiver&) = delete;


    /**
     * @brief Delete move constructor
     */
    BufferedReceiver(BufferedReceiver&&) = delete;


    /**
     * @brief Delete assignment operator
     */
    BufferedReceiver& operator=(BufferedReceiver const&) = delete;

    /**
     * @brief Destroy the Buffered Receiver object
     */
    virtual ~BufferedReceiver() override = default;

    /**
     * @brief 'receive' receives one message
     * and is called from an endless loop in thread_run
     */
    virtual bool receive() = 0;

    /**
     * @brief thread_run receives until the thread is stopped by calling .stop()
     * 
     */
    void thread_run() override
    {
        while (running)
        {
            receive();
        }
    }

protected:
    /**
     * @brief Construct a new Buffered Receiver object
     * 
     * @param addr address the receiver connects to
     * @param port port the receiver connects to 
     * @param timeout timeout for receiver
     * @param buffer buffer to write the incoming messages
     */
    BufferedReceiver(   const std::string& addr, 
                        uint16_t port, 
                        std::chrono::milliseconds timeout, 
                        typename util::ConcurrentRingBuffer<BUFF_T>::Ptr buffer)
    : receiver_{addr, port, timeout}
    , buffer_{buffer}
    {}

    /// Receiver that's used to get data
    Receiver<RECV_T> receiver_;

    /// Received data is written into buffer
    typename util::ConcurrentRingBuffer<BUFF_T>::Ptr buffer_;
    
    /// individual message that is received
    RECV_T msg_;
};

/**
 * @brief BufferedImuStampedReceiver: receives imu message, write imu message into buffer
 */
class BufferedImuStampedReceiver : public BufferedReceiver<msg::ImuStamped, msg::ImuStamped>
{
public:
    /**
     * @brief Construct a new Buffered ImuStamped Receiver object
     * 
     * @param addr address the receiver connects to
     * @param port port the receiver connects to 
     * @param timeout timeout for receiver
     * @param buffer buffer to write the incoming messages
     */
    BufferedImuStampedReceiver( const std::string& addr, 
                                uint16_t port, 
                                std::chrono::milliseconds timeout,
                                msg::ImuStampedBuffer::Ptr buffer)
    : BufferedReceiver{addr, port, timeout, buffer}
    {}

    /**
     * @brief Destroy the Buffered Imu Stamped Receiver object
     */
    ~BufferedImuStampedReceiver() final = default;

    /**
     * @brief Receive ImuStamped (non blocking) and write into buffer, if received
     *
     * @return
     */
    bool receive() final
    {
        if (receiver_.receive(msg_))
        {
            buffer_->push_nb(std::move(msg_));
            return true;
        }

        return false;
    }

    using UPtr = std::unique_ptr<BufferedImuStampedReceiver>;
};

/**
 * @brief BufferedPclStampedReceiver: Receiver PointCloudStamped, write to PointCloud*Ptr*Stamped Buffer
 * 
 */
class BufferedPclStampedReceiver : public BufferedReceiver<msg::PointCloudPtrStamped, msg::PointCloudStamped>
{
public:
    /**
     * @brief Construct a new Buffered Pcl Stamped Receiver object
     * 
     * @param addr address the receiver connects to
     * @param port port the receiver connects to 
     * @param timeout timeout for receiver
     * @param buffer buffer to write the incoming messages
     */
    BufferedPclStampedReceiver( const std::string& addr, 
                                uint16_t port, 
                                std::chrono::milliseconds timeout,
                                msg::PointCloudPtrStampedBuffer::Ptr buffer)
    : BufferedReceiver{addr, port, timeout, buffer}
    {}

    /**
     * @brief Destroy the Buffered Pcl Stamped Receiver object
     */
    ~BufferedPclStampedReceiver() final = default;

    /**
     * @brief Receive PointCloudStamped (non blocking), convert to PointCloud*Ptr*Stamped if received, and save
     *
     * @return true if message received and converted and saved
     * @return false if no message received
     */
    bool receive() final
    {
        if (receiver_.receive(msg_))
        {
            auto& [ pcl, ts ] = msg_;
            buffer_->push_nb(std::move(msg::PointCloudPtrStamped{std::make_shared<msg::PointCloud>(std::move(pcl)), ts }));
            return true;
        }

        return false;
    }

    using UPtr = std::unique_ptr<BufferedPclStampedReceiver>;
};


} // namespace fastsense::comm