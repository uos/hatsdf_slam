#pragma once

/**
 * @file queue_bridge.h
 * @author Marcel Flottmann
 * @author Julian Gaal
 * @date 2020-10-06
 */

#include <util/time.h>
#include <util/concurrent_ring_buffer.h>
#include <msg/point_cloud.h>
#include <util/process_thread.h>
#include <comm/sender.h>

namespace fastsense::comm
{

template<typename T_QUEUE, typename T_MSG, bool FORCE>
class QueueBridgeBase : public util::ProcessThread
{
public:
    /// typedef for shared_ptr of in/out buffer
    using BufferType = std::shared_ptr<util::ConcurrentRingBuffer<T_QUEUE>>;

    /**
     * @brief Construct a new Queue Bridge Base object
     * 
     * @param in input buffer
     * @param out output buffer
     * @param port port to send to
     * @param send if true, send, if not, directly to output buffer
     */
    QueueBridgeBase(const BufferType& in, const BufferType& out, uint16_t port, bool send = true) :
        in_{in}, out_{out}, sender_{port}, send_{send}
    {}

    /**
     * @brief Destroy the Queue Bridge Base object
     */
    virtual ~QueueBridgeBase() = default;

    /// delete copy constructor
    QueueBridgeBase(const QueueBridgeBase& other) = delete;

    /// delete move constructor
    QueueBridgeBase(QueueBridgeBase&& other) = delete;

    /// delete assignment operator
    QueueBridgeBase& operator=(const QueueBridgeBase& other) = delete;

    /// delete move assignment operator
    QueueBridgeBase& operator=(QueueBridgeBase&& other) = delete;


protected:
    /// Input Buffer
    BufferType in_;

    /// Output Buffer
    BufferType out_;

    /// Sender of Type of input buffer
    Sender<T_MSG> sender_;

    /// To send of not. If not: pop and directly into output buffer
    bool send_;

    /**
     * @brief Endless loop that pops from input buffer of type T, 
     * sends via sender<T> and pushes popped data into output buffer
     */
    void thread_run() override
    {
        T_QUEUE val;
        while (this->running)
        {
            if (!this->in_->pop_nb(&val, DEFAULT_POP_TIMEOUT))
            {
                continue;
            }
            if (this->out_)
            {
                if constexpr (FORCE)
                {
                    this->out_->push_nb(val, true);
                }
                else
                {
                    this->out_->push(val);
                }
            }

            if (send_)
            {
                this->send(val);
            }

        }
    }

    /**
     * @brief Send value of type T_QUEUE (from input buffer)
     * 
     * @param val value of type T_QUEUE (from input buffer)
     */
    virtual void send(const T_QUEUE& val) = 0;
};

/**
 * @brief QueueBridge Specialization where data is poped, send and pushed to same buffer
 * 
 * @tparam T any type
 * @tparam FORCE push non blocking in output buffer
 */
template<typename T, bool FORCE>
class QueueBridge : public QueueBridgeBase<T, T, FORCE>
{
public:
    using QueueBridgeBase<T, T, FORCE>::QueueBridgeBase;

    /// default destructor
    ~QueueBridge() override = default;

    /// delete copy constructor
    QueueBridge(const QueueBridge& other) = delete;

    /// delete move constructor
    QueueBridge(QueueBridge&& other) = delete;

    /// delete assignment operator
    QueueBridge& operator=(const QueueBridge& other) = delete;

    /// delete move assignment operator
    QueueBridge& operator=(QueueBridge&& other) = delete;

protected:
    void send(const T& val) override
    {
        this->sender_.send(val);
    }
};

/**
 * @brief QueueBridge specialization for shared_ptr<T>. Make copy of T and send T
 * 
 * @tparam T std::shared_ptr<T>
 * @tparam FORCE if true, push non blocking in output buffer
 */
template<typename T, bool FORCE>
class QueueBridge<std::shared_ptr<T>, FORCE> : public QueueBridgeBase<std::shared_ptr<T>, T, FORCE>
{
public:
    using QueueBridgeBase<std::shared_ptr<T>, T, FORCE>::QueueBridgeBase;

    /// default destructor
    ~QueueBridge() override = default;

    /// delete copy constructor
    QueueBridge(const QueueBridge& other) = delete;

    /// delete move constructor
    QueueBridge(QueueBridge&& other) = delete;

    /// delete assignment operator
    QueueBridge& operator=(const QueueBridge& other) = delete;

    /// delete move assignment operator
    QueueBridge& operator=(QueueBridge&& other) = delete;

protected:
    void send(const std::shared_ptr<T>& val) override
    {
        this->sender_.send(*val);
    }
};

/**
 * @brief QueueBridgeBase specialization for PointCloudPtrStamped (trenz to ROS). 
 * Creates a copy of PointCloud from PointCloud::Ptr and sends a Stamped<PointCloud>
 * 
 * @tparam T msg::Stamped<std::shared_ptr<msg::PointCloud>>
 * @tparam FORCE if true, push non blocking in output buffer
 */
template<bool FORCE>
class QueueBridge<msg::Stamped<std::shared_ptr<msg::PointCloud>>, FORCE> : public QueueBridgeBase<msg::Stamped<std::shared_ptr<msg::PointCloud>>, msg::PointCloudStamped, FORCE>
{
public:
    using QueueBridgeBase<msg::Stamped<std::shared_ptr<msg::PointCloud>>, msg::PointCloudStamped, FORCE>::QueueBridgeBase;
    
    /// default destructor
    ~QueueBridge() override = default;

    /// delete copy constructor
    QueueBridge(const QueueBridge& other) = delete;

    /// delete move constructor
    QueueBridge(QueueBridge&& other) = delete;

    /// delete assignment operator
    QueueBridge& operator=(const QueueBridge& other) = delete;

    /// delete move assignment operator
    QueueBridge& operator=(QueueBridge&& other) = delete;

protected:
    void send(const msg::Stamped<std::shared_ptr<msg::PointCloud>>& val) override
    {
        msg::PointCloud pcl = *val.data_;
        this->sender_.send(msg::PointCloudStamped{std::move(pcl), val.timestamp_});
    }
};

/**
 * @brief QueueBridge specialization for Stamped<T>
 * 
 * @tparam T Stamped<T>
 * @tparam FORCE if true, push non blocking in output buffer
 */
template<typename T, bool FORCE>
class QueueBridge<msg::Stamped<T>, FORCE> : public QueueBridgeBase<msg::Stamped<T>, msg::Stamped<T>, FORCE>
{
public:
    using QueueBridgeBase<msg::Stamped<T>, msg::Stamped<T>, FORCE>::QueueBridgeBase;
    
    /// default destructor
    ~QueueBridge() override = default;

    /// delete copy constructor
    QueueBridge(const QueueBridge& other) = delete;

    /// delete move constructor
    QueueBridge(QueueBridge&& other) = delete;

    /// delete assignment operator
    QueueBridge& operator=(const QueueBridge& other) = delete;

    /// delete move assignment operator
    QueueBridge& operator=(QueueBridge&& other) = delete;

protected:
    void send(const msg::Stamped<T>& val) override
    {
        this->sender_.send(val);
    }
};

} // namespace fastsense::comm