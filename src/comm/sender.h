#pragma once

/**
 * @file sender.h
 * @author Julian Gaal
 */

#include <string>
#include <msg/zmq_converter.h>
#include <comm/zmq_context_manager.h>

namespace fastsense::comm
{

/**
 * @brief Wrapper around ZeroMQ publisher that can send arbitrary data type T
 *
 * @tparam T data type to send
 */
template <typename T>
class Sender
{
public:
    /**
     * @brief Construct a new Sender object
     *
     * @param port Port to send on
     */
    Sender(uint16_t port)
    :   socket_{ZMQContextManager::getContext(), zmq::socket_type::pub}
    {
        socket_.setsockopt(ZMQ_SNDHWM, 2);
        socket_.bind("tcp://*:" + std::to_string(port));
    }

    /// Destroy the Sender object
    ~Sender() = default;

    /// Delete copy constructor
    Sender(const Sender&) = delete;


    /// @brief Delete move constructor
    Sender(Sender&&) = delete;


    /// Delete assignment operator
    Sender& operator=(Sender const&) = delete;

    /// Delete move assignment operator
    Sender& operator=(Sender&&) = delete;

    /**
     * @brief Send data (with size known at compile time)
     *
     * @tparam TT type used to compile only if T does NOT inherit from msg::ZMQConverter (therefore only static members)
     * @param data data to send
     * @param flag zeromq send flags, default: dontwait (non-blocking)
     */
    template < typename TT = T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    void send(const T& data, zmq::send_flags flag = zmq::send_flags::dontwait)
    {
        auto length = sizeof(T);
        zmq::message_t msg(length);
        memcpy(msg.data(), &data, length);
        socket_.send(msg, flag);
    }

    /**
     * @brief Send data (with size not known at compile time)
     *
     * @tparam TT type used to compile ONLY IF T does inherits from msg::ZMQConverter (therefore data with dynamic size)
     * @param data Data to send (must inherit from msg::ZMQConverter)
     */
    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    void send(const T& data)
    {
        zmq::multipart_t multi = data.to_zmq_msg();
        multi.send(socket_);
    }

private:
    /// ZeroMQ socket handles connection
    zmq::socket_t socket_;
};

} // namespace fastsense::comm
