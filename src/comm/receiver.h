#pragma once

/**
 * @file receiver.h
 * @author Julian Gaal
 * @author Marcel Flottmann
 */

#include <msg/point_cloud.h>
#include <comm/zmq_context_manager.h>

namespace fastsense::comm
{

/**
 * @brief Receiver wraps zeromq via cppzmq and supports receiving data of type T
 *
 * @tparam T data type that will be received
 */
template <typename T>
class Receiver
{
public:
    /**
     * @brief Construct a new Receiver object
     *
     * @param addr which address receiver should listen to
     * @param port which port receiver should listen to
     */
    Receiver(std::string addr, uint16_t port, std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
    :   socket_{ZMQContextManager::getContext(), zmq::socket_type::sub}
    ,   timeout_{timeout}
    ,   pollitems_{{socket_, 0, ZMQ_POLLIN, 0}}
    {
        if (addr.empty())
        {
            throw std::runtime_error("Can't connect to address ''");
        }

        if (timeout.count() < 0)
        {
            throw std::runtime_error("Invalid timeout chosen");
        }
        
        socket_.connect("tcp://" + addr + ":" + std::to_string(port));

        // no filters, subscribe to all
        socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }

    /**
     * @brief Delete copy constructor
     */
    Receiver(const Receiver&) = delete;


    /**
     * @brief Delete move constructor
     */
    Receiver(Receiver&&) = delete;


    /**
     * @brief Delete assignment operator
     */
    Receiver& operator=(Receiver const&) = delete;

    /**
     * @brief Destroy the Receiver object
     */
    virtual ~Receiver() = default;

    /**
     * @brief Performs poll on socket
     * 
     * @return true if successfully polled
     * @return false if poll took longer than timeout
     */
    bool poll_successful()
    {
        zmq::poll(pollitems_.data(), 1, timeout_);
        return pollitems_[0].revents & ZMQ_POLLIN;
    }

    /**
     * @brief Receive a message of static size, by reference
     *
     * @tparam TT type used to compile only if T DOES NOT inherit from msg::ZMQConverter (therefore members of dynamic size)
     * @param target where message is copied to
     * @param flags ZMQ receive flags, default 'none' -> blocking
     */
    template < typename TT = T, std::enable_if_t < !std::is_base_of_v<msg::ZMQConverter, TT>, int > = 0 >
    bool receive(T& target, zmq::recv_flags flags = zmq::recv_flags::none)
    {
        if (poll_successful())
        {
            zmq::message_t msg;
            socket_.recv(msg, flags);
            target = *static_cast<T*>(msg.data());
            return true;
        } 
        
        return false;
    }

    /**
     * @brief Receive a message with elements of dynamic size (by reference)
     *
     * @tparam TT type used to compile ONLY IF T inherits from msg::ZMQConverter (therefore members of dynamic size)
     * @param target where message is copied to
     */
    template <typename TT = T, std::enable_if_t<std::is_base_of_v<msg::ZMQConverter, TT>, int> = 0>
    bool receive(T& target)
    {
        if (poll_successful())
        {
            zmq::multipart_t multi;
            multi.recv(socket_);
            target.from_zmq_msg(multi);
            return true;
        }

        return false;
    }

private:
    /// ZeroMQ socket
    zmq::socket_t socket_;

    /// timeout
    std::chrono::milliseconds timeout_;

    /// poll items
    std::vector<zmq::pollitem_t> pollitems_;
};

} // namespace fastsense::comm
