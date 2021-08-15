/**
 * @file zmq_context_manager.h
 * @author Marcel Flottmann
 * @author Julian Gaal
 * @date 2020-10-07
 */

#pragma once

#include <zmq.hpp>

namespace fastsense::comm
{

/**
 * @brief Manages zmq context and makes sure there's only one alive during runtime
 * 
 */
class ZMQContextManager
{
public:
    /// Delete default constructor
    ZMQContextManager() = delete;

    /// default destructor 
    ~ZMQContextManager() = default;

    /// delete copy constructor
    ZMQContextManager(const ZMQContextManager& other) = delete;

    /// delete move constructor
    ZMQContextManager(ZMQContextManager&& other) = delete;

    /// delete assignment operator
    ZMQContextManager& operator=(const ZMQContextManager& other) = delete;

    /// delete move assignment operator
    ZMQContextManager& operator=(ZMQContextManager&&) = delete;

    /**
     * @brief Get the Context object, singleton
     * 
     * @return zmq::context_t& zmq context on single thread
     */
    static zmq::context_t& getContext()
    {
        // Create context with one IO thread
        static zmq::context_t context{1};
        return context;
    }
};

} // namespace fastsense::comm
