#pragma once

/**
 * @file runner.h
 * @author Marcel Flottmann
 */

/**
 * @brief Runner object: manages starting and stopping ProcessThread objects
 * 
 * @tparam T : object inheriting ProcessThread
 */
template<typename T>
class Runner
{
private:
    T& object;
public:
    /**
     * @brief Construct a new Runner object: start the thread
     * 
     * @param obj 
     */
    explicit Runner(T& obj) : object(obj)
    {
        object.start();
    }

    /**
     * @brief Destroy the Runner object: stop the thread
     */
    ~Runner()
    {
        object.stop();
    }
};