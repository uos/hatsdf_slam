#pragma once

/**
 * @file logger.h
 * @author Marcel Flottmann
 */

#include <mutex>
#include <sstream>
#include <util/time.h>
#include <iomanip>
#include <vector>
#include <memory>
#include <algorithm>

#include "sink.h"

namespace fastsense::util::logging
{

enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};

class Logger
{
    Logger();

    static Logger& getInst();

    template<typename T, typename ...Args>
    void print(std::ostringstream& msg, const T& val, const Args& ...args);

    void print(std::ostringstream& msg);

    template<typename ...Args>
    void writeMessage(LogLevel level, const Args& ...args);

    const char* const logLevelToString[5] =
    {
        "[DEBUG]   ",
        "[INFO]    ",
        "[WARNING] ",
        "[ERROR]   ",
        "[FATAL]   ",
    };

    std::mutex mtx;

    LogLevel currentLogLevel;

    std::vector<std::shared_ptr<sink::Sink>> sinks;

public:
    /**
     * @brief delete assignment operator because of pointer member variable
     *
     * @return Logger& other Buffer
     */
    Logger& operator=(Logger&) = delete;

    /**
     * @brief delete copy constructor because of pointer member variable
     * @param Logger& other buffer
     */
    Logger(Logger&) = delete;

    /// delete move assignment operator
    Logger& operator=(Logger&&) noexcept = delete;

    /// delete move constructor
    Logger(Logger&&) = delete;

    /// default destructor
    ~Logger() = default;

    template<typename ...Args>
    static void debug(const Args& ...args);

    template<typename ...Args>
    static void info(const Args& ...args);

    template<typename ...Args>
    static void warning(const Args& ...args);

    template<typename ...Args>
    static void error(const Args& ...args);

    template<typename ...Args>
    static void fatal(const Args& ...args);

    template<typename ...Args>
    static void log(LogLevel level, const Args& ...args);

    static void setLoglevel(LogLevel level);

    static void addSink(const std::shared_ptr<sink::Sink>& s);

    static void removeSink(const std::shared_ptr<sink::Sink>& s);
};

} // namespace fastsense::util::logging

#include "logger.tcc"
