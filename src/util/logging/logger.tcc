#pragma once

/**
 * @file logger.tcc
 * @author Marcel Flottmann
 */

namespace fastsense::util::logging
{

inline Logger::Logger() : mtx{}, currentLogLevel{LogLevel::Warning}, sinks{}
{
}

inline Logger& Logger::getInst()
{
    static Logger logger;
    return logger;
}

template<typename T, typename ...Args>
void Logger::print(std::ostringstream& msg, const T& val, const Args& ...args)
{
    msg << val;
    print(msg, args...);
}

inline void Logger::print(std::ostringstream& msg)
{
    msg << '\n';
}

template<typename ...Args>
void Logger::writeMessage(LogLevel level, const Args& ...args)
{
    if (level >= currentLogLevel)
    {
        std::ostringstream msg;

        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()  % 1000;
        msg << std::put_time(std::localtime(&t), "%F %T.");
        auto default_fill = msg.fill();
        msg << std::setfill('0') << std::setw(3) << ms << ' ';
        msg << std::setfill(default_fill);

        msg << logLevelToString[(int)level];
        print(msg, args...);
        auto msg_str = msg.str();

        std::lock_guard guard(mtx);
        for (auto& sink : sinks)
        {
            sink->write(msg_str);
        }
    }
}

template<typename ...Args>
void Logger::debug(const Args& ...args)
{
    getInst().writeMessage(LogLevel::Debug, args...);
}

template<typename ...Args>
void Logger::info(const Args& ...args)
{
    getInst().writeMessage(LogLevel::Info, args...);
}

template<typename ...Args>
void Logger::warning(const Args& ...args)
{
    getInst().writeMessage(LogLevel::Warning, args...);
}

template<typename ...Args>
void Logger::error(const Args& ...args)
{
    getInst().writeMessage(LogLevel::Error, args...);
}

template<typename ...Args>
void Logger::fatal(const Args& ...args)
{
    getInst().writeMessage(LogLevel::Fatal, args...);
}

template<typename ...Args>
void Logger::log(LogLevel level, const Args& ...args)
{
    getInst().writeMessage(level, args...);
}

inline void Logger::setLoglevel(LogLevel level)
{
    getInst().currentLogLevel = level;
}

inline void Logger::addSink(const std::shared_ptr<sink::Sink>& s)
{
    getInst().sinks.push_back(s);
}

inline void Logger::removeSink(const std::shared_ptr<sink::Sink>& s)
{
    auto& sinks = getInst().sinks;
    auto newEnd = std::remove(sinks.begin(), sinks.end(), s);
    sinks.erase(newEnd, sinks.end());
}

} // namespace fastsense::util::logging
