#pragma once

#include <ros/ros.h>
#include <util/time.h>

namespace fastsense::bridge
{

/**
 * @brief Convert std::chrono timestamp from trenz board into ros::Time
 * 
 * @param ts std::chrono time_point
 * @param discard_timestamp if set to true, replace timestamp with ros::Time::now()
 * @return ros::Time::now() if (discard_timestamp), else converted std::chrono -> ros::Time
 */
inline ros::Time timestamp_to_rostime(const fastsense::util::HighResTimePoint& ts, bool discard_timestamp)
{
	if (discard_timestamp)
	{
		return ros::Time::now();
	}

	auto tstamp = ts.time_since_epoch();
	int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp).count() % 1'000'000'000UL;
    return ros::Time(sec, nsec);
}

}