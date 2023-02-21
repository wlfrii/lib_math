/**--------------------------------------------------------------------
 *																		
 *   				   Mathematics extension library 					
 *																		
 * Description:													
 * This file is part of lib_math. You can redistribute it and or modify 
 * it to construct your own project. It is wellcome to use this library
 * in your scientific research work.
 * 
 * @file 		timer.h 
 * 
 * @brief 		Design some interfaces for time counting.
 * 
 * @author		Longfei Wang
 * 
 * @date		2019/12/14
 * 
 * @license		
 * 
 * Copyright (C) 2019 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * 
 * -------------------------------------------------------------------*/
#ifndef LIB_MATH_MTIMER_H_LF
#define LIB_MATH_MTIMER_H_LF
#include <typeinfo>
#include <string>
#include <ctime>
#include <chrono>
/** Ignorethe waring in Windows. */
#if defined _WIN32 | defined _WIN64
#pragma warning( disable : 4996 ) // disable the error of localtime
#endif // Windows


namespace mmath{
namespace timer
{

/**
 * @brief Return current time point
 * Class std::chrono::steady_clock represents a monotonic clock. The time points of 
 * this clock cannot decrease as physical time moves forward and the time between ticks 
 * of this clock is constant. This clock is not related to wall clock time (for example,
 * it can be time since last reboot), and is most suitable for measuring intervals.
 */
inline std::chrono::steady_clock::time_point getCurrentTimePoint()
{
	return std::chrono::steady_clock::now();
}


/**
 * @brief Get the duration since the start time point
 * 
 * @param start_time_point The start time point returned by getCurrentTimePoint()
 * @return The time duration represented by std::chrono::milliseconds
 */
inline float getDurationSince(const ::std::chrono::steady_clock::time_point &start_time_point)
{
	std::chrono::steady_clock::time_point now = getCurrentTimePoint();
	long long ms = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_point).count();
	return ms / 1000.f;
}


/**
 * @brief Get tcurrent time point since epoch. This time counter is not stable than steady clock.
 * Thus, getCurrentTimePoint() is recommended.
 * 
 * @tparam Tp  The unit of the time, default is std::chrono::milliseconds
 * @return  The time duration represented by Tp
 */
template<typename Tp = std::chrono::milliseconds>
Tp getCurrentTimePointSinceEpoch()
{
	return ::std::chrono::duration_cast<Tp>(::std::chrono::system_clock::now().time_since_epoch());
}


/**
 * @brief Get the duration since a start time point returned by getCurrentTimePointSinceEpoch()
 * 
 * @tparam Tp  The unit of the time, default is std::chrono::milliseconds, and only should be 
 *             one of [nanoseconds, microseconds, milliseconds, seconds, minutes] for now
 * @param start_time_point  The start time point returned by getCurrentTimePointSinceEpoch()
 * @param unit  The time evaluated unit, default is std::chrono::milliseconds
 * @return float  The time duration represented by milliseconds
 */
template<typename Tp = std::chrono::milliseconds>
float getDurationSince(const Tp &start_time_point)
{
	auto current_time_point = getCurrentTimePointSinceEpoch();

	float k = 1;
	if (typeid(Tp) == typeid(std::chrono::nanoseconds)) {
		k = 1.f / 1000000.f;
	}
	else if (typeid(Tp) == typeid(std::chrono::microseconds)) {
		k = 1.f / 1000.f;
	}
	else if (typeid(Tp) == typeid(std::chrono::seconds)) {
		k = 1000.f;
	}
	else if (typeid(Tp) == typeid(std::chrono::minutes)) {
		k = 1000000.f;
	}
	else{
		return -1;
	}
	return (current_time_point - start_time_point).count() * k;
}


/**
 * @brief Get the current time string, as "YYmmdd_HHMMSS" format.
 * 
 * @return ::std::string 
 */
inline ::std::string getCurrentTimeStr()
{
	time_t timep;
	time(&timep);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&timep));

	return std::string(tmp);
}

} // timer
} // mmath


/**
 * @brief 
 * \param FUNC  The function to be timed
 * \param DURATION  The time duration tolerance
 * \param FMT, ...  The message to be printed if the time comsumption greate than the DURATION
 */
#define MMATH_TIMER_COUNT_VOID_FUNC_TIME(FUNC, DURATION, FMT, ...) \
	do{ \
		  	auto TIME_POINT = mmath::timer::getCurrentTimePoint();\
		    FUNC;\
		    float ms = mmath::timer::getDurationSince(start); \
			if (ms > DURATION) { printf("[%.4f] ms elapsed:\t " #FMT, ms, ##__VA_ARGS__);} \
	 } while (0)


/**
 * @brief 
 * \param FUNC  The function to be timed
 * \param DURATION  The time duration tolerance
 * \param RET  The returned value from FUNC
 * \param FMT, ...  The message to be printed if the time comsumption greate than the DURATION
 */
#define MMATH_TIMER_COUNT_NONVOID_FUNC_TIME(FUNC, DURATION, RET, FMT, ...)  \
	do{  \
		  	auto TIME_POINT = mmath::timer::getCurrentTimePoint();\
		    RET = FUNC;\
		    float ms = mmath::timer::getDurationSince(start); \
			if (ms > DURATION) { printf("[%.4f] ms elapsed:\t " #FMT, ms, ##__VA_ARGS__);} \
	 } while (0)


#endif // LIB_MATH_MTIMER_H_LF
