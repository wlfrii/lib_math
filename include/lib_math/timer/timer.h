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
 * @license		MIT
 * 
 * Copyright (C) 2019-Now Longfei Wang.
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
namespace timer{

/**
 * @brief Return current time point.
 * 
 * @note Class std::chrono::steady_clock represents a monotonic clock. The time 
 * points of this clock cannot decrease as physical time moves forward and the 
 * time between ticks of this clock is constant. This clock is not related to 
 * wall clock time (for example, it can be time since last reboot), and is most
 * suitable for measuring intervals.
 */
inline std::chrono::steady_clock::time_point getCurrentTimePoint() {
	return std::chrono::steady_clock::now();
}


/**
 * @brief Get the duration since the start time point, in milliseconds.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the other functions only in what argument(s) it accepts.
 * 
 * @param [in] start_time_point The start time point returned by 
 * 						        mmath::timer::getCurrentTimePoint().
 * 
 * @return The time duration represented by std::chrono::milliseconds
 * 
 * @see mmath::timer::getCurrentTimePoint().
 */
inline float getDurationSince(
	const ::std::chrono::steady_clock::time_point &start_time_point) {
	std::chrono::steady_clock::time_point now = getCurrentTimePoint();
	long long ms = std::chrono::duration_cast<std::chrono::microseconds>(
											now - start_time_point).count();
	return ms / 1000.f;
}


/**
 * @brief Get current time point since epoch. 
 * 
 * @remark This time counter is not stable than steady clock.
 * Thus, mmath::timer::getCurrentTimePoint() is recommended.
 * 
 * @tparam Tp The unit of the time, default unit is std::chrono::milliseconds.
 * 
 * @return  The time duration represented by Tp.
 * 
 */
template<typename Tp = std::chrono::milliseconds>
Tp getCurrentTimePointSinceEpoch() {
	return std::chrono::duration_cast<Tp>(
		std::chrono::system_clock::now().time_since_epoch());
}


/**
 * @brief Get the duration since a start time point since epoch.
 * 
 * @remark This is an overloaded function, provided for convenience. It differs 
 * from the other functions only in what argument(s) it accepts.
 * 
 * @tparam Tp  The unit of the time, default unit is std::chrono::milliseconds,
 *             and only should be one of [nanoseconds, microseconds, 
 * 			   milliseconds, seconds, minutes] for now.
 * @param [in] start_time_point  The start time point returned by 
 *                               mmath::timer::getCurrentTimePointSinceEpoch().
 * 
 * @return The time duration represented by Tp.
 * 
 * @see mmath::timer::getCurrentTimePointSinceEpoch().
 */
template<typename Tp = std::chrono::milliseconds>
float getDurationSince(const Tp &start_time_point) {
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
 * @brief Get the current time information in "YYmmdd_HHMMSS" string format.
 * 
 * @return Current time information in std::string.
 */
inline ::std::string getCurrentTimeStr() {
	time_t timep;
	time(&timep);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&timep));

	return std::string(tmp);
}


/**
 * @brief Get the current time information in "YYmmdd_HHMM" string format.
 * 
 * @return Current time information in std::string.
 */
inline ::std::string getCurrentMinuteStr() {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M", localtime(&timep));

    return std::string(tmp);
}


/**
 * @brief Get the current time information in "YYmmdd" string format.
 * 
 * @return Current time information in std::string.
 */
inline ::std::string getCurrentDateStr() {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d", localtime(&timep));

    return std::string(tmp);
}


/**
 * @brief Get the current time point information in "YYmmdd_HHMMSS_MS" string format.
 * 
 * @return Current time information in std::string.
 */
inline ::std::string getCurrentTimeStr() {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&timep));

    auto now = std::chrono::system_clock::now();
    uint64_t cms = 
	std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
    char strms[32];
    sprintf(strms, "_%03d", cms);

    return std::string(tmp) + strms;
}

} // timer
} // mmath


/**
 * @brief A macro for counting function time comsumption.
 * \param FUNC  The function to be timed
 * \param DURATION  The time duration tolerance
 * \param FMT, ...  The message to be printed if the time comsumption greater
 *                  than the DURATION
 */
#define MMATH_TIMER_COUNT_VOID_FUNC_TIME(FUNC, DURATION, FMT, ...) \
	do{ \
		  	auto start = mmath::timer::getCurrentTimePoint();\
		    FUNC;\
		    float ms = mmath::timer::getDurationSince(start); \
			if (ms > DURATION) { printf("[%.4f ms] elapsed:\t " FMT, ms, ##__VA_ARGS__);} \
	 } while (0)


/**
 * @brief A macro for counting function time comsumption.
 * \param FUNC  The function to be timed
 * \param DURATION  The time duration tolerance
 * \param RET  The returned value from FUNC
 * \param FMT, ...  The message to be printed if the time comsumption greater
 *                  than the DURATION
 */
#define MMATH_TIMER_COUNT_NONVOID_FUNC_TIME(FUNC, DURATION, RET, FMT, ...)  \
	do{  \
		  	auto start = mmath::timer::getCurrentTimePoint();\
		    RET = FUNC;\
		    float ms = mmath::timer::getDurationSince(start); \
			if (ms > DURATION) { printf("[%.4f ms] elapsed:\t " FMT, ms, ##__VA_ARGS__);} \
	 } while (0)


#endif // LIB_MATH_MTIMER_H_LF
