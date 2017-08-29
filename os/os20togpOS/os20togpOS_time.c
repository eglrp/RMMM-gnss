/**
 * @file    os20togpOS_time.c
 * @brief   OS20 time to genericOS message wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_timei.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

os20_clock_t  _ST_TimeoutImmediate;
os20_clock_t  _ST_TimeoutInfinity;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
tInt time_after( const os20_clock_t time1, const os20_clock_t time2)
{
	return gpOS_time_after(time1, time2);
}

os20_clock_t time_minus( const os20_clock_t time1, const os20_clock_t time2)
{
	return gpOS_time_minus(time1, time2);
}

os20_clock_t time_plus( const os20_clock_t time1, const os20_clock_t time2)
{
	return gpOS_time_plus(time1, time2);
}

os20_clock_t time_now( void)
{
	return gpOS_time_now();
}

void timer_set_clock( const os20_clock_t clock)
{
	gpOS_timer_set_clock(clock);
}

void timer_timeout_handler( void)
{
	gpOS_timer_timeout_handler();
}

void timer_tick_handler( void)
{
	gpOS_timer_tick_handler();
}

os20_clock_t timer_ticks_per_usec( void)
{
	return gpOS_timer_ticks_per_usec();
}

os20_clock_t timer_ticks_per_msec( void)
{
	return gpOS_timer_ticks_per_msec();
}

os20_clock_t timer_ticks_per_sec( void)
{
	return gpOS_timer_ticks_per_sec();
}

