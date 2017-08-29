/**
 * @file    os20togpOS_time.h
 * @brief   OS20 time functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_TIME_H
#define OS20TOGPOS_TIME_H

/*****************************************************************************
   includes
*****************************************************************************/
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

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
#define time_after(start_time, delay)          gpOS_time_after(start_time, delay)
#define time_minus(first_time, second_time)    gpOS_time_minus(first_time, second_time)
#define time_plus(first_time, second_time)     gpOS_time_plus(first_time, second_time)
#define time_now()                             gpOS_time_now()

#define timer_set_clock(clock)                 gpOS_timer_set_clock(clock)
#define timer_timeout_handler()                gpOS_timer_timeout_handler()
#define timer_tick_handler()                   gpOS_timer_tick_handler()

#define timer_ticks_per_usec()                 gpOS_timer_ticks_per_usec()
#define timer_ticks_per_msec()                 gpOS_timer_ticks_per_msec()
#define timer_ticks_per_sec()                  gpOS_timer_ticks_per_sec()

#endif /* OS20TOGPOS_TIME_H */
