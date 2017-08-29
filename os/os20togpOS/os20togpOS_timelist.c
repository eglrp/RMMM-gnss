/**
 * @file    os20togpOS_timelist.c
 * @brief   OS20 timelist to genericOS message wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_timelisti.h"
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
void timer_list_init( os20_clock_t *timer_list, const tInt num_timers)
{
	gpOS_timer_list_init(timer_list, num_timers);
}

void timer_list_update(os20_clock_t *timer_list, const tInt timer_index, const tInt timer_interval)
{
	gpOS_timer_list_update(timer_list, timer_index, timer_interval);
}

void timer_list_calc_nearest(os20_clock_t *timer_list, const tInt num_timers, tInt *nearest, os20_clock_t *nearest_time)
{
	gpOS_timer_list_calc_nearest(timer_list, num_timers, nearest, nearest_time);
}

void timer_list_update_absolute(os20_clock_t *timer_list, const tInt timer_index, const os20_clock_t timer_value)
{
	gpOS_timer_list_update_absolute(timer_list, timer_index, timer_value);
}

