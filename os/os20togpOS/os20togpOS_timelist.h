/**
 * @file    os20togpOS_timelist.h
 * @brief   OS20 time list functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_TIMELIST_H
#define OS20TOGPOS_TIMELIST_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_timelisti.h"

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
#define timer_list_init(timer_list, num_of_timers)                                 gpOS_timer_list_init(timer_list, num_of_timers)
#define timer_list_update(timer_list, idx, delay)                                  gpOS_timer_list_update(timer_list, idx, delay)
#define timer_list_calc_nearest(timer_list, num_of_timers, nearest, nearest_time)  gpOS_timer_list_calc_nearest(timer_list, num_of_timers, nearest, nearest_time)
#define timer_list_update_absolute(timer_list, idx, timer_value)                   gpOS_timer_list_update_absolute(timer_list, idx, timer_value)

#endif /* OS20TOGPOS_TIMELIST_H */
