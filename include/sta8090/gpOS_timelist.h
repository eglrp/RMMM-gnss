/**
 * @file    timelist.h
 * @brief   OS time lists definitions and macros.
 *
 * @addtogroup OS
 */

#ifndef gpOS_TIMELIST_H
#define gpOS_TIMELIST_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_types.h"

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

extern void gpOS_timer_list_init            ( gpOS_clock_t *timer_list, const tInt num_of_timers);
extern void gpOS_timer_list_update          ( gpOS_clock_t *timer_list, const tInt idx, const tInt delay);
extern void gpOS_timer_list_calc_nearest    ( gpOS_clock_t *timer_list, const tInt num_of_timers, tInt *nearest, gpOS_clock_t *nearest_time);
extern void gpOS_timer_list_update_absolute ( gpOS_clock_t *timer_list, const tInt idx, const gpOS_clock_t timer_value);

#endif /* gpOS_TIMELIST_H */

