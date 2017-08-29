/**
 * @file    ostime.h
 * @brief   OS20 time handling definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef gpOS_TIME_H
#define gpOS_TIME_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define gpOS_TIMEOUT_IMMEDIATE (&_OS_TimeoutImmediate)
#define gpOS_TIMEOUT_INFINITY  (&_OS_TimeoutInfinity)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef tUInt gpOS_clock_t;                  /**< OS20 clock type */

/*****************************************************************************
   exported variables
*****************************************************************************/

extern gpOS_clock_t   _OS_TimeoutImmediate;  /**< immediate timeout constant */
extern gpOS_clock_t   _OS_TimeoutInfinity;   /**< infinity timeout constant */

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tInt           gpOS_time_after                 ( const gpOS_clock_t start_time, const gpOS_clock_t delay);
extern gpOS_clock_t   gpOS_time_minus                 ( const gpOS_clock_t first_time, const gpOS_clock_t second_time);
extern gpOS_clock_t   gpOS_time_plus                  ( const gpOS_clock_t first_time, const gpOS_clock_t second_time);
extern gpOS_clock_t   gpOS_time_now                   ( void);

extern void           gpOS_timer_set_clock            ( void *bsp_timer_cfg_ptr, const tUInt update_value);
extern void           gpOS_timer_tick_handler         ( void);
extern void           gpOS_timer_timeout_handler      ( void);
extern void           gpOS_timer_timeout_task_update  ( const tUInt frequency_ratio, const tUInt ratio_scale, const tUInt current_time, const tUInt update_first);

extern gpOS_clock_t   gpOS_timer_ticks_per_usec       ( void);
extern gpOS_clock_t   gpOS_timer_ticks_per_msec       ( void);
extern gpOS_clock_t   gpOS_timer_ticks_per_sec        ( void);

extern void           gpOS_timer_init                 ( void *bsp_timer_cfg_ptr);
extern void           gpOS_timer_start                ( void);
extern void           gpOS_timer_suspend              ( void);
extern void           gpOS_timer_reset_timeout        ( void);

#endif /* gpOS_TIME_H */
