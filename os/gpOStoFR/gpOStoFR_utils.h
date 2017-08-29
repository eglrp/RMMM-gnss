/**
 * @file    OStoFR_utils.h
 * @brief   OS wrapper messages internal definitions and macros.
 *
 * @addtogroup gpOS_WRAPPER
 */

#ifndef OS20FR_UTILS_H
#define OS20FR_UTILS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "FreeRTOS.h"
#include "gpOS_time.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*
 * Since FreeRTOS does not provide creation of a task in suspend mode,
 * Task is created with low prio (so will not start) and is immediatly
 * suspended by creator
 */
#define PRIO_FOR_CREATION_OF_SUSPENDED_TASK (tskIDLE_PRIORITY + 1)

#define O2F_W_ASSERT(cond) if(cond) ; else vFreeRTOS_Abort()

#define O2F_W_OS_TICKS_PER_SEC ((gpOS_clock_t)gpOS_timer_ticks_per_sec())

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

/*
 * Time convert from OS to FreeRTOS
 */

extern TickType_t   OStoFR_timeout ( const gpOS_clock_t *os_absolute_time_p);
extern TickType_t   OStoFR_ticks   ( gpOS_clock_t os_ticks);
extern gpOS_clock_t FRtoOS_ticks   ( TickType_t frtos_ticks);

#endif /* OS20FR_UTILS_H */
