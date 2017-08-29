/**
 * @file    gpOS_common.h
 * @brief   OS top level include file.
 *
 * @addtogroup OS
 */

#ifndef gpOS_COMMON_H
#define gpOS_COMMON_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_types.h"
#include "gpOS_interrupt.h"
#include "gpOS_memory.h"
#include "gpOS_time.h"
#include "gpOS_semaphore.h"
#include "gpOS_message.h"
#include "gpOS_mutex.h"
#include "gpOS_task.h"
#include "gpOS_kernel.h"
#include "gpOS_timelist.h"
#include "gpOS_wakelock.h"

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

extern const tChar *     gpOS_version        ( void);

#endif /* gpOS_COMMON_H */
