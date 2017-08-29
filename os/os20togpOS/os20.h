/**
 * @file    os20.h
 * @brief   OS20 top level include file.
 *
 * @addtogroup OS20
 */

#ifndef OS20_H
#define OS20_H

/*****************************************************************************
   includes
*****************************************************************************/

#if defined( __GNUC__)
#undef OS20_LITE
#endif

#include "gpOS.h"
#include "OS20toOS_interrupt.h"
#include "OS20toOS_kernel.h"
#include "OS20toOS_memory.h"
#include "OS20toOS_message.h"
#include "OS20toOS_mutex.h"
#include "OS20toOS_semaphore.h"
#include "OS20toOS_task.h"
#include "OS20toOS_time.h"
#include "OS20toOS_timelist.h"
#include "OS20toOS_types.h"

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

#endif /* OS20_H */
