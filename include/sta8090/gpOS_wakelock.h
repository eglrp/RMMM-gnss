/**
 * @file    gpOS_wakelock.h
 * @brief   OS wakelock definitions and macros.
 *
 * @addtogroup OS
 */

/*****************************************************************************
   includes
*****************************************************************************/

#ifndef GPOS_WAKELOCK_H
#define GPOS_WAKELOCK_H

#include "typedefs.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef tU8 gpOS_wakelockid_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t     gpOS_wakelock_init              ( gpOS_partition_t *partition);

extern gpOS_error_t     gpOS_wakelock_register          ( gpOS_wakelockid_t *id);
extern gpOS_error_t     gpOS_wakelock_acquire           ( gpOS_wakelockid_t id);
extern gpOS_error_t     gpOS_wakelock_release           ( gpOS_wakelockid_t id, const gpOS_clock_t *interval_ms );
extern gpOS_error_t     gpOS_wakelock_status            ( gpOS_wakelockid_t id, boolean_t *status);

/* To be called only in priviledge mode - unprotected functions !!! */
extern gpOS_error_t     gpOS_wakelock_get_value_unp     ( tU32 *value);

extern void             gpOS_wakelock_task_next_activity( gpOS_clock_t** nearest_timer);

#endif /* GPOS_WAKELOCK_H */
