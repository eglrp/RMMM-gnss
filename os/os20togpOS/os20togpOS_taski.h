/**
 * @file    os20togpOS_taski.h
 * @brief   OS20 tasks internal definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_TASKI_H
#define OS20TOGPOS_TASKI_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define OS20_TASK_PRIORITY_LEVELS     gpOS_TASK_PRIORITY_LEVELS
#define OS20_TASK_MAX_USR_PRIORITY    gpOS_TASK_MAX_USR_PRIORITY
#define OS20_TASK_MIN_USR_PRIORITY    gpOS_TASK_MIN_USR_PRIORITY

#define OS20_TASK_STATE_ACTIVE        gpOS_TASK_STATE_ACTIVE
#define OS20_TASK_STATE_TERMINATED    gpOS_TASK_STATE_TERMINATED
#define OS20_TASK_STATE_SUSPENDING    gpOS_TASK_STATE_SUSPENDING
#define OS20_TASK_STATE_SUSPENDED     gpOS_TASK_STATE_SUSPENDED

#define OS20_TASK_FLAGS_ACTIVE        gpOS_TASK_FLAGS_ACTIVE
#define OS20_TASK_FLAGS_SUSPENDED     gpOS_TASK_FLAGS_SUSPENDED

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

#define task_state_t            gpOS_task_state_t
#define task_flags_t            gpOS_task_flags_t
#define task_exit_status_t      gpOS_task_exit_status_t
#define task_param_t            gpOS_task_param_t
#define task_function_t         gpOS_task_function_t
#define task_t                  gpOS_task_t
#define task_onexit_function_t  gpOS_task_onexit_function_t

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* OS20TOGPOS_TASKI_H */
