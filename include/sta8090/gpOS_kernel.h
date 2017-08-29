/**
 * @file    kernel.h
 * @brief   OS kernel definitions and macros.
 *
 * @addtogroup OS
 */

#ifndef gpOS_KERNEL_H
#define gpOS_KERNEL_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef void *gpOS_syscall_param_t;                             /**< System call parameter type */

typedef void (*gpOS_syscall_func_t)   ( gpOS_syscall_param_t);  /**< System call type */

typedef tUInt *gpOS_kernel_config_t;        /**< ARM system stacks sizes table */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   gpOS_kernel_init               ( gpOS_partition_t *part, gpOS_kernel_config_t kernel_cfg);
extern gpOS_error_t   gpOS_kernel_start              ( void);
extern gpOS_task_t *  gpOS_kernel_get_active_task    ( void);
extern void           gpOS_kernel_set_timeslice      ( gpOS_bool_t new_ts_status);
extern gpOS_bool_t    gpOS_kernel_get_timeslice      ( void);
extern void           gpOS_kernel_user_system_call   ( gpOS_syscall_func_t syscall_func, gpOS_syscall_param_t syscall_param);
extern void           gpOS_kernel_lock               ( void);
extern void           gpOS_kernel_unlock             ( void);

extern tUInt          gpOS_kernel_get_context_switch_number      ( void);
extern tUInt          gpOS_kernel_get_interrupts_occurred_number ( void);
extern tUInt          gpOS_kernel_get_interrupts_spent_time      ( void);

#endif /* gpOS_KERNEL_H */
