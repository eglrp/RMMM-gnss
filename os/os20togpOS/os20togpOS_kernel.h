/**
 * @file    os20togpOS_kernel.h
 * @brief   OS20 kernel functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_KERNEL_H
#define OS20TOGPOS_KERNEL_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_kerneli.h"

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

#define kernel_get_active_task()                                gpOS_kernel_get_active_task()
#define kernel_set_timeslice(new_status)                        gpOS_kernel_set_timeslice(new_status)
#define kernel_get_timeslice()                                  gpOS_kernel_get_timeslice()
#define kernel_user_system_call(syscall_func, syscall_param)    gpOS_kernel_user_system_call(syscall_func, syscall_param)
#define kernel_lock()                                           gpOS_kernel_lock()
#define kernel_unlock()                                         gpOS_kernel_unlock()

#define kernel_get_context_switch_number()                      gpOS_kernel_get_context_switch_number()
#define kernel_get_interrupts_occurred_number()                 gpOS_kernel_get_interrupts_occurred_number()
#define kernel_get_interrupts_spent_time()                      gpOS_kernel_get_interrupts_spent_time()

#endif /* OS20TOOS_KERNEL_H */
