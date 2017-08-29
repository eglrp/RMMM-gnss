/**
 * @file    os20togpOS_task.h
 * @brief   OS20 tasks functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_TASK_H
#define OS20TOGPOS_TASK_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_taski.h"

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
#define task_create(proc_func, proc_param, stack_size,  priority, name, flags)   gpOS_task_create(proc_func, proc_param, stack_size,  priority, name, flags)
#define task_create_p(custom_part, proc_func, proc_param, stack_size, priority, name, flags) 	OS_task_create_p(custom_part, proc_func, proc_param, stack_size, priority, name, flags)

#define task_delete(task)                       gpOS_task_delete(task)

#define task_delay(delay)                       gpOS_task_delay(delay)
#define task_delay_until(timeout)               gpOS_task_delay_until(timeout)
#define task_suspend(task)                      gpOS_task_suspend(task)
#define task_resume(task)                       gpOS_task_resume(task)

#define task_get_head()                         gpOS_task_get_head()
#define task_get_next(task)                     gpOS_task_get_next(task)
#define task_get_id()                           gpOS_task_get_id()

#define task_set_priority(task, new_priority)   gpOS_task_set_priority(task, new_priority)
#define task_get_priority(task)                 gpOS_task_get_priority(task)
#define task_get_stack_base(task)               gpOS_task_get_stack_base(task)
#define task_get_stack_ptr(task)                gpOS_task_get_stack_ptr(task)
#define task_get_stack_size(task)               gpOS_task_get_stack_size(task)
#define task_get_stack_used(task)               gpOS_task_get_stack_used(task)
#define task_get_cpuusage(task)                 gpOS_task_get_cpuusage(task)
#define task_get_runtime(task)                  gpOS_task_get_runtime(task)
#define task_get_name(task)                     gpOS_task_get_name(task)
#define task_get_data(task)                     gpOS_task_get_data(task)
#define task_set_data(task, data)               gpOS_task_set_data(task, data)
#define task_get_timeout(task)                  gpOS_task_get_timeout(task)

#define task_exit(exit_status)                  gpOS_task_exit(exit_status)
#define task_set_onexit_function(function)      gpOS_task_set_onexit_function(function)
#define task_wait(task_list, ntasks, timeout)   gpOS_task_wait(task_list, ntasks, timeout)

#define task_lock()                             gpOS_task_lock()
#define task_unlock()                           gpOS_task_unlock()

#endif /* OS20TOGPOS_TASK_H */
