/**
 * @file    FRtoOS_kernel.c
 * @brief   Wrapper OS - FreeRTOS kernel implementation.
 *
 * @addtogroup gpOS_WRAPPER
 */

#include "gpOS_kernel.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FR_svci.h"

gpOS_task_t *gpOS_kernel_get_active_task( void)
{
  return gpOS_task_get_id();
}

void gpOS_kernel_set_timeslice( gpOS_bool_t new_ts_status)
{
}

gpOS_bool_t gpOS_kernel_get_timeslice( void)
{
  return gpOS_FALSE;
}

void gpOS_kernel_user_system_call( gpOS_syscall_func_t syscall_func, gpOS_syscall_param_t syscall_param)
{
  xSVC_user_system_call( syscall_func, syscall_param);
}

void  gpOS_kernel_lock( void)
{
  vTaskSuspendAll();
}

void  gpOS_kernel_unlock( void)
{
  xTaskResumeAll();
}

tUInt gpOS_kernel_get_context_switch_number( void)
{
  return xOS_stats_get_context_switch_number();
}

tUInt gpOS_kernel_get_interrupts_occurred_number( void)
{
  return xOS_stats_get_interrupts_occurred_number();
}

tUInt gpOS_kernel_get_interrupts_spent_time( void)
{
  return 0;
}

