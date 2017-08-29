/**
 * @file    os20togpOS_kernel.c
 * @brief   OS20 kernel to genericOS kernel wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_kerneli.h"
#include "os20togpOS_taski.h"
#include "os20togpOS_types.h"

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

task_t *kernel_get_active_task( void)
{
  return gpOS_kernel_get_active_task();
}

void kernel_set_timeslice( os20_bool_t new_status)
{
	gpOS_kernel_set_timeslice(new_status);
}

os20_bool_t kernel_get_timeslice( void)
{
  return gpOS_kernel_get_timeslice();
}

void kernel_user_system_call( os20_syscall_func_t func, os20_syscall_param_t param)
{
  gpOS_kernel_user_system_call( func, param);
}

void kernel_lock( void)
{
  gpOS_kernel_lock();
}

void kernel_unlock( void)
{
  gpOS_kernel_unlock();
}

tUInt kernel_get_context_switch_number( void)
{
  return gpOS_kernel_get_context_switch_number();
}

tUInt kernel_get_interrupts_occurred_number( void)
{
  return gpOS_kernel_get_interrupts_occurred_number();
}

tUInt kernel_get_interrupts_spent_time( void)
{
  return gpOS_kernel_get_interrupts_spent_time();
}
