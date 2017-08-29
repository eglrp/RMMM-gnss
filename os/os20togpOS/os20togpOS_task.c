/**
 * @file    os20togpOS_task.c
 * @brief   OS20 task to genericOS message wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_taski.h"
#include "os20togpOS_memoryi.h"
#include "os20togpOS_timei.h"
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
task_t* task_create( task_function_t function, task_param_t param, const tSize stack_size, const tInt priority, const tChar *name, task_flags_t flags)
{
	return gpOS_task_create(function, param, stack_size,  priority, name, flags);
}

task_t* task_create_p( partition_t *partition_ptr, task_function_t function, task_param_t param, const tSize stack_size, const tInt priority, const tChar *name, task_flags_t flags)
{
	return gpOS_task_create_p(partition_ptr, function, param, stack_size, priority, name, flags);
}

os20_error_t task_delete( task_t* task)
{
	return gpOS_task_delete(task);
}

void task_delay( const os20_clock_t delay)
{
	gpOS_task_delay(delay);
}

void task_delay_until( const os20_clock_t timeout)
{
	gpOS_task_delay_until(timeout);
}

os20_error_t task_suspend( task_t* task)
{
	return gpOS_task_suspend(task);
}

os20_error_t task_resume( task_t* task)
{
	return gpOS_task_resume(task);
}

task_t* task_get_head( void)
{
	return gpOS_task_get_head();
}

task_t* task_get_next( task_t* task)
{
	return gpOS_task_get_next(task);
}

task_t* task_get_id( void)
{
	return gpOS_task_get_id();
}

tInt task_set_priority( task_t *task, const tInt new_priority)
{
	return gpOS_task_set_priority(task, new_priority);
}

tInt task_get_priority( task_t* task)
{
	return gpOS_task_get_priority(task);
}

void* task_get_stack_base( task_t* task)
{
	return gpOS_task_get_stack_base(task);
}

void* task_get_stack_ptr( task_t* task)
{
	return gpOS_task_get_stack_ptr(task);
}

tSize task_get_stack_size( task_t* task)
{
	return gpOS_task_get_stack_size(task);
}

tSize task_get_stack_used( task_t* task)
{
	return gpOS_task_get_stack_used(task);
}

tUInt task_get_cpuusage( task_t* task)
{
	return gpOS_task_get_cpuusage(task);
}

tUInt task_get_runtime( task_t* task)
{
	return gpOS_task_get_runtime(task);
}

const tChar* task_get_name (task_t* task)
{
	return gpOS_task_get_name(task);
}

void* task_get_data (task_t* task)
{
	return gpOS_task_get_data(task);
}

void* task_set_data(task_t* task, void* data)
{
	return gpOS_task_set_data(task, data);
}

os20_clock_t task_get_timeout(task_t* task)
{
	return gpOS_task_get_timeout(task);
}

void task_exit(task_exit_status_t exit_status)
{
	gpOS_task_exit(exit_status);
}

task_onexit_function_t task_set_onexit_function(task_onexit_function_t function)
{
	return gpOS_task_set_onexit_function(function);
}

tInt task_wait( task_t** task_list, tInt ntasks, const os20_clock_t* timeout)
{
  const os20_clock_t *os_timeout = timeout;

  if( os_timeout == &_ST_TimeoutImmediate)
  {
    os_timeout = gpOS_TIMEOUT_IMMEDIATE;
  }
  else if( os_timeout == &_ST_TimeoutInfinity)
  {
    os_timeout = gpOS_TIMEOUT_INFINITY;
  }

	return gpOS_task_wait(task_list, ntasks, os_timeout);
}

void task_lock( void)
{
	gpOS_task_lock();
}

void task_unlock( void)
{
	gpOS_task_unlock();
}

