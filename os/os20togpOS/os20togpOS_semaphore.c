/**
 * @file    os20togpOS_semaphore.c
 * @brief   OS20 semaphore to genericOS message wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_semaphorei.h"
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
semaphore_t *semaphore_create_fifo( tInt count)
{
	return gpOS_semaphore_create(SEM_FIFO, count);
}

semaphore_t *semaphore_create_fifo_p( partition_t *partition_ptr, const tInt count)
{
	return gpOS_semaphore_create_p(SEM_FIFO, partition_ptr, count);
}

semaphore_t *semaphore_create_priority( const tInt count)
{
	return gpOS_semaphore_create(SEM_PRIO, count);
}

semaphore_t *semaphore_create_priority_p( partition_t *partition_ptr, const tInt count)
{
	return gpOS_semaphore_create_p(SEM_PRIO, partition_ptr, count);
}

os20_error_t semaphore_delete( semaphore_t* semaphore)
{
	return gpOS_semaphore_delete(semaphore);
}

void semaphore_wait( semaphore_t* semaphore)
{
	gpOS_semaphore_wait(semaphore);
}

os20_error_t semaphore_wait_timeout( semaphore_t* semaphore, const os20_clock_t* timeout)
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

	return gpOS_semaphore_wait_timeout(semaphore, os_timeout);
}

void semaphore_signal( semaphore_t* semaphore)
{
	gpOS_semaphore_signal(semaphore);
}

tInt semaphore_value( semaphore_t *semaphore)
{
	return gpOS_semaphore_value(semaphore);
}

