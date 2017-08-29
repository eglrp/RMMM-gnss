/**
 * @file    os20togpOS_mutex.c
 * @brief   OS20 mutex to genericOS message wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_mutexi.h"
#include "os20togpOS_memoryi.h"
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
mutex_t *mutex_create_fifo( void)
{
	return gpOS_mutex_create( MUTEX_FIFO);
}

mutex_t *mutex_create_fifo_p( partition_t *partition_ptr)
{
	return gpOS_mutex_create_p( MUTEX_FIFO, partition_ptr);
}

os20_error_t mutex_delete( mutex_t *mutex)
{
	return gpOS_mutex_delete(mutex);
}

void mutex_lock(mutex_t *mutex)
{
	gpOS_mutex_lock(mutex);
}

os20_error_t mutex_trylock(mutex_t *mutex)
{
	return gpOS_mutex_trylock(mutex);
}

os20_error_t mutex_release(mutex_t *mutex)
{
	return gpOS_mutex_release(mutex);
}

tUInt mutex_locked(mutex_t *mutex)
{
	return gpOS_mutex_locked(mutex);
}

