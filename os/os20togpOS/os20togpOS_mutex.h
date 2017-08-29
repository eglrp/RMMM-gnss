/**
 * @file    os20togpOS_mutex.h
 * @brief   OS20 mutexes functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_MUTEX_H
#define OS20TOGPOS_MUTEX_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_mutexi.h"

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
#define mutex_create_fifo()                 gpOS_mutex_create( MUTEX_FIFO)
#define mutex_create_fifo_p(custom_part)    gpOS_mutex_create_p( MUTEX_FIFO, custom_part)

#define mutex_delete(mutex)                 gpOS_mutex_delete(mutex)

#define mutex_lock(mutex)                   gpOS_mutex_lock(mutex)
#define mutex_trylock(mutex)                gpOS_mutex_trylock(mutex)
#define mutex_release(mutex)                gpOS_mutex_release(mutex)
#define mutex_locked(mutex)                 gpOS_mutex_locked(mutex)

#endif /* OS20TOGPOS_MUTEX_H */
