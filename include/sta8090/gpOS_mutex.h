/**
 * @file    mutex.h
 * @brief   OS mutexes definitions and macros.
 *
 * @addtogroup OS
 */

#ifndef gpOS_MUTEX_H
#define gpOS_MUTEX_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gpOS_memory.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef enum gpOS_mutex_type_e
{
  MUTEX_FIFO
} gpOS_mutex_type_t;

typedef struct gpOS_mutex_s  gpOS_mutex_t;        /**< OS20 mutex type */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_mutex_t *   gpOS_mutex_create       ( gpOS_mutex_type_t type);
extern gpOS_mutex_t *   gpOS_mutex_create_p     ( gpOS_mutex_type_t type, gpOS_partition_t *custom_part);
extern gpOS_error_t     gpOS_mutex_delete       ( gpOS_mutex_t *mutex);

extern void             gpOS_mutex_lock         ( gpOS_mutex_t *mutex);
extern gpOS_error_t     gpOS_mutex_trylock      ( gpOS_mutex_t *mutex);
extern gpOS_error_t     gpOS_mutex_release      ( gpOS_mutex_t *mutex);
extern tUInt            gpOS_mutex_locked       ( gpOS_mutex_t *mutex);

#endif /* gpOS_MUTEX_H */
