/**
 * @file    OStoFR_mutex.h
 * @brief   OS mutexes definitions and macros.
 *
 * @addtogroup gpOS_WRAPPER
 */

#ifndef OSTOFR_MUTEX_H
#define OSTOFR_MUTEX_H

// Fallback for generic OS
#ifndef gpOS_MUTEX_H
#define gpOS_MUTEX_H
#endif

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gpOS_memory.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define gpOS_mutex_create( param) fis_mutex_create_p(__FILE__, __LINE__, NULL)
#define gpOS_mutex_create_p( param, custom_part) fis_mutex_create_p(__FILE__, __LINE__, custom_part)

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

extern gpOS_mutex_t * fis_mutex_create_p      ( char *file, int line, gpOS_partition_t *custom_part);
extern gpOS_error_t   gpOS_mutex_delete       ( gpOS_mutex_t *mutex);

extern void           gpOS_mutex_lock         ( gpOS_mutex_t *mutex);
extern gpOS_error_t   gpOS_mutex_trylock      ( gpOS_mutex_t *mutex);
extern gpOS_error_t   gpOS_mutex_release      ( gpOS_mutex_t *mutex);
extern tUInt          gpOS_mutex_locked       ( gpOS_mutex_t *mutex);

#endif /* OSTOFR_MUTEX_H */
