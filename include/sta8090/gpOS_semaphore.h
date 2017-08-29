/**
 * @file    semaphore.h
 * @brief   OS20 semaphores definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef gpOS_SEMAPHORE_H
#define gpOS_SEMAPHORE_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS_task.h"
#include "gpOS_time.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct gpOS_semaphore_s gpOS_semaphore_t;

typedef enum gpOS_sem_type_e
{
  SEM_FIFO,
  SEM_PRIO
} gpOS_sem_type_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_semaphore_t * gpOS_semaphore_create         ( const gpOS_sem_type_t type, const tInt start_value);
extern gpOS_semaphore_t * gpOS_semaphore_create_p       ( const gpOS_sem_type_t type, gpOS_partition_t *custom_part, const tInt start_value);
extern gpOS_error_t       gpOS_semaphore_delete         ( gpOS_semaphore_t *semaphore);

extern void           gpOS_semaphore_wait         ( gpOS_semaphore_t *semaphore);
extern gpOS_error_t   gpOS_semaphore_wait_timeout ( gpOS_semaphore_t *semaphore, const gpOS_clock_t *timeout);
extern void           gpOS_semaphore_signal       ( gpOS_semaphore_t *semaphore);
extern tInt           gpOS_semaphore_value        ( gpOS_semaphore_t *semaphore);

#endif /* gpOS_SEMAPHORE_H */
