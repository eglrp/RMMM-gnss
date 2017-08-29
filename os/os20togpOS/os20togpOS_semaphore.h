/**
 * @file    os20togpOS_semaphore.h
 * @brief   OS20 semaphores functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_SEMAPHORE_H
#define OS20TOGPOS_SEMAPHORE_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_semaphorei.h"

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
#define semaphore_create_fifo(start_value)                    gpOS_semaphore_create(SEM_FIFO, start_value)
#define semaphore_create_fifo_p(partition, start_value)       gpOS_semaphore_create_p(SEM_FIFO, partition, start_value)
#define semaphore_create_priority(start_value)                gpOS_semaphore_create(SEM_PRIO, start_value)
#define semaphore_create_priority_p(partition, start_value)   gpOS_semaphore_create_p(SEM_PRIO, partition, start_value)

#define semaphore_delete(sem)                                 gpOS_semaphore_delete(sem)

#define semaphore_wait(sem)                                   gpOS_semaphore_wait(sem)
#define semaphore_wait_timeout(sem, timeout)                  gpOS_semaphore_wait_timeout(sem, timeout)
#define semaphore_signal(sem)                                 gpOS_semaphore_signal(sem)
#define semaphore_value(sem)                                  gpOS_semaphore_value(sem)

#endif /* OS20TOGPOS_SEMAPHORE_H */
