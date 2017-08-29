/**
 * @file    OStoFR_semaphore.h
 * @brief   OS semaphores definitions and macros.
 *
 * @addtogroup gpOS_WRAPPER
 */

#ifndef OSTOFR_SEMAPHORE_H
#define OSTOFR_SEMAPHORE_H

// Fallback for generic OS
#ifndef gpOS_SEMAPHORE_H
#define gpOS_SEMAPHORE_H
#endif

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define gpOS_semaphore_create(param, start_value) \
            fis_semaphore_create_p(__FILE__, __LINE__, NULL, start_value)
#define gpOS_semaphore_create_p(param, partition, start_value) \
            fis_semaphore_create_p(__FILE__, __LINE__, partition, start_value)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef enum gpOS_sem_type_e
{
  SEM_NULL,
  SEM_FIFO,
  SEM_PRIO
} gpOS_sem_type_t;

typedef struct gpOS_semaphore_s   gpOS_semaphore_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_semaphore_t * fis_semaphore_create_p        ( char *file, int line, gpOS_partition_t *custom_part, const int start_value);
extern gpOS_error_t       gpOS_semaphore_delete         ( gpOS_semaphore_t *sem);

extern void               gpOS_semaphore_wait           ( gpOS_semaphore_t *semaphore);
extern gpOS_error_t       gpOS_semaphore_wait_timeout   ( gpOS_semaphore_t *sem, const gpOS_clock_t *timeout);
extern void               gpOS_semaphore_signal         ( gpOS_semaphore_t *sem);
extern int                gpOS_semaphore_value          ( gpOS_semaphore_t *sem);


#endif /* OSTOFR_SEMAPHORE_H */
