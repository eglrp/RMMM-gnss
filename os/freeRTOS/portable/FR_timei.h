/**
 * @file    freeRTOS_timei.h
 * @brief   FreeRTOS time internal handling definitions and macros.
 *
 */

#ifndef FREERTOS_TIMEI_H
#define FREERTOS_TIMEI_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "semphr.h"
#include "gpOS_time.h"
#include "queue.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef enum
{
  TIMER_SVC_NONE,
  TIMER_SVC_FREERTOS_TICK,
  TIMER_SVC_TASK_DELAY,
  TIMER_SVC_QUEUE_DELAY
} timer_svc_id_e;

typedef struct svc_job_s       svc_job_t;

typedef struct svc_job_s
{
   uint32_t       timer_svc_type;
   void*          svc_handler;
   void*          message;
   gpOS_clock_t     timer_timeout;
   svc_job_t*     timer_queue_next;
} svc_job_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

extern BaseType_t   xReschedule_fromISR_required;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void       taskHiResDelay            ( gpOS_clock_t delay);
extern void       timer_svc_create          ( svc_job_t *this_job, timer_svc_id_e timer_svc_type, void *timer_svc_handle, gpOS_clock_t exit_time, void *message);
extern BaseType_t timer_svc_insert          ( svc_job_t *this_job);
extern BaseType_t timer_svc_remove          ( svc_job_t *this_job);
extern BaseType_t timer_generate_TickCount  ( TickType_t xTickCount, boolean_t Comp );
extern void       timer_remove_TickCount    ( void );
extern void       timer_svc_timeout_jobs_update (const tS32 frequency_ratio, const tU32 ratio_scale, const tU32 current_time, const boolean_t update_first);

#endif /* FREERTOS_TIMEI_H */
