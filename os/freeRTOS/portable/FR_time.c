/**
 * @file    freeRTOS_time.c
 * @brief   FreeRTOS time handling implementation.
 *
 */

/*****************************************************************************
   includes
*****************************************************************************/

/*{{{  include*/
#include "freeRTOS.h"
#include "FR_timei.h"
#include "gpOS_bsp.h"
#include "task.h"
/*}}}  */

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define TIMER_MIN_INSERTION_TIME    ((int32_t)(gpOS_timer_ticks_per_msec()/20))

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*{{{  typedefs*/
typedef enum
{
  gpOS_TIMER_UNINIT,
  gpOS_TIMER_SUSPENDED,
  gpOS_TIMER_RUNNING
} timer_status_t;
/*}}}  */

/*{{{  typedefs*/
typedef struct timer_handler_s
{
  timer_status_t  status;
  svc_job_t *    queue_head;
  svc_job_t *    queue_tail;
  unsigned    clock_speed;
  unsigned    LastTimeout_time;
} timer_handler_t;
/*}}}  */

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
/*{{{  decs*/
static timer_handler_t timer_handler = { gpOS_TIMER_UNINIT, NULL, NULL, 0xFFFFFFFF, 0 };
static svc_job_t generate_TickCount;
/*}}}  */

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Suspend the current task for the specified HighResolution period
 *
 * \param   delay  Duration of suspend
 *
 ***********************************************/

/***********************************************************/
/* When using taskHiResDelay(), vTaskResume() on same task */
/* must not be used during the delay                       */
/***********************************************************/

void taskHiResDelay( gpOS_clock_t delay)
{
  svc_job_t job;
  TaskHandle_t myHandle = xTaskGetCurrentTaskHandle();

  // Enter critical section to make sure timout will not occur before suspend().
  // Critical lock will be suspended until task delay completes
  taskENTER_CRITICAL();

  timer_svc_create(&job, TIMER_SVC_TASK_DELAY,
                   (void *)myHandle, delay, NULL);


  if(pdTRUE == timer_svc_insert(&job)) {
    vTaskSuspend(myHandle);
  }

  taskEXIT_CRITICAL();
}

/********************************************//**
 * \brief   Change clock period of timer module
 *
 * \param   new_period  New clock period
 *
 ***********************************************/
void gpOS_timer_set_clock( void *bsp_timer_cfg_ptr, const unsigned update_value)
{
  if( (timer_handler.status == gpOS_TIMER_SUSPENDED))
  {
    gpOS_bsp_timer_set_clock( bsp_timer_cfg_ptr, update_value);
    timer_handler.clock_speed = gpOS_bsp_timer_get_clock_speed();
  }
}

/********************************************//**
 * \brief   Used to request the number of ticks in 1 second
 *
 * \param   void
 * \return  The number of clock ticks in 1 second
 *
 * \see     ticks_per_msec, ticks_per_usec
 *
 ***********************************************/
gpOS_clock_t gpOS_timer_ticks_per_sec( void)
{
  return timer_handler.clock_speed;
}

/********************************************//**
 * \brief   Used to request the number of ticks in 1 millisecond
 *
 * \param   void
 * \return  The number of clock ticks in 1 millisecond
 *
 * \see     ticks_per_sec, ticks_per_usec
 *
 ***********************************************/
gpOS_clock_t gpOS_timer_ticks_per_msec( void)
{
  return timer_handler.clock_speed / (int32_t)1e3;
}

/********************************************//**
 * \brief   Used to request the number of ticks in 1 microsecond
 *
 * \param   void
 * \return  The number of clock ticks in 1 microsecond
 *
 * \see     ticks_per_sec, ticks_per_msec
 *
 ***********************************************/
gpOS_clock_t gpOS_timer_ticks_per_usec( void)
{
  return timer_handler.clock_speed / (int32_t)1e6;
}

/********************************************//**
 * \brief   Returns the number of clock ticks that have elapsed
 *
 * \param   void
 * \return  Number of clock ticks
 *
 ***********************************************/
gpOS_clock_t gpOS_time_now( void)
{
  return gpOS_bsp_timer_time_now();
}

/********************************************//**
 * \brief   Adds two time values together returning the result
 *
 * \param   time1   Time1 specifies the first time value
 * \param   time2   Time2 specifies the second time value
 * \return  The result of adding Time1 to Time2 will be returned
 *
 * \see     gpOS_time_minus, gpOS_time_after
 *
 ***********************************************/
gpOS_clock_t gpOS_time_plus( gpOS_clock_t time1, gpOS_clock_t time2)
{
  return time1 + time2;
}

/********************************************//**
 * \brief   Subtracts one time from another and returns the result
 *
 * \param   time1   specifies the time from which Time2 will be subtracted
 * \param   time2   specifies the time to be subtracted from Time1
 * \return  The result of the subtraction will be returned
 *
 * \see     gpOS_time_plus, gpOS_time_after
 *
 ***********************************************/
gpOS_clock_t gpOS_time_minus( gpOS_clock_t time1, gpOS_clock_t time2)
{
  return time1 - time2;
}


/********************************************//**
 * \brief   Compares two time values to see if one is after than the other
 *
 * \param   time1   specifies the time from which Time2 will be subtracted
 * \param   time2   specifies the time to be subtracted from Time1
 * \return  If time1 - time2 is less than 0, 0 will be returned, otherwise 1 is returned
 *
 * \see     gpOS_time_plus, gpOS_time_minus
 *
 ***********************************************/
int gpOS_time_after( gpOS_clock_t time1, gpOS_clock_t time2)
{
  return ((int32_t)time1 - (int32_t)time2) > 0;
}

/*! \cond internal_docs */

/********************************************//**
 * \brief   Handles timeout event
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void gpOS_timer_timeout_handler( void)
{
  svc_job_t*   front_job;
  svc_job_t*   second_job;
  int32_t      time_left = 0;
  BaseType_t   TickCount_detect = pdFALSE;

  /* enter critical section */
  portENTER_CRITICAL();

  /* Save the received timeout time */
  timer_handler.LastTimeout_time = gpOS_time_now();

  front_job = timer_handler.queue_head;

  /* If the timer has not really expired, enqueue again last job.
     This must be check for timers that must interrupt more than once
     before timeout expiring. */
  if( front_job != NULL)
  {
    time_left = front_job->timer_timeout - (int32_t)gpOS_time_now();

    if(time_left >= TIMER_MIN_INSERTION_TIME)
    {
      gpOS_bsp_timer_reset_timeout( front_job->timer_timeout, true);

      portEXIT_CRITICAL();
      return;
    }
  }

 /* else look for next job that must wait on timer  */
  while( front_job != NULL)
  {

    /* Execute action associated to recieved timeout */
    if (front_job->timer_svc_type == TIMER_SVC_FREERTOS_TICK)
    {

      extern boolean_t ulTickFlag;

      TickCount_detect = pdTRUE;
      xReschedule_fromISR_required |= xTaskIncrementTick();

      ulTickFlag = pdTRUE;
    }

    if (front_job->timer_svc_type == TIMER_SVC_QUEUE_DELAY)
    {
      xQueueSendFromISR(front_job->svc_handler, front_job->message, &xReschedule_fromISR_required);
    }

    if (front_job->timer_svc_type == TIMER_SVC_TASK_DELAY)
    {
      xReschedule_fromISR_required |= xTaskResumeFromISR(front_job->svc_handler);
    }

    /* Get the following job and check if timeout can be considered as expired too */
    second_job = front_job->timer_queue_next;

    // if a job is on timer queue of current job, use that
    if( second_job != NULL)
    {
      time_left                 = second_job->timer_timeout - (int32_t)gpOS_time_now();
      timer_handler.queue_head  = second_job;

      // if time left for next job is less than minimum, dequeue also next job from timer queue
      if (time_left <= TIMER_MIN_INSERTION_TIME)
      {
        front_job = second_job;
      }
      // else reset bsp timeout
      else
      {
        gpOS_bsp_timer_reset_timeout( second_job->timer_timeout, true);
        front_job = NULL;
      }
    }
    // else clean timer queue
    else
    {
      front_job = NULL;
      timer_handler.queue_head  = NULL;
      timer_handler.queue_tail  = NULL;
    }
  }

  if (TickCount_detect == pdTRUE)
  {
    timer_generate_TickCount(1, TRUE);
  }

  // exit critical section
  portEXIT_CRITICAL();
}

/********************************************//**
 * \brief   Handles timeslice tick event
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void gpOS_timer_tick_handler( void)
{
  xReschedule_fromISR_required = xTaskIncrementTick();
}

/********************************************//**
 * \brief   Handles periodic tick freeRTOS
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
BaseType_t timer_generate_TickCount( TickType_t xTickCount, boolean_t Comp)
{
  BaseType_t return_value;

  if (generate_TickCount.timer_svc_type == TIMER_SVC_NONE)
  {
//    generate_TickCount.timer_timeout      = (int32_t)gpOS_time_now();
    generate_TickCount.timer_svc_type     = TIMER_SVC_FREERTOS_TICK;
    generate_TickCount.svc_handler        = NULL;
    generate_TickCount.message            = NULL;
    generate_TickCount.timer_queue_next   = NULL;
  }

  if( Comp == pdFALSE )
  {
    generate_TickCount.timer_timeout = (int32_t)(((gpOS_time_now()/(portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec())) + xTickCount)*(portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec()));
  }
  else
  {
    generate_TickCount.timer_timeout = (int32_t)(((gpOS_time_plus(gpOS_time_now(), TIMER_MIN_INSERTION_TIME)/(portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec())) + xTickCount)*(portTICK_PERIOD_MS * gpOS_timer_ticks_per_msec()));

    if(gpOS_time_after(gpOS_time_plus(gpOS_time_now(), 4*TIMER_MIN_INSERTION_TIME), generate_TickCount.timer_timeout) )
    {
      generate_TickCount.timer_timeout = gpOS_time_plus(gpOS_time_now(), 8*TIMER_MIN_INSERTION_TIME);
    }

  }
//  generate_TickCount.timer_timeout = OS_time_plus( generate_TickCount.timer_timeout, (portTICK_PERIOD_MS * OS_timer_ticks_per_msec()) );
  return_value = timer_svc_insert(&generate_TickCount);
  if (return_value == pdFALSE)
  {
    generate_TickCount.message = (void *)gpOS_time_now();
  }

  return return_value;
}

/********************************************//**
 * \brief   Remove tick freeRTOS
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void timer_remove_TickCount( void)
{
  timer_svc_remove(&generate_TickCount);
}

/********************************************//**
 * \brief   Insert a new job in timer list
 *
 * \param   this_job   job to be added in the timer list
 * \return  FAILURE if timestamp is too near
 *
 ***********************************************/
void timer_svc_create( svc_job_t *this_job, timer_svc_id_e timer_svc_type, void *timer_svc_handle, gpOS_clock_t exit_time, void *message)
{
  this_job->timer_timeout      = exit_time;
  this_job->timer_svc_type     = timer_svc_type;
  this_job->svc_handler        = timer_svc_handle;
  this_job->message            = message;
  this_job->timer_queue_next   = NULL;
}

/********************************************//**
 * \brief   Insert a new job in timer list
 *
 * \param   this_job   job to be added in the timer list
 * \return  FAILURE if timestamp is too near
 *
 ***********************************************/
BaseType_t timer_svc_insert( svc_job_t *this_job)
{
  svc_job_t*    last_job;
  svc_job_t*    next_job;
  int32_t       time_left;

  // enter critical section
  portENTER_CRITICAL();

  time_left = this_job->timer_timeout - (int32_t)gpOS_time_now();

  // if time left for timeout is less than minimum, do not enqueue job on timer
  // queue and exit
  if (time_left <= TIMER_MIN_INSERTION_TIME)
  {
    portEXIT_CRITICAL();
    return (pdFALSE);
  }

  last_job = NULL;
  next_job = timer_handler.queue_head;

  // search where this job must be inserted in timer queue
  while( next_job != NULL)
  {
    // if this job exit time is smaller than current job looked in the queue,
    // exit loop as this job must be inserted between last_job e next_job
    if( ((int32_t)(this_job->timer_timeout - next_job->timer_timeout)) < 0)
    {
      break;
    }
    last_job = next_job;
    next_job = next_job->timer_queue_next;
  }

  // enqueue this job on timer queue
  this_job->timer_queue_next   = next_job;

  // if this job must be the first to be dequeued, update head of timer queue
  // and force a new timeout reset on bsp
  if(last_job == NULL)
  {
    timer_handler.queue_head = this_job;
    gpOS_bsp_timer_reset_timeout( this_job->timer_timeout, true);
  }
  // else enqueue this job on last job timer queue
  else
  {
    last_job->timer_queue_next = this_job;
  }

  // if no more jobs are on this job queue, update tail
  if( next_job == NULL)
  {
    timer_handler.queue_tail = this_job;
  }

  // exit critical section
  portEXIT_CRITICAL();

  return pdTRUE;
}

/********************************************//**
 * \brief   Remove a job from timer list
 *
 * \param   this_job   job to remove
 * \return  void
 *
 ***********************************************/
BaseType_t timer_svc_remove( svc_job_t *this_job)
{
  svc_job_t*    last_job;
  svc_job_t*    next_job;
  BaseType_t    return_status = pdTRUE;

  // enter critical section
  portENTER_CRITICAL();

  last_job = NULL;
  next_job = timer_handler.queue_head;

  while ((next_job != this_job) && (next_job != NULL))
  {
    last_job = next_job;
    next_job = next_job->timer_queue_next;
  }

  if( next_job == this_job)
  {
    next_job = this_job->timer_queue_next;

    if(last_job == NULL)
    {
      timer_handler.queue_head = next_job;

      if (next_job != NULL)
      {
        gpOS_bsp_timer_reset_timeout( next_job->timer_timeout, true);
      }
    }
    else
    {
      last_job->timer_queue_next = next_job;
    }
    return_status = pdTRUE;
  }
  else
  {
    return_status = pdFALSE;
  }

  this_job->timer_timeout    = 0;
  this_job->timer_queue_next = NULL;

  // exit critical section
  portEXIT_CRITICAL();

  return (return_status);
}

/********************************************//**
 * \brief   Initialize timer module
 *
 * \param   void *bsp_timer_cfg_ptr
 * \return  void
 *
 ***********************************************/
void gpOS_timer_init( void *bsp_timer_cfg_ptr)
{
  gpOS_bsp_timer_init( bsp_timer_cfg_ptr);

  timer_handler.status      = gpOS_TIMER_SUSPENDED;

  timer_handler.clock_speed = gpOS_bsp_timer_get_clock_speed();
}

/********************************************//**
 * \brief   Restart timers
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void gpOS_timer_start( void)
{
  portENTER_CRITICAL(); //OS_interrupt_lock();

  if( timer_handler.status == gpOS_TIMER_SUSPENDED)
  {
    gpOS_bsp_timer_start();

    timer_handler.status = gpOS_TIMER_RUNNING;
  }

  portEXIT_CRITICAL(); //OS_interrupt_unlock();
}

/********************************************//**
 * \brief   Suspend timers
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void gpOS_timer_suspend( void)
{
  portENTER_CRITICAL(); //OS_interrupt_lock();

  if( timer_handler.status == gpOS_TIMER_RUNNING)
  {
    gpOS_bsp_timer_stop();

    timer_handler.status = gpOS_TIMER_SUSPENDED;
  }

  portEXIT_CRITICAL(); //OS_interrupt_unlock();
}

/********************************************//**
 * \brief   Reset task timeout
 *
 * \param   void
 * \return  void
 *
 ***********************************************/

void gpOS_timer_reset_timeout( void)
{
  if(( timer_handler.queue_head != NULL)&&(timer_handler.status == gpOS_TIMER_SUSPENDED))
  {
    gpOS_bsp_timer_reset_timeout( timer_handler.queue_head->timer_timeout, false);
  }
}


/********************************************//**
 * \brief   Update job timeouts
 *
 * \param   frequency: ratio ratio to apply to current timeout value
 * \param   ratio_scale: indicates the value (power of 2) by which the frequency_ratio
 *          has been multiplied before being put as parameter of this function
 * \param   current_time: time value before the call to this function
 * \param   update_first: indicate if only the next coming task must be updated or not
 * \return  none
 *
 ***********************************************/
void gpOS_timer_timeout_task_update(const tUInt frequency_ratio,  const tUInt ratio_scale, const tUInt current_time, const tUInt update_first)
{
  xSVC_timer_task_update( frequency_ratio, ratio_scale, current_time, update_first);
}

/**********************************************************************************//**
 * \brief   SVC call to update timeout for each job
 *
 * \param   frequency: ratio ratio to apply to current timeout value
 * \param   ratio_scale: indicates the value (power of 2) by which the frequency_ratio
 *          has been multiplied before being put as parameter of this function
 * \param   current_time: time value before the call to this function
 * \param   update_first: indicate if only the next coming task must be updated or not
 * \return  none
 *
 **************************************************************************************/
void timer_svc_timeout_jobs_update(const tS32 frequency_ratio, const tU32 ratio_scale, const tU32 current_time, const boolean_t update_first)
{
  svc_job_t *curr_job = timer_handler.queue_head;

  // This function must only be called if the frequency_ratio is different from 1.
  // It is advised to use only "powers of two" values for frequency_ratio.

  // Go through the list of tasks that delayed execution
  if (update_first)
  {
    // update timeout for next coming task only
    if( curr_job != NULL)
    {
      if ( frequency_ratio > 0 )
      {
        curr_job->timer_timeout = (tU32)((((tU64)curr_job->timer_timeout - (tU64)current_time) * (tU64)frequency_ratio) / (tU64)ratio_scale);
      }
      else
      {
        curr_job->timer_timeout = (tU32)((((tU64)curr_job->timer_timeout - (tU64)current_time) * (tU64)ratio_scale) / (tU64)frequency_ratio);
      }
    }
  }
  else
  {
    // skip the first task of the list
    if (curr_job != NULL)
    {
      curr_job = curr_job->timer_queue_next;

      if ( frequency_ratio > 0 )
      {
        while( curr_job != NULL)
        {
          curr_job->timer_timeout = (tU32)((((tU64)curr_job->timer_timeout - (tU64)current_time) * (tU64)frequency_ratio) / (tU64)ratio_scale);
          curr_job = curr_job->timer_queue_next;
        }
      }
      else
      {
        while( curr_job != NULL)
        {
          curr_job->timer_timeout = (tU32)((((tU64)curr_job->timer_timeout - (tU64)current_time) * (tU64)ratio_scale) / (tU64)frequency_ratio);
          curr_job = curr_job->timer_queue_next;
        }
      }
    }
  }
}

/*! \endcond */

/*}}}  */
