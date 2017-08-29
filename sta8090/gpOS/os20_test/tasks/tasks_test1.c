/*****************************************************************************
   FILE:          tasks_test1.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Tasks
                  (OS20+ operative system specification, Chapter 3)
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2011 STMicroelectronics,
------------------------------------------------------------------------------
   Developers:
      AO:   Aldo Occhipinti
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
 ------------+------+------------------------------------------------------
             |      |
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "gpOS_types.h"
#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define TASKS_TEST1_WS_SIZE 2048
#define NUM_ITERATIONS 5

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef struct tasks_test1_manager_s
{
    gpOS_partition_t *     part;
    gpOS_task_t *          task;

} tasks_test1_manager_t;
static tasks_test1_manager_t *tasks_test1_manager = NULL;
int tasks_test1_task_priority = 9;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/


/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static gpOS_task_exit_status_t tasks_test1_process       ( void *);

/********************************************//**
 * \brief
 *
 * \param
 * \return
 ***********************************************/
static gpOS_task_exit_status_t tasks_test1_process( void *p)
{
  int i;
  gpOS_clock_t time[NUM_ITERATIONS], delay[NUM_ITERATIONS];

  GPS_DEBUG_MSG(( "\r\n[Tasks] [1] [ i] [time_i] [time_i-1] [delay_time]\r\n\r\n"));

  time[ 0] = gpOS_time_now();

  for ( i= 1; i<= NUM_ITERATIONS; i++)
  {
    time[i]  = gpOS_time_now();
    delay[i] = gpOS_time_minus(time[i], time[i-1]);
    GPS_DEBUG_MSG(( "[Tasks] [1] [%2d] [%d] [%d] [%d]\r\n", i, time[i], time[i-1], delay[i] ));
    gpOS_task_delay( gpOS_timer_ticks_per_sec());
  }

  GPS_DEBUG_MSG(( "\r\n\r\n"));

  return gpOS_SUCCESS;

}


/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
gpOS_error_t tasks_test1_run( gpOS_partition_t *part)
{

  tasks_test1_manager->task = gpOS_task_create_p( part, tasks_test1_process, NULL, TASKS_TEST1_WS_SIZE, tasks_test1_task_priority, "Task T1 P1", gpOS_TASK_FLAGS_ACTIVE);

  if( tasks_test1_manager->task == NULL)
    {

      gpOS_task_delete( tasks_test1_manager->task);
      return gpOS_FAILURE;
    }

  tasks_test1_manager->part = part;

  return gpOS_SUCCESS;
}
/*}}}  */
