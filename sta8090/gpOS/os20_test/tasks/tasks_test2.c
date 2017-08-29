/*****************************************************************************
   FILE:          tasks_test2.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Test One: task creation example
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
#define TASKS_TEST2_FIRST_PROCESS_WS_SIZE  1024
#define TASKS_TEST2_SECOND_PROCESS_WS_SIZE 1024
#define TICKS_DELAYED 2000
#define NUM_ITERATIONS 20

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef struct tasks_test2_manager_s
{
    gpOS_partition_t *     part;

} tasks_test2_manager_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
static gpOS_task_t *curr_task;
static tU32 curr_pri;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
static tasks_test2_manager_t *tasks_test2_manager = NULL;
int tasks_test2_first_process_priority  = 12;
int tasks_test2_second_process_priority = 4;
static gpOS_task_t *tasks_test2_task1;
static gpOS_task_t *tasks_test2_task2;

static gpOS_task_exit_status_t tasks_test2_first_process  ( void *);
static gpOS_task_exit_status_t tasks_test2_second_process ( void *);

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
static gpOS_task_exit_status_t tasks_test2_first_process(  void *p)
{
  int i;

  for ( i= 1; i<= 5000 * NUM_ITERATIONS; i++);

  curr_task = gpOS_kernel_get_active_task();
  curr_pri  = gpOS_task_get_priority( curr_task );

  GPS_DEBUG_MSG(( "\r\n[Tasks] [2] [1] [%d] [%d]\r\n", curr_pri, gpOS_time_now()));

  gpOS_task_set_priority( curr_task, 4 );

  //gpOS_task_delay( 20000000);
  gpOS_task_delay( gpOS_timer_ticks_per_sec());

  curr_task = gpOS_kernel_get_active_task();
  curr_pri  = gpOS_task_get_priority( curr_task );

  GPS_DEBUG_MSG(( "\r\n[Tasks] [2] [1] [%d] [%d]\r\n", curr_pri, gpOS_time_now()));

  return gpOS_SUCCESS;
}

static gpOS_task_exit_status_t tasks_test2_second_process(  void *p)
{
  int i;

  for ( i= 1; i<= 5000 * NUM_ITERATIONS; i++);

  curr_task = gpOS_kernel_get_active_task();
  curr_pri  = gpOS_task_get_priority( curr_task );

  GPS_DEBUG_MSG(( "\r\n[Tasks] [2] [2] [%d] [%d]\r\n", curr_pri, gpOS_time_now()));

  gpOS_task_set_priority( curr_task, 12 );

  //gpOS_task_delay( 20000000);
  gpOS_task_delay( gpOS_timer_ticks_per_sec());

  curr_task = gpOS_kernel_get_active_task();
  curr_pri  = gpOS_task_get_priority( curr_task );

  GPS_DEBUG_MSG(( "\r\n[Tasks] [2] [2] [%d] [%d]\r\n", curr_pri, gpOS_time_now()));

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
gpOS_error_t tasks_test2_run( gpOS_partition_t *part)
{



  tasks_test2_task1 = gpOS_task_create( tasks_test2_first_process,  NULL, TASKS_TEST2_FIRST_PROCESS_WS_SIZE,  tasks_test2_first_process_priority,  "Task T2 P1", gpOS_TASK_FLAGS_ACTIVE);
  tasks_test2_task2 = gpOS_task_create( tasks_test2_second_process, NULL, TASKS_TEST2_SECOND_PROCESS_WS_SIZE, tasks_test2_second_process_priority, "Task T2 P2", gpOS_TASK_FLAGS_ACTIVE);

  if(( tasks_test2_task1 == NULL) ||
     ( tasks_test2_task2 == NULL))
    {

      gpOS_task_delete( tasks_test2_task1);
      gpOS_task_delete( tasks_test2_task2);

      return gpOS_FAILURE;
    }

  tasks_test2_manager->part    = part;

  GPS_DEBUG_MSG(( "\r\n[Tasks] [2] [Task_Num] [Current Priority] [Time]\r\n"));

  return gpOS_SUCCESS;
}
/*}}}  */
