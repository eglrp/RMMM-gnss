/*****************************************************************************
   FILE:          tasks_test3.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Test Three: different task can use the same process
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
#define TASKS_TEST3_PROCESS_SIZE  1024

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef struct tasks_test2_manager_s
{
    gpOS_partition_t *     part;

} tasks_test3_manager_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
static gpOS_task_t *curr_task;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
static tasks_test3_manager_t *tasks_test3_manager = NULL;
int tasks_test3_process_priority  = 5;
static gpOS_task_t *tasks_test3_task1;
static gpOS_task_t *tasks_test3_task2;
static gpOS_task_t *tasks_test3_task3;

static gpOS_task_exit_status_t tasks_test3_process  ( void *);

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
static gpOS_task_exit_status_t tasks_test3_process(  void *p)
{

  gpOS_task_delay( 1 * gpOS_timer_ticks_per_sec());

  curr_task = gpOS_kernel_get_active_task();

  GPS_DEBUG_MSG(( "\r\n[Tasks] [3] [%s] [%d]", gpOS_task_get_name(curr_task), gpOS_time_now()));

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
gpOS_error_t tasks_test3_run( gpOS_partition_t *part)
{

  tasks_test3_task1 = gpOS_task_create( tasks_test3_process, NULL, TASKS_TEST3_PROCESS_SIZE, tasks_test3_process_priority, "Task T3 P1", gpOS_TASK_FLAGS_ACTIVE);
  tasks_test3_task2 = gpOS_task_create( tasks_test3_process, NULL, TASKS_TEST3_PROCESS_SIZE, tasks_test3_process_priority, "Task T3 P2", gpOS_TASK_FLAGS_ACTIVE);
  tasks_test3_task3 = gpOS_task_create( tasks_test3_process, NULL, TASKS_TEST3_PROCESS_SIZE, tasks_test3_process_priority, "Task T3 P3", gpOS_TASK_FLAGS_ACTIVE);

  if(( tasks_test3_task1 == NULL) ||
     ( tasks_test3_task2 == NULL) ||
     ( tasks_test3_task3 == NULL))
    {

      gpOS_task_delete( tasks_test3_task1);
      gpOS_task_delete( tasks_test3_task2);

      return gpOS_FAILURE;
    }

  tasks_test3_manager->part    = part;

  GPS_DEBUG_MSG(( "\r\n[Tasks] [3] [Task Name] [Time]\r\n"));

  return gpOS_SUCCESS;
}
/*}}}  */
