/*****************************************************************************
   FILE:          tasks_test4.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Test 4: passing parameter to a Task
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2016 STMicroelectronics,
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
#define TASKS_TEST4_PROCESS_SIZE  1024

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef struct tasks_test4_manager_s
{

  gpOS_partition_t *     part;

} tasks_test4_manager_t;

struct mytask_params {

  gpOS_clock_t    param_1;
  tChar          *param_2;

};

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/


/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
static tasks_test4_manager_t *tasks_test4_manager = NULL;
int tasks_test4_process_priority  = 5;
static gpOS_task_t *tasks_test4_task;

static gpOS_task_exit_status_t tasks_test4_process  ( void *);

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
static gpOS_task_exit_status_t tasks_test4_process(  gpOS_task_param_t voidp)
{

  struct mytask_params *param = (struct mytask_params *)voidp;

  gpOS_task_delay( 1 * gpOS_timer_ticks_per_sec());

  GPS_DEBUG_MSG(( "\r\n[Tasks] [4] [%d] [%s]\r\n", param->param_1, param->param_2));

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
gpOS_error_t tasks_test4_run( gpOS_partition_t *part)
{

  static struct mytask_params params;

  params.param_1 = gpOS_time_now();
  params.param_2 = "this is test4 for Task section";

  tasks_test4_task = gpOS_task_create( tasks_test4_process, (gpOS_task_param_t)&params, TASKS_TEST4_PROCESS_SIZE, tasks_test4_process_priority, "Task T4 P1", gpOS_TASK_FLAGS_ACTIVE);


  if(( tasks_test4_task == NULL ))
    {

      gpOS_task_delete( tasks_test4_task);

      return gpOS_FAILURE;
    }

  tasks_test4_manager->part    = part;

  GPS_DEBUG_MSG(( "\r\n[Tasks] [4] [param_1] [param_2]\r\n"));

  return gpOS_SUCCESS;
}
/*}}}  */
