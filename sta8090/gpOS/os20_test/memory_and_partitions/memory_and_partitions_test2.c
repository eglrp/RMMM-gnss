/*****************************************************************************
   FILE:          memory_and_partitions_test2.c
   PROJECT:       STA8090 GNSS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Memory and Partitions
                  (OS20+ operative system specification, Chapter 2
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
#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define TEST2_WS_SIZE 2048

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
typedef struct test_manager_s
{
  gpOS_task_t *          task;
  gpOS_message_queue_t * msg_queue;
  gpOS_clock_t      measure_time;
} test_manager_t;

static test_manager_t test2_manager;
int memory_test2_task_priority = 14;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param dummy task_param_t
 * \return task_exit_status_t
 ***********************************************/
gpOS_ISR gpOS_task_exit_status_t memory_test2_process( gpOS_task_param_t dummy)
{
  gpOS_task_t*  task;
  size_t   size;
  size_t   used;

  GPS_DEBUG_MSG(( "\r\n[Memory] [2] [TASK] [STACK SIZE] [STACK USED] [HEAP SIZE] [HEAP FREE]\r\n\r\n"));

  task = gpOS_task_get_head();

  while( task != NULL)
    {
      used = gpOS_task_get_stack_used( task);
      size = gpOS_task_get_stack_size( task);

      GPS_DEBUG_MSG(("[Memory] [2] [%s] [%d] [%d] [%d] [%d]\r\n",
                       gpOS_task_get_name( task),
                       size,
                       used,
                       gpOS_memory_getheapsize(),
                       gpOS_memory_getheapfree()));

      task = gpOS_task_get_next( task);
      gpOS_task_delay( gpOS_timer_ticks_per_sec());

    }

  GPS_DEBUG_MSG(("\r\n"));

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return os20_error_t
 ***********************************************/
gpOS_error_t memory_and_partitions_test2_run( gpOS_partition_t *part)
{

   test2_manager.task = gpOS_task_create ( memory_test2_process, NULL, TEST2_WS_SIZE, memory_test2_task_priority, "Memory T2 P1", gpOS_TASK_FLAGS_ACTIVE);

   if( test2_manager.task == NULL)
   {
     gpOS_task_delete( test2_manager.task);
     GPS_DEBUG_MSG(( " task_create() failed\n"));

     return gpOS_FAILURE;
   }

   return gpOS_SUCCESS;
}
/*}}}  */
