/*****************************************************************************
   FILE:          time_clocks_test1.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Tasks
                  (OS20+ operative system specification, Chapter 11)
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
#define TASKS_TEST1_WS_SIZE 1024
#define SUSPEND_TIME        20 * gpOS_timer_ticks_per_msec()
/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
typedef struct time_clocks_test1_manager_s
{
    gpOS_partition_t *     part;
    gpOS_task_t *          task;

} time_clocks_test1_manager_t;
static time_clocks_test1_manager_t *time_clocks_test1_manager = NULL;
int time_clocks_test1_task_priority = 9;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static gpOS_task_exit_status_t time_clocks_test1_process       ( void *);

/********************************************//**
 * \brief
 *
 * \param
 * \return
 ***********************************************/
static gpOS_task_exit_status_t time_clocks_test1_process( void *p)
{
  gpOS_clock_t write_time = SUSPEND_TIME;
  gpOS_clock_t time_when_suspend_ends;
  gpOS_clock_t t1, t2;
  int i;

  gpOS_task_delay( gpOS_timer_ticks_per_sec());
  GPS_DEBUG_MSG(( "\r\n[Time Clocks] [1] [i] [t1] [time_when_suspend_ends] [t2] [suspend_time] [delta_t2]\r\n\r\n"));
  gpOS_task_delay( gpOS_timer_ticks_per_sec());
  //DEBUG_MSG(( "[%d] [%d] [%d]\r\n", timer_ticks_per_usec(), gpOS_timer_ticks_per_msec(),timer_ticks_per_sec() ));

  for ( i=0; i<3; i++)
  {
    t1 = gpOS_time_now();
    time_when_suspend_ends = gpOS_time_plus( t1, write_time);

    while ( gpOS_time_after( time_when_suspend_ends, gpOS_time_now() ))
    {
    }

    t2 = gpOS_time_now();

    gpOS_task_delay( 1 * gpOS_timer_ticks_per_sec());

    GPS_DEBUG_MSG(( "[RT Clocks] [1] [%d] [%2d] [%d] [%d] [%d] [%d]\r\n", i, t1, time_when_suspend_ends, t2, ( time_when_suspend_ends - t1), ( t2 - time_when_suspend_ends) ));
  }

  GPS_DEBUG_MSG(( "\r\n"));

  return gpOS_SUCCESS;

}


/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
gpOS_error_t time_clocks_test1_run( gpOS_partition_t *part)
{

  time_clocks_test1_manager->task = gpOS_task_create_p( part, time_clocks_test1_process, NULL, TASKS_TEST1_WS_SIZE, time_clocks_test1_task_priority, "RT Clocks T1 P1", gpOS_TASK_FLAGS_ACTIVE);

  if( time_clocks_test1_manager->task == NULL)
    {

      gpOS_task_delete( time_clocks_test1_manager->task);
      return gpOS_FAILURE;
    }

  time_clocks_test1_manager->part    = part;

  return gpOS_SUCCESS;
}
/*}}}  */
