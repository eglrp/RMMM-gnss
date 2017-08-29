/*****************************************************************************
   FILE:          kernel_test1.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Kernel
                  (OS20+ operative system specification, Chapter 1
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics,
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
#include "gpOS_task.h"
#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define T5_WS_SIZE      1024
#define TICKS_DELAYED 200000

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/


/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static gpOS_task_t *kernel_test1_task1;
static gpOS_task_t *kernel_test1_task2;

unsigned cnt1 = 0, cnt2 = 0;

static gpOS_task_exit_status_t kernel_test1_running1( void *dummy)
{
  gpOS_task_t *task_id = gpOS_task_get_id();
  boolean_t exit_flag = FALSE;

  while( exit_flag == FALSE)
  {
    unsigned delay;

    for( delay = 0; delay < 100; delay++);

    cnt1++;
  }

  // should never reach this
  return -1;
}

static gpOS_task_exit_status_t kernel_test1_running2( void *dummy)
{
  gpOS_task_t *task_id = gpOS_task_get_id();
  boolean_t exit_flag = FALSE;

  while( exit_flag == FALSE)
  {
    unsigned delay;

    for( delay = 0; delay < 100; delay++);

    cnt2++;
  }

  // should never reach this
  return -1;
}

gpOS_error_t kernel_test1_run( void)
{
  unsigned cnt = 0;

  gpOS_task_delay( 100 * TICKS_DELAYED);
  GPS_DEBUG_MSG(( "\r\n[Kernel] [1] [Timeslice ON/OFF] [Counter Task1] [Counter Task2]\r\n\r\n"));
  gpOS_task_delay( 100 * TICKS_DELAYED);

  if(( kernel_test1_task1 == NULL) ||
     ( kernel_test1_task2 == NULL))
  {
    kernel_test1_task1 = gpOS_task_create( kernel_test1_running1, NULL, T5_WS_SIZE, 8, "Kernel T1 P1", gpOS_TASK_FLAGS_ACTIVE);
    kernel_test1_task2 = gpOS_task_create( kernel_test1_running2, NULL, T5_WS_SIZE, 8, "Kernel T1 P2", gpOS_TASK_FLAGS_ACTIVE);

    if( (kernel_test1_task1 == NULL) || (kernel_test1_task2 == NULL))
    {
      return gpOS_FAILURE;
    }
  }

  cnt = cnt1 = cnt2 = 0;

  gpOS_kernel_set_timeslice( gpOS_FALSE);

  while( cnt < 10)
  {
    gpOS_task_delay( gpOS_timer_ticks_per_sec());
    GPS_DEBUG_MSG(( "." ));
    cnt++;
  }

  GPS_DEBUG_MSG(( "done\r\n" ));

  GPS_DEBUG_MSG(( "\r\n[Kernel] [1] [%s] [%d] [%d]\r\n", gpOS_kernel_get_timeslice() ? "ON" : "OFF", cnt1, cnt2));

  cnt = cnt1 = cnt2 = 0;

  gpOS_kernel_set_timeslice( gpOS_TRUE);

  while( cnt < 10)
  {
    gpOS_task_delay( gpOS_timer_ticks_per_sec());
    GPS_DEBUG_MSG(( "." ));
    cnt++;
  }

  GPS_DEBUG_MSG(( "done\r\n" ));

  GPS_DEBUG_MSG(( "\r\n[Kernel] [1] [%s] [%d] [%d]\r\n", gpOS_kernel_get_timeslice() ? "ON" : "OFF", cnt1, cnt2));

  /* go back to swicth OFF the timeslices for the other os20 examples */
  gpOS_kernel_set_timeslice( gpOS_FALSE);

  return gpOS_SUCCESS;
}
/*}}}  */
