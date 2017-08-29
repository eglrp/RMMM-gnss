/*****************************************************************************
   FILE:          tasks_main.c
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
#include "gnss_debug.h"
#include "test_io.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

extern gpOS_error_t tasks_test1_run( gpOS_partition_t *part);
extern gpOS_error_t tasks_test2_run( gpOS_partition_t *part);
extern gpOS_error_t tasks_test3_run( gpOS_partition_t *part);
extern gpOS_error_t tasks_test4_run( gpOS_partition_t *part);

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
tVoid tasks_test_start( gpOS_partition_t *fast_part)
{
  char in_char;
  tBool exit = FALSE;

  while(exit == FALSE)
  {
    GPS_DEBUG_MSG(( "\r\nSelect an OS20+ Example on Tasks:\r\n"));
    GPS_DEBUG_MSG(( "1. Creating a Simple Task\r\n"));
    GPS_DEBUG_MSG(( "2. Managing Task's Priority\r\n"));
    GPS_DEBUG_MSG(( "3. Different Tasks can share the same section of code\r\n"));
    GPS_DEBUG_MSG(( "4. Passing parameters to a Task\r\n"));
    GPS_DEBUG_MSG(( "e. Exit\r\n"));

    test_io_read( &in_char, 1);

    switch( in_char)
    {
      case '1':

        tasks_test1_run( NULL);

      break;

      case '2':

        tasks_test2_run( NULL);

      break;

      case '3':

        tasks_test3_run( NULL);

      break;

      case '4':

        tasks_test4_run( NULL);

      break;

      case 'e':
      case 'E':

        exit = TRUE;

      break;

      default:

      GPS_DEBUG_MSG(("Selection Not Valid\r\n"));

      break;
    }

    gpOS_task_delay( 10 * gpOS_timer_ticks_per_sec());
  }
}
