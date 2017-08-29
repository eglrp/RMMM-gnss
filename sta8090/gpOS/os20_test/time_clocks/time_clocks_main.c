/*****************************************************************************
   FILE:          time_clocks_main.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Time Clocks
                  (OS20+ operative system specification, Chapter 11)
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2013 STMicroelectronics,
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

extern gpOS_error_t time_clocks_test1_run( gpOS_partition_t *part);

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
tVoid time_clocks_test_start( gpOS_partition_t *fast_part)
{
  char in_char;
  tBool exit = FALSE;

  while( exit == FALSE)
  {
    GPS_DEBUG_MSG(( "Select an OS20+ Example on Tasks:\r\n"));
    GPS_DEBUG_MSG(( "1. Computing and Applying a Suspend Time\r\n"));
    GPS_DEBUG_MSG(( "e. Exit\r\n"));

    test_io_read( &in_char, 1);

    switch( in_char)
    {
      case '1':

        time_clocks_test1_run( NULL);

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

