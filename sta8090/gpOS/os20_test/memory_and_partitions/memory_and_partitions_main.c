/*****************************************************************************
   FILE:          memory_and_partitions_main.c
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
#include "test_io.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

extern gpOS_error_t memory_and_partitions_test1_run( gpOS_partition_t *);
extern gpOS_error_t memory_and_partitions_test2_run( gpOS_partition_t *);

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
tVoid memory_and_partitions_test_start( gpOS_partition_t *fast_part)
{
  char in_char;
  tBool exit = FALSE;

  while(exit == FALSE)
  {
    GPS_DEBUG_MSG(( "Select an OS20+ Example on Memory & Partitions:\r\n"));
    GPS_DEBUG_MSG(( "1. Creating a Partition and Allocating Memory\r\n"));
    GPS_DEBUG_MSG(( "2. Dumping Memory Info From a Task\r\n"));
    GPS_DEBUG_MSG(( "e. Exit\r\n"));

    test_io_read( &in_char, 1);

    switch( in_char)
    {
      case '1':

        memory_and_partitions_test1_run( NULL);

      break;

      case '2':

        memory_and_partitions_test2_run( NULL);

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
