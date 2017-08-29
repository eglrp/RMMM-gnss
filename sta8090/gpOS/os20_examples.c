/*****************************************************************************
   FILE:          os20_examples.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   Module to run and test STA8090 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Aldo Occhipinti
           on : 2016.04.18
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"
#include "clibs.h"

// LLD for STA8090
#include "lld_gpio.h"

// OS related
#include "gpOS.h"
#include "gnss_debug.h"
#include "platform.h"
#include "frontend.h"

#include "os20_examples.h"
#include "test_io.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define OS20_EXAMPLES_VERSION_STRING  "2.1.0"

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

// Version string
const tChar *os20_examples_ver  = MCR_VERSION_STRING( "OS20_EXAMPLES", OS20_EXAMPLES_VERSION_STRING);

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

extern tVoid kernel_test_start                ( gpOS_partition_t *);
extern tVoid memory_and_partitions_test_start ( gpOS_partition_t *);
extern tVoid tasks_test_start                 ( gpOS_partition_t *);
extern tVoid semaphores_test_start            ( gpOS_partition_t *);
extern tVoid message_handling_test_start      ( gpOS_partition_t *);
extern tVoid interrupts_test_start            ( gpOS_partition_t *);
extern tVoid exceptions_test_start            ( gpOS_partition_t *);
extern tVoid time_clocks_test_start           ( gpOS_partition_t *);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 *
 ***********************************************/
static tVoid os20_examples_start( gpOS_partition_t *fast_part)
{
  char input_buf[2];

  while(1)
  {
    GPS_DEBUG_MSG(( "Select an OS20+ Example to Run:\r\n"));
    GPS_DEBUG_MSG(( "1. Kernel\r\n"));
    GPS_DEBUG_MSG(( "2. Memory & Partitions\r\n"));
    GPS_DEBUG_MSG(( "3. Tasks\r\n"));
    GPS_DEBUG_MSG(( "4. Semaphores\r\n"));
    GPS_DEBUG_MSG(( "5. Mutexes\r\n"));
    GPS_DEBUG_MSG(( "6. Message Handling\r\n"));
    GPS_DEBUG_MSG(( "7. Real Time Clocks\r\n"));
    GPS_DEBUG_MSG(( "8. Interrupts\r\n"));
    GPS_DEBUG_MSG(( "9. Exceptions\r\n"));

    GPS_DEBUG_MSG(("\r\n"));

    do
    {
      GPS_DEBUG_MSG(("Select: "));
    }
    while( test_io_read( input_buf, 1) == 0);

    switch( input_buf[0])
    {
      case '1':
         kernel_test_start( fast_part);
      break;

      case '2':
        memory_and_partitions_test_start( fast_part);
      break;

      case '3':
        tasks_test_start( fast_part);
      break;

      case '4':
        semaphores_test_start( fast_part);
      break;

      case '5':
        GPS_DEBUG_MSG(("Sorry No Examples on Mutexes\r\n"));
      break;

      case '6':
        message_handling_test_start( fast_part);
      break;

      case '7':
        time_clocks_test_start( fast_part);
      break;

      case '8':
        interrupts_test_start( fast_part);
      break;

      case '9':
        exceptions_test_start( fast_part);
      break;


      default:
        break;
    }
    GPS_DEBUG_MSG(("================================\r\n"));
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param fast_part gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t os20_examples_init( gpOS_partition_t *fast_part)
{
  GPS_DEBUG_MSG(( "======================================================================\r\n"));
  GPS_DEBUG_MSG(( "Test suite version: %s\r\n", os20_examples_version));
  GPS_DEBUG_MSG(( "LLD version: %s\r\n%s\r\n", lld_ver));
  GPS_DEBUG_MSG(( "OS version: %s\r\n", gpOS_version()));
  GPS_DEBUG_MSG(( "SVC version: %s\r\n", svc_version()));
  GPS_DEBUG_MSG(( "======================================================================\r\n"));

  /**< Start Test Main menu */
  os20_examples_start( NULL);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return const tChar*
 *
 ***********************************************/
const tChar *os20_examples_version( tVoid)
{
  return os20_examples_ver;
}
