/**
 * @file    OStoFR_utils.c
 * @brief   Wrapper OS - FreeRTOS utilities function implementation.
 *
 * @addtogroup gpOS_WRAPPER
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "gpOStoFR_taski.h"
#include "gpOStoFR_utils.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
gpOS_clock_t  _OS_TimeoutImmediate;
gpOS_clock_t  _OS_TimeoutInfinity;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/************************************************************************
Timeout convert from OS to FreeRTOS
************************************************************************/
TickType_t OStoFR_timeout(const gpOS_clock_t *os_absolute_time_p)
{
  TickType_t fRtos_relative_time;
  int32_t time_delta;

  if(os_absolute_time_p == gpOS_TIMEOUT_INFINITY)
  {
    fRtos_relative_time = portMAX_DELAY;
  }
  else if(os_absolute_time_p == gpOS_TIMEOUT_IMMEDIATE)
  {
    fRtos_relative_time = 0;
  }
  else
  {
    gpOS_clock_t now = gpOS_time_now();

    time_delta = (int32_t)gpOS_time_minus(*os_absolute_time_p, now);

    if(time_delta >= 0)
    {
      fRtos_relative_time = OStoFR_ticks(time_delta);
    }
    else
    {
      fRtos_relative_time = 0;
    }
  }

  return fRtos_relative_time;
}


/************************************************************************
Tick convert from OS to FreeRTOS
************************************************************************/
TickType_t OStoFR_ticks(gpOS_clock_t os_ticks)
{
  return (TickType_t)((((tFloat)os_ticks)/O2F_W_OS_TICKS_PER_SEC) * configTICK_RATE_HZ);
}

/************************************************************************
Tick convert from FreeRTOS to OS
************************************************************************/
gpOS_clock_t FRtoOS_ticks(TickType_t frtos_ticks)
{
  return (gpOS_clock_t)((((tFloat)frtos_ticks)/configTICK_RATE_HZ) * O2F_W_OS_TICKS_PER_SEC);
}

