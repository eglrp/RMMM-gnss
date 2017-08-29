/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Implements a PWR.
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

#if defined(__STA8088__)
#include "lld_clk_ctrl_sta8088.h"
#elif defined(__STA8090__)
#include "lld_prcc_sta8090.h"
#endif
#include "lld_arm946.h"
#include "lld_rtc.h"

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_pwr.h"
#include "rtc.h"

#include "clibs.h"

#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_PWR_TRACE 0

#define SVC_PWR_MAX_PERIPHERALSLOCKS (sizeof(tU32) * 8U) /* 32 bits */
#define SVC_PWR_HANDLER_SIZE         (sizeof( svc_pwr_handler_t))

#define SVC_PWR_RTC_TICKS_PER_SECOND (0x8000U) /* RTC frac resolution */
#define SVC_PWR_RTC_CRITICAL_ALARM_LIMIT (2U) /* 2 seconds */

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief PWR handler
 ***********************************************/
typedef struct svc_pwr_handler_s
{
  gpOS_partition_t *          partition;        /**< Partition used for svc_pwr */
  tU32                        mem_used;         /**< Memory used for svc_pwr */
  gpOS_semaphore_t *          access_sem;       /**< PWR access semaphore */
  gpOS_semaphore_t *          access_peripherallocks_sem;   /**< Peripheral lock access semaphore */

  /* pheripherals handling */
  tU32                        peripherallocks;        /**< peripherals lock bit for svc_pwr - 32 peripherals available */
  tU8                         peripherallocks_cnt;    /**< peripherals locks counter */

  tUInt                       next_wakeup;      /**< Next time to enter in standby   */
  tUInt                       force_standby_duration; /**< Next forced standby duration   */

  gpOS_clock_t                startup_time;     /**< Last startup time */
  boolean_t                   wakeup_RTC;       /**< wakeup RTC */
  boolean_t                   wakeup_PIN;       /**< wakeup pin */

  gpOS_wakelockid_t           wakelockid;
} svc_pwr_handler_t;


typedef struct svc_pwr_StandByTime_s
{
  tU32  rtc_int_val;
  tU32  rtc_fra_val;
  gpOS_clock_t gpOS_time;

}svc_pwr_StandByTime_t;

#if (SVC_PWR_TRACE == 1)

#define MAX_ITEM_DBG 5U

typedef struct svc_pwr_StandByItemDbg
{
  gpOS_clock_t                startup_time;
  svc_pwr_StandByTime_t       EnterStandby;
  tU32                        WakeupAlarm;
  gpOS_clock_t                time_next_activity;
  gpOS_clock_t                time_next_activity_corrected;
  tUInt                       next_wakeup;
  tU32                        peripherallocks;


}svc_pwr_StandByItemDbg_t;

typedef struct svc_pwr_StandByDbg
{
  tUInt CurrentItem;
  svc_pwr_StandByItemDbg_t StandByItemDbg[MAX_ITEM_DBG];

}svc_pwr_StandByDbg_t;

#define SVC_PWR_DBG_ITEM_IDX svc_pwr_dbg.CurrentItem
#endif


/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
#if defined(__STA8090__)
#pragma arm section zidata = "SRAM_STDBY_DATA"
#endif
SRAM_STDBY_DATA static svc_pwr_StandByTime_t  svc_pwr_StandByTime;          /**< RTC reference stored at standby entry */
SRAM_STDBY_DATA static boolean_t              svc_pwr_standby_status;       /**< Enable the platform to go in standby */
SRAM_STDBY_DATA static boolean_t              svc_pwr_force_standby_status; /**< Force standby state as soon as possible */
#if (SVC_PWR_TRACE == 1)
SRAM_STDBY_DATA static svc_pwr_StandByDbg_t   svc_pwr_dbg;
#endif
#if defined(__STA8090__)
#pragma arm section zidata
#endif

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_pwr_handler_t *svc_pwr_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static boolean_t svc_pwr_IsStandbyWakeupState( boolean_t * wakeup_RTC,boolean_t * wakeup_PIN );

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initialize PWR svc
 *
 * \param partition Partition used for memory allocation
 * \param bus_speed Bus speed
 * \return gpOS_SUCCESS if all goes ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_pwr_init( gpOS_partition_t *partition )
{
  boolean_t wakeup_RTC,wakeup_PIN;
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_pwr_handler != NULL)
  {
    return gpOS_SUCCESS;    /*lint !e904 Return statement before end of function */
  }

  svc_pwr_handler = gpOS_memory_allocate_p( partition, SVC_PWR_HANDLER_SIZE);    /*lint !e9079 conversion from pointer to void to pointer to other type */

  if( svc_pwr_handler == NULL)
  {
    return gpOS_FAILURE;    /*lint !e904 Return statement before end of function */
  }

  _clibs_memset( svc_pwr_handler, 0, SVC_PWR_HANDLER_SIZE);
  svc_pwr_handler->partition   = partition;

  svc_pwr_handler->access_sem = gpOS_semaphore_create_p( SEM_FIFO, partition, 0);
  if( svc_pwr_handler->access_sem == NULL)
  {
    gpOS_memory_deallocate_p( partition, svc_pwr_handler);
    return gpOS_FAILURE;    /*lint !e904 Return statement before end of function */
  }

  svc_pwr_handler->access_peripherallocks_sem = gpOS_semaphore_create_p( SEM_FIFO, partition, 0);
  if( svc_pwr_handler->access_peripherallocks_sem == NULL)
  {
    gpOS_memory_deallocate_p( partition, svc_pwr_handler->access_sem);
    gpOS_memory_deallocate_p( partition, svc_pwr_handler);
    return gpOS_FAILURE;    /*lint !e904 Return statement before end of function */
  }

  /* Reserve a wakelock for SVC PWR */
  gpOS_wakelock_register(&svc_pwr_handler->wakelockid);

  svc_pwr_handler->mem_used    = mem_at_start - gpOS_memory_getheapfree_p( partition);

  /* if restart is not standby, erase clock info */
  if( (svc_pwr_StandbyWakeupState(&wakeup_RTC,&wakeup_PIN) == FALSE) || (svc_pwr_force_standby_status == TRUE ))
  {
    svc_pwr_StandByTime.rtc_fra_val = 0;
    svc_pwr_StandByTime.rtc_int_val = 0;
    svc_pwr_standby_status = FALSE;
    svc_pwr_force_standby_status = FALSE;
#if (SVC_PWR_TRACE == 1)
    _clibs_memset( &svc_pwr_dbg, 0, sizeof( svc_pwr_StandByDbg_t));
#endif
  }
  else
  {
    // Standby enable, load wakeup sourc
    svc_pwr_standby_status = svc_pwr_StandbyWakeupState(&svc_pwr_handler->wakeup_RTC,&svc_pwr_handler->wakeup_PIN);
  }

  gpOS_semaphore_signal( svc_pwr_handler->access_sem);
  gpOS_semaphore_signal( svc_pwr_handler->access_peripherallocks_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Update startup time
 *
 * \param StartupTime in ms
 * \return none
 *
 ***********************************************/
 void svc_pwr_StartupTime( tUInt StartupTime )
 {
    if( svc_pwr_handler != NULL )
    {
      svc_pwr_handler->startup_time = StartupTime;

#if (SVC_PWR_TRACE == 1)
      svc_pwr_dbg.StandByItemDbg[SVC_PWR_DBG_ITEM_IDX].startup_time = svc_pwr_handler->startup_time;
      if( svc_pwr_StartupMode() != SVC_PWR_STARTUP_POWER_ON)
      {
        tUInt LastIndex = SVC_PWR_DBG_ITEM_IDX - 1U;

        if( SVC_PWR_DBG_ITEM_IDX == 0U )
        {
          LastIndex = ( MAX_ITEM_DBG - 1U);
        }

        GPS_DEBUG_MSG(( "[svcpwr][stdby] dbg index %d\r\n", LastIndex ));
        GPS_DEBUG_MSG(( "[svcpwr][stdby] leave at rtc I: %d F:%d Alarm at %d\r\n", svc_pwr_dbg.StandByItemDbg[LastIndex].EnterStandby.rtc_int_val,
        svc_pwr_dbg.StandByItemDbg[LastIndex].EnterStandby.rtc_fra_val,
        svc_pwr_dbg.StandByItemDbg[LastIndex].WakeupAlarm ));
        GPS_DEBUG_MSG(( "[svcpwr][stdby] ost %d crt %d duration %d stt %d\r\n", svc_pwr_dbg.StandByItemDbg[LastIndex].time_next_activity,
        svc_pwr_dbg.StandByItemDbg[LastIndex].time_next_activity_corrected,
        svc_pwr_dbg.StandByItemDbg[LastIndex].next_wakeup,
        svc_pwr_dbg.StandByItemDbg[LastIndex].startup_time
        ));
      }
#endif
    }
 }

/********************************************//**
 * \brief declare a new peripheral on pwr
 *
 * \param peripherallockid_t  identifier
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_pwr_peripherallock_register( peripherallockid_t *id )
{
  gpOS_error_t error = gpOS_SUCCESS;

  if((svc_pwr_handler == NULL) || (svc_pwr_handler->peripherallocks_cnt >= SVC_PWR_MAX_PERIPHERALSLOCKS))
  {
    error = gpOS_FAILURE;
  }
  else
  {
    gpOS_semaphore_wait( svc_pwr_handler->access_peripherallocks_sem);

    *id = svc_pwr_handler->peripherallocks_cnt;
    svc_pwr_handler->peripherallocks_cnt++;

    gpOS_semaphore_signal( svc_pwr_handler->access_peripherallocks_sem);
  }

  return error;
}

/********************************************//**
 * \brief indicate that peripheral has began an activity
 *
 * \param peripherallockid_t  peripheral identifier
 * \return void
 *
 ***********************************************/
void svc_pwr_peripherallock_acquire( peripherallockid_t id)
{
  /*gpOS_error_t error = gpOS_SUCCESS;*/

  if((svc_pwr_handler == NULL) || (id >= SVC_PWR_MAX_PERIPHERALSLOCKS))
  {
    /* error case */
    /*error = gpOS_FAILURE;*/
  }
  else
  {
    gpOS_semaphore_wait( svc_pwr_handler->access_peripherallocks_sem);

    // Unitary instruction,no protection
    gpOS_interrupt_lock();
    MCR_SETBIT(svc_pwr_handler->peripherallocks, id);
    gpOS_interrupt_unlock();

    gpOS_semaphore_signal( svc_pwr_handler->access_peripherallocks_sem);
  }
}

/********************************************//**
 * \brief indicate that peripheral has stopped its activity
 *
 * \param peripherallockid_t  peripheral identifier
 * \return void
 *
 ***********************************************/
void svc_pwr_peripherallock_release( peripherallockid_t id)
{
  /*gpOS_error_t error = gpOS_SUCCESS;*/

  if((svc_pwr_handler == NULL) || (id >= SVC_PWR_MAX_PERIPHERALSLOCKS))
  {
    /* error case */
    /*error = gpOS_FAILURE;*/
  }
  else
  {
    gpOS_semaphore_wait( svc_pwr_handler->access_peripherallocks_sem);

    // Unitary instruction,no protection
    gpOS_interrupt_lock();
    MCR_CLEARBIT(svc_pwr_handler->peripherallocks, id);
    gpOS_interrupt_unlock();

    gpOS_semaphore_signal( svc_pwr_handler->access_peripherallocks_sem);
  }
}

/********************************************//**
 * \brief indicate that peripheral has stopped its activity during isr
 *
 * \param peripherallockid_t  peripheral identifier
 * \return void
 *
 ***********************************************/
void GENERIC_CODE_ISR svc_pwr_isr_peripherallock_release( peripherallockid_t id)
{
  /*gpOS_error_t error = gpOS_SUCCESS;*/

  if((svc_pwr_handler == NULL) || (id >= SVC_PWR_MAX_PERIPHERALSLOCKS))
  {
    /* error case */
    /*error = gpOS_FAILURE;*/
  }
  else
  {
    // Unitary instruction,no protection
    gpOS_interrupt_lock();
    MCR_CLEARBIT(svc_pwr_handler->peripherallocks, id);
    gpOS_interrupt_unlock();
  }
}

/********************************************//**
 * \brief Allow/Forbid the platform to enter low power mode when all wakelocks are released
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_pwr_set_lowpower_allowed( boolean_t status)
{
  gpOS_error_t error = gpOS_SUCCESS;

  if(status == FALSE)
  {
    error = gpOS_wakelock_acquire(svc_pwr_handler->wakelockid);
  }
  else
  {
    error = gpOS_wakelock_release(svc_pwr_handler->wakelockid, gpOS_TIMEOUT_INFINITY);
  }

  return error;
}


/********************************************//**
 * \brief Allow/Forbid the platform to enter low power mode when all wakelocks are released
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_pwr_get_lowpower_allowed( boolean_t *status)
{
  gpOS_error_t error = gpOS_SUCCESS;

  error = gpOS_wakelock_status( svc_pwr_handler->wakelockid, status);
  if(error == gpOS_SUCCESS)
  {
    *status = (*status == FALSE)?TRUE:FALSE;
  }
  else
  {
    *status = FALSE;
  }

  return error;
}
/********************************************//**
 * \brief
 *
 * \param tU16 duration
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_pwr_force_standby( tU16 duration )
{
  gpOS_error_t error = gpOS_SUCCESS;

  if(svc_pwr_handler == NULL)
  {
    error = gpOS_FAILURE;
  }
  else
  {
    gpOS_semaphore_wait( svc_pwr_handler->access_sem);
    //svc_pwr_standby_status                   = FALSE;
    svc_pwr_handler->force_standby_duration  = duration;
    svc_pwr_force_standby_status             = TRUE;
    gpOS_semaphore_signal( svc_pwr_handler->access_sem);
  }

  return error;
}

/********************************************//**
 * \brief Allow/Forbid the platform to leave in standby according to system timers or standby_duration
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_pwr_set_standby_allowed( boolean_t status )
{
  gpOS_error_t error = gpOS_SUCCESS;

  if( svc_pwr_handler == NULL )
  {
    error = gpOS_FAILURE;
  }
  else
  {
    gpOS_semaphore_wait( svc_pwr_handler->access_sem);

    svc_pwr_standby_status = status;         /**< Verify that standby still currently activated after stand-by restart */

    gpOS_semaphore_signal( svc_pwr_handler->access_sem);

    GPS_DEBUG_MSG(("[svcpwr] standby allowed =  %d  \r\n",svc_pwr_standby_status));
  }

  return error;
}

/********************************************//**
 * \brief Get standby allowed
 *
 * \param boolean_t true if allowed
 * \return gpOS_error_t
 *
 ***********************************************/
boolean_t svc_pwr_get_standby_allowed( void )
{
  boolean_t allowed = FALSE;

  if( svc_pwr_handler != NULL )
  {
    allowed = svc_pwr_standby_status;
  }

  return allowed;
}

/********************************************//**
 * \brief Get standby status
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_pwr_get_standby_status( boolean_t *status )
{
  gpOS_error_t error = gpOS_SUCCESS;
  tU32 DRI;
  tU32 DRF;

  *status = FALSE;

  if( (svc_pwr_handler == NULL) || (status == NULL))
  {
    error = gpOS_FAILURE;
  }
  else
  {
    tU32 svc_pwr_wakelock_id_temp;
    tU32 svc_pwr_wakelock_value;
    /* No semaphore handling - SHOULD BE CALLED ONLY FROM IDLE TASK */

    /* Store a checkpoint in the past for further time relationship after standby wakeup */
    LLD_RTC_Get_DRI_DRF_Registers(&DRI,&DRF);

    /* Prepare for forced standby check - need to bypass SVC_PWR wakelock */
    svc_pwr_wakelock_id_temp = 0U;
    MCR_SETBIT(svc_pwr_wakelock_id_temp, svc_pwr_handler->wakelockid);

    gpOS_wakelock_get_value_unp( &svc_pwr_wakelock_value);

    /* Case of FORCESTANDBY - bypass the svc_pwr wakelock */
    if(((svc_pwr_wakelock_value & ~svc_pwr_wakelock_id_temp) == 0x0U) && (svc_pwr_force_standby_status == TRUE) && (svc_pwr_handler->peripherallocks == 0x0U))
    {
      if(svc_pwr_handler->force_standby_duration != 0U)
      {
        svc_pwr_handler->next_wakeup = gpOS_time_plus(svc_pwr_handler->force_standby_duration, DRI);
      }
      else
      {
        /* Will wait an infinite time */
        svc_pwr_handler->next_wakeup = 0U;
      }
      *status = TRUE;
    }
    // Test next timer in a second step to save processing time
    else if (((svc_pwr_wakelock_value == 0x0U) && (svc_pwr_standby_status == TRUE) && (svc_pwr_handler->peripherallocks == 0x0U)))
    {
      gpOS_clock_t time_next_activity;
      gpOS_clock_t *time_next_activity_p = &time_next_activity;

      /* Next time wake-up reset */
      svc_pwr_handler->next_wakeup  = 0;

      /* Check next timer wakeup */
      gpOS_wakelock_task_next_activity(&time_next_activity_p);

      if( time_next_activity_p != gpOS_TIMEOUT_INFINITY )
      {
        // Check if next wakeup time is not too closed of time now
        if(gpOS_time_after( time_next_activity, DRI) != 0)
        {
          // Get next activity & apply startup time correction
          svc_pwr_handler->next_wakeup = time_next_activity;

          /* If next timer closer than standby wake up, abort */
#if (SVC_PWR_TRACE == 1)
          svc_pwr_dbg.StandByItemDbg[SVC_PWR_DBG_ITEM_IDX].next_wakeup = svc_pwr_handler->next_wakeup;
#endif
          if((gpOS_time_minus(svc_pwr_handler->next_wakeup, DRI))>= SVC_PWR_RTC_CRITICAL_ALARM_LIMIT)
          {
            * status = TRUE;

#if (SVC_PWR_TRACE == 1)
            /* Log the next standby request */
            _clibs_memcpy(&svc_pwr_dbg.StandByItemDbg[SVC_PWR_DBG_ITEM_IDX].EnterStandby,&svc_pwr_StandByTime,sizeof(svc_pwr_StandByTime_t));
            svc_pwr_dbg.StandByItemDbg[SVC_PWR_DBG_ITEM_IDX].WakeupAlarm = svc_pwr_handler->next_wakeup;
            svc_pwr_dbg.StandByItemDbg[SVC_PWR_DBG_ITEM_IDX].peripherallocks = svc_pwr_handler->peripherallocks;
#endif
          }
#if (SVC_PWR_TRACE == 1)
          svc_pwr_dbg.CurrentItem = (svc_pwr_dbg.CurrentItem + 1U ) % MAX_ITEM_DBG;
#endif
        }
      }
      else
      {
        /* No time registered in the timer list - will leave in standby forever until an HW event occurs */
        * status = TRUE;
      }
    }
    else
    {
      /* Status should stay set to FALSE */
    }

    if( * status == TRUE )
    {
      /* Store a checkpoint in the past for further time relationship after standby wakeup */
      svc_pwr_StandByTime.gpOS_time = gpOS_time_now();
      svc_pwr_StandByTime.rtc_int_val = DRI;
      if(DRF > SVC_PWR_RTC_TICKS_PER_SECOND)
      {
        DRF = SVC_PWR_RTC_TICKS_PER_SECOND;
      }
      svc_pwr_StandByTime.rtc_fra_val = (tU32)SVC_PWR_RTC_TICKS_PER_SECOND - DRF;
    }
  } /*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */


  return error;
}

/********************************************//**
 * \brief  Get startup mode
 *
 * \param  void
 * \return gnss_startup_mode_t
 ***********************************************/
svc_pwr_startup_mode_t svc_pwr_StartupMode( )
{
  svc_pwr_startup_mode_t mode = SVC_PWR_STARTUP_POWER_ON;

  /* WAKEUP pin has a higher priority, anyway, the Timers will be rescheduled and RTC wakeup or not, all activities will be re-evaluated */

  if( (svc_pwr_handler->wakeup_PIN == TRUE) && (svc_pwr_standby_status == TRUE) )
  {
    mode = SVC_PWR_STARTUP_WAKEUP_PIN;
  }
  else if( (svc_pwr_handler->wakeup_RTC == TRUE) && (svc_pwr_standby_status == TRUE))
  {
    mode = SVC_PWR_STARTUP_WAKEUP_RTC;
  }
  else
  {
    mode = SVC_PWR_STARTUP_POWER_ON;
  }

  return mode;
}

/********************************************//**
 * \brief  Get wakeup condition
 *
 * \param  void
 * \return boolean_t true if standby wakeup
 ***********************************************/
static boolean_t svc_pwr_IsStandbyWakeupState( boolean_t * wakeup_RTC,boolean_t * wakeup_PIN )
{
  boolean_t WakeupState = FALSE;

  if( ( wakeup_RTC != NULL ) && (wakeup_PIN != NULL))
  {
 #if defined ( __STA8090__ )
    LLD_PRCC_IsSTDBYWakeUpSource( wakeup_RTC, wakeup_PIN );
#endif
#if defined ( __STA8088__ )
    LLD_CLK_CTRL_IsWakeupStandbyRTC( wakeup_RTC );
    LLD_CLK_CTRL_IsWakeupStandbyPin( wakeup_PIN );
#endif

    if(((* wakeup_RTC) == TRUE) || ((*wakeup_PIN) == TRUE ))
    {
      WakeupState = TRUE;
    }
  }

 /**< Return Wakeup pin and standby allowed */
 return WakeupState;

}

/********************************************//**
 * \brief  Get startup condition
 *
 * \param  void
 * \return boolean_t true if standby wakeup
 ***********************************************/
boolean_t svc_pwr_StandbyWakeupState( boolean_t * WakeUpRTC, boolean_t * WakeUpPin )
{
  boolean_t WakeupState = FALSE;

  if( (svc_pwr_IsStandbyWakeupState(WakeUpRTC,WakeUpPin) == TRUE) && (svc_pwr_standby_status == TRUE ) )
  {
    WakeupState = TRUE;
  }

  /**< Return Wakeup pin and standby allowed */
  return WakeupState;
}



/********************************************//**
 * \brief  Get startup time adjustment after standby
 *
 * \param  void
 * \return boolean_t true time available
 ***********************************************/
boolean_t svc_pwr_get_timer_adjustment( gpOS_clock_t * StandbyDuration_rtcbase, gpOS_clock_t * VirtualPreviousOsTime )
{
  boolean_t bTimeAvailable = FALSE;

  tUInt DRI,DRF;
  tDouble rtc_time_now_secs,rtc_time_stdby_secs,rtc_time_delta,stdby_duration;
  gpOS_clock_t Time;

  // Check wakeup & previous time available
  if( (svc_pwr_StartupMode() != SVC_PWR_STARTUP_POWER_ON) && (svc_pwr_StandByTime.rtc_int_val != 0U) )
  {
    gnss_error_t rtc_error;

    rtc_error = rtc_drv_rtc_accurate_read(&DRI,&DRF,&Time,(tDouble)SVC_PWR_RTC_TICKS_PER_SECOND);
    if( rtc_error != GNSS_NO_ERROR)
    {
      GPS_DEBUG_MSG(("[svcpwr][stdby] RTC not ready \r\n"));
    }
    else
    {
      // Compute time since standby
      rtc_time_stdby_secs = (tDouble)svc_pwr_StandByTime.rtc_int_val + ((tDouble)svc_pwr_StandByTime.rtc_fra_val/(tDouble)SVC_PWR_RTC_TICKS_PER_SECOND);

      // compute duration with RTC only
      rtc_time_now_secs = (tDouble)DRI + ((tDouble)DRF/(tDouble)SVC_PWR_RTC_TICKS_PER_SECOND);
      rtc_time_delta = rtc_time_now_secs - rtc_time_stdby_secs;
      stdby_duration = rtc_time_delta * (tDouble) gpOS_timer_ticks_per_sec();
      * StandbyDuration_rtcbase =  (gpOS_clock_t) stdby_duration;
      * VirtualPreviousOsTime = * StandbyDuration_rtcbase + svc_pwr_StandByTime.gpOS_time;

      bTimeAvailable = TRUE;
    }
  }

  return bTimeAvailable;
}


/********************************************//**
 * \brief Enter standby
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
void svc_pwr_enter_standby( void )
{
  if((svc_pwr_handler != NULL) && ((svc_pwr_force_standby_status == true) || (svc_pwr_standby_status == true )) )
  {
    //LLD_PRCC_StopModeDisable();
    if( svc_pwr_handler->next_wakeup != 0U )
    {
      /* Ensure RTC is up and running */
      if(LLD_RTC_IsEnabled() == FALSE)
      {
        LLD_RTC_Enable(0U);
      }

      /* Clean interrupt */
      LLD_RTC_ClearInterrupt();

      /* Set duration in match register */
      LLD_RTC_SetMatchRegister( svc_pwr_handler->next_wakeup );

      // Arm interrupt for next alarm
      LLD_RTC_EnableInterrupt();

#if defined( __STA8088__ )
      LLD_CLK_CTRL_EnableWakeupFromRTC();
#elif defined( __STA8090__ )
      LLD_PRCC_SetWakeupFromRTC( TRUE);
      LLD_PRCC_EnableWakeup();
#endif
    }
    else
    {
#if defined( __STA8088__ )
      LLD_CLK_CTRL_DisableWakeupFromRTC();
#elif defined( __STA8090__ )
      LLD_PRCC_SetWakeupFromRTC( FALSE);
      LLD_PRCC_EnableWakeup();
#endif
    }
  }
  else
  {
#if defined( __STA8088__ )
    LLD_CLK_CTRL_EnableWakeupFromRTC();
#elif defined( __STA8090__ )
    LLD_PRCC_SetWakeupFromRTC( TRUE);
    LLD_PRCC_EnableWakeup();
#endif
  }

#if defined( __STA8088__ )
  LLD_CLK_CTRL_StandByEnable();
#elif defined( __STA8090__ )
  /* Otherwise RTC has been set with Timer handling */
  LLD_PRCC_SetWakeupFromSTDBYpin ( TRUE);
  /* In force standby case, deactivate standby for next run */
  LLD_PRCC_PeripheralClkDis( LLD_PRCC_PERIPHID_SRAM);
  //LLD_PRCC_StandByEnable();
  LLD_PRCC_EnterStandby();
  /* In case standby mode cannot be entered, the sw must go on */
  LLD_PRCC_PeripheralClkEn( LLD_PRCC_PERIPHID_SRAM);
#endif
}

#if defined( __STA8090__ )
/********************************************//**
 * \brief Perform peripheral and clock source check before going in WFI
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
void GENERIC_CODE_ISR svc_pwr_pre_WFI( void )
{

/* ---------------------------------- */
/* ALL CODE HEREAFTER MUST BE IN ITCM */

  /* Maybe call a function from platform.c here */
  LLD_PRCC_PeripheralClkDis( LLD_PRCC_PERIPHID_SQIO);
  LLD_PRCC_PeripheralClkDis( LLD_PRCC_PERIPHID_FSMC);
}

/********************************************//**
 * \brief Perform peripheral and clock source check when going out of WFI
 *
 * \param boolean_t status
 * \return gpOS_error_t
 *
 ***********************************************/
void GENERIC_CODE_ISR svc_pwr_post_WFI( void )
{
/* ---------------------------------- */
/* ALL CODE HEREAFTER MUST BE IN ITCM */

  /* Maybe call a function from platform.c here */
  LLD_PRCC_PeripheralClkEn( LLD_PRCC_PERIPHID_SQIO);
  LLD_PRCC_PeripheralClkEn( LLD_PRCC_PERIPHID_FSMC);

/* ALL CODE HEREDEFORE MUST BE IN ITCM */
/* ---------------------------------- */

  MCR_NOP();
  MCR_NOP();
  MCR_NOP();
  MCR_NOP();
  MCR_NOP();
}
#endif /* __STA8090__ */
