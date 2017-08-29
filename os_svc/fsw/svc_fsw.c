/*******************************************************************************
 *                            (C) 2013 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Runtime switch service
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"

#include "clibs.h"
#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_fsw.h"
#include "gpOS_bsp.h"

#include "gnss_debug.h"

#include "frontend.h"

#include "svc_mtu.h"

#include "lld.h" // for ISR region

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_FSW_RIOSC_CALIB_THLOW (0x39F)
#define SVC_FSW_RIOSC_CALIB_THHI  (0x401)

#define SVC_FSW_STACK_SIZE    512
#define SVC_FSW_TASK_PRIORITY 5

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  SVC_FSW_TCXO_TO_OSCI_CLK,
  SVC_FSW_OSCI_CLK_TO_TCXO,
  SVC_FSW_NO_CLK_TRANSITION
} clock_transition_t;

typedef enum
{
  SVC_FSW_FREQUENCY_DECREASE,
  SVC_FSW_FREQUENCY_INCREASE,
  SVC_FSW_NO_FREQUENCY_CHANGE
} frequency_state_t;

typedef struct
{
  svc_fsw_cfg_t         config;
  svc_mcu_clkcfg_t      curr_cfg;
} svc_fsw_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_fsw_handler_t *svc_fsw_handler;

// used to save configuration when needed (in case of init for instance)
static svc_mcu_corefreqcfg_t svc_fsw_saved_configuration;

static gpOS_task_t *svc_fsw_riosc_calib_task;

static gpOS_semaphore_t *svc_fsw_ring_calib_access_sem;

static boolean_t svc_fsw_RIOSC_calibration_task_ongoing;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static tVoid svc_fsw_increase_frequency(tS32 freq_ratio, clock_transition_t clock_transition);
static tVoid svc_fsw_decrease_frequency(tU32 freq_ratio, clock_transition_t clock_transition);

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/************************************************************//**
 * \brief   Task in charge of Ring Oscillator Calibration
 *
 * \param   *param additional parameter
 * \return  task_exit_status_t
 *
 ****************************************************************/
#define RIOSC_CALIB_DELAY_IN_S 10U

static gpOS_task_exit_status_t  svc_fsw_calib(const gpOS_task_param_t param)
{
  boolean_t exit_flag = FALSE;
  tU32 freq = 0;

  while( exit_flag == FALSE)
  {
    tU32 trim_out = 0;
    gpOS_clock_t timeout = gpOS_time_now() + (RIOSC_CALIB_DELAY_IN_S*gpOS_timer_ticks_per_sec());

    if (svc_fsw_RIOSC_calibration_task_ongoing == TRUE)
    {
      gpOS_semaphore_wait_timeout(svc_fsw_ring_calib_access_sem, &timeout);

      if (svc_fsw_RIOSC_calibration_task_ongoing == FALSE)
      {
        // Used in case not running with RIOSC anymore once timeout has expired
        gpOS_semaphore_wait(svc_fsw_ring_calib_access_sem );
      }
      else
      {
        svc_mcu_suspend_transfers(svc_fsw_handler->config.mclkid);

        svc_mcu_setRIOSCconfig(2, SVC_FSW_RIOSC_CALIB_THLOW, SVC_FSW_RIOSC_CALIB_THHI);
        freq = svc_mcu_getRIOSCfrequency(&trim_out);

        svc_mcu_busclk_set( svc_fsw_handler->config.mclkid, freq);
        svc_mcu_update_data_rates( svc_fsw_handler->config.mclkid);

        svc_mcu_restore_transfers(svc_fsw_handler->config.mclkid);

        GPS_DEBUG_MSG(("[fsw] CALIB New freq: %d Hz --- trim_out: %d \r\n", freq, trim_out));
      }
    }
    else
    {
      // Wait for RIOSC calibration request
      gpOS_semaphore_wait(svc_fsw_ring_calib_access_sem );
    }
  }

  // should never reach this
  return -1;
}

/************************************************************//**
 * \brief   Init of frequency switch
 *
 * \param   part memory partition
 *
 * \return  os_error_t
 *
 ****************************************************************/
gpOS_error_t svc_fsw_init( gpOS_partition_t* part, const svc_fsw_cfg_t *cfg_ptr)
{
  gpOS_error_t error = gpOS_FAILURE;

  svc_fsw_handler = (svc_fsw_handler_t *)gpOS_memory_allocate_p( part, sizeof( svc_fsw_handler_t));
  svc_fsw_riosc_calib_task = gpOS_task_create_p( part, svc_fsw_calib, NULL, SVC_FSW_STACK_SIZE, SVC_FSW_TASK_PRIORITY, "svc_fsw_calib_ws", gpOS_TASK_FLAGS_ACTIVE);

  if( (svc_fsw_handler != NULL) && (svc_fsw_riosc_calib_task != NULL))
  {
    svc_fsw_ring_calib_access_sem = gpOS_semaphore_create( SEM_FIFO, 0);
    if(svc_fsw_ring_calib_access_sem != NULL)
    {
      /* Set initial configuration */
      _clibs_memcpy( &svc_fsw_handler->config, cfg_ptr, sizeof( svc_fsw_cfg_t));
      _clibs_memcpy( &svc_fsw_handler->curr_cfg, &svc_fsw_handler->config.clkcfg_table[cfg_ptr->clkcfg_start], sizeof( svc_mcu_clkcfg_t));

      error = gpOS_SUCCESS;
    }
    else
    {
      /* Delete allocated data */
      gpOS_task_delete( svc_fsw_riosc_calib_task);
    }
  }

  return error;
}

/************************************************************//**
 * \brief   Set external frequency value for systimer
 *
 * \param   extfreq   new external frequency
 *
 * \return  none
 *
 ****************************************************************/
tVoid svc_fsw_setextfreq( tUInt extfreq)
{
  svc_fsw_handler->config.systimer_extfreq = extfreq;
}

/************************************************************//**
 * \brief   Increase ARM frequency and associated peripherals
 *
 * \param   freq_ratio ratio to apply when using frequency
 * \param   clock_transition  indicate if switching
 *          from TCXO to OSCI, etc...
 * \return  none
 *
 ****************************************************************/

static tVoid svc_fsw_increase_frequency(tS32 freq_ratio, clock_transition_t clock_transition)
{
  tU32 trim_out = 0;
  gpOS_clock_t current_time;

  gpOS_interrupt_lock();

  // Lock operating system timer
  gpOS_timer_suspend();

  svc_fsw_handler->config.set_clkcfg_cb( &svc_fsw_handler->curr_cfg, TRUE);

  if (clock_transition == SVC_FSW_OSCI_CLK_TO_TCXO)
  {
    // Enable G3 device. This lets f0 to be provided to MTU.
    svc_mcu_RFenable(TRUE);
  }

  svc_mcu_busclk_set( svc_fsw_handler->config.mclkid, (tU32)svc_fsw_handler->curr_cfg.bus_speed);

  if (freq_ratio != 1)
  {
    current_time = gpOS_time_now();

    // Update timeout values for the next coming task only. The rest is done once timer are ON again.
    gpOS_timer_timeout_task_update((tU32)freq_ratio, svc_fsw_handler->config.freqratiofactor, current_time, TRUE);
    gpOS_timer_set_clock( &svc_fsw_handler->curr_cfg.systimer_cfg, TRUE);
  }
  else
  {
    gpOS_timer_set_clock( &svc_fsw_handler->curr_cfg.systimer_cfg, FALSE);
  }

  gpOS_bsp_timer_update_timeout(freq_ratio, svc_fsw_handler->config.freqratiofactor);

  gpOS_timer_start();

  if (freq_ratio != 1)
  {
    // Update timeout values for all existing tasks except first one which has been already updated
    gpOS_timer_timeout_task_update((tU32)freq_ratio, svc_fsw_handler->config.freqratiofactor, current_time, FALSE);  /*lint !e644 Variable may not have been initialized */
  }

  gpOS_interrupt_unlock();

  // Change baudrate/datarates according to new frequency clock
  if (svc_fsw_handler->curr_cfg.bus_speed  <= svc_fsw_handler->config.maxoscifreq)
  {
    // "Ring Oscillator" case, need to use the measured frequency.
    svc_mcu_busclk_set( svc_fsw_handler->config.mclkid, svc_mcu_getRIOSCfrequency(&trim_out));
    svc_mcu_update_data_rates( svc_fsw_handler->config.mclkid);

    if (svc_fsw_RIOSC_calibration_task_ongoing == FALSE)
    {
      // Resume Ring Oscillator calibration task
      svc_fsw_RIOSC_calibration_task_ongoing  = TRUE;
      gpOS_semaphore_signal(svc_fsw_ring_calib_access_sem);
    }
  }
  else
  {
    svc_mcu_update_data_rates( svc_fsw_handler->config.mclkid);

    if (clock_transition == SVC_FSW_OSCI_CLK_TO_TCXO)
    {
      // Disable Ring Oscillator
     svc_mcu_setRIOSCconfig(0, 0, 0);
    }
  }
}

/************************************************************//**
 * \brief   Decrease ARM frequency and associated peripherals
 *
 * \param   freq_ratio ratio to apply when using frequency
 * \param   clock_transition  indicate if switching
 *          from TCXO to OSCI, etc...
 * \return  none
 *
 ****************************************************************/

static tVoid svc_fsw_decrease_frequency(tU32 freq_ratio, clock_transition_t clock_transition)
{
  if (clock_transition == SVC_FSW_TCXO_TO_OSCI_CLK)
  {
    // Prepare new source clock and enable it (i.e. Ring Oscillator in case of frequency decrease)
    svc_mcu_setRIOSCconfig(1, 0, 0);

    svc_mcu_setRIOSCconfig(2, SVC_FSW_RIOSC_CALIB_THLOW, SVC_FSW_RIOSC_CALIB_THHI);

  }

  gpOS_interrupt_lock();

  if (freq_ratio != 1U)
  {
    gpOS_clock_t current_time = gpOS_time_now();
    // Update timeout values for all existing tasks, according to F0 factor (e.g.: 4*F0 -> (1/8)*F0)
    gpOS_timer_timeout_task_update(freq_ratio, svc_fsw_handler->config.freqratiofactor, current_time, FALSE);
    gpOS_timer_suspend();
    gpOS_timer_timeout_task_update(freq_ratio, svc_fsw_handler->config.freqratiofactor, current_time, TRUE);
  }
  else
  {
    gpOS_timer_suspend();
  }

  svc_fsw_handler->config.set_clkcfg_cb( &svc_fsw_handler->curr_cfg, FALSE);

  if (svc_fsw_handler->curr_cfg.bus_speed  > svc_fsw_handler->config.maxoscifreq)
  {
    svc_mcu_busclk_set( svc_fsw_handler->config.mclkid, (tU32)svc_fsw_handler->curr_cfg.bus_speed);
  }
  else
  {
    tU32 trim_out = 0;

    if(clock_transition == SVC_FSW_TCXO_TO_OSCI_CLK)
    {
      // Disable G3/G3EP clock
      svc_mcu_RFenable(FALSE);
      // Disable TCXO
      svc_mcu_changeclockmode(2);
      // Configure power supply settings
      svc_fsw_handler->config.set_pwrcfg_cb( svc_fsw_handler->config.pwrlowfreqid );

      /* Perform a second calibration with RF OFF - better results */
      svc_mcu_setRIOSCconfig(2, SVC_FSW_RIOSC_CALIB_THLOW, SVC_FSW_RIOSC_CALIB_THHI);
    }

    svc_mcu_busclk_set( svc_fsw_handler->config.mclkid, svc_mcu_getRIOSCfrequency(&trim_out));
  }

  if (freq_ratio != 1U)
  {
    gpOS_timer_set_clock( &svc_fsw_handler->curr_cfg.systimer_cfg, TRUE);
  }
  else
  {
    gpOS_timer_set_clock( &svc_fsw_handler->curr_cfg.systimer_cfg, FALSE);
  }

  gpOS_bsp_timer_update_timeout((tS32)freq_ratio, svc_fsw_handler->config.freqratiofactor);

  gpOS_timer_start();

  gpOS_interrupt_unlock();

  if (svc_fsw_handler->curr_cfg.bus_speed  <= svc_fsw_handler->config.maxoscifreq)
  {

    // "Ring Oscillator" case, need to use the measured frequency.
    svc_mcu_update_data_rates( svc_fsw_handler->config.mclkid);

    if (svc_fsw_RIOSC_calibration_task_ongoing == FALSE)
    {
      // Resume Ring Oscillator calibration task
      svc_fsw_RIOSC_calibration_task_ongoing  = TRUE;
      gpOS_semaphore_signal(svc_fsw_ring_calib_access_sem);
    }
  }
  else
  {
    svc_mcu_update_data_rates( svc_fsw_handler->config.mclkid);
  }
}


/********************************************//**
 * \brief   Set new ARM clock frequency
 *
 * \param   platform_ARM_frequency_t  targeted ARM and peripherals frequency
 * \param   svc_fsw_mode_t mode used (forced or normal).
            If Forced mode used, the frequency switch will occur even if it provokes a baudrate change.
 * \return  os_error_t error if frequency change not possible
 *
 ***********************************************/
gpOS_error_t svc_fsw_set_corefreq( svc_mcu_corefreqcfg_t config, boolean_t forced_mode)
{
  clock_transition_t clock_transition = SVC_FSW_NO_CLK_TRANSITION;
  tS32 freq_ratio = 1;
  frequency_state_t new_frequency_status = SVC_FSW_NO_FREQUENCY_CHANGE;

  // Save previous values first
  tS32 previous_factor = svc_fsw_handler->curr_cfg.systimer_cfg.factor;
  svc_mcu_corefreqcfg_t previous_corefreqid = svc_fsw_handler->curr_cfg.corefreqid;

  tU32 previous_clk_source = (tU32)svc_fsw_handler->curr_cfg.clk_source;
  svc_mcu_busfreq_t previous_bus_speed = svc_fsw_handler->curr_cfg.bus_speed ;

  // Check if frequency switch is compatible with current baudrates.
  // If not and forced_mode set to false, then abort frequency switch and exit the function
  if (forced_mode == FALSE)
  {
    if (svc_mcu_check_data_rates_feasibility(svc_fsw_handler->config.mclkid, (tU32)svc_fsw_handler->config.clkcfg_table[config].bus_speed) == gpOS_FAILURE)
    {
      GPS_DEBUG_MSG(("[fsw] FreqSwitch cancelled: the according baud rate change is not reachable!\r\n"));
      return (gpOS_FAILURE);      /*lint !e904 Return statement before end of function */
    }
  }

  // Suspend Ring Oscillator calibration task during the transition if it was active
  if (svc_fsw_RIOSC_calibration_task_ongoing == TRUE)
  {
    svc_fsw_RIOSC_calibration_task_ongoing = FALSE;
    gpOS_semaphore_signal(svc_fsw_ring_calib_access_sem);
  }

  // Update stored configuration: svc_fsw_handler->curr_cfg = platform_frequency_switch_handler[config]
  // done by copying element by element to gain execution time compared to memcpy
  svc_fsw_handler->curr_cfg.corefreqid           = config;
  svc_fsw_handler->curr_cfg.clk_48f0_enabled      = svc_fsw_handler->config.clkcfg_table[config].clk_48f0_enabled;
  svc_fsw_handler->curr_cfg.clk_arm_speed         = svc_fsw_handler->config.clkcfg_table[config].clk_arm_speed;
  svc_fsw_handler->curr_cfg.clk_source            = svc_fsw_handler->config.clkcfg_table[config].clk_source;
  svc_fsw_handler->curr_cfg.clk_divider           = svc_fsw_handler->config.clkcfg_table[config].clk_divider;
  svc_fsw_handler->curr_cfg.bus_speed             = svc_fsw_handler->config.clkcfg_table[config].bus_speed;
  svc_fsw_handler->curr_cfg.systimer_cfg.factor   = svc_fsw_handler->config.clkcfg_table[config].systimer_cfg.factor;

  if( svc_fsw_handler->config.clkcfg_table[config].systimer_cfg.ext_freq == 0U)
  {
    svc_fsw_handler->curr_cfg.systimer_cfg.ext_freq = 0;
  }
  else
  {
    svc_fsw_handler->curr_cfg.systimer_cfg.ext_freq = (tU32)svc_fsw_handler->config.systimer_extfreq;
  }
  svc_fsw_handler->curr_cfg.systimer_cfg.prescaler = svc_fsw_handler->config.clkcfg_table[config].systimer_cfg.prescaler;

  // Save configuration number for future use, such as GNSS restart
  svc_fsw_saved_configuration = config;


  if (previous_corefreqid != config)
  {

    // Call here the function to indicate that frequency switch is starting
    GPS_DEBUG_MSG(("[fsw] Start FreqSwitch to config: %d -> %d to %d @time: %u\r\n", config, previous_bus_speed, svc_fsw_handler->curr_cfg.bus_speed, gpOS_bsp_timer_time_now()));

    // Set to avoid UART freeze when reducing frequency in case an debug message is called just some ms before
    // the svc_mcu_suspend will wait that all transfer are over...no need for a task delay here
    // gpOS_task_delay(100U*gpOS_timer_ticks_per_msec());

    // Suspend tranfers when possible (i.e. when Master in case of I²C, MSP and SSP)
    svc_mcu_suspend_transfers(svc_fsw_handler->config.mclkid);


    // Check if it is a frequency increase or decrease
    if (previous_corefreqid < config)
    {
      new_frequency_status = SVC_FSW_FREQUENCY_DECREASE;

      if ((svc_fsw_handler->curr_cfg.bus_speed  <= svc_fsw_handler->config.maxoscifreq)&&(previous_bus_speed > svc_fsw_handler->config.maxoscifreq))
      {
        clock_transition = SVC_FSW_TCXO_TO_OSCI_CLK;
      }
    }
    else if (previous_corefreqid > config)
    {
      new_frequency_status = SVC_FSW_FREQUENCY_INCREASE;

      if ((svc_fsw_handler->curr_cfg.bus_speed > svc_fsw_handler->config.maxoscifreq)&&(previous_bus_speed <= svc_fsw_handler->config.maxoscifreq))
      {
        clock_transition = SVC_FSW_OSCI_CLK_TO_TCXO;

        svc_fsw_handler->config.set_pwrcfg_cb( config );
        // Enable 26MHz TCXO
        svc_mcu_changeclockmode(1);
        svc_mcu_RFenable(FALSE);

        FE_reset();

        #if defined(ENABLE_EXTERNAL_5630)
        if( gnss_get_cut_version() == GNSS_BSP_T2_CUT_BC)
        {
          FE_reset_ext();
          FE_Set_NRDiv_ext( NDIV_EXT, RDIV_EXT);
        }
        #endif
      }
    }
    else
    {
      /* Empty */
    }

    if (previous_factor != svc_fsw_handler->curr_cfg.systimer_cfg.factor)
    {
      if (previous_factor < svc_fsw_handler->curr_cfg.systimer_cfg.factor)
      {
        freq_ratio = (tS32)(((tU64)svc_fsw_handler->curr_cfg.systimer_cfg.factor * (tU64)svc_fsw_handler->config.freqratiofactor) / (tU64)previous_factor);
      }
      else
      {
        // bit of sign added to transmit the usage of F0 factor
        freq_ratio = ((-1)*(tS32)(((tU64)previous_factor * (tU64)svc_fsw_handler->config.freqratiofactor) / (tU64)svc_fsw_handler->curr_cfg.systimer_cfg.factor));
      }
    }

    if (new_frequency_status == SVC_FSW_FREQUENCY_INCREASE)
    {
      svc_fsw_increase_frequency(freq_ratio, clock_transition);
    }
    else if (new_frequency_status == SVC_FSW_FREQUENCY_DECREASE)
    {
      svc_fsw_decrease_frequency((tU32)freq_ratio, clock_transition);
    }
    else
    {
      /* Empty */
    }

    svc_mcu_restore_transfers(svc_fsw_handler->config.mclkid);

    // "0" corresponds to PLL case for both platforms.
    if ((previous_clk_source == 0U)&&(svc_fsw_handler->curr_cfg.clk_source != 0))
    {
      svc_mcu_setPLL( FALSE);
    }

    GPS_DEBUG_MSG(("[fsw] End FreqSwitch: %d -> bus speed %d to %d - t: %u\r\n", config, previous_bus_speed, svc_fsw_handler->curr_cfg.bus_speed, gpOS_bsp_timer_time_now()));

    // For debug
    if (svc_fsw_handler->curr_cfg.bus_speed  <= svc_fsw_handler->config.maxoscifreq)
    {
      tU32 trim_out = 0;
      tU32 freq = 0;
      freq = svc_mcu_getRIOSCfrequency(&trim_out);
      GPS_DEBUG_MSG(("[fsw] CalibRes -> freq: %d Hz, trim_out: %d \r\n", freq, trim_out));
    }

  // Call here the function to indicate that frequency switch is finished.
  }
  else
  {
    GPS_DEBUG_MSG(("[fsw] No FreqSwitch needed: same config %d as before\r\n", config));
  }

  return gpOS_SUCCESS;
}

/***************************************************//**
 * \brief   Get pointer to complete frequency
 *          configuration parameters
 *
 * \param   none
 * \return  platform_clkcfg_t* pointer to configuration
 *
 *******************************************************/
svc_fsw_cfg_t* svc_fsw_get_fswcfg( tVoid)
{
  return( &svc_fsw_handler->config);
}

/***************************************************//**
 * \brief   Get pointer to complete frequency
 *          configuration parameters
 *
 * \param   none
 * \return  platform_clkcfg_t* pointer to configuration
 *
 *******************************************************/
svc_mcu_clkcfg_t* svc_fsw_get_clkcfg( tVoid)
{
  return( &svc_fsw_handler->curr_cfg);
}

/********************************************//**
 * \brief   Save frequency configuration number
 *
 * \param   config_number configuration number
 * \return  none
 *
 ***********************************************/
tVoid svc_fsw_save_configuration( svc_mcu_corefreqcfg_t config_number)
{
  svc_fsw_saved_configuration = config_number;
}

/********************************************//**
 * \brief   Get frequency configuration number
 *
 * \param   none
 * \return  saved configuration number
 *
 ***********************************************/
svc_mcu_corefreqcfg_t svc_fsw_get_configuration(tVoid)
{
  return( svc_fsw_saved_configuration);
}
