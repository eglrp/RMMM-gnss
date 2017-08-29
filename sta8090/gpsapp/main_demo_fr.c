/*****************************************************************************
   FILE:          main_freeRTOS.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The main freeRTOS application to run and test STA8090 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2014 STMicroelectronics, (S2S - SWD) Le Mans (FRANCE)
------------------------------------------------------------------------------
   Created by : Fabrice Pointeau
           on : 2014.05.19
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"
#include "gpOS_crt.h"
#include "svc_mcu.h"
#include "svc_pwr.h"
#include "svc_fsw.h"

// LLD for STA8088
#include "lld.h"
#include "lld_rtc.h"
#include "lld_gpio.h"

// freeRTOS related
#include "freeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpOS_types.h"
#include "FR_svci.h"
#include "gpOS_bsp.h"

// GPS library
#include "gnssapp.h"
#include "gnssapp_plugins.h"
#include "gnss_debug.h"

// Platform releated
#include "platform.h"

// Modules related
#include "in_out.h"
#include "sw_config.h"

#if defined( EXAMPLE_NOC )
#include "svc_can.h"
#include "NOC_app.h"
#endif

#if defined (EXAMPLE_PVT)
// Needed for PVT example
#include "PVT_app.h"
#endif

#if defined (EXAMPLE_POW)
// Needed for POW example
#include "POW_app.h"
#endif

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define IDLE_TASK_STACK_SIZE   2816/4   /* size of the idle task stack */
#define SYS_MODE_STACK_SIZE    512  /* size of the interrupt stack for all interrupts. This must be the worst case if all interrupt happen concurrently */

#if defined( EXAMPLE_NOC )
#define CAN_USED      0
#endif

#if defined (__ARMCC_VERSION)
#define OS_ROOT_TASK_STACK_SIZE   (2048)  /* The main() stack size in bytes */
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
#if defined (__ARMCC_VERSION)
// Root task size
const tUInt os_root_task_stack_size = OS_ROOT_TASK_STACK_SIZE;
#endif

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
static gpOS_partition_t *fast_partition;
static TaskHandle_t xMainIdleProcess;

#ifdef FREERTOS_CHECK_SEMAPHORES
int set_trace = 0;
#endif

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static void main_idle_process( void *pvParameters );

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief   Start NMEA Over CAN application
 *
 * \param   void
 * \return  gpOS_error_t
 *
 ***********************************************/
#ifdef EXAMPLE_NOC
gpOS_error_t gnssapp_NoC_start( gpOS_partition_t *part)
{

  tBool som = FALSE;

  svc_can_init( part, PLATFORM_BUSCLK_ID_CAN_CLK);

  if( svc_can_open_port( CAN_USED, gpOS_INTERRUPT_NOPRIORITY, LLD_CAN_125KBPS, LLD_CAN_STD_ID, som, 8) == gpOS_FAILURE)
  {

     return gpOS_FAILURE;

  }

  if( CAN_start( CAN_USED, part, som) == LLD_ERROR)
  {

    GPS_DEBUG_MSG(("[main]: LLD_ERROR CAN_start failed\n"));

    return( gpOS_FAILURE);

  }

  return( gpOS_SUCCESS);
}
#endif

/********************************************//**
 * \brief Starts freeRTOS
 *
 * \param void
 * \return gpOS_SUCCESS if all ok, else gpOS_FAILURE
 *
 ***********************************************/
static int main_freeRTOS_start( void )
{

  svc_mcu_clkcfg_t* curr_clkcfg;

  gpOS_interrupt_lock();

  gpOS_crt_init();

  gpOS_interrupt_init( fast_partition, SYS_MODE_STACK_SIZE);

  // Initialize OS Services and set available bus clocks
  platform_mcu_init( NULL);
  platform_mcu_setspeed( PLATFORM_MCU_SPEED_26MHZ);

  gpOS_wakelock_init( fast_partition);

  curr_clkcfg = svc_fsw_get_clkcfg();

  gpOS_timer_init( &curr_clkcfg->systimer_cfg);

  gpOS_timer_start();

  gpOS_memory_protection_init();

  gpOS_interrupt_unlock();

  gpOS_bsp_timer_set_timeslice( FALSE);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief idle task process
 *
 * \param param dummy parameter
 * \return
 *
 ***********************************************/
static void main_idle_process( void *pvParameters )
{
  gpOS_wakelockid_t wakelock_main;
  sw_config_t *sw_config_ptr;
  tU8 spm_conf;

  /**< Starts freeRTOS */
  if( main_freeRTOS_start() == gpOS_FAILURE)
  {
    GPS_DEBUG_MSG(( "[main] ERROR CANNOT START FREERTOS SERVICES !!!\r\n" ));
  }

  /**< Initialize platform basic peripherals */
  if( platform_config() == gpOS_FAILURE)
  {
    GPS_DEBUG_MSG(( "[main] ERROR CANNOT CONFIGURE PLATFORM !!!\r\n" ));
  }

  /**< Get pointer to default Configuration Data Blocks */
  if( sw_config_get_default( &sw_config_ptr) == GNSS_ERROR)
  {
    GPS_DEBUG_MSG(( "[main] ERROR CANNOT GET SPM DEFAULT VALUES !!!\r\n" ));
  }
  else
  {
    /**< If SPM is disabled, SPM configuration byte should be 0 */
    if( sw_config_ptr->spm == 0U)
    {
      spm_conf = 0U;
    }
    else
    {
      spm_conf = (tU8)sw_config_ptr->spm_configuration;
    }
  }

  /**< Initialize fundamental drivers */
  if( platform_start_services( fast_partition, spm_conf) == gpOS_FAILURE)
  {
    GPS_DEBUG_MSG(( "[main] ERROR CANNOT START SERVICES !!!\r\n" ));
  }

  gpOS_wakelock_register(&wakelock_main);
  gpOS_wakelock_acquire(wakelock_main);

  timer_remove_TickCount();
  if(timer_generate_TickCount( 1, FALSE ) == pdFALSE)
  {
    /* First tick insertion failed, try 2 */
    timer_generate_TickCount(2, FALSE);
  }

  /* This Task is started with highest priority to launch the GNSS initialisation */
  /**< Start NVM module */
  if( gnssapp_init( fast_partition) == gpOS_FAILURE)
  {
    GPS_DEBUG_MSG(( "[main] ERROR CANNOT START GNSS LIBRARY !!!\r\n" ));
  }

  #ifdef EXAMPLE_NOC
  /**< Start NoC application */
  if( gnssapp_NoC_start( NULL) == gpOS_FAILURE)
  {
    GPS_DEBUG_MSG(( "[main] ERROR CANNOT START NOC DEMO !!!\r\n" ));
  }
  #endif

 #ifdef EXAMPLE_POW
  /**< Start low power application */
  pow_app_init(fast_partition);
 #endif

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
  vOS_stats_init();
#endif

  /* Set to low priority for regular tracing */
  vTaskPrioritySet( xMainIdleProcess, tskIDLE_PRIORITY + 4);

  #ifdef FREERTOS_CHECK_SEMAPHORES
  set_trace = 1;
  #endif

  /* Do nothing - introduced to let Navigate start and acquire a wakelock */
  gpOS_task_delay( 10*gpOS_timer_ticks_per_msec());

  gpOS_wakelock_release(wakelock_main, gpOS_TIMEOUT_INFINITY);

#if defined (EXAMPLE_PVT)
  /* call PVT example init */
  pvt_app_init(fast_partition);
#endif

  while(TRUE)
  {
    /* Do nothing */
    vTaskDelay( 1000/portTICK_PERIOD_MS );

    #ifdef FREERTOS_CHECK_SEMAPHORES
    if (set_trace == 2) {
      GPS_DEBUG_MSG(( "<<<<<<<<<<<<<  SEMAPHORES ERRORS >>>>>>>>>>>>>>>>>\r\n"));
      set_trace = 0;
    }
    #endif
    {
      int clk_source, clk_speed;

      platform_get_clock_configuration_status( &clk_source, &clk_speed);

      GPS_DEBUG_MSG(( "[main]CPU usage: %3.2f clk src %d speed %d\r\n", (double)(svc_mcu_getcpuusage()) / 100.0, clk_source, clk_speed));
      GPS_DEBUG_MSG(( "[main]MEM usage: heap total size %8d, req %8d, free %8d\r\n", gpOS_memory_getheapsize(), gpOS_memory_getheaprequested(), gpOS_memory_getheapfree()));
      if( fast_partition != NULL)
      {
        GPS_DEBUG_MSG(( "[main]MEM usage: fast total size %8d, req %8d, free %8d\r\n", gpOS_memory_getheapsize_p( fast_partition), gpOS_memory_getheaprequested_p( fast_partition), gpOS_memory_getheapfree_p( fast_partition)));
      }
      //GPS_DEBUG_MSG(( "[main]Runtime: %8d\r\n", gpOS_task_get_runtime( NULL) ));
      GPS_DEBUG_MSG(( "[main]CS: %8d, IRQs: %d\r\n", xOS_stats_get_context_switch_number(), xOS_stats_get_interrupts_occurred_number() ));
    }
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Main procedure
 *
 * \param void
 * \return int
 *
 ***********************************************/
int main(void)
{
  void *fast_partition_base;
  tU32 fast_partition_size = 0;

  platform_get_fast_partition( &fast_partition_base, &fast_partition_size);

  if( fast_partition_size != 0)
  {
    fast_partition = gpOS_memory_create_partition( gpOS_MEMORY_TYPE_SIMPLE, fast_partition_base, fast_partition_size);
  }
  else
  {
    fast_partition = NULL;
  }

  // Initialize vector interrupt handlers
  vOS_vectors_stack_init(fast_partition);

  // Initialize interrupt hw
  gpOS_bsp_interrupt_handler_init();

  xTaskCreate( main_idle_process, "idle_task_ws", IDLE_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 15, &xMainIdleProcess );
	/* We want to start in supervisor mode. Operation will switch to system mode when the first task starts.*/
	vTaskStartScheduler();

	/* Should never reach here! */
	return 0;
}

/*}}}  */


