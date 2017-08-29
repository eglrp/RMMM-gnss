/*****************************************************************************
   FILE:          main.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The main application to run and test STA8090 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 2007.07.25
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"

// LLD for STA8088
#include "lld.h"
#include "lld_rtc.h"
#include "lld_gpio.h"

// OS related
#include "gpOS.h"

// services related
#include "svc_mcu.h"
#include "svc_pwr.h"

// GPS library
#include "gnss_debug.h"

// Platform related
#include "platform.h"

#include "test_io.h"
#include "os20_examples.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/**< OS20 stacks defines */
#define OS20_SYS_MODE_STACK_SIZE    512  /* size of the interrupt stack for all interrupts. This must be the worst case if all interrupt happen concurrently */
#define OS20_IRQ_MODE_STACK_SIZE    512  /* size of IRQ mode */
#define OS20_SVC_MODE_STACK_SIZE    256  /* size of SVC mode */
#define OS20_ABORT_MODE_STACK_SIZE  1024  /* size of ABORT mode */
#if defined( VFP_SUPPORT)
#define OS20_UND_MODE_STACK_SIZE    1024  /* size of UND mode */
#else
#define OS20_UND_MODE_STACK_SIZE    0     /* size of SVC mode */
#endif
#define OS20_IDLE_TASK_STACK_SIZE   256   /* size of the idle task stack */
#if defined( __GNUC__)
#define OS20_ROOT_TASK_STACK_SIZE   (2048+768)  /* The main() stack size in bytes */
#else
#define OS20_ROOT_TASK_STACK_SIZE   (2048)  /* The main() stack size in bytes */
#endif

/* set up a pragma to enforce no semi hosting SWI's */
#ifndef ARM_SEMIHOSTING_ON
#if defined (__ARMCC_VERSION)
#if ((__ARMCC_VERSION >= 120000) && (__ARMCC_VERSION < 200000))
#pragma import(__use_no_semihosting_swi)
#elif (__ARMCC_VERSION >= 310000)
#pragma import(__use_no_semihosting)
#endif
#endif
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

// Root task size
const tUInt os20_root_task_stack_size = OS20_ROOT_TASK_STACK_SIZE;

tDouble slv_ticks_per_second = 1023000;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

// Idle task data
gpOS_task_t *os20_idle_task;

static  gpOS_partition_t *fast_partition;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/* callback for svc_pwr */
extern gpOS_error_t rtc_drv_rtc_accurate_read( tUInt *rtc_int_val, tUInt *rtc_fra_val, gpOS_clock_t *cpu_time, tUInt accurate_tps)
{
  tU32 fractional;

  gpOS_interrupt_lock();

  *cpu_time = gpOS_time_now();
  LLD_RTC_Get_DRI_DRF_Registers((tU32 *)rtc_int_val, &fractional);

  *rtc_fra_val = 0x8000U - fractional;

  gpOS_interrupt_unlock();

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief idle task process
 *
 * \param param dummy parameter
 * \return
 *
 ***********************************************/
static gpOS_task_exit_status_t main_os20_idle_process( void* param)
{
  boolean_t exit_flag = FALSE;

  while( exit_flag == FALSE)
  {
    svc_mcu_enter_wfi();
  }

  // should never reach this
  return -1;
}

/********************************************//**
 * \brief Starts OS20
 *
 * \param void
 * \return gpOS_SUCCESS if all ok, else gpOS_FAILURE
 *
 ***********************************************/
static tInt main_os20_start( void *fast_partition_base, tU32 fast_partition_size)
{
  tUInt kernel_stacks[] = {
    OS20_SVC_MODE_STACK_SIZE,
    OS20_UND_MODE_STACK_SIZE,
    OS20_ABORT_MODE_STACK_SIZE,
    OS20_IRQ_MODE_STACK_SIZE,
    0
  };

  gpOS_bsp_timer_cfg_t timer_bsp_cfg = { LLD_MTU_PRESCALER_DIV_1, 0};

  if( fast_partition_size != 0)
  {
    fast_partition = gpOS_memory_create_partition( gpOS_MEMORY_TYPE_SIMPLE, fast_partition_base, fast_partition_size);
  }
  else
  {
    fast_partition = NULL;
  }

  if( gpOS_kernel_init( fast_partition, kernel_stacks) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;
  };

  gpOS_interrupt_init( fast_partition, OS20_SYS_MODE_STACK_SIZE);

  // Initialize OS services and set available bus speeds
  svc_mcu_init( NULL);
  platform_set_tcxo_speed( TCXO_26MHZ);
  svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_MCLK, 26000000U);
  svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_USB_CLK, 26000000U);
  svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_CAN_CLK, 26000000U);

  gpOS_timer_init( &timer_bsp_cfg);

  gpOS_timer_start();

  gpOS_kernel_start();

  os20_idle_task = gpOS_task_create( main_os20_idle_process, NULL, OS20_IDLE_TASK_STACK_SIZE, gpOS_TASK_MIN_USR_PRIORITY, "OS20_idle_task_ws", gpOS_TASK_FLAGS_ACTIVE);

  if( os20_idle_task == NULL)
  {
    return gpOS_FAILURE;
  }

  gpOS_kernel_set_timeslice( FALSE);
  return gpOS_SUCCESS;
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Main procedure
 *
 * \param void
 * \return tInt
 *
 ***********************************************/
tInt main(void)
{
  void *fast_partition_base;
  tU32 fast_partition_size = 0;
  wakelockid_t wakelock_main;

  platform_get_fast_partition( &fast_partition_base, &fast_partition_size);

  /**< Starts OS20 */
  if( main_os20_start( fast_partition_base, fast_partition_size) == gpOS_FAILURE)
  {
    return( LLD_ERROR);
  }

  /**< Initialize platform basic peripherals */
  if( platform_config() == gpOS_FAILURE)
  {
    return( LLD_ERROR);
  }

  /**< Initialize fundamental drivers */
  if( platform_start_services( (gpOS_partition_t *)fast_partition) == gpOS_FAILURE)
  {
    return( LLD_ERROR);
  }

  svc_pwr_wakelock_register(&wakelock_main);
  svc_pwr_wakelock_acquire(wakelock_main);

  /**< Start test I/O */
  if( test_io_init( (gpOS_partition_t *)fast_partition) == gpOS_FAILURE)
  {
    return( LLD_ERROR);
  }

  /**< Start Os20 Examples */
  if( os20_examples_init( (gpOS_partition_t *)fast_partition) == gpOS_FAILURE)
  {
    return( LLD_ERROR);
  }


  #if defined( OS20_MEMORYALLOCATIONCHECK )
  {
    extern tUInt os20_memory_pointers_table[];
    extern tUInt os20_memory_requests_table[];
    extern tUInt os20_memory_reqtype_table[];
    extern tUInt os20_memory_requests_idx;
    tUInt cnt;

    for( cnt = 0; cnt < os20_memory_requests_idx; cnt++)
    {
      if( os20_memory_reqtype_table[cnt] == 0)
      {
        GPS_DEBUG_MSG(( "[main]MEM: allocate %3d, ptr %8d, size %8d\r\n", cnt, os20_memory_pointers_table[cnt], os20_memory_requests_table[cnt]));
      }
      else
      {
        GPS_DEBUG_MSG(( "[main]MEM: request %3d, ptr %8d, size %8d\r\n", cnt, os20_memory_pointers_table[cnt], os20_memory_requests_table[cnt]));
      }
    }
  }
  #endif

  gpOS_task_set_priority( gpOS_kernel_get_active_task(), 4);

  /* Do nothing - introduced to let Navigate start and acquire a wakelock */
  gpOS_task_delay( 10*gpOS_timer_ticks_per_msec());

  svc_pwr_wakelock_release(wakelock_main,SVC_PWR_NO_NEXT_ACTIVITY_PLANNED);

  /* Do dummy stuff */
  while(TRUE)
  {
    /* Do nothing */
    gpOS_task_delay( gpOS_timer_ticks_per_sec());

    {
      tInt clk_source, clk_divisor;

      platform_get_clock_configuration_status(&clk_source, &clk_divisor);

      GPS_DEBUG_MSG(( "[main]CPU usage: %3.2f clk src %d divisor %d \r\n", (tDouble)(svc_mcu_getcpuusage()) / 100.0, clk_source, clk_divisor));
      GPS_DEBUG_MSG(( "[main]MEM usage: heap total size %8d, req %8d, free %8d\r\n", gpOS_memory_getheapsize(), gpOS_memory_getheaprequested(), gpOS_memory_getheapfree()));
      if( fast_partition_size > 0)
      {
        GPS_DEBUG_MSG(( "[main]MEM usage: fast total size %8d, req %8d, free %8d\r\n", gpOS_memory_getheapsize_p( fast_partition), gpOS_memory_getheaprequested_p( fast_partition), gpOS_memory_getheapfree_p( fast_partition)));
      }
      GPS_DEBUG_MSG(( "[main]Runtime: %8d\r\n", gpOS_task_get_runtime( NULL) ));  /*lint !e9080 : integer null pointer constant used [MISRA 2012 Rule 11.9, required]*/
      GPS_DEBUG_MSG(( "[main]CS: %8d, IRQs num: %d, IRQs time: %d\r\n", gpOS_kernel_get_context_switch_number(), gpOS_kernel_get_interrupts_occurred_number(), gpOS_kernel_get_interrupts_spent_time() ));
    }
  }
}

/*}}}  */


