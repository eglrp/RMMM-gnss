//!
//!   \file     in_out.c
//!   \brief    <i><b> Input/output management source file</b></i>
//!   \author   Giovanni De Angelis
//!   \version  1.0
//!   \date     2012.02.15
//!   \bug      Unknown
//!   \warning  None
//!

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"
#include "in_out.h"

// OS related
#include "gpOS.h"
#include "svc_uart.h"

#if defined( USB_LINKED )
#include "svc_usb.h"
#include "svc_gpio.h"
#endif

// GPS library
#include "gnss_debug.h"

#include "platform.h"
#include "sw_config.h"
#include "nmea.h"

#if defined( __STA8088__ )
#include "lld_clk_ctrl_sta8088.h"
#endif
#if defined( __STA8090__ )
#include "lld_prcc_sta8090.h"
#endif

#if defined( SDLOG_LINKED )
#include "sdlog.h"
#include "lld_gpio.h"
#endif

// RTCM related
#ifdef RTCM_LINKED
#include "dgps.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*! \cond internal_docs */

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define GPS_DEBUG_RX_FIFO_SIZE    1024
#define GPS_DEBUG_TX_FIFO_SIZE    1024
#define NMEA_RX_FIFO_SIZE         1024
#define NMEA_TX_FIFO_SIZE         1024
#define RTCM_RX_FIFO_SIZE         1024
#define RTCM_TX_FIFO_SIZE         1024

#if defined( __STA8088__ ) || defined( __STA8090__ )
#define IN_OUT_EXTSAL
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef tU32 (*in_out_t)( tUInt, tChar *, tU32, gpOS_clock_t *);

typedef struct
{
  gpOS_partition_t *part;

  tUInt nmea_id;
  tUInt debug_id;
  tUInt rtcm_id;

  in_out_t nmea_read_func;
  in_out_t nmea_write_func;
  in_out_t debug_write_func;
  in_out_t debug_read_func;
  in_out_t rtcm_read_func;

  nmea_inout_t          nmea_input_func;
  nmea_inout_t          nmea_output_func;
  gnss_debug_writeout_t debug_output_func;
  gnss_debug_inout_t    debug_input_func;
  dgps_read_t           rtcm_input_func;

  tU8           output_cfg;
  tU8           cpu_clock_speed;
  tU8           debug_port_mode;
  tU8           rtcm_mode;

  #ifdef SDLOG_LINKED
  sdlog_file_t * gnss_debug_file_hdlr;
  sdlog_file_t * nmea_file_hdlr;
  #endif

  #ifdef USB_LINKED
  tU32 usb_det_pin_id;

  gpOS_semaphore_t *  usb_detect_sem;
  gpOS_semaphore_t *  nmea_read_access_sem;
  gpOS_semaphore_t *  nmea_write_access_sem;
  gpOS_semaphore_t *  debug_read_access_sem;
  gpOS_semaphore_t *  debug_write_access_sem;
  gpOS_task_t *       usb_detect_task;
  #endif
} in_out_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

extern tU8 nmea_com;
extern tU8 gnss_debug_com;
extern tU8 rtcm_com;

extern tU32 nmea_port_baud_rate;
extern tU32 debug_port_baud_rate;
extern tU32 rtcm_port_baud_rate;

tU8 stream_task_priority = 4;

extern tU8 usb_det_configuration;
extern tU8 usb_det_pin;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static in_out_handler_t *in_out_handler;


/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static tU32 in_out_nmea_extern_read     ( tChar *buf, tU32 len, gpOS_clock_t *);
static tU32 in_out_nmea_extern_write    ( tChar *buf, tU32 len, gpOS_clock_t *);
static tU32 in_out_debug_extern_write   ( tChar *buf, tU32 len, gpOS_clock_t *);
static tU32 in_out_debug_extern_read    ( tChar *buf, tU32 len, gpOS_clock_t *);
static tU32 in_out_dummy_read           ( tChar *buf, tU32 len, gpOS_clock_t *);

#if defined( __STA2064__ ) || defined( __STA8088__ ) || defined( __STA8090__ )
static tU32 in_out_dummy_write          ( tChar *buf, tU32 len, gpOS_clock_t *);
#endif

#ifdef SDLOG_LINKED
static tU32 in_out_nmea_file_read       ( tChar *buf, tU32 len, gpOS_clock_t *);
static tU32 in_out_nmea_file_write      ( tChar *buf, tU32 len, gpOS_clock_t *);
static tU32 in_out_debug_file_write     ( tChar *buf, tU32 len, gpOS_clock_t *);
#endif

#if defined( RTCM_LINKED )
static tU32 in_out_rtcm_extern_read     ( tChar *buf, tU32 len, gpOS_clock_t *);
#endif

#ifdef USB_LINKED
static LLD_GPIO_StateTy         usb_init_detect_pin   ( void);
static LLD_GPIO_StateTy         usb_check_detect_pin  ( void);

static void                     usb_detect_callback   ( const void * param);
static gpOS_task_exit_status_t  usb_detect_process    ( gpOS_task_param_t dummy);
#endif

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
static tU32 in_out_nmea_extern_read( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 read_chars = 0;
#if defined( USB_LINKED)
  gpOS_clock_t read_timeout;

  gpOS_semaphore_wait( in_out_handler->nmea_read_access_sem);

  if(timeout == gpOS_TIMEOUT_INFINITY)
  {
     read_timeout = gpOS_time_plus( gpOS_time_now(), NAV_CPU_TICKS_PER_SECOND);
  }
  else
  {
    read_timeout = *timeout;
  }

  read_chars = in_out_handler->nmea_read_func( in_out_handler->nmea_id, buf, len, &read_timeout);

  gpOS_semaphore_signal( in_out_handler->nmea_read_access_sem);

#else
  read_chars = in_out_handler->nmea_read_func( in_out_handler->nmea_id, buf, len, gpOS_TIMEOUT_INFINITY);
#endif

  return read_chars;
}

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
static tU32 in_out_nmea_extern_write( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 written_chars = 0;
#if defined( USB_LINKED)
  gpOS_clock_t write_timeout;

  gpOS_semaphore_wait( in_out_handler->nmea_write_access_sem);

  if(timeout == gpOS_TIMEOUT_INFINITY)
  {
    write_timeout = gpOS_time_plus( gpOS_time_now(), NAV_CPU_TICKS_PER_SECOND);
  }
  else
  {
    write_timeout = *timeout;
  }

  written_chars = in_out_handler->nmea_write_func( in_out_handler->nmea_id, buf, len, &write_timeout);

  gpOS_semaphore_signal( in_out_handler->nmea_write_access_sem);

#else
  written_chars = in_out_handler->nmea_write_func( in_out_handler->nmea_id, buf, len, gpOS_TIMEOUT_INFINITY);
#endif

  return written_chars;
}

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
static tU32 in_out_debug_extern_read( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 read_chars = 0;
#if defined( USB_LINKED)
  gpOS_clock_t read_timeout;

  gpOS_semaphore_wait( in_out_handler->debug_read_access_sem);

  if(timeout == gpOS_TIMEOUT_INFINITY)
  {
     read_timeout = gpOS_time_plus( gpOS_time_now(), NAV_CPU_TICKS_PER_SECOND);
  }
  else
  {
    read_timeout = *timeout;
  }

  read_chars = in_out_handler->debug_read_func( in_out_handler->debug_id, buf, len, &read_timeout);

  gpOS_semaphore_signal( in_out_handler->debug_read_access_sem);

#else
  read_chars = in_out_handler->debug_read_func( in_out_handler->debug_id, buf, len, gpOS_TIMEOUT_INFINITY);
#endif

  return read_chars;
}

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
GNSS_FAST static tU32 in_out_debug_extern_write( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 written_chars = 0;
#if defined( USB_LINKED)
  gpOS_clock_t write_timeout;

  gpOS_semaphore_wait( in_out_handler->debug_write_access_sem);

  if(timeout == gpOS_TIMEOUT_INFINITY)
  {
    write_timeout = gpOS_time_plus( gpOS_time_now(), NAV_CPU_TICKS_PER_SECOND);
  }
  else
  {
    write_timeout = *timeout;
  }

  written_chars = in_out_handler->debug_write_func( in_out_handler->debug_id, buf, len, &write_timeout);

  gpOS_semaphore_signal( in_out_handler->debug_write_access_sem);

#else
  written_chars = in_out_handler->debug_write_func( in_out_handler->debug_id, buf, len, gpOS_TIMEOUT_INFINITY);
#endif

  return written_chars;
}

/********************************************//**
 * \brief
 *
 * \param buf tChar*
 * \param len tU32
 * \return void
 *
 ***********************************************/
static tU32 in_out_dummy_read( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 read_chars = 0;
  /* Do nothing */
  gpOS_task_delay( gpOS_timer_ticks_per_sec());

  return read_chars;
}

/********************************************//**
 * \brief
 *
 * \param buf tChar*
 * \param len tU32
 * \return void
 *
 ***********************************************/
#if defined( __STA2064__ ) || defined( __STA8088__ ) || defined( __STA8090__ )
static tU32 in_out_dummy_write( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 written_chars = 0;

  return written_chars;
}
#endif

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
#ifdef SDLOG_LINKED
static tU32 in_out_nmea_file_read( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 read_chars = 0;

  gpOS_task_delay( gpOS_timer_ticks_per_msec());
  *buf = 0x0;

  return read_chars;
}

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
static tU32 in_out_nmea_file_write( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 written_chars = 0;

  written_chars = sdlog_write( in_out_handler->nmea_file_hdlr, buf, len);

  return written_chars;
}

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
static tU32 in_out_debug_file_read( tUInt id, tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 read_chars = 0;

  gpOS_task_delay( gpOS_timer_ticks_per_msec());
  *buf = 0x0;

  return read_chars;
}

/********************************************//**
 * \brief Callback called by NMEA to read chars
 *
 * \param buf pointer to char buffer
 * \param len number of chars to read
 * \return void
 *
 ***********************************************/
static tU32 in_out_debug_file_write( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 written_chars = 0;

  written_chars = sdlog_write( in_out_handler->gnss_debug_file_hdlr, buf, len);

  return written_chars;
}
#endif

/********************************************//**
 * \brief Callback passed for DGPS module configuration
 *        used to read chars.
 *
 * \param buf Destination buffer pointer
 * \param len Characters to read
 * \return void
 *
 ***********************************************/
#if defined( RTCM_LINKED )
static tU32 in_out_rtcm_extern_read( tChar *buf, tU32 len, gpOS_clock_t *timeout)
{
  tU32 read_chars = 0;

  read_chars = in_out_handler->rtcm_read_func( in_out_handler->rtcm_id, buf, len, timeout);

  return read_chars;
}
#endif

/********************************************//**
 * \brief
 *
 * \param void
 * \return void
 *
 ***********************************************/
#ifdef USB_LINKED
static LLD_GPIO_StateTy usb_init_detect_pin( void)
{
  LLD_GPIO_StateTy  usb_det_status;
  tU32 usb_det_pin_id;

  usb_det_pin_id = ( tU32)( usb_det_pin & 0x3F);

  if( usb_det_pin_id < 32)
  {
    svc_gpio_open_port( SVC_GPIO_PORT_0, gpOS_INTERRUPT_NOPRIORITY);
    svc_gpio_interrupt_install( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)usb_det_pin_id, usb_detect_callback);
    svc_gpio_pin_enable( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)usb_det_pin_id);
    svc_gpio_interrupt_enable( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)usb_det_pin_id, SVC_GPIO_RISING_AND_FALLING_EDGE);

    LLD_GPIO_ReadPin( usb_det_pin_id, &usb_det_status);
  }
  else if( usb_det_pin_id < 64)
  {
    svc_gpio_open_port( SVC_GPIO_PORT_1, gpOS_INTERRUPT_NOPRIORITY);
    svc_gpio_interrupt_install( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)( usb_det_pin_id - 32), usb_detect_callback);
    svc_gpio_pin_enable( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)( usb_det_pin_id - 32));
    svc_gpio_interrupt_enable( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)( usb_det_pin_id - 32), SVC_GPIO_RISING_AND_FALLING_EDGE);

    LLD_GPIO_ReadPin( usb_det_pin_id, &usb_det_status);
  }

  in_out_handler->usb_det_pin_id = usb_det_pin_id;

  return usb_det_status;
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return LLD_GPIO_StateTy
 *
 ***********************************************/
static LLD_GPIO_StateTy usb_check_detect_pin( void)
{
  LLD_GPIO_StateTy status;

  LLD_GPIO_ReadPin( in_out_handler->usb_det_pin_id, &status);

  return status;
}

/********************************************//**
 * \brief
 *
 * \param hdlr_ptr svc_gpio_port_handler_t*
 * \return void
 *
 ***********************************************/
static LLD_ISR_GPIO void usb_detect_callback( const void * param)
{
  gpOS_semaphore_signal( in_out_handler->usb_detect_sem);
}

/********************************************//**
 * \brief
 *
 * \param dummy gpOS_task_param_t
 * \return gpOS_task_exit_status_t
 *
 ***********************************************/
static gpOS_task_exit_status_t usb_detect_process( gpOS_task_param_t dummy)
{
  boolean_t exit_flag = FALSE;
  LLD_GPIO_StateTy usb_det_status;

  while( exit_flag == FALSE)
  {
    gpOS_semaphore_wait( in_out_handler->usb_detect_sem);

    usb_det_status = usb_check_detect_pin();

    // USB detect line is high
    if( usb_det_status == LLD_GPIO_HIGH)
    {
      // Enable PLL if it was OFF
      if(( in_out_handler->cpu_clock_speed & 0x0F) == 0x02)
      {
        #if defined( __STA8088__ )
        platform_set_cpu_clock_speed( 0, LLD_CLK_CTRL_CLKSRC_PLL, LLD_CLK_CTRL_CLKSEL_52);
        #endif
        #if defined( __STA8090__ )
        // Do nothing! STA8090 does not have PLL
        #endif
      }

      // Debug stream is set on USB
      if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_DEBUG_USB)
      {
        // Try to open USB VCOM. If failed reset original CPU clock speed
        if( svc_usb_open_vcom( 0, gpOS_INTERRUPT_NOPRIORITY, GPS_DEBUG_TX_FIFO_SIZE, GPS_DEBUG_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
        {
          if(( in_out_handler->cpu_clock_speed & 0x0F) == 0x02)
          {
            #if defined( __STA8088__)
            platform_set_cpu_clock_speed( 0, LLD_CLK_CTRL_CLKSRC_TWICE, 0);
            #endif
            #if defined( __STA8090__)
            platform_set_cpu_clock_speed( LLD_PRCC_ARMCLKSRC_192f0, LLD_PRCC_CLKSEL_48);
            #endif
          }
        }
        else
        {
          gpOS_task_lock();

          in_out_handler->debug_write_func = ( in_out_t)svc_usb_write;
          in_out_handler->debug_read_func = ( in_out_t)svc_usb_read;
          in_out_handler->debug_id = 0;

          gpOS_task_unlock();
        }
      }
      // Debug stream is set on USB
      else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_NMEA_USB)
      {
        // Try to open USB VCOM. If failed reset original CPU clock speed
        if( svc_usb_open_vcom( 0, gpOS_INTERRUPT_NOPRIORITY, NMEA_TX_FIFO_SIZE, NMEA_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
        {
          if(( in_out_handler->cpu_clock_speed & 0x0F) == 0x02)
          {
            #if defined( __STA8088__ )
            platform_set_cpu_clock_speed( 0, LLD_CLK_CTRL_CLKSRC_TWICE, 0);
            #endif
            #if defined( __STA8090__ )
            platform_set_cpu_clock_speed( LLD_PRCC_ARMCLKSRC_192f0, LLD_PRCC_CLKSEL_48);
            #endif
          }
        }
        else
        {
          gpOS_task_lock();

          in_out_handler->nmea_write_func = ( in_out_t)svc_usb_write;
          in_out_handler->nmea_read_func = ( in_out_t)svc_usb_read;
          in_out_handler->nmea_id = 0;

          gpOS_task_unlock();
        }
      }
    }
    // USB detect line is low
    else
    {
      // Debug stream is set on USB
      if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_DEBUG_USB)
      {
        gpOS_task_lock();

        in_out_handler->debug_write_func = ( in_out_t)svc_uart_write;
        in_out_handler->debug_read_func = ( in_out_t)svc_uart_read;
        in_out_handler->debug_id = gnss_debug_com;

        gpOS_task_unlock();

        svc_usb_close_vcom( 0);

        // Reset original CPU clock speed
        if(( in_out_handler->cpu_clock_speed & 0x0F) == 0x02)
        {
          #if defined( __STA8088__ )
          platform_set_cpu_clock_speed( 0, LLD_CLK_CTRL_CLKSRC_TWICE, 0);
          #endif
          #if defined( __STA8090__)
          platform_set_cpu_clock_speed( LLD_PRCC_ARMCLKSRC_192f0, LLD_PRCC_CLKSEL_48);
          #endif
        }
      }
      // Debug stream is set on USB
      else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_NMEA_USB)
      {
        gpOS_task_lock();

        in_out_handler->nmea_write_func = ( in_out_t)svc_uart_write;
        in_out_handler->nmea_read_func = ( in_out_t)svc_uart_read;
        in_out_handler->nmea_id = nmea_com;

        gpOS_task_unlock();

        svc_usb_close_vcom( 0);

        // Reset original CPU clock speed
        if(( in_out_handler->cpu_clock_speed & 0x0F) == 0x02)
        {
          #if defined( __STA8088__ )
          platform_set_cpu_clock_speed( 0, LLD_CLK_CTRL_CLKSRC_TWICE, 0);
          #endif
          #if defined( __STA8090__)
          platform_set_cpu_clock_speed( LLD_PRCC_ARMCLKSRC_192f0, LLD_PRCC_CLKSEL_48);
          #endif
        }
      }
    }
  }
  // should never reach this
  return -1;
}
#endif

/*! \endcond */

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param
 * \param
 * \return
 *
 ***********************************************/
gpOS_error_t in_out_start( gpOS_partition_t *part)
{
  boolean_t uart_needed = FALSE;
#ifdef SDLOG_LINKED
  boolean_t sdcard_needed = FALSE;
#endif
#ifdef USB_LINKED
  boolean_t usb_needed = FALSE;
#endif

  #if defined( IN_OUT_EXTSAL )
  boolean_t sal_ext_sqi;

  sal_ext_sqi = platform_get_sal_ext_sqi_status();
  #endif

  in_out_handler = gpOS_memory_allocate_p( part, sizeof( in_out_handler_t));

  if( in_out_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  _clibs_memset( in_out_handler, 0x0, sizeof( in_out_handler_t));

  in_out_handler->part = part;
  #ifdef USB_LINKED
  in_out_handler->nmea_read_access_sem = gpOS_semaphore_create_p( SEM_FIFO, part, 1);
  if( in_out_handler->nmea_read_access_sem == NULL)
  {
    return gpOS_FAILURE;
  }

  in_out_handler->nmea_write_access_sem = gpOS_semaphore_create_p( SEM_FIFO, part, 1);
  if( in_out_handler->nmea_write_access_sem == NULL)
  {
    return gpOS_FAILURE;
  }

  in_out_handler->debug_read_access_sem = gpOS_semaphore_create_p( SEM_FIFO, part, 1);
  if( in_out_handler->debug_read_access_sem == NULL)
  {
    return gpOS_FAILURE;
  }

  in_out_handler->debug_write_access_sem = gpOS_semaphore_create_p( SEM_FIFO, part, 1);
  if( in_out_handler->debug_write_access_sem == NULL)
  {
    return gpOS_FAILURE;
  }
  #endif

  // Read current CPU clock and output redirection configuration
  sw_config_get_param( CURRENT_CONFIG_DATA, OUTPUT_CFG_ID, &in_out_handler->output_cfg);
  sw_config_get_param( CURRENT_CONFIG_DATA, CPU_CLOCK_SPEED, &in_out_handler->cpu_clock_speed);
  sw_config_get_param( CURRENT_CONFIG_DATA, GPS_DEBUG_MODE_ID, &in_out_handler->debug_port_mode);
  in_out_handler->rtcm_mode = sw_config_get_software_switch_status( RTCM_ON_OFF_SWITCH);

  if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_DEBUG_UART)
  {
    // If hw is a SAL with external SQI UART0 is not available
    #if defined( IN_OUT_EXTSAL )
    if(( sal_ext_sqi == TRUE) && ( gnss_debug_com == 0))
    {
      in_out_handler->debug_id          = gnss_debug_com;
      in_out_handler->debug_output_func = in_out_dummy_write;
      in_out_handler->debug_input_func  = in_out_dummy_read;
    }
    else
    #endif
    {
      in_out_handler->debug_write_func  = ( in_out_t)svc_uart_write;
      in_out_handler->debug_read_func   = ( in_out_t)svc_uart_read;
      in_out_handler->debug_id          = gnss_debug_com;
      in_out_handler->debug_output_func = in_out_debug_extern_write;
      in_out_handler->debug_input_func  = in_out_debug_extern_read;
      uart_needed = TRUE;
    }
  }
#ifdef SDLOG_LINKED
  else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_DEBUG_FILE)
  {
    in_out_handler->debug_output_func   = in_out_debug_file_write;
    in_out_handler->debug_read_func     = in_out_debug_file_read;
    sdcard_needed = TRUE;
  }
#endif
#ifdef USB_LINKED
  else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_DEBUG_USB)
  {
    in_out_handler->debug_write_func    = ( in_out_t)svc_usb_write;
    in_out_handler->debug_read_func     = ( in_out_t)svc_usb_read;
    in_out_handler->debug_id            = 0;
    in_out_handler->debug_output_func   = in_out_debug_extern_write;
    in_out_handler->debug_input_func    = in_out_debug_extern_read;
    usb_needed = TRUE;
  }
#endif

  if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_NMEA_UART)
  {
    // If hw is a SAL with external SQI UART0 is not available
    #if defined( IN_OUT_EXTSAL )
    if(( sal_ext_sqi == TRUE) && ( nmea_com == 0))
    {
      in_out_handler->nmea_id           = nmea_com;
      in_out_handler->nmea_output_func  = in_out_dummy_write;
      in_out_handler->nmea_input_func   = in_out_dummy_read;
    }
    else
    #endif
    {
      in_out_handler->nmea_write_func   = ( in_out_t)svc_uart_write;
      in_out_handler->nmea_read_func    = ( in_out_t)svc_uart_read;
      in_out_handler->nmea_id           = nmea_com;
      in_out_handler->nmea_output_func  = in_out_nmea_extern_write;
      in_out_handler->nmea_input_func   = in_out_nmea_extern_read;
      uart_needed = TRUE;
    }
  }
#ifdef SDLOG_LINKED
  else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_NMEA_FILE)
  {
    in_out_handler->nmea_output_func    = in_out_nmea_file_write;
    in_out_handler->nmea_input_func     = in_out_nmea_file_read;
    sdcard_needed = TRUE;
  }
#endif
#ifdef USB_LINKED
  else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_NMEA_USB)
  {
    in_out_handler->nmea_write_func     = ( in_out_t)svc_usb_write;
    in_out_handler->nmea_read_func      = ( in_out_t)svc_usb_read;
    in_out_handler->nmea_id             = 0;
    in_out_handler->nmea_output_func    = in_out_nmea_extern_write;
    in_out_handler->nmea_input_func     = in_out_nmea_extern_read;
    usb_needed = TRUE;
  }
#endif

#if defined( RTCM_LINKED )
  if( sw_config_get_software_switch_status( RTCM_ON_OFF_SWITCH ) )
  {
    // If hw is a SAL with external SQI UART0 is not available
    #if defined( IN_OUT_EXTSAL )
    if(( sal_ext_sqi == TRUE) && ( rtcm_com == 0))
    #else
    if( rtcm_com == 0)
    #endif
    {
      in_out_handler->rtcm_input_func = in_out_dummy_read;
    }
    else
    {
      in_out_handler->rtcm_read_func  = (in_out_t)svc_uart_read;
      in_out_handler->rtcm_id         = rtcm_com;
      in_out_handler->rtcm_input_func = ( dgps_read_t)in_out_rtcm_extern_read;
      uart_needed = TRUE;
    }
  }
#endif

#ifdef SDLOG_LINKED
  if( sdcard_needed == TRUE)
  {
    if(sdlog_init( part, 0) == gpOS_FAILURE)
    {
      in_out_handler->nmea_write_func   = ( in_out_t)svc_uart_write;
      in_out_handler->nmea_read_func    = ( in_out_t)svc_uart_read;
      in_out_handler->debug_write_func  = ( in_out_t)svc_uart_write;
      in_out_handler->debug_read_func   = ( in_out_t)svc_uart_read;
      in_out_handler->nmea_id           = nmea_com;
      in_out_handler->debug_id          = gnss_debug_com;
      in_out_handler->nmea_output_func  = in_out_nmea_extern_write;
      in_out_handler->nmea_input_func   = in_out_nmea_extern_read;
      in_out_handler->debug_output_func = in_out_debug_extern_write;
      in_out_handler->debug_input_func   = in_out_debug_extern_read;
      uart_needed = TRUE;
    }
    else
    {
      if( in_out_handler->debug_output_func == in_out_debug_file_write)
      {
        in_out_handler->gnss_debug_file_hdlr = sdlog_open_file( "/ST_logs", "dbg");

        if( in_out_handler->gnss_debug_file_hdlr == NULL)
        {
          return gpOS_FAILURE;
        }
      }

      if( in_out_handler->nmea_output_func == in_out_nmea_file_write)
      {
        in_out_handler->nmea_file_hdlr = sdlog_open_file( "/ST_logs", "nmea");

        if( in_out_handler->nmea_file_hdlr == NULL)
        {
          return gpOS_FAILURE;
        }
      }
    }
  }
#endif

  /**< If USB is needed, initialize service and coms */
#ifdef USB_LINKED
  if( usb_needed == TRUE)
  {
    LLD_GPIO_StateTy status = LLD_GPIO_LOW;

    svc_usb_init( part , PLATFORM_BUSCLK_ID_USB_CLK);

    // USB detect feature is enabled
    if( usb_det_configuration == 1)
    {
      uart_needed = 1;

      // Create semaphore to signal USB detect pin status is changed
      in_out_handler->usb_detect_sem = gpOS_semaphore_create_p( SEM_FIFO, in_out_handler->part, 0);

      if( in_out_handler->usb_detect_sem == NULL)
      {
        gpOS_semaphore_delete( in_out_handler->usb_detect_sem);
        return gpOS_FAILURE;
      }

      // Create task process to handle USB detect event
      in_out_handler->usb_detect_task = gpOS_task_create_p( part, usb_detect_process, NULL, 1024, 11, "In/Out update Task", gpOS_TASK_FLAGS_ACTIVE);

      if( in_out_handler->usb_detect_task == NULL)
      {
        gpOS_task_delete( in_out_handler->usb_detect_task);
        gpOS_semaphore_delete( in_out_handler->usb_detect_sem);
        return gpOS_FAILURE;
      }

      // Read current status of USB detect pin
      status = usb_init_detect_pin();

      // Check if USB detect pin status is low
      if( status == LLD_GPIO_LOW)
      {
        if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_DEBUG_USB)
        {
          in_out_handler->debug_write_func  = ( in_out_t)svc_uart_write;
          in_out_handler->debug_read_func   = ( in_out_t)svc_uart_read;
          in_out_handler->debug_id          = gnss_debug_com;
          in_out_handler->debug_output_func = in_out_debug_extern_write;
          in_out_handler->debug_input_func  = in_out_debug_extern_read;
        }
        else if( in_out_handler->output_cfg & SWCFG_OUTPUT_CFG_NMEA_USB)
        {
          in_out_handler->nmea_write_func   = ( in_out_t)svc_uart_write;
          in_out_handler->nmea_read_func    = ( in_out_t)svc_uart_read;
          in_out_handler->nmea_id           = nmea_com;
          in_out_handler->nmea_output_func  = in_out_nmea_extern_write;
          in_out_handler->nmea_input_func   = in_out_nmea_extern_read;
        }
      }
    }
    // USB detect feature is NOT enabled
    if(( usb_det_configuration == 0) || ( status == LLD_GPIO_HIGH))
    {
      // Enable PLL if it was OFF
      if(( in_out_handler->cpu_clock_speed & 0x0F) == 0x02)
      {
        #if defined( __STA8088__ )
        platform_set_cpu_clock_speed( 0, LLD_CLK_CTRL_CLKSRC_PLL, LLD_CLK_CTRL_CLKSEL_52);
        #endif
        #if defined( __STA8090__)
        // Do nothing! STA8090 does not have PLL
        #endif
      }

      if( in_out_handler->debug_write_func == ( in_out_t)svc_usb_write)
      {
        if( svc_usb_open_vcom( 0, gpOS_INTERRUPT_NOPRIORITY, GPS_DEBUG_TX_FIFO_SIZE, GPS_DEBUG_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
        {
          return( gpOS_FAILURE);
        }
      }
      else if( in_out_handler->nmea_write_func == ( in_out_t)svc_usb_write)
      {
        if( svc_usb_open_vcom( 0, gpOS_INTERRUPT_NOPRIORITY, NMEA_TX_FIFO_SIZE, NMEA_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
        {
          return( gpOS_FAILURE);
        }
      }
    }
  }
  else
  {
    #if defined( __STA8088__  )
    LLD_CLK_CTRL_PeripheralClkDis( LLD_CLK_CTRL_PERIPHID_UART1);
    #elif defined( __STA8090__ )
    LLD_PRCC_PeripheralClkDis( LLD_PRCC_PERIPHID_UART1);
    #endif
  }
#endif

  /**< If UART is needed, initialize service and coms */
  if( uart_needed == TRUE)
  {
    svc_uart_init( part, PLATFORM_BUSCLK_ID_MCLK);

#if defined( USB_LINKED )
    if(( in_out_handler->debug_write_func == ( in_out_t)svc_uart_write) || ( in_out_handler->debug_write_func == ( in_out_t)svc_usb_write))
#else
    if(( in_out_handler->debug_write_func == ( in_out_t)svc_uart_write))
#endif
    {
      if((( in_out_handler->debug_port_mode & 0x0F )== 0) || (( in_out_handler->debug_port_mode & SWCFG_NMEA_INPUT_ON_DEBUG) != 0) || (( in_out_handler->debug_port_mode & SWCFG_DUAL_NMEA_PORT_ENABLE) != 0))
      {
        if( svc_uart_open_port( gnss_debug_com, gpOS_INTERRUPT_NOPRIORITY, (LLD_UART_BaudRateTy)debug_port_baud_rate, GPS_DEBUG_TX_FIFO_SIZE, GPS_DEBUG_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
        {
          return( gpOS_FAILURE);
        }
      }
    }

#if defined( USB_LINKED )
    if(( in_out_handler->nmea_write_func == ( in_out_t)svc_uart_write) || ( in_out_handler->nmea_write_func == ( in_out_t)svc_usb_write))
#else
    if(( in_out_handler->nmea_write_func == ( in_out_t)svc_uart_write))
#endif
    {
      if( svc_uart_open_port( nmea_com, gpOS_INTERRUPT_NOPRIORITY, (LLD_UART_BaudRateTy)nmea_port_baud_rate, NMEA_TX_FIFO_SIZE, NMEA_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
      {
        return( gpOS_FAILURE);
      }
    }

#if defined( RTCM_LINKED )
    if(((( in_out_handler->debug_output_func != in_out_debug_extern_write) || ( gnss_debug_com != rtcm_com)) && ( nmea_com != rtcm_com) && ( in_out_handler->rtcm_input_func == in_out_rtcm_extern_read ))
	    || (( gnss_debug_com == rtcm_com) && !((( in_out_handler->debug_port_mode & 0x0F )== 0) || (( in_out_handler->debug_port_mode & SWCFG_NMEA_INPUT_ON_DEBUG) != 0) || (( in_out_handler->debug_port_mode & SWCFG_DUAL_NMEA_PORT_ENABLE) != 0))))
    {
      if( in_out_handler->rtcm_mode == 1)
      {
        if( svc_uart_open_port( rtcm_com, gpOS_INTERRUPT_NOPRIORITY, (LLD_UART_BaudRateTy)rtcm_port_baud_rate, RTCM_TX_FIFO_SIZE, RTCM_RX_FIFO_SIZE, stream_task_priority) == gpOS_FAILURE)
        {
          return( gpOS_FAILURE);
        }
      }
    }
#endif
  }
  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param port in_out_port_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t in_out_open_port( in_out_port_t port)
{
  gpOS_error_t error = gpOS_SUCCESS;

  switch( port)
  {
    case IN_OUT_DEBUG_PORT:
      if(( in_out_handler->debug_write_func == ( in_out_t)svc_uart_write))
      {
        error = svc_uart_open_port( gnss_debug_com, gpOS_INTERRUPT_NOPRIORITY, LLD_UART_921600_BPS, GPS_DEBUG_TX_FIFO_SIZE, GPS_DEBUG_RX_FIFO_SIZE, stream_task_priority);
      }
#if defined( USB_LINKED )
      else if(( in_out_handler->debug_write_func == ( in_out_t)svc_usb_write))
      {
        error = svc_usb_open_vcom( 0, gpOS_INTERRUPT_NOPRIORITY, GPS_DEBUG_TX_FIFO_SIZE, GPS_DEBUG_RX_FIFO_SIZE, stream_task_priority);
      }
#endif
      break;

    default:
      error = gpOS_FAILURE;
      break;
  }

  return error;
}

/********************************************//**
 * \brief
 *
 * \param port in_out_port_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t in_out_close_port( in_out_port_t port)
{
  gpOS_error_t error = gpOS_SUCCESS;

  switch( port)
  {
    case IN_OUT_DEBUG_PORT:
      if(( in_out_handler->debug_write_func == ( in_out_t)svc_uart_write))
      {
        error = svc_uart_close_port( gnss_debug_com);
      }
#if defined( USB_LINKED )
      else if(( in_out_handler->debug_write_func == ( in_out_t)svc_usb_write))
      {
        error = svc_usb_close_vcom( 0);
      }
#endif
      break;

    default:
      error = gpOS_FAILURE;
      break;
  }
  return error;
}

gpOS_error_t in_out_get_debug_cfg( gnss_debug_inout_t *debug_input_func, gnss_debug_writeout_t *debug_output_func)
{
  if( in_out_handler == NULL)
  {
    *debug_output_func = NULL;
    *debug_input_func = NULL;
    return gpOS_FAILURE;
  }

  *debug_input_func = in_out_handler->debug_input_func;
  *debug_output_func = in_out_handler->debug_output_func;

  return gpOS_SUCCESS;
}

gpOS_error_t in_out_get_nmea_cfg( nmea_inout_t *nmea_input_func, nmea_inout_t *nmea_output_func)
{
  if( in_out_handler == NULL)
  {
    *nmea_input_func = NULL;
    *nmea_output_func = NULL;
    return gpOS_FAILURE;
  }

  *nmea_input_func = in_out_handler->nmea_input_func;
  *nmea_output_func = in_out_handler->nmea_output_func;

  return gpOS_SUCCESS;
}

gpOS_error_t in_out_get_rtcm_cfg( dgps_read_t *rtcm_input_func)
{
  if( in_out_handler == NULL)
  {
    *rtcm_input_func = NULL;
    return gpOS_FAILURE;
  }

  *rtcm_input_func = in_out_handler->rtcm_input_func;

  return gpOS_SUCCESS;
}

#ifdef __cplusplus
}
#endif

// End of file
