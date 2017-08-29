/*********************************************************************************************
   FILE:          POW_app.c
----------------------------------------------------------------------------------------------
   DESCRIPTION: Every 15 seconds, application print out its activity
                Every 10 seconds, GNSS activity perform a fix
----------------------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2016 STMicroelectronics
 **********************************************************************************************/



/*****************************************************************************
   includes
 *****************************************************************************/
#include "POW_app.h"

#include "gnss_data.h"
#include "libs/cJSON.h"
#include "macros.h"       /* to use BACKUP area           */
#include "gnss_api.h"     /* to use GNSS api              */
#include "gnss_defs.h"
#include "gnss_debug.h"   /* to send data to debug port   */
#include "svc_pwr.h"      /* to use power services        */
#include "svc_uart.h"
#include "gpOS_wakelock.h"/* to use wakelock services     */
#include "gnssapp.h"      /* to configure power services  */
#include "clibs.h"        /* to use clibs memory services */
#include "lld_rtc.h"      /* to use RTC                   */
#include "lld_gpio.h"
#include "gps_nvm.h"

/* define DEMO_USE_FREERTOS_API into make file to use FreeRTOS API instead of gpOS API */

#if !defined (DEMO_USE_FREERTOS_API)
#include "gpOS.h"         /* to use OS related functions  */
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
 *****************************************************************************/
#define POW_APP_TASK_STACK_SIZE   10000     /* size of the sample application task stack */
#define POW_RTC_TICKS_PER_SECOND   0x8000  /* RTC frac resolution */

/*****************************************************************************
   global variable definitions
 *****************************************************************************/

tInt pow_app_task_priority = 0;   /* task priority value (value set arbitrary here)*/
tUInt fix_period_s=10;
tUInt report_every_n_fixes = 2;
tUInt COM_PORT = 0;
tUInt MAX_READ_BUFFER_SIZE = 1500;

/* Backup area declaration */
#if defined(__STA8090__)
#pragma arm section zidata = "SRAM_STDBY_DATA"
#endif
SRAM_STDBY_DATA gpOS_clock_t pow_next_timer_activity;     /* Next activity time */
SRAM_STDBY_DATA tUInt        pow_application_activity;    /* Activity counter   */
SRAM_STDBY_DATA tUInt        n_positions;
SRAM_STDBY_DATA Gnss_data    positions[POSITIONS_ARRAY_SIZE];
SRAM_STDBY_DATA Config       config;
SRAM_STDBY_DATA tUInt        testing;
#if defined(__STA8090__)
#pragma arm section zidata
#endif

static pow_manager_t       pow_manager;

/*****************************************************************************
   function prototypes (scope: module-local)
 *****************************************************************************/
/* Function which is called by "POW_app_task" */
static gpOS_task_exit_status_t demo_app_process( void *p );
static void main_activity(tUInt round);

/*****************************************************************************
   function implementations
 *****************************************************************************/

/* Creation of "POW_app_task" */
void pow_app_init(gpOS_partition_t *part)
{

  // Register on service power
  gpOS_wakelock_register( &pow_manager.wakelock_id );

  // Prevent standby entry by acquiring wakelock
  gpOS_wakelock_acquire(pow_manager.wakelock_id);

  /* COLD startup, activate low power mode */
  if( svc_pwr_StartupMode() == SVC_PWR_STARTUP_POWER_ON )
  {
    gnss_low_power_periodic_mode_t  periodic;
    gnss_low_power_cyclic_mode_t  cyclic;
    gnss_app_lowpow_standby_type_t Standby;

    /* Will illustrate the number of wakeup for Application activity */
    pow_application_activity = 0;

    /* Plan activity in POW_APPLICATION_ACTIVITY_PERIOD seconds */
    pow_next_timer_activity = gpOS_time_now();

    n_positions=0;
    testing=0;


    //config.fleet_id[0] = '0';                      // Empty in the COLD STARTUP
    //strcpy(config.fleet_id, "5721489412194304");
    //GPS_DEBUG_MSG( ( "[CLOE_demo] fleet id %s \r\n", config.fleet_id));

    nvm_status_t status;
    void *test ;
    nvm_status_t status2 ;
    Config result;
    nvm_status_t status3;
    char *p ;

    GPS_DEBUG_MSG( ( "[CLOE_demo] Reading current configuration if it exists \r\n"));
    status = nvm_create(128,1,1,sizeof(Config));
    status3 = nvm_copy(128,1, &result);

    if(strlen(result.fleet_id)==16)
    {
      GPS_DEBUG_MSG( ( "[CLOE_demo] Configuration found\r\n"));
      config = result;
    }
    else
    {
      GPS_DEBUG_MSG( ( "[CLOE_demo] No configuration detected \r\n"));
      _clibs_memset(config.fleet_id, 0, sizeof(config.fleet_id));
      _clibs_memset(config.imei, 0, sizeof(config.imei));
      config.fix_period_s = fix_period_s;
      config.report_every_n_fixes = report_every_n_fixes;
    }

    //strcpy(config.imei, "357353080001193");

    GPS_DEBUG_MSG( ( "[CLOE_demo] COLD STARTUP, setup periodic mode with standby \r\n"));

    /* set periodic mode at period of 10 seconds */
    periodic.periodic_mode = TRUE;                  /* < Activate periodic mode           */
    periodic.RTC_refresh = 0;                       /* < No RTC refresh                   */
    periodic.EPH_refresh = 1;                       /* < Ephemeris refresh                */
    periodic.NoFixOffTime = 10;                     /* < wait fix during 60 seconds       */
    periodic.NoFixTimeout = 10;                     /* < if no fix retry after 60 seconds */
    periodic.fix_on_time = 1;                       /* < wait 1 fix                       */
    periodic.fix_period = config.fix_period_s; /* < provide a fix every 25 seconds   */

    GPS_DEBUG_MSG( ( "[CLOE_demo] Provide %d fix every %d seconds \r\n",periodic.fix_on_time,periodic.fix_period));
    GPS_DEBUG_MSG( ( "[CLOE_demo] Application activity synchronized with GNSS \r\n"));


    /* activate standby */
    Standby = GNSSAPP_LOW_POWER_STANDBY_ENABLE;

    /* disable cyclic mode */;
    _clibs_memset(&cyclic,0,sizeof(gnss_low_power_cyclic_mode_t));

    /* Activate GNSS periodic mode for a standby period of 10 seconds */
    gnssapp_low_power_setup_update( Standby , &cyclic, & periodic );               /* periodic setup */

  }
  /* Come back from standby state */
  /* STARTUP_WAKEUP_PIN or STARTUP_WAKEUP_RTC */
  else
  {
    gpOS_clock_t StandbyDuration_rtcbase;
    gpOS_clock_t VirtualPreviousOsTime;
    gpOS_clock_t TimeNow = gpOS_time_now();

    // Update previous timer value from backup ram
    svc_pwr_get_timer_adjustment( &StandbyDuration_rtcbase, &VirtualPreviousOsTime );
    GPS_DEBUG_MSG( ( "\r\n[CLOE_demo] STARTUP - LEAVE STANDBY after %d second(s)-\r\n",StandbyDuration_rtcbase/gpOS_timer_ticks_per_sec()));
  }

  gpOS_task_create_p( part, demo_app_process, NULL, POW_APP_TASK_STACK_SIZE, pow_app_task_priority + 15, "POW_app_task", gpOS_TASK_FLAGS_ACTIVE );

  /* GPIO setup */
  LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN0,(LLD_GPIO_ModeTy)LLD_GPIO_ALTERNATE_NONE);
  LLD_GPIO_SetDirectionInput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN0);

  LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN11,(LLD_GPIO_ModeTy)LLD_GPIO_ALTERNATE_NONE);
  LLD_GPIO_SetDirectionInput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN11);

  LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN13,(LLD_GPIO_ModeTy)LLD_GPIO_ALTERNATE_NONE);
  LLD_GPIO_SetDirectionOutput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN13);
  LLD_GPIO_SetStateLow( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN13 );

  LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN25,(LLD_GPIO_ModeTy)LLD_GPIO_ALTERNATE_MODE_A);
  LLD_GPIO_SetDirectionInput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN25);

  LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN26,(LLD_GPIO_ModeTy)LLD_GPIO_ALTERNATE_NONE);
  LLD_GPIO_SetDirectionInput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( LLD_GPIO_PinTy )LLD_GPIO_PIN26);

  /* Open UART port */
  if(svc_uart_open_port(COM_PORT, gpOS_INTERRUPT_NOPRIORITY, LLD_UART_921600_BPS, 128, 128, 4) == gpOS_FAILURE)
  {
    GPS_DEBUG_MSG(( "[CLOE_demo] Open UART FAILED\r\n"));
  }
  else
  {
    GPS_DEBUG_MSG(( "[CLOE_demo] Open UART OK\r\n"));
  }

}


/* Function which is called by "sample_app_task" */
static gpOS_task_exit_status_t demo_app_process( void *p )
{
  GPS_DEBUG_MSG( ( "[CLOE_demo] Task is running\r\n"));

  GPS_DEBUG_MSG(( "[CLOE_demo] Waiting for modem to become online... \r\n"));
  if(!at_listen_sysstart()){
    report_error(" - +SYSSTART never received");
  }else{
    GPS_DEBUG_MSG(( "[CLOE_demo] +SYSSTART Received \r\n"));
  }

  /*while(TRUE){
    gpOS_task_delay(1*1000*1000*gpOS_timer_ticks_per_usec()); //1 segundo?
    at_showphystat();
    at_showpdn();
  }*/

  gpOS_clock_t std_at_timeout_ms = 100;

  at_disable_echo();
  at_check();
  at_activate_modem();


  if(!at_configure_socket(SOCKET_ID,PDN,0,0,600,50)){
    report_error(" - sqnscfg failed");
  }
  if(!at_extended_configure_socket(SOCKET_ID,1,0,0)){
    report_error(" - sqnscfgext failed");
  }

  while(strlen(config.fleet_id)==0){            // Normally we come from a COLD STARTUP
    GPS_DEBUG_MSG( ( "[CLOE_demo] Activating the tracker... \r\n"));
    at_get_imei(config.imei);
    if(activate_tracker(config.imei)==FALSE || strlen(config.fleet_id)!= 16)
    {
      _clibs_memset(config.fleet_id, 0, sizeof(config.fleet_id));
      GPS_DEBUG_MSG( ( "[CLOE_demo] The tracker couldn't be activated \r\n"));
    }
    else
    {
      GPS_DEBUG_MSG( ( "[CLOE_demo] Activated! \r\n"));
    }

  }

  GPS_DEBUG_MSG( ( "[CLOE_demo] Tracker activated IMEI = %s  and fleet = %s\r\n", config.imei, config.fleet_id));

  //gpOS_task_delay(1*1000*1000*gpOS_timer_ticks_per_usec()); //1 segundo?


  while ( TRUE )
  {
    // Activity processing
    main_activity(pow_application_activity ++);

    // Wait for standby wake-up, generated by GNSS periodic mode
    gpOS_wakelock_release(pow_manager.wakelock_id, gpOS_TIMEOUT_INFINITY );

    // suspend activity with a delay superior to GNSS period.
    gpOS_task_delay( 1.1 * config.fix_period_s * gpOS_timer_ticks_per_sec());

    // Prevent standby entry by acquiring wakelock
    gpOS_wakelock_acquire(pow_manager.wakelock_id);
  }
}
/* Main application activity */
static void main_activity(tUInt round)
{
  tU32 DRI,DRF;
  double ActivityTime;

  // Get RTC time
  LLD_RTC_Get_DRI_DRF_Registers(&DRI,&DRF);
  if(DRF > POW_RTC_TICKS_PER_SECOND)
  {
    DRF = POW_RTC_TICKS_PER_SECOND;
  }
  DRF = POW_RTC_TICKS_PER_SECOND - DRF;

  ActivityTime = ( DRI % 60 ) + (double)DRF/(double)POW_RTC_TICKS_PER_SECOND;

  GPS_DEBUG_MSG( ( "[CLOE_demo] Task activity round %d Time : %.2f s\r\n",round, ActivityTime  ));

  gnss_fix_store_local(NULL);

  if(gnss_fix_get_pos_status() != NO_FIX)
  {
    position_t pos;
    velocity_t vel;
    char buffer[27];
    _clibs_memset(buffer, 0, sizeof(buffer));
    at_get_network_time(buffer);

    gnss_fix_get_fil_pos_vel_local(&pos, &vel, NULL);


    GPS_DEBUG_MSG( ( "[CLOE_demo] Position : %.5f, %.5f detected\r\n",pos.latitude, pos.longitude  ));
    GPS_DEBUG_MSG( ( "[CLOE_demo] Saving... \r\n"  ));
    positions[n_positions].pos=pos;
    parse_date_response(buffer);
    _clibs_strcpy(positions[n_positions].timestamp, buffer);
    GPS_DEBUG_MSG( ( "[CLOE_demo] Position saved \r\n"  ));
    n_positions++;
    if(n_positions%config.report_every_n_fixes==0)
    {
      if(send_positions()==TRUE)
      {
        n_positions=0;
        _clibs_memset(positions, 0, sizeof(positions));
      }
      else
      {
        GPS_DEBUG_MSG( ( "[CLOE_demo] Couldn't send the positions \r\n"  ));
      }
      if(n_positions>=POSITIONS_ARRAY_SIZE){
        n_positions=0;
      }
    }
  }

  else{
    GPS_DEBUG_MSG( ( "[CLOE_demo] No FIX yet \r\n" ));
  }

}

