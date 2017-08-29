/*****************************************************************************
   FILE:          rtcm_plugins.c
   PROJECT:       STA2062 GPS application
   SW PACKAGE:    STA2062 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The main application to run and test STA2062 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 201
*****************************************************************************/

/*!
 * @file    rtcm_plugins.c
 * @brief   Plugin for RTCM differential module
 */

/*****************************************************************************
   includes
*****************************************************************************/

#if defined( RTCM_LINKED)

#include "gpOS.h"
#include "gnssapp.h"
#include "gnssapp_plugins.h"
#include "gnss_debug.h"
#include "dgps.h"
#include "sw_config.h"
#include "in_out.h"

#ifdef RTCM3_ENC_LINKED
#include "gnss_events.h"
#include "rtcm3_enc.h"
#endif

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
#ifdef RTCM3_ENC_LINKED

typedef enum
{
  RTCM3_ENC_NMEA_CMDID_SET_MTI,
  RTCM3_ENC_NMEA_CMDID_NUMBER,
} rtcm3_enc_nmea_cmdid_t;

static const tChar * const rtcm3_enc_nmea_cmd_strings[] =
{
  "$PSTMRTCMESETMTI",
  NULL
};

typedef tU32    (*rtcm3_enc_inout_t)      ( tChar *, tU32, gpOS_clock_t*);

typedef struct plugin_rtcm3_enc_manager_s {

  gnss_events_synch_handler_t *outmsg_synchdlr_ptr;
  rtcm3_enc_inout_t            ioport_write;
  boolean_t                    out_processing_enabled;

} plugin_rtcm3_enc_manager_t;

#endif

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
#ifdef RTCM3_ENC_LINKED
static plugin_rtcm3_enc_manager_t plugin_rtcm3_enc_mgr;
#endif

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

#ifdef RTCM3_ENC_LINKED
static tInt         dgps_plugin_rtcm3_enc_cmdif_parse        ( tChar *, tUInt, tChar *);
static gpOS_error_t dgps_plugin_rtcm3_enc_init               ( gpOS_partition_t *part);
static void         dgps_plugin_rtcm3_enc_output_processing  ( void);

static void rtcm3_enc_nmea_cmdif_exec_set_mti( tChar *cmd_par );

static const nmea_support_cmdif_exec_t rtcm3_enc_nmea_cmdif_exec_table[] =
{
  rtcm3_enc_nmea_cmdif_exec_set_mti,
  NULL
};

#endif

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

static gpOS_error_t dgps_plugin_init( gpOS_partition_t *part)
{
  gpOS_error_t error = gpOS_SUCCESS;

  if( sw_config_get_software_switch_status( RTCM_ON_OFF_SWITCH ) )
  {
    tInt rtcm_port = 0, nmea_port = 0, debug_port = 0, debug_mode = 0;
    sw_config_get_param(CURRENT_CONFIG_DATA,NMEA_PORT_NUMBER_ID,&nmea_port);
    sw_config_get_param(CURRENT_CONFIG_DATA,RTCM_PORT_NUMBER_ID,&rtcm_port);
    sw_config_get_param(CURRENT_CONFIG_DATA,DEBUG_PORT_NUMBER_ID,&debug_port);
    sw_config_get_param(CURRENT_CONFIG_DATA,GPS_DEBUG_MODE_ID,&debug_mode);


    if((rtcm_port == nmea_port) || ((rtcm_port == debug_port) && ((debug_mode & SWCFG_NMEA_INPUT_ON_DEBUG) != 0)))
    {
      dgps_start( NULL, FALSE);
      nmea_set_cmdif_msg_forward_callback((nmea_cmdif_msg_forward_callback_t)dgps_add_new_data);
      nmea_rtcm_input_chan_enable((tInt)(rtcm_port == debug_port));
    }
    #if ! defined(__linux__)
    else
    {
      dgps_read_t rtcm_input;

      in_out_get_rtcm_cfg( &rtcm_input);
      dgps_start( rtcm_input, TRUE);
    }
    #endif
  }

  #ifdef RTCM3_ENC_LINKED
  if(sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, RTCM3_ENC_ON_OFF_SWITCH ) != 0)
  {
    error = dgps_plugin_rtcm3_enc_init( part);

    if((error == gpOS_SUCCESS) && (sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, NMEA_IN_OUT_INTERFACE_SWITCH ) != 0))
    {
      nmea_set_if_mode_ext( NMEA_INTERNAL_IF_MODE, NMEA_EXTERNAL_IF_MODE);
    }
  }
  #endif

  return error;
}

static gpOS_error_t dgps_plugin_suspend( void)
{
  return gpOS_SUCCESS;
}

static gpOS_error_t dgps_plugin_restart( void)
{
  return gpOS_SUCCESS;
}

static gpOS_error_t dgps_plugin_api( const gnssapp_plugins_cmd_t cmd, gnssapp_plugins_cmd_param_t *param)
{
  gpOS_error_t error = gpOS_SUCCESS;

  switch( cmd)
  {
    case GNSSAPP_PLUGINS_CMD_INIT:
      error = dgps_plugin_init( ( gpOS_partition_t *)param->data_ptr);
      break;

    case GNSSAPP_PLUGINS_CMD_GETVER:
      break;

    case GNSSAPP_PLUGINS_CMD_START:
      error = dgps_plugin_restart();
      break;

    case GNSSAPP_PLUGINS_CMD_SUSPEND:
      error = dgps_plugin_suspend();
      break;

    case GNSSAPP_PLUGINS_CMD_CUSTOM:

    default:
      error = gpOS_FAILURE;
      break;
  }

  return error;
}


#ifdef RTCM3_ENC_LINKED

static void dgps_plugin_rtcm3_enc_write_callback(const tUChar *buff, const tU32 len)
{
  plugin_rtcm3_enc_mgr.ioport_write( (tChar *)buff, len, gpOS_TIMEOUT_INFINITY);
}

static void dgps_plugin_rtcm3_enc_output_processing(void)
{
  gnss_events_install( GNSS_EVENTID_FIXREADY, plugin_rtcm3_enc_mgr.outmsg_synchdlr_ptr);

  GPS_DEBUG_MSG( ("[rtcm3e] gnss_events_install ok\r\n" ) );

  while(plugin_rtcm3_enc_mgr.out_processing_enabled == TRUE)
  {
    gnss_events_wait(GNSS_EVENTID_FIXREADY, plugin_rtcm3_enc_mgr.outmsg_synchdlr_ptr);

    if(plugin_rtcm3_enc_mgr.out_processing_enabled != FALSE)  // Check if disabled while waiting the event
    {
      rtcm3_enc_send_configured_msg( 0xFFFFFFFF, dgps_plugin_rtcm3_enc_write_callback);
    }
  }

  gnss_events_uninstall( GNSS_EVENTID_FIXREADY, plugin_rtcm3_enc_mgr.outmsg_synchdlr_ptr);
}

static gpOS_error_t dgps_plugin_rtcm3_enc_init( gpOS_partition_t *part)
{
  nmea_inout_t nmea_input, nmea_output;

  plugin_rtcm3_enc_mgr.outmsg_synchdlr_ptr = gnss_events_synch_handler_create();

  if( plugin_rtcm3_enc_mgr.outmsg_synchdlr_ptr == GNSS_EVENTS_SYNCHANDLER_NONE)
  {
    return(gpOS_FAILURE);  /*lint !e904 Return statement before end of function */
  }

  in_out_get_nmea_cfg( &nmea_input, &nmea_output);

  plugin_rtcm3_enc_mgr.ioport_write = (rtcm3_enc_inout_t)nmea_output;

  if(rtcm3_enc_init_p(part) == GNSS_ERROR)
  {
    ERROR_MSG("[rtcm3e]: Error - Init failed\r\n");
    return(gpOS_FAILURE);
  }

  //nmea_set_external_cmdif_callback((nmea_external_cmdif_callback_t)NULL);
  nmea_set_external_outmsg_callback((nmea_external_outmsg_callback_t)dgps_plugin_rtcm3_enc_output_processing);

  plugin_rtcm3_enc_mgr.out_processing_enabled = TRUE;

  GPS_DEBUG_MSG( ("[rtcm3e] plugin init OK\r\n" ) );

  return(gpOS_SUCCESS);
}

static tInt dgps_plugin_rtcm3_enc_cmdif_parse( tChar *cmd, tUInt cmd_size, tChar *cmd_par)
{
  tUInt cmd_id;

  if( nmea_support_cmdif_getid( cmd, cmd_size, rtcm3_enc_nmea_cmd_strings, RTCM3_ENC_NMEA_CMDID_NUMBER, &cmd_id) == NMEA_NOT_VALID)
  {
    return NMEA_NOT_VALID;
  }

  rtcm3_enc_nmea_cmdif_exec_table[cmd_id]( cmd_par );

  return NMEA_OK;
}

static void rtcm3_enc_nmea_cmdif_exec_set_mti( tChar *cmd_par )
{
  tInt field_count, index = 0;
  tInt arg0,arg1;

  field_count = _clibs_sscanf(cmd_par,",%d,%d",&arg0,&arg1);

  if( field_count == 2)
  {
    rtcm3_enc_set_mti( (tU16)arg0, (tS16)arg1);
  }
}
#endif


/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

gnssapp_plugins_handler_t dgps_gnssapp_plugin_handler =
{
  NULL,
  GNSSAPP_PLUGINS_ID_RTCM,
  dgps_plugin_api,
  #ifdef RTCM3_ENC_LINKED
  dgps_plugin_rtcm3_enc_cmdif_parse,
  #else
  NULL,
  #endif
  NULL,
  (gnssapp_plugins_stbin_cmdif_callback_t)NULL,
  (gnssapp_plugins_stbin_outmsg_callback_t)NULL
};

#endif
