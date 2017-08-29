/*!
 * @file    nmea_ext_plugins.c
 * @brief   Plugin for extended NMEA features
 */

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"
#include "gpOS.h"
#include "gnssapp.h"
#include "gnssapp_plugins.h"
#include "gnss_debug.h"
#include "waas.h"
#include "sw_config.h"
#include "nmea_support.h"
#include "nmea_ext.h"

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

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

static gpOS_error_t nmea_ext_plugin_handle_output_msg( void *param)
{
  return nmea_ext_handle_output_msg( (nmea_support_ext_params_t *)param);
}

static tInt nmea_ext_plugin_cmdif_parse( tChar *input_cmd_msg, tUInt cmd_size, tChar *cmd_par)
{
  return nmea_ext_cmdif_parse( input_cmd_msg, cmd_size, cmd_par);
}

static gpOS_error_t nmea_ext_plugin_api( const gnssapp_plugins_cmd_t cmd, gnssapp_plugins_cmd_param_t *param)
{
  gpOS_error_t error = gpOS_SUCCESS;

  switch( cmd)
  {
    case GNSSAPP_PLUGINS_CMD_INIT:
      if( nmea_ext_init( (gpOS_partition_t *)param->data_ptr) == NMEA_ERROR)
      {
        error = gpOS_FAILURE;
      }
      break;

    case GNSSAPP_PLUGINS_CMD_GETVER:
      break;

    case GNSSAPP_PLUGINS_CMD_START:
      break;

    case GNSSAPP_PLUGINS_CMD_SUSPEND:
      break;

    case GNSSAPP_PLUGINS_CMD_CUSTOM:

    default:
      error = gpOS_FAILURE;
      break;
  }

  return error;
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

gnssapp_plugins_handler_t nmea_ext_gnssapp_plugin_handler =
{
  NULL,
  GNSSAPP_PLUGINS_ID_NMEA_EXT,
  nmea_ext_plugin_api,
  nmea_ext_plugin_cmdif_parse,
  nmea_ext_plugin_handle_output_msg,
  (gnssapp_plugins_stbin_cmdif_callback_t)NULL,
  (gnssapp_plugins_stbin_outmsg_callback_t)NULL
};

