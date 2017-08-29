/*****************************************************************************
   FILE:          gnssapp_plugins.c
------------------------------------------------------------------------------
   DESCRIPTION:   Plugin for gnssapp
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on :
*****************************************************************************/

/*!
 * @file    gnssapp_plugins.c
 * @brief   Plugin for gnssapp
 */

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"

#include "gpOS.h"
#include "gnss_debug.h"
#include "gnssapp_plugins.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef struct
{
  gpOS_partition_t *          part;
  gnssapp_plugins_handler_t * first;
  gnssapp_plugins_handler_t * last;
} gnssapp_plugins_manager_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static gnssapp_plugins_manager_t gnssapp_plugins_manager;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

static void gnssapp_plugins_install( gnssapp_plugins_handler_t *new_plugin_handler)
{
  if( gnssapp_plugins_manager.first == NULL)
  {
    gnssapp_plugins_manager.first = new_plugin_handler;
    gnssapp_plugins_manager.last = new_plugin_handler;
  }
  else
  {
    gnssapp_plugins_manager.last->next = new_plugin_handler;
    gnssapp_plugins_manager.last = new_plugin_handler;
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

gpOS_error_t gnssapp_plugins_init( gpOS_partition_t *part)
{
  gnssapp_plugins_cmd_param_t gpparam;
  gnssapp_plugins_handler_t *plugin_handler;

  gnssapp_plugins_manager.part    = part;
  gnssapp_plugins_manager.first   = NULL;
  gnssapp_plugins_manager.last    = NULL;

  {
    extern gnssapp_plugins_handler_t nmea_ext_gnssapp_plugin_handler;

    gnssapp_plugins_install( &nmea_ext_gnssapp_plugin_handler);
  }

  #if defined RTCM_LINKED
  {
    extern gnssapp_plugins_handler_t dgps_gnssapp_plugin_handler;

    gnssapp_plugins_install( &dgps_gnssapp_plugin_handler);
  }
  #endif

  #if defined WAAS_LINKED
  {
    extern gnssapp_plugins_handler_t waas_gnssapp_plugin_handler;

    gnssapp_plugins_install( &waas_gnssapp_plugin_handler);
  }
  #endif

  #if defined ST_AGPS
  {
    extern gnssapp_plugins_handler_t stagps_gnssapp_plugin_handler;

    gnssapp_plugins_install( &stagps_gnssapp_plugin_handler);
  }
  #endif

  #if defined(DR_CODE_LINKED) && !defined(DR_DISABLE)
  {
    extern gnssapp_plugins_handler_t dr_gnssapp_plugin_handler;

    gnssapp_plugins_install( &dr_gnssapp_plugin_handler);
  }
  #endif

  gpparam.data_ptr  = (void *)part;

  plugin_handler = gnssapp_plugins_manager.first;

  while( plugin_handler != NULL)
  {
    if( plugin_handler->api_handler( GNSSAPP_PLUGINS_CMD_INIT, &gpparam) == gpOS_FAILURE)
    {
      return gpOS_FAILURE;
    }

    plugin_handler = plugin_handler->next;
  }

  return gpOS_SUCCESS;
}

gpOS_error_t gnssapp_plugins_get_nvm_size( tUInt *nvm_size_ptr)
{
  gpOS_error_t error = gpOS_FAILURE;

  #if defined ST_AGPS
  #if !defined (STAGPS_USE_OLD_PGPS_SEED)
  *nvm_size_ptr = (2 * 128 * 1024);
  #else
  *nvm_size_ptr = (2 * 128 * 1024);
  #endif
  error = gpOS_SUCCESS;
  #endif

  return error;
}

gpOS_error_t gnssapp_plugins_start( void)
{
  gnssapp_plugins_handler_t *plugin_handler;

  plugin_handler = gnssapp_plugins_manager.first;

  while( plugin_handler != NULL)
  {
    plugin_handler->api_handler( GNSSAPP_PLUGINS_CMD_START, NULL);

    plugin_handler = plugin_handler->next;
  }

  return gpOS_SUCCESS;
}

gpOS_error_t gnssapp_plugins_suspend( void)
{
  gnssapp_plugins_handler_t *plugin_handler;

  plugin_handler = gnssapp_plugins_manager.first;

  while( plugin_handler != NULL)
  {
    plugin_handler->api_handler( GNSSAPP_PLUGINS_CMD_SUSPEND, NULL);

    plugin_handler = plugin_handler->next;
  }

  return gpOS_SUCCESS;
}

gpOS_error_t gnssapp_plugins_nmea_outmsg_transmit( void *param)
{
  gnssapp_plugins_handler_t *plugin_handler;

  plugin_handler = gnssapp_plugins_manager.first;

  while( plugin_handler != NULL)
  {
    if( plugin_handler->outmsg_handler != NULL)
    {
      plugin_handler->outmsg_handler( param);
    }

    plugin_handler = plugin_handler->next;
  }

  return gpOS_SUCCESS;
}



tInt gnssapp_plugins_nmea_cmdif_parse( tChar *input_cmd_msg, tUInt cmd_size, tChar *par_ptr)
{
  gnssapp_plugins_handler_t *plugin_handler;

  plugin_handler = gnssapp_plugins_manager.first;

  while( plugin_handler != NULL)
  {
    if( plugin_handler->cmdif_handler != NULL)
    {
      if( plugin_handler->cmdif_handler(input_cmd_msg, cmd_size, par_ptr) == NMEA_OK)
      {
        return NMEA_OK;
      }
    }

    plugin_handler = plugin_handler->next;
  }

  return NMEA_NOT_VALID;
}

gpOS_error_t gnssapp_plugins_cmd( const gnssapp_plugins_id_t id, const gnssapp_plugins_cmd_t cmd, gnssapp_plugins_cmd_param_t *param)
{
  gnssapp_plugins_handler_t *plugin_handler;

  plugin_handler = gnssapp_plugins_manager.first;

  while( plugin_handler != NULL)
  {
    if( plugin_handler->id == id)
    {
      return plugin_handler->api_handler( cmd, param);
    }

    plugin_handler = plugin_handler->next;
  }

  return gpOS_FAILURE;
}

#if defined( STBIN_LINKED )
stbin_status_t gnssapp_plugins_stbin_cmdif_parse(tU16 msg_length,
                                                tUChar *msg,
                                                const stbin_req_msg_parms *parms)
{
  gnssapp_plugins_handler_t *plugin_handler;
  stbin_status_t rc = STBIN_NOTFOUND;

  plugin_handler = gnssapp_plugins_manager.first;

  while((STBIN_NOTFOUND == rc) && (plugin_handler != NULL))
  {
    if( plugin_handler->stbin_cmdif_handler != NULL)
      rc = plugin_handler->stbin_cmdif_handler(msg_length, msg, parms);

    plugin_handler = plugin_handler->next;
  }

  return rc;
}

void gnssapp_plugins_stbin_outmsg_transmit(const tU32 *msg_list,
                                            const stbin_outProcess_input *in)
{
  gnssapp_plugins_handler_t *plugin_handler;

  plugin_handler = gnssapp_plugins_manager.first;

  while(plugin_handler != NULL)
  {
    if( plugin_handler->stbin_outmsg_handler != NULL)
      plugin_handler->stbin_outmsg_handler(msg_list, in);

    plugin_handler = plugin_handler->next;
  }
}
#endif

