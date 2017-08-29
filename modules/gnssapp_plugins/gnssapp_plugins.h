/*****************************************************************************
   FILE:          gnssapp_plugins.h
------------------------------------------------------------------------------
   DESCRIPTION:   Plugin for gnssapp
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 2007.07.25
*****************************************************************************/

#ifndef GNSSAPP_PLUGINS
#define GNSSAPP_PLUGINS

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gnss_defs.h"
#include "nmea_support.h"
#include "stbin.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define GNSSAPP_PLUGINS_NUMBER                  5

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef tInt           (*gnssapp_plugins_nmea_cmdif_callback_t)   ( tChar *, tUInt, tChar *);
typedef gpOS_error_t   (*gnssapp_plugins_nmea_outmsg_callback_t)  ( void *);
typedef stbin_status_t (*gnssapp_plugins_stbin_cmdif_callback_t)  ( tU16, tUChar *, const stbin_req_msg_parms *);
typedef void           (*gnssapp_plugins_stbin_outmsg_callback_t) ( const tU32 *, const stbin_outProcess_input *);

typedef enum
{
  GNSSAPP_PLUGINS_ID_NMEA_EXT,
  GNSSAPP_PLUGINS_ID_WAAS,
  GNSSAPP_PLUGINS_ID_RTCM,
  GNSSAPP_PLUGINS_ID_STAGPS,
  GNSSAPP_PLUGINS_ID_DR,
  GNSSAPP_PLUGINS_ID_PLITE
} gnssapp_plugins_id_t;

typedef enum
{
  GNSSAPP_PLUGINS_CMD_INIT,
  GNSSAPP_PLUGINS_CMD_GETVER,
  GNSSAPP_PLUGINS_CMD_START,
  GNSSAPP_PLUGINS_CMD_SUSPEND,
  GNSSAPP_PLUGINS_CMD_CMDIF_PARSE,
  GNSSAPP_PLUGINS_CMD_OUTMSG_TRANSMIT,
  GNSSAPP_PLUGINS_CMD_CUSTOM
} gnssapp_plugins_cmd_t;

typedef struct
{
  tUInt  id;
  void *        data_ptr;
} gnssapp_plugins_cmd_param_t;

typedef gpOS_error_t (*gnssapp_plugin_api_t)( const gnssapp_plugins_cmd_t, gnssapp_plugins_cmd_param_t *);

struct gnssapp_plugins_handler_s;

typedef struct gnssapp_plugins_handler_s
{
  struct gnssapp_plugins_handler_s *        next;

  gnssapp_plugins_id_t                      id;
  gnssapp_plugin_api_t                      api_handler;
  gnssapp_plugins_nmea_cmdif_callback_t     cmdif_handler;
  gnssapp_plugins_nmea_outmsg_callback_t    outmsg_handler;
  gnssapp_plugins_stbin_cmdif_callback_t    stbin_cmdif_handler;
  gnssapp_plugins_stbin_outmsg_callback_t   stbin_outmsg_handler;
} gnssapp_plugins_handler_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   gnssapp_plugins_init          ( gpOS_partition_t *);
extern gpOS_error_t   gnssapp_plugins_get_nvm_size  ( tUInt *);
extern gpOS_error_t   gnssapp_plugins_start         ( void);
extern gpOS_error_t   gnssapp_plugins_suspend       ( void);
extern gpOS_error_t   gnssapp_plugins_cmd           ( const gnssapp_plugins_id_t id, const gnssapp_plugins_cmd_t cmd, gnssapp_plugins_cmd_param_t *param);

extern gpOS_error_t   gnssapp_plugins_nmea_outmsg_transmit    ( void *);
extern gpOS_error_t   gnssapp_plugins_nmea_outmsg2_transmit   ( void *);
extern tInt           gnssapp_plugins_nmea_cmdif_parse        ( tChar *, tUInt, tChar *);
extern stbin_status_t gnssapp_plugins_stbin_cmdif_parse       ( tU16 msg_length, tUChar *msg, const stbin_req_msg_parms *parms);
extern void           gnssapp_plugins_stbin_outmsg_transmit   ( const tU32 *msg_list, const stbin_outProcess_input *in);

#endif /* GNSSAPP_PLUGINS */
