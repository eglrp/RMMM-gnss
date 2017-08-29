/*!
 * @file    waas_plugins.c
 * @brief   Plugin for WAAS/EGNOS/SBAS module
 */

/*****************************************************************************
   includes
*****************************************************************************/

#if defined( WAAS_LINKED)

#include "clibs.h"
#include "gpOS.h"
#include "gnssapp.h"
#include "gnssapp_plugins.h"
#include "gnss_debug.h"
#include "waas.h"
#include "sw_config.h"
#include "nmea.h"
#include "nmea_support.h"
#include "waas_plugin.h"

#if defined( STBIN_LINKED )
#include "stbin_waas_plugin.h"
#endif

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SBAS_VERBOSE_NMEA 0

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  WAAS_NMEA_CMDID_SETSTATUS,
  WAAS_NMEA_CMDID_SETSAT,
  WAAS_NMEA_CMDID_SETSATLIST,
  WAAS_NMEA_CMDID_SETPERIODICBACKUP,
  WAAS_NMEA_CMDID_FORCEBACKUP,
  WAAS_NMEA_CMDID_TEST,
  WAAS_NMEA_CMDID_MULTICHAN,
  WAAS_NMEA_CMDID_AUTOSEARCH,

  WAAS_NMEA_CMDID_NUMBER,
} waas_nmea_cmdid_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static const tChar * const waas_nmea_cmd_strings[] =
{
  "$PSTMSBASONOFF",
  "$PSTMSBASSAT",
  "$PSTMSBASSATLIST",
  "$PSTMSBASPBACKUPONOFF",
  "$PSTMSBASFORCEBACKUP",
  "$PSTMSBASTEST",
  "$PSTMSBASMCH",
  "$PSTMSBASAUTOSEARCH",

  NULL
};

static const tSbasInfo waas_nmea_sbas_list[SLOT_PER_SERVICE * SBAS_PROVIDER_NUMBER] =
{
  { EGNOS     ,   1U,  120,  -15.5},
  { EGNOS     ,   1U,  124,   21.5},
  { EGNOS     ,   1U,  126,   64.0},
  { NO_SERVICE,   0U,    0,    0.0},

  { WAAS      ,   1U,  122, -142.0},
  { WAAS      ,   1U,  134,  178.0},
  { WAAS      ,   1U,  135, -133.0},
  { WAAS      ,   1U,  138, -107.3},

  { MSAS      ,   1U,  129,  140.0},
  { MSAS      ,   0U,  137,    0.0},
  { NO_SERVICE,   0U,    0,    0.0},
  { NO_SERVICE,   0U,    0,    0.0},
};

static waas_plugin_handler_t *waas_plugin_handler;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static tInt      waas_plugin_nmea_cmdif_parse            ( tChar *, tUInt, tChar *);

static void     waas_nmea_cmdif_exec_setstatus          ( tChar *);
static void     waas_nmea_cmdif_exec_setsat             ( tChar *);
static void     waas_nmea_cmdif_exec_setsatlist         ( tChar *);
static void     waas_nmea_cmdif_exec_setperiodicbackup  ( tChar *);
static void     waas_nmea_cmdif_exec_forcebackup        ( tChar *);
static void     waas_nmea_cmdif_exec_test               ( tChar *);
static void     waas_nmea_cmdif_exec_multichan          ( tChar *);
static void     waas_nmea_cmdif_exec_autosearch         ( tChar *);

static gpOS_error_t   waas_plugin_nmea_outmsg_transmit    ( void *param);

static void         waas_nmea_outmsg_send_SBAS          ( void *);
static void         waas_nmea_outmsg_send_DIFF          ( void);
static void         waas_nmea_outmsg_send_SBASMCH       ( chanid_t);
static void         waas_nmea_outmsg_send_SBASMSG       ( waas_msg_dump_message_t *);
static void         waas_nmea_outmsg_send_SBASM         ( waas_msg_dump_message_t *);
static boolean_t    waas_nmea_get_sbas_message          ( waas_msg_dump_message_t *);
static void         waas_nmea_flush_sbas_msgs           ( void);

#if SBAS_VERBOSE_NMEA
static void         waas_nmea_outmsg_send_SBASSATCORR   ( void);
static void         waas_nmea_outmsg_send_SBASGEO       ( void);
static void         waas_nmea_outmsg_send_SBASSATPAR    ( void);
#endif

static gpOS_error_t waas_plugin_get_sat_params        ( nmea_support_sat_params_t *);

static const nmea_support_cmdif_exec_t waas_nmea_cmdif_exec_table[] =
{
  waas_nmea_cmdif_exec_setstatus,
  waas_nmea_cmdif_exec_setsat,
  waas_nmea_cmdif_exec_setsatlist,
  waas_nmea_cmdif_exec_setperiodicbackup,
  waas_nmea_cmdif_exec_forcebackup,
  waas_nmea_cmdif_exec_test,
  waas_nmea_cmdif_exec_multichan,
  waas_nmea_cmdif_exec_autosearch,

  NULL
};

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief   Initialize WAAS plugin
 *
 * \param   gpinit  Plugin parameters structure
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
static gpOS_error_t waas_plugin_init( gpOS_partition_t *part)
{
  waas_plugin_handler = (waas_plugin_handler_t *)gpOS_memory_allocate_p( part, sizeof( waas_plugin_handler_t));

  if( waas_plugin_handler == NULL)
  {
    ERROR_MSG( "[waas]: ERROR waas_plugin_init failed\r\n" );
    return( gpOS_FAILURE );
  }

  waas_plugin_handler->sat_id                 = WAAS_SATELLITE;
  waas_plugin_handler->status                 = WAAS_STATUS_OFF;
  waas_plugin_handler->debug_integrity_check  = FALSE;

  if( waas_init_p( part) == GNSS_ERROR )
  {
    ERROR_MSG( "[main]: ERROR waas_plugin_init failed\r\n" );
    return( gpOS_FAILURE );
  }

  {
    tInt waas_sat_id = 0;
    tUInt sbas_sat_enabled_mask = 0;
    tUInt sbas_autosearch_timeout1 = 0;
    tUInt sbas_autosearch_timeout2 = 0;
    tSatelliteParams sbas_sat1;
    tSatelliteParams sbas_sat2;

    sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_PRN_ID, &waas_sat_id );

    waas_set_status(( gnss_waas_status_t )sw_config_get_software_switch_status( WAAS_ON_OFF_SWITCH ) );

    waas_set_satellite( waas_sat_id );

    if(sw_config_get_software_switch_status( WAAS_AUTOSEARCH_ON_OFF_SWITCH ))
    {
      tInt last_decoded_sat;
      if(waas_autosearch_get_last_decoded_sat((tU16*)&last_decoded_sat)== GNSS_NO_ERROR)
      {
        waas_set_satellite( last_decoded_sat );
      }
      waas_enable_autosearch();
    }
    else
    {
      waas_disable_autosearch();
    }

    sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITES_ENABLE_MASK_ID, &sbas_sat_enabled_mask );

    sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_AUTO_SEARCH_TIMEOUT_1_ID, &sbas_autosearch_timeout1 );

    sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_AUTO_SEARCH_TIMEOUT_2_ID, &sbas_autosearch_timeout2 );

    sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_1_ID, &(sbas_sat1.param));

    GPS_DEBUG_MSG(( "sat_id_1 %d\r\n",sbas_sat1.SatParam.sat_id ) );
    if (WAAS_SAT_ID_VALID(sbas_sat1.SatParam.sat_id) == TRUE)
    {
      tDouble longitude = (tDouble)sbas_sat1.SatParam.longitude;
      if (sbas_sat1.SatParam.sense == 1)
      {
        longitude = -longitude;
      }
      GPS_DEBUG_MSG(( "sat_id_1 %d LON %d Sense %d Service %d \r\n",sbas_sat1.SatParam.sat_id,
                     sbas_sat1.SatParam.longitude,sbas_sat1.SatParam.sense,sbas_sat1.SatParam.service) );
      waas_insert_sat(sbas_sat1.SatParam.sat_id,longitude,(tSbasService)sbas_sat1.SatParam.service);
    }

    sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_2_ID, &(sbas_sat2.param));

    if (WAAS_SAT_ID_VALID(sbas_sat2.SatParam.sat_id) == TRUE)
    {
      tDouble longitude = (tDouble)sbas_sat2.SatParam.longitude;
      if (sbas_sat2.SatParam.sense == 1)
      {
        longitude = -longitude;
      }
      GPS_DEBUG_MSG(( "sat_id_2 %d LON %d Sense %d Service %d \r\n",sbas_sat2.SatParam.sat_id,
                     sbas_sat2.SatParam.longitude,sbas_sat2.SatParam.sense,sbas_sat2.SatParam.service) );
      waas_insert_sat(sbas_sat2.SatParam.sat_id,longitude,(tSbasService)sbas_sat2.SatParam.service);
    }

    waas_autosearch_set_sat_enabled_mask(sbas_sat_enabled_mask) ;

    waas_autosearch_set_timeout((sbas_autosearch_timeout2&0xFFFF),((sbas_autosearch_timeout2>>16)&0xFFFF),
                                (sbas_autosearch_timeout1&0xFFFF),((sbas_autosearch_timeout1>>16)&0xFFFF));

  }

  waas_plugin_handler->sbas_list = &waas_nmea_sbas_list;

  #if defined( STBIN_LINKED )
  stbin_waas_plugin_init( waas_plugin_handler);
  #endif

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Suspend WAAS module
 *
 * \return  gpOS_SUCCESS if all ok
 *
 ***********************************************/
static gpOS_error_t waas_plugin_suspend( void)
{
  waas_disable_autosearch();

  waas_set_status( WAAS_STATUS_OFF);

  while( waas_suspend_done() == FALSE)
  {
    gpOS_task_delay( gpOS_timer_ticks_per_sec() / 10);
  }

  waas_nmea_flush_sbas_msgs();

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Restart WAAS module
 *
 * \return  gpOS_SUCCESS if all ok
 *
 ***********************************************/
static gpOS_error_t waas_plugin_restart( void)
{
  tUInt sbas_sat_enabled_mask = 0;
  tInt waas_sat_id = 0;
  tUInt sbas_autosearch_timeout1 = 0;
  tUInt sbas_autosearch_timeout2 = 0;
  tSatelliteParams sbas_sat1;
  tSatelliteParams sbas_sat2;

  sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_PRN_ID, &waas_sat_id );

  waas_set_status(( gnss_waas_status_t )sw_config_get_software_switch_status( WAAS_ON_OFF_SWITCH ));

  waas_set_satellite( waas_sat_id );

  if(sw_config_get_software_switch_status( WAAS_AUTOSEARCH_ON_OFF_SWITCH ))
  {
    tInt last_decoded_sat;

    if(waas_autosearch_get_last_decoded_sat((tU16 *)&last_decoded_sat)== GNSS_NO_ERROR)
    {
      waas_set_satellite( last_decoded_sat );
    }
    waas_enable_autosearch();
  }
  else
  {
    waas_disable_autosearch();
  }

  sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITES_ENABLE_MASK_ID, &sbas_sat_enabled_mask );

  sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_AUTO_SEARCH_TIMEOUT_1_ID, &sbas_autosearch_timeout1 );

  sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_AUTO_SEARCH_TIMEOUT_2_ID, &sbas_autosearch_timeout2 );

  sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_1_ID, &(sbas_sat1.param));

  if (WAAS_SAT_ID_VALID(sbas_sat1.SatParam.sat_id) == TRUE)
  {
    tDouble longitude = (tDouble)sbas_sat1.SatParam.longitude;
    if (sbas_sat1.SatParam.sense == 1)
    {
      longitude = -longitude;
    }
    waas_insert_sat(sbas_sat1.SatParam.sat_id,longitude,(tSbasService)sbas_sat1.SatParam.service);
  }

  sw_config_get_param( CURRENT_CONFIG_DATA, SBAS_SATELLITE_PARAM_2_ID, &(sbas_sat2.param));

  if (WAAS_SAT_ID_VALID(sbas_sat2.SatParam.sat_id) == TRUE)
  {
    tDouble longitude = (tDouble)sbas_sat2.SatParam.longitude;

    if (sbas_sat2.SatParam.sense == 1)
    {
      longitude = -longitude;
    }

    waas_insert_sat(sbas_sat2.SatParam.sat_id,longitude,(tSbasService)sbas_sat2.SatParam.service);
  }

  waas_autosearch_set_sat_enabled_mask(sbas_sat_enabled_mask) ;

  waas_autosearch_set_timeout((sbas_autosearch_timeout2&0xFFFF),((sbas_autosearch_timeout2>>16)&0xFFFF),
                              (sbas_autosearch_timeout1&0xFFFF),((sbas_autosearch_timeout1>>16)&0xFFFF));

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Fill external satellite parameters structure for WAAS
 *
 * \param   sat_params  Parameter structure to fill
 * \return  gpOS_SUCCESS if satellite is used
 *
 ***********************************************/
static gpOS_error_t waas_plugin_get_sat_params( nmea_support_sat_params_t *sat_params)
{
  satid_t sat_id = sat_params->satid;

  if( sat_id == waas_get_satellite())
  {
    tInt el,az;
    tDouble lon;

    sat_params->CN0 = gnss_get_sat_cn0( sat_id);
    el = waas_sat_get_elevation( sat_id);
    az = waas_sat_get_azimuth( sat_id);

    if  ((waas_sat_lon_get( sat_id,&lon)== GNSS_NO_ERROR) && (el >= 0))
    {
      sat_params->elevation = el;
      sat_params->azimuth   = az;
    }
    else
    {
      sat_params->elevation = 0;
      sat_params->azimuth   = 0;
    }

    return gpOS_SUCCESS;
  }

  return gpOS_FAILURE;
}

/********************************************//**
 * \brief   Send SBASMSG message
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBASMSG(waas_msg_dump_message_t *sbas_message_ptr)
{
  gnss_waas_status_t waas_status = waas_get_status();

  if (waas_status == WAAS_STATUS_ON)
  {
    tInt        index =0;
    tInt        i;
    tInt        year;
    tInt        month;
    tInt        day;
    tInt        hours;
    tInt        mins;
    tInt        secs;
    tInt        msecs;
    satid_t     sbas_sat_id;
    chanid_t    sbas_channel_id;
    gnss_time_t time;
    tChar       out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

    sbas_sat_id     = sbas_message_ptr->sat_id;
    time.week_n     = sbas_message_ptr->waas_frame_time.week_n;
    time.tow        = sbas_message_ptr->waas_frame_time.tow;
    sbas_channel_id = 0;
    gnss_get_date(time.week_n, (tDouble) time.tow, &year, &month, &day);
    gnss_get_utc_time((tDouble) time.tow, &hours, &mins, &secs, &msecs);

    index += _clibs_sprintf(out_msg + index, "$PSTMSBASMSG,");
    index += _clibs_sprintf(out_msg + index, "%03d,%02d,", sbas_sat_id, sbas_channel_id);
    index += _clibs_sprintf(out_msg + index, "%02d%02d%02d,%02d%02d%02d,", hours, mins, secs,day,month,year % 100);

    for(i = 0; i < WAAS_FRAME_SIZE; i++)
    {
      index += _clibs_sprintf(out_msg + index, "%02X",sbas_message_ptr->buf[i]);
    }
    index += _clibs_sprintf(out_msg + index, "*%02X", nmea_support_checksum(out_msg));

    index += _clibs_sprintf(out_msg + index, "\r\n");

    nmea_send_outmsg(out_msg, index, SBAS_NMEA_MSG);

    #if SBAS_VERBOSE_NMEA
    waas_nmea_outmsg_send_SBASSATCORR();
    waas_nmea_outmsg_send_SBASGEO();
    waas_nmea_outmsg_send_SBASSATPAR();
    #endif
  }
}

/********************************************//**
 * \brief   Send SBASM message
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBASM(waas_msg_dump_message_t *sbas_message_ptr)
{
  gnss_waas_status_t waas_status = waas_get_status();

  if (waas_status == WAAS_STATUS_ON)
  {
    tInt  index =0;
    tInt  i;
    tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

    index += _clibs_sprintf(out_msg + index, "$PSTMSBASM,");
    index += _clibs_sprintf(out_msg + index, "%03d,", sbas_message_ptr->sat_id );

    for(i = 0; i < WAAS_FRAME_SIZE; i++)
    {
      index += _clibs_sprintf(out_msg + index, "%02X",sbas_message_ptr->buf[i]);
    }

    index += _clibs_sprintf(out_msg + index, "*%02X", nmea_support_checksum(out_msg));

    index += _clibs_sprintf(out_msg + index, "\r\n");

    nmea_send_outmsg(out_msg, index, SBAS_NMEA_MSG);
  }
 }

/********************************************//**
 * \brief   Get WAAS frame from queue
 *
 * \return  None
 *
 ***********************************************/
static boolean_t waas_nmea_get_sbas_message( waas_msg_dump_message_t *sbas_message)
{
  tInt i;
  boolean_t sbas_message_available = FALSE;

  gnss_waas_status_t waas_status = waas_get_status();

  if (waas_status == WAAS_STATUS_ON)
  {
    waas_msg_dump_message_t *sbas_message_ptr;

    sbas_message_ptr = waas_msg_get_dump_msg();

    if (sbas_message_ptr != NULL)
    {
      sbas_message_available = TRUE;

      sbas_message->sat_id          = sbas_message_ptr->sat_id;
      sbas_message->waas_frame_time = sbas_message_ptr->waas_frame_time;

      for(i = 0; i < WAAS_FRAME_SIZE; i++)
      {
        sbas_message->buf[i] = sbas_message_ptr->buf[i];
      }

      waas_msg_dump_msg_release(sbas_message_ptr);
    }
  }
  else
  {
    sbas_message->sat_id                 = 0;
    sbas_message->waas_frame_time.tow    = 0.0;
    sbas_message->waas_frame_time.week_n = 0;

    for(i = 0; i < WAAS_FRAME_SIZE; i++)
    {
      sbas_message->buf[i] = 0;
    }
  }

  return sbas_message_available;
}

#if SBAS_VERBOSE_NMEA
/********************************************//**
 * \brief   Send SBASSATPAR message
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBASSATPAR(void)
{
  tSatData sat_db[33];
  tInt slot2prn[33];
  tInt slot=0;
  tInt j = 0;
  tInt msg_num = 0;
  tInt sat_count = 0;
  tInt tot_num_msg = 0;
  visible_sats_data_t sats_visible;
  tU16 fast,fast_udrei,dx,dy,dz,dafo;
  tInt time_fast,time_slow;

  waas_get_sat_params(sat_db, slot2prn);
  gnss_get_sats_visible(&sats_visible);

  tot_num_msg = ((sats_visible.list_size - 1) / 3) + 1;

  for(msg_num = 1, sat_count = 0; sat_count < sats_visible.list_size; msg_num++)
  {
    tInt i = 0;
    tInt index = 0;
    tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

    index += _clibs_sprintf(out_msg + index, "$PSTMSBASSATPAR,%1d,%1d,%02d", tot_num_msg, msg_num, sats_visible.list_size);

    for(i = 0; (i < 3) && ((i + sat_count) < sats_visible.list_size); i++)
    {
      slot = waas_prn_to_slot(sats_visible.list[i+sat_count].satid,slot2prn);
      if (slot > 0)
      {
        fast_udrei = sat_db[slot].fast_udrei;

        if (fast_udrei >= 14)
        {
          index += _clibs_sprintf(out_msg + index, ",%02d,,,,,,",sats_visible.list[i+sat_count].satid);
        }
        else
        {
          time_fast = sat_db[slot].tow_fast;
          time_slow = sat_db[slot].tow_slow;
          fast = sat_db[slot].fast;
          dx = sat_db[slot].dx;
          dy = sat_db[slot].dy;
          dz = sat_db[slot].dz;
          dafo = sat_db[slot].dafo;
          index += _clibs_sprintf(out_msg + index, ",%02d,%05d,%05d,%03X,%03X,%03X,%03X,%03X",sats_visible.list[i+sat_count].satid,time_fast,time_slow,fast,dx,dy,dz,dafo);
        }
      }
      else
      {
        index += _clibs_sprintf(out_msg + index, ",,,,,,,");
      }
    }

    sat_count += i;

    for(j = i; j < 3; j++)
    {
      index += _clibs_sprintf(out_msg + index, ",,,,,,,");
    }

    index += _clibs_sprintf(out_msg + index, "*%02X", nmea_support_checksum(out_msg));

    index += _clibs_sprintf(out_msg + index, "\r\n");

    nmea_send_outmsg(out_msg, index, SBAS_NMEA_MSG);
  }
}
#endif

#if SBAS_VERBOSE_NMEA
/********************************************//**
 * \brief   Send SBASGEO message
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBASGEO(void)
{
  tInt j = 0;
  tInt msg_num = 0;
  tInt sat_count = 0;
  tInt tot_num_msg = 0;
  tGeoSatParam geo_params;

  visible_sats_data_t sats_visible;

  gnss_get_sats_visible(&sats_visible);

  tot_num_msg = ((sats_visible.list_size - 1) / 3) + 1;

  for(msg_num = 1, sat_count = 0; sat_count < sats_visible.list_size; msg_num++)
  {
    tInt i = 0;
    tInt index = 0;
    tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

    index += _clibs_sprintf(out_msg + index, "$PSTMSBASGEO,%1d,%1d,%02d", tot_num_msg, msg_num, sats_visible.list_size);

    for(i = 0; (i < 3) && ((i + sat_count) < sats_visible.list_size); i++)
    {
      waas_get_geo_params(sats_visible.list[i+sat_count].satid, &geo_params);
      index += _clibs_sprintf(out_msg + index, ",%02d,%.2f,%.2f,%.2f",sats_visible.list[i+sat_count].satid,geo_params.xdiff,geo_params.ydiff,geo_params.zdiff);
    }

    sat_count += i;

    for(j = i; j < 3; j++)
    {
      index += _clibs_sprintf(out_msg + index, ",,,,");
    }

    index += _clibs_sprintf(out_msg + index, "*%02X", nmea_support_checksum(out_msg));

    index += _clibs_sprintf(out_msg + index, "\r\n");

    nmea_send_outmsg(out_msg, index, SBAS_NMEA_MSG);
  }
}
#endif

#if SBAS_VERBOSE_NMEA
/********************************************//**
 * \brief   Send SBASSATCORR message
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBASSATCORR(void)
{
  tInt j = 0;
  tInt msg_num = 0;
  tInt sat_count = 0;
  tInt tot_num_msg = 0;
  tGpsSatCorrection sat_correction;

  tInt time_fast,time_slow;
  tDouble fast,slow,dx,dy,dz,clock;
  visible_sats_data_t sats_visible;

  gnss_get_sats_visible(&sats_visible);

  tot_num_msg = ((sats_visible.list_size - 1) / 3) + 1;

  for(msg_num = 1, sat_count = 0; sat_count < sats_visible.list_size; msg_num++)
  {
    tInt i = 0;
    tInt index = 0;
    tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

    index += _clibs_sprintf(out_msg + index, "$PSTMSBASSATCORR,%1d,%1d,%02d", tot_num_msg, msg_num, sats_visible.list_size);

    for(i = 0; (i < 3) && ((i + sat_count) < sats_visible.list_size); i++)
    {
      waas_get_sat_correction(sats_visible.list[i+sat_count].satid, &sat_correction);

      time_fast = sat_correction.tow_fast;
      fast = sat_correction.fast;
      time_slow = sat_correction.tow_slow;
      slow = sat_correction.slow;
      dx = sat_correction.dx;
      dy = sat_correction.dy;
      dz = sat_correction.dz;
      clock = sat_correction.clock;
      index += _clibs_sprintf(out_msg + index, ",%02d,%05d,%3.3f,%05d,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f",sats_visible.list[i+sat_count].satid,time_fast,fast,time_slow,slow,dx,dy,dz,clock);
    }

    sat_count += i;

    for(j = i; j < 3; j++)
    {
      index += _clibs_sprintf(out_msg + index, ",,,,,,");
    }

    index += _clibs_sprintf(out_msg + index, "*%02X", nmea_support_checksum(out_msg));

    index += _clibs_sprintf(out_msg + index, "\r\n");

    nmea_send_outmsg(out_msg, index, SBAS_NMEA_MSG);
  }
}
#endif

/********************************************//**
 * \brief   Transmits WAAS/SBAS information about the status and the
 *          the azimuth, the elevation and the power of SBAS satellite
 *          tracked.
 *
 * \param   data_p    void*
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBAS(void *data_p)
{
  tInt index     = 0;
  gnss_waas_status_t waas_status;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  index += _clibs_sprintf(out_msg, "$PSTMSBAS");
  waas_status = waas_get_status();
  waas_plugin_handler->sat_id = waas_get_satellite();

  if (waas_status == WAAS_STATUS_ON)
  {
    tInt waas_el;
    tInt waas_az;
    tInt waas_signal;
    tDouble waas_lon;

    waas_el = waas_sat_get_elevation( waas_plugin_handler->sat_id);
    waas_az = waas_sat_get_azimuth( waas_plugin_handler->sat_id);
    waas_signal = gnss_get_sat_cn0( waas_plugin_handler->sat_id);


    if (gnss_fix_get_diff_status_local(data_p) == DIFF_STATUS_ON)
    {
      index += _clibs_sprintf(out_msg + index, ",1,2");// waas status and acquire flag status
    }
    else
    {
      if (waas_signal > 0 )
    {
      index += _clibs_sprintf(out_msg + index, ",1,1");// waas status and acquire flag status
    }
    else
    {
      index += _clibs_sprintf(out_msg + index, ",1,0");// waas status and acquire flag status
    }
    }

    if  ((waas_sat_lon_get(waas_plugin_handler->sat_id,&waas_lon)== GNSS_NO_ERROR) && (waas_el >= 0))
    {
      index += _clibs_sprintf(out_msg + index, ",%03d,%02d,%03d", waas_plugin_handler->sat_id, waas_el, waas_az);
    }
    else
    {
      index += _clibs_sprintf(out_msg + index, ",%03d,,", waas_plugin_handler->sat_id);
    }

    if(waas_signal >= 0)
    {
      index += _clibs_sprintf(out_msg + index, ",%02d", waas_signal);
    }
    else
    {
      index += _clibs_sprintf(out_msg + index, ",");
    }

  }
  else
  {
    index += _clibs_sprintf(out_msg + index, ",0,0,,,,");
  }

  index += _clibs_sprintf(out_msg + index, "*%02X\r\n", nmea_support_checksum(out_msg));

  nmea_send_outmsg(out_msg, index, WAAS_NMEA_MSG);

  waas_nmea_outmsg_send_SBASMCH(0);
  waas_nmea_outmsg_send_SBASMCH(1);
}

/********************************************//**
 * \brief   Transmits WAAS/SBAS information about the status and the
 *          the azimuth, the elevation and the power of SBAS satellite
 *          tracked.
 *
 * \param   waas_channel chanid_t
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_SBASMCH( chanid_t waas_channel)
{
  tInt index     = 0;
  gnss_waas_status_t waas_ch_status;
  satid_t waas_ch_satid;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  index += _clibs_sprintf(out_msg, "$PSTMSBASMCH,%d",waas_channel);
  waas_get_multi_ch( waas_channel, &waas_ch_satid, &waas_ch_status);

  if (waas_ch_status == WAAS_STATUS_ON)
  {
    tInt waas_el;
    tInt waas_az;
    tInt waas_signal;
    tDouble waas_lon;
    satid_t waas_sat_to_decode = waas_get_satellite();

    if(waas_sat_to_decode == waas_ch_satid)
    {
      waas_el = waas_sat_get_elevation(waas_ch_satid);
      waas_az = waas_sat_get_azimuth(waas_ch_satid);

      if  ((waas_sat_lon_get(waas_ch_satid,&waas_lon)== GNSS_NO_ERROR) && (waas_el >= 0))
      {
        index += _clibs_sprintf(out_msg + index, ",%03d,%02d,%03d", waas_ch_satid, waas_el, waas_az);
      }
      else
      {
        index += _clibs_sprintf(out_msg + index, ",%03d,,", waas_ch_satid);
      }
    }
    else
    {
      index += _clibs_sprintf(out_msg + index, ",%03d,,", waas_ch_satid);
    }

    waas_signal = gnss_get_sat_cn0(waas_ch_satid);
    if(waas_signal >= 0)
    {
      index += _clibs_sprintf(out_msg + index, ",%02d", waas_signal);
    }
    else
    {
      index += _clibs_sprintf(out_msg + index, ",");
    }

  }
  else
  {
    index += _clibs_sprintf(out_msg + index, ",0,0,,,,");
  }

  index += _clibs_sprintf(out_msg + index, "*%02X\r\n", nmea_support_checksum(out_msg));
  nmea_send_outmsg(out_msg, index, WAAS_NMEA_MSG);
}

/********************************************//**
 * \brief   Transmits inoforamtion on which visible satellites are
 *          in differential mode.
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_outmsg_send_DIFF(void)
{
  tInt index     = 0;
  tInt sat_count;
  tInt i;
  tInt satid;
  tInt diff_count = 0;
  tInt sats_vis_list = 0;
  tInt iod;
  tDouble epoch;
  tDouble range;
  tDouble rate;
  boolean_t available;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  visible_sats_data_t sats_visible;


  gnss_get_sats_visible(&sats_visible);

  waas_plugin_handler->status = waas_get_status(); // to get waas status
  index += _clibs_sprintf(out_msg, "$PSTMDIFF");

  sats_vis_list = sats_visible.list_size;

  if( (sats_vis_list == 12) && (waas_plugin_handler->status == WAAS_STATUS_ON))
  {
    sats_vis_list = sats_visible.list_size - 1;
  }

  for(sat_count = 0;  sat_count < sats_vis_list; sat_count++)
  {
    gnss_get_diff_params(sats_visible.list[sat_count].satid,&iod,&epoch,&range,&rate,&available);

    if(available)
    {
      diff_count++;
    }
  }

  index += _clibs_sprintf(out_msg + index, ",%02d,%02d", sats_visible.list_size, diff_count);

  for(sat_count = 0;  sat_count < sats_vis_list; sat_count++)
  {
    satid = sats_visible.list[sat_count].satid;
    gnss_get_diff_params(sats_visible.list[sat_count].satid,&iod,&epoch,&range,&rate,&available);
    index += _clibs_sprintf(out_msg + index, ",%02d,%d", satid, available);
  }

  for(i = 0; i < (12 - sats_vis_list); i++)
  {
    index += _clibs_sprintf(out_msg + index, ",");
  }

  index += _clibs_sprintf(out_msg + index, "*%02X\r\n", nmea_support_checksum(out_msg));

  nmea_send_outmsg(out_msg, index, DIFF_NMEA_MSG);
}

/********************************************//**
 * \brief   Execute set periodic backup command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_setperiodicbackup( tChar *cmd_par)
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt value;

  field_count = _clibs_sscanf( cmd_par,",%d",&value);

  if(field_count == 1)
  {
    if(value == 1)
    {
      waas_enable_periodic_data_backup(TRUE);
      index = _clibs_sprintf(out_msg, "$PSTMWAASPERIODICBACKUPON");
    }
    else
    {
      waas_enable_periodic_data_backup(FALSE);
      index = _clibs_sprintf(out_msg, "$PSTMWAASPERIODICBACKUPOFF");
    }
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMWAASPERIODICBACKUPERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute multichannel command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_multichan( tChar *cmd_par)
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt test;
  tInt err = 0;

  field_count = _clibs_sscanf(cmd_par,",%d",&test);

  index = _clibs_sprintf(out_msg, "$PSTMSBASMCH");

  if(field_count == 1)
  {
    switch(test)
    {
      case 0: //set multi ch prn and status
        {
          tInt prn,ch,status;

          field_count = _clibs_sscanf(cmd_par,",%d,%d,%d,%d",&test,&ch,&prn,&status);
          if(field_count == 4)
          {
            waas_set_multi_ch(ch,prn,(gnss_waas_status_t)status);
            index += _clibs_sprintf(out_msg + index,"OK,%d,%d,%d,%d",test,ch,prn,status);
          }
          else
          {
            err |= (1<<1);
          }
        }
        break;

      case 1: //get multi ch sat_id and status
        {
          satid_t sat_id;
          tInt chan_id;
          gnss_waas_status_t status;

          field_count = _clibs_sscanf(cmd_par,",%d,%d", &test, &chan_id);
          if(field_count == 2)
          {
            waas_get_multi_ch( chan_id, &sat_id, &status);
            index += _clibs_sprintf(out_msg + index,"OK,%d,%d,%d,%d",test,chan_id,sat_id,status);
          }
          else
          {
            err |= (1<<2);
          }
        }
        break;

      case 2: //set prn to decode
        {
          tInt satid;

          field_count = _clibs_sscanf(cmd_par,",%d,%d", &test, &satid);
          if(field_count == 2)
          {
            waas_set_prn_to_decode(satid);
            index += _clibs_sprintf(out_msg + index,"OK,%d,%d",test,satid);
          }
          else
          {
            err |= (1<<3);
          }

        }
        break;

      case 3: //get prn to decode
        {
          satid_t satid;
          waas_get_prn_to_decode( &satid);
          index += _clibs_sprintf(out_msg + index,"OK,%d,%d",test,satid);
        }
        break;
    }
  }
  else
  {
    err |= (1<<1);
  }

  if (err != 0)
  {
    index += _clibs_sprintf(out_msg + index,"GNSS_ERROR,%d",err);
  }

  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute autosearch command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_autosearch(tChar *cmd_par)
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt value;

  field_count = _clibs_sscanf(cmd_par,",%d",&value);

  if(field_count == 1)
  {
    if(value == 1)
    {
      waas_enable_autosearch();
      index = _clibs_sprintf(out_msg, "$PSTMSBASAUTOSEARCON");
      GPS_DEBUG_MSG(("\r\n[nmea][waas] AUTOSEARCH_ENABLED \r\n"));
    }
    else
    {
      waas_disable_autosearch();
      index = _clibs_sprintf(out_msg, "$PSTMSBASAUTOSEARCOFF");
      GPS_DEBUG_MSG(("\r\n[nmea][waas] AUTOSEARCH_DISABLED \r\n"));
    }
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMSBASAUTOSEARCHERROR");
    GPS_DEBUG_MSG(("\r\n[nmea][waas] AUTOSEARCH_ERROR %d  %d\r\n",field_count,value));
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute set WAAS status command
 *
 * \param   cmd_par   Command parameters
 * \return  void
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_setstatus( tChar *cmd_par )
{
  waas_plugin_handler->status = waas_get_status();

  if ( waas_plugin_handler->status == WAAS_STATUS_ON )
  {
    GPS_DEBUG_MSG( ( "[nmea][waas]You have just switched the SBAS off\r\n" ) );
    waas_plugin_handler->status = WAAS_STATUS_OFF;
  }
  else
  {
    GPS_DEBUG_MSG( ( "[nmea][waas]You have just switched the SBAS on\r\n" ) );
    waas_plugin_handler->status = WAAS_STATUS_ON;
  }
  waas_set_status( waas_plugin_handler->status );
}

/********************************************//**
 * \brief   Execute set WAAS satellite command
 *
 * \param   cmd_par   Command parameters
 * \return  void
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_setsat( tChar *cmd_par )
{
  tInt field_count = 0;
  tInt waas_sat_changed;

  field_count = _clibs_sscanf( cmd_par, ",%3d",&waas_sat_changed);

  if(field_count == 1)
  {
    if (waas_sat_changed != 0)
    {
      if( (waas_sat_changed < 120) || (waas_sat_changed > 138) )
      {
        if (waas_sat_changed == 6)
        {
          if( waas_plugin_handler->debug_integrity_check == FALSE)
          {
            waas_plugin_handler->debug_integrity_check = TRUE;
          }
          else
          {
            waas_plugin_handler->debug_integrity_check = FALSE;
          }
          waas_set_integrity_check( waas_plugin_handler->debug_integrity_check);
          GPS_DEBUG_MSG(("[nmea][waas]Nmea command to discard sat %d for no integrity simulation integrity check %d\r\n",waas_sat_changed,waas_plugin_handler->debug_integrity_check));
        }
      }
      else
      {
        waas_set_satellite( waas_sat_changed );
        GPS_DEBUG_MSG( ( "[nmea][waas]You have changed the SBAS sat... now the SBAS sat is %d\r\n", waas_plugin_handler->sat_id ) );
      }
    }
    else
    {
      if ( waas_automatic_search( &waas_plugin_handler->sat_id ) == GNSS_ERROR )
      {
        GPS_DEBUG_MSG( ( "[nmea][waas]Error Automatic SBAS sat search not possible\r\n" ) );
      }
      else
      {
        waas_set_satellite( waas_plugin_handler->sat_id );
        GPS_DEBUG_MSG( ( "[nmea][waas]Automatic SBAS sat choice...now the SBAS available sat is: %d\r\n", waas_plugin_handler->sat_id ) );
      }
    }
  }
  else
  {
    GPS_DEBUG_MSG( ( "[nmea][waas]You are trying to change the SBAS sat... but the prn is wrong\r\n" ) );
  }
}

/********************************************//**
 * \brief   Execute set satellites list command
 *
 * \param   cmd_par   Command parameters
 * \return  void
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_setsatlist( tChar *cmd_par )
{
  waas_set_sbas_list( (tSbasInfo *)waas_nmea_sbas_list, SBAS_PROVIDER_NUMBER ); // set the sbas satellite list. In this case it is with 3 provider
}

/********************************************//**
 * \brief   Force backup of waas data
 *
 * \param   cmd_par   Command parameters
 * \return  void
 *
 ***********************************************/
static void waas_nmea_cmdif_exec_forcebackup( tChar *cmd_par )
{
  waas_execute_data_backup();
}

/********************************************//**
 * \brief   Execute test on WAAS module
 *
 * \param   cmd_par   Command parameters
 * \return  void
 *
 ***********************************************/
static void _execute_sbas_test(void)
{
  GPS_DEBUG_MSG( ( "\r\n\r\n\r\n[nmea][waas]Initialising SBAS Test...\r\n" ) );

  waas_set_status(WAAS_STATUS_OFF);
  gnss_reset_diff_params();
  gpOS_task_delay(2000000);
  waas_set_status(WAAS_STATUS_ON);
}

static void waas_nmea_cmdif_exec_test( tChar *cmd_par )
{
  _execute_sbas_test();

  nmea_outmsg_transmit_short_header();
}

/********************************************//**
 * \brief   Check NMEA command for WAAS commands
 *
 * \param   input_cmd_msg   Command string
 * \return  void
 *
 ***********************************************/
static tInt waas_plugin_nmea_cmdif_parse( tChar *cmd, tUInt cmd_size, tChar *cmd_par)
{
  tUInt cmd_id;

  if( nmea_support_cmdif_getid( cmd, cmd_size, waas_nmea_cmd_strings, WAAS_NMEA_CMDID_NUMBER, &cmd_id) == NMEA_NOT_VALID)
  {
    return NMEA_NOT_VALID;
  }

  waas_nmea_cmdif_exec_table[cmd_id]( cmd_par );

  #if 0
  switch(cmd_id)
  {
    case WAAS_NMEA_CMDID_SETSTATUS:
      waas_nmea_cmdif_exec_setstatus( cmd_par );
      break;

    case WAAS_NMEA_CMDID_SETSAT:
      waas_nmea_cmdif_exec_setsat( cmd_par );
      break;

    case WAAS_NMEA_CMDID_SETSATLIST:
      waas_nmea_cmdif_exec_setsatlist( cmd_par );
      break;

    case WAAS_NMEA_CMDID_SETPERIODICBACKUP:
      waas_nmea_cmdif_exec_setperiodicbackup( cmd_par );
      break;

    case WAAS_NMEA_CMDID_FORCEBACKUP:
      waas_nmea_cmdif_exec_forcebackup( cmd_par );
      break;

    case WAAS_NMEA_CMDID_TEST:
      waas_nmea_cmdif_exec_test( cmd_par );
      break;

    case WAAS_NMEA_CMDID_MULTICHAN:
      waas_nmea_cmdif_exec_multichan( cmd_par );
      break;

    case WAAS_NMEA_CMDID_AUTOSEARCH :
      waas_nmea_cmdif_exec_autosearch( cmd_par );
      break;

    default:
      break;
  }
  #endif

  return NMEA_OK;
}

/********************************************//**
 * \brief   Transmit WAAS related NMEA messages
 *
 * \param   param   NMEA parameters
 * \return  gpOS_SUCCESS
 *
 ***********************************************/
static gpOS_error_t waas_plugin_nmea_outmsg_transmit( void *param)
{
  nmea_support_ext_params_t *nmea_params = (nmea_support_ext_params_t *)param;
  tUInt nmea_msg_list[2];

  nmea_msg_list[0] = nmea_params->msg_list[0];
  nmea_msg_list[1] = nmea_params->msg_list[1];

  if( nmea_msg_list_check(nmea_msg_list,WAAS_NMEA_MSG))
  {
    waas_nmea_outmsg_send_SBAS(nmea_params->data_p);
  }

  if( nmea_msg_list_check(nmea_msg_list,DIFF_NMEA_MSG))
  {
    waas_nmea_outmsg_send_DIFF();
  }

  if(( nmea_msg_list_check(nmea_msg_list,SBAS_NMEA_MSG)) || (nmea_msg_list_check(nmea_msg_list,SBAS_NMEA_M)))
  {
    waas_msg_dump_message_t sbas_message;
    boolean_t               sbas_message_available = FALSE;

    do {
      sbas_message_available = waas_nmea_get_sbas_message(&sbas_message);

      if (sbas_message_available == TRUE)
      {
        if( nmea_msg_list_check(nmea_msg_list,SBAS_NMEA_MSG))
        {
          waas_nmea_outmsg_send_SBASMSG(&sbas_message);
        }

        if( nmea_msg_list_check(nmea_msg_list,SBAS_NMEA_M))
        {
          waas_nmea_outmsg_send_SBASM(&sbas_message);
        }
      }
    } while (sbas_message_available == TRUE);
  }
  else
  {
    waas_nmea_flush_sbas_msgs();
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Flush queue from WAAS frames
 *
 * \return  None
 *
 ***********************************************/
static void waas_nmea_flush_sbas_msgs(void)
{
  waas_msg_dump_message_t sbas_message;
  boolean_t               sbas_message_available = FALSE;

  do {
    sbas_message_available = waas_nmea_get_sbas_message(&sbas_message);

  } while (sbas_message_available == TRUE);
}

/********************************************//**
 * \brief   Execute WAAS plugin command
 *
 * \param   cmd const   Command ID
 * \param   param       Command parameter
 * \return  gpOS_SUCCESS if all ok
 *
 ***********************************************/
static gpOS_error_t waas_plugin_api( const gnssapp_plugins_cmd_t cmd, gnssapp_plugins_cmd_param_t *param)
{
  gpOS_error_t error = gpOS_SUCCESS;

  switch( cmd)
  {
    case GNSSAPP_PLUGINS_CMD_INIT:
      error = waas_plugin_init( ( gpOS_partition_t *)param->data_ptr);
      break;

    case GNSSAPP_PLUGINS_CMD_GETVER:
      *((const tChar **)param->data_ptr) = waas_version();
      break;

    case GNSSAPP_PLUGINS_CMD_START:
      error = waas_plugin_restart();
      break;

    case GNSSAPP_PLUGINS_CMD_SUSPEND:
      error = waas_plugin_suspend();
      break;

    case GNSSAPP_PLUGINS_CMD_CUSTOM:
      switch( param->id)
      {
        case 0:
          error = waas_plugin_get_sat_params( (nmea_support_sat_params_t *)param->data_ptr);
          break;

        case 1:
          waas_invalidate_data_correction();
          break;

        default:
          error = gpOS_FAILURE;
          break;
      }
	  break;

    default:
      error = gpOS_FAILURE;
      break;
  }

  return error;
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

gnssapp_plugins_handler_t waas_gnssapp_plugin_handler =
{
  NULL,
  GNSSAPP_PLUGINS_ID_WAAS,
  waas_plugin_api,
  waas_plugin_nmea_cmdif_parse,
  waas_plugin_nmea_outmsg_transmit,
  #if defined( STBIN_LINKED )
  stbin_waas_plugin_cmdif_parse,
  stbin_waas_plugin_outmsg_transmit
  #else
  (gnssapp_plugins_stbin_cmdif_callback_t)NULL,
  (gnssapp_plugins_stbin_outmsg_callback_t)NULL
  #endif
};

#endif
