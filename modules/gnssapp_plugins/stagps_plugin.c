/*****************************************************************************
   FILE:          stagps_plugins.c
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
 * @file    stagps_plugins.c
 * @brief   Plugin for assisted GPS
 */

/*****************************************************************************
   includes
*****************************************************************************/

#if defined( ST_AGPS)

#include "clibs.h"
#include "gpOS.h"
#include "gnssapp.h"
#include "gnssapp_plugins.h"
#include "gnss_debug.h"
#include "sw_config.h"

#include "st_agps.h"
#include "st_agps_testing.h"
#include "pgps.h"

#include "nmea_support.h"
#include "stagps_plugin.h"
#include "gnss.h"

#if defined( STBIN_LINKED )
#include "stbin_stagps_plugin.h"
#endif // defined

/*****************************************************************************
   external declarations
*****************************************************************************/

extern tInt gnss_navigate_task_priority;
extern tInt st_agps_task_priority;

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define PGPS_PWDGEN_MAX_FIELD_LENGTH  50
#define PASSWORD_LENGTH               41
#define STAGPS_POLYS_PER_DAYS         2
#define STAGPS_SIZEOF_STORAGE         28
#define STAGPS_STORAGE_LINE_SIZE      (STAGPS_POLYS_PER_DAYS * 5 * STAGPS_SIZEOF_STORAGE)


  #define STAGPS_MAX_DAYS               14

#define STAGPS_NMEA_EPHDBDEPTH        12
#define STAGPS_NUM_OF_LISTS           2
#define STAGPS_MAX_LIST_LEN           (NMEA_GSV_MAX_SATS * 2)
#define STAGPS_BUFEER_SIZE            (300)
#define STAGPS_POLYS_PER_SATELLITE    (STAGPS_POLYS_PER_DAYS * STAGPS_MAX_DAYS)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  STAGPS_NMEA_CMDID_SETSTATUS,
  STAGPS_NMEA_CMDID_GETSTATUS,
  STAGPS_NMEA_CMDID_INVALIDATE,

  STAGPS_NMEA_CMDID_DUMPEPHINDB,
  STAGPS_NMEA_CMDID_SAVEEPHINDB,
  STAGPS_NMEA_CMDID_CLEAREPHINDB,
  STAGPS_NMEA_CMDID_SETREALEPHUPDSTATUS,
  STAGPS_NMEA_CMDID_SETPREDEPHUPDSTATUS,
  STAGPS_NMEA_CMDID_DUMPPOLYDB,
  STAGPS_NMEA_CMDID_SAVEPOLYDB,
  STAGPS_NMEA_CMDID_SETSTATUS_POLYMGR,
  STAGPS_NMEA_CMDID_INVALIDATEEPHEMS,
  STAGPS_NMEA_CMDID_SETBARCOL,

  STAGPS_NMEA_CMDID_PGPS_SETSATSEED,
  STAGPS_NMEA_CMDID_PGPS_GETNEXTSAT,
  STAGPS_NMEA_CMDID_PGPS_STARTPROP,
  STAGPS_NMEA_CMDID_PGPS_GENERATEPWD,

  STAGPS_NMEA_CMDID_FORCE_KEPLERIZATION,
  STAGPS_NMEA_CMDID_SETCONSTMASK,

  STAGPS_NMEA_CMDID_NUMBER
} stagps_nmea_cmdid_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

stagps_plugin_handler_t *stagps_plugin_handler;

static boolean_t stagps_enginesuspended = FALSE;
static boolean_t stagps_nmea_satbarcolor_status = TRUE;

static const tChar * const stagps_nmea_cmd_strings[] =
{
  "$PSTMSTAGPSONOFF",         // STAGPS_NMEA_CMDID_SETSTATUS,
  "$PSTMGETAGPSSTATUS",       // STAGPS_NMEA_CMDID_GETSTATUS,
  "$PSTMSTAGPSINVALIDATE",    // STAGPS_NMEA_CMDID_INVALIDATE,

  "$PSTMDUMPINDB",            // STAGPS_NMEA_CMDID_DUMPEPHINDB,
  "$PSTMINDBEPHEM",           // STAGPS_NMEA_CMDID_SAVEEPHINDB,
  "$PSTMINDBCLEAR",           // STAGPS_NMEA_CMDID_CLEAREPHINDB,
  "$PSTMREPHUPONOFF",         // STAGPS_NMEA_CMDID_SETREALEPHUPDSTATUS,
  "$PSTMPEPHUPONOFF",         // STAGPS_NMEA_CMDID_SETPREDEPHUPDSTATUS,
  "$PSTMPOLYDUMP",            // STAGPS_NMEA_CMDID_DUMPPOLYDB,
  "$PSTMPOLYSAVE",            // STAGPS_NMEA_CMDID_SAVEPOLYDB,
  "$PSTMPOLMGRONOFF",         // STAGPS_NMEA_CMDID_SETSTATUS_POLYMGR,
  "$PSTMCLRSATEPH",           // STAGPS_NMEA_CMDID_INVALIDATEEPHEMS,
  "$PSTMBARCOLONOFF",         // STAGPS_NMEA_CMDID_SETBARCOL,

  "$PSTMSTAGPSSATSEED",       // STAGPS_NMEA_CMDID_PGPS_SETSATSEED,
  "$PSTMSTAGPSSATSEEDNEXT",   // STAGPS_NMEA_CMDID_PGPS_GETNEXTSAT,
  "$PSTMSTAGPSFORCEPROP",     // STAGPS_NMEA_CMDID_PGPS_STARTPROP,
  "$PSTMSTAGPSPASSGEN",       // STAGPS_NMEA_CMDID_PGPS_GENERATEPWD,

  "$PSTMAGPSKEPL",            // STAGPS_NMEA_CMDID_FORCE_KEPLERIZATION,
  "$PSTMSTAGPSSETCONSTMASK",  // STAGPS_NMEA_CMDID_SETCONSTMASK,
  NULL                        // STAGPS_NMEA_CMDID_NUMBER
};

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static tInt            stagps_plugin_nmea_cmdif_parse                 ( tChar *, tUInt, tChar *);

static void           stagps_nmea_cmdif_exec_setstatus        ( tChar *);
static void           stagps_nmea_cmdif_exec_getstatus        ( tChar *);
static void           stagps_nmea_cmdif_exec_invalidate       ( tChar *);

static void           stagps_nmea_cmdif_exec_dumpephindb        ( tChar *);
static void           stagps_nmea_cmdif_exec_saveephindb        ( tChar *);
static void           stagps_nmea_cmdif_exec_clearephindb       ( tChar *);
static void           stagps_nmea_cmdif_exec_realephupdstatus   ( tChar *);
static void           stagps_nmea_cmdif_exec_predephupdstatus   ( tChar *);
static void           stagps_nmea_cmdif_exec_polydump           ( tChar *);
static void           stagps_nmea_cmdif_exec_polysave           ( tChar *);
static void           stagps_nmea_cmdif_exec_setstatus_polymgr  ( tChar *);
static void           stagps_nmea_cmdif_exec_invalidateephem    ( tChar *);
static void           stagps_nmea_cmdif_exec_setbarcol          ( tChar *);

static void           stagps_nmea_cmdif_exec_setsatseed         ( tChar *);
static void           stagps_nmea_cmdif_exec_getnextsat         ( tChar *);
static void           stagps_nmea_cmdif_exec_pgpsprop           ( tChar *);
static void           stagps_nmea_cmdif_exec_generatepwd        ( tChar *);

static void           stagps_nmea_cmdif_keplerize               ( tChar *);

static gpOS_error_t   stagps_plugin_nmea_outmsg_transmit        ( void *param);
static void           stagps_nmea_cmdif_setconstmask            ( tChar *);

static void           stagps_nmea_outmsg_send_AGPS              (void *);

static const nmea_support_cmdif_exec_t stagps_nmea_cmdif_exec_table[] =
{
  stagps_nmea_cmdif_exec_setstatus,
  stagps_nmea_cmdif_exec_getstatus,
  stagps_nmea_cmdif_exec_invalidate,

  stagps_nmea_cmdif_exec_dumpephindb,
  stagps_nmea_cmdif_exec_saveephindb,
  stagps_nmea_cmdif_exec_clearephindb,
  stagps_nmea_cmdif_exec_realephupdstatus,
  stagps_nmea_cmdif_exec_predephupdstatus,
  stagps_nmea_cmdif_exec_polydump,
  stagps_nmea_cmdif_exec_polysave,
  stagps_nmea_cmdif_exec_setstatus_polymgr,
  stagps_nmea_cmdif_exec_invalidateephem,
  stagps_nmea_cmdif_exec_setbarcol,

  stagps_nmea_cmdif_exec_setsatseed,
  stagps_nmea_cmdif_exec_getnextsat,
  stagps_nmea_cmdif_exec_pgpsprop,
  stagps_nmea_cmdif_exec_generatepwd,

  stagps_nmea_cmdif_keplerize,
  stagps_nmea_cmdif_setconstmask,

  NULL
};

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

static gpOS_error_t stagps_plugin_init( gpOS_partition_t *part)
{
  stagps_plugin_handler = (stagps_plugin_handler_t *)gpOS_memory_allocate_p( part, sizeof( stagps_plugin_handler_t));

  if( stagps_plugin_handler == NULL)
  {
    ERROR_MSG( "[main]: ERROR stagps_plugin_init failed\r\n" );
    return( gpOS_FAILURE );
  }

  stagps_plugin_handler->sat_idx_counter = 0;
  stagps_plugin_handler->sat_idx_pgps_max = 59;

  if( ST_AGPS_init( part) == GNSS_ERROR)
  {
    ERROR_MSG("[main]: LLD_ERROR ST_AGPS_init failed\r\n");
    return gpOS_FAILURE;
  }

  if( sw_config_get_software_switch_status( STAGPS_ON_OFF_SWITCH ) )
  {
    ST_AGPS_start();
  }

  #if defined( STBIN_LINKED )
  stbin_stagps_plugin_init( stagps_plugin_handler);
  #endif // defined

  return gpOS_SUCCESS;
}

static gpOS_error_t stagps_plugin_suspend( void)
{
  if (ST_AGPS_suspended() == TRUE)
  {
    stagps_enginesuspended = TRUE;
  }
  else
  {
    gpOS_task_t *sagps_task;
    ST_AGPS_suspend( ST_AGPS_SUSPEND_IMMEDIATELY);

    sagps_task = (gpOS_task_t *)ST_AGPS_get_task_ptr(ST_AGPS_MGRID_SAGPS);
    gpOS_task_set_priority(sagps_task, gnss_navigate_task_priority);

    while( ST_AGPS_suspended() == FALSE)
    {
      gpOS_task_delay( gpOS_timer_ticks_per_sec());
    }
    stagps_enginesuspended = FALSE;
  }
  return gpOS_SUCCESS;
}

static gpOS_error_t stagps_plugin_restart( void)
{
  if (stagps_enginesuspended == FALSE)
  {
    gpOS_task_t *sagps_task;

    ST_AGPS_start();
    ST_AGPS_start_initial_prediction();

    sagps_task = (gpOS_task_t *)ST_AGPS_get_task_ptr(ST_AGPS_MGRID_SAGPS);
    gpOS_task_set_priority(sagps_task, st_agps_task_priority);
  }
  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Execute set status command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_setstatus(tChar *cmd_par)
{
  tChar out_msg[256];
  tInt status;
  tInt field_count = 0;
  tInt index;

  field_count =  _clibs_sscanf( cmd_par, ",%d" ,&status);

  if(field_count == 1)
  {
    if (status == 1)
    {
      ST_AGPS_start();
      ST_AGPS_start_initial_prediction();
      index = _clibs_sprintf(out_msg, "$PSTMPOLSTARTED");
    }
    else
    {
      ST_AGPS_suspend( ST_AGPS_SUSPEND_IMMEDIATELY);
      index = _clibs_sprintf(out_msg, "$PSTMPOLSUSPENDED");
    }
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMPOLONOFFERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute get status command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_getstatus( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt index;
  tInt status_code;

  status_code = ST_AGPS_TST_get_state();
  index = _clibs_sprintf(out_msg, "$PSTMAGPSSTATUS,%d", status_code);
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute invalidate command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_invalidate( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt field_count, mask;
  tInt sat_idx;
  tInt index;

  field_count=_clibs_sscanf( cmd_par,",%d",&mask);

  if( field_count == 1)
  {
    boolean_t suspended = ST_AGPS_suspended();

    if(!suspended)
    {
      ST_AGPS_suspend( ST_AGPS_SUSPEND_IMMEDIATELY);
    }
    for( sat_idx = 0; sat_idx < gnss_get_predicted_linked_sat_ids(); sat_idx++)
    {
      GPS_DEBUG_MSG(("[st-agps/invalidate] Invalidating sat %d, id %d, mask %d\r\n", gnss_stagps_gnsslib_id_to_sat_id(sat_idx), sat_idx, mask));
      ST_AGPS_invalidate( gnss_stagps_gnsslib_id_to_sat_id(sat_idx), mask);
    }
    if(!suspended)
    {
      ST_AGPS_start();
      ST_AGPS_start_initial_prediction();
    }
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSINVALIDATEOK,%d",mask);
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSINVALIDATEERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute dump of predicted ephemerides command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_dumpephindb( tChar *cmd_par)
{
  tChar out_msg[256];
  ephemeris_raw_t ephemeris;
  tChar* epointer = (tChar*)(&ephemeris);
  tInt index;
  tInt index2;
  tInt i;
  tInt sat_id;

  ST_AGPS_TST_ephdb_claim();
  for (sat_id = STAGPS_MIN_SAT_ID; sat_id <= STAGPS_MAX_SAT_ID; sat_id++)
  {
    if (STAGPS_SAT_ID_VALID(sat_id) == TRUE)
    {
      for (i=0;i<12;i++)
      {
        if(ST_AGPS_TST_ephmgr_dump_eph(sat_id,&ephemeris) == GNSS_NO_ERROR)
        {
          if( ephemeris.gps.available == 1)
          {
            index = _clibs_sprintf(out_msg, "%s,%i,%i,", stagps_nmea_cmd_strings[STAGPS_NMEA_CMDID_DUMPEPHINDB], sat_id, sizeof(ephemeris_raw_t));
            for (index2 = 0; index2 < sizeof(ephemeris_raw_t); index2++)
            {
              index += _clibs_sprintf(out_msg + index, "%02x",epointer[index2]);
            }
            index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
            nmea_send_msg_to_uart(out_msg, index);
            index = _clibs_sprintf(out_msg, "%sINFO,%i,%d,%d,%d*%02x\r\n", stagps_nmea_cmd_strings[STAGPS_NMEA_CMDID_DUMPEPHINDB], sat_id,ephemeris.gps.week,ephemeris.gps.toe,(ephemeris.gps.iodc & 0xFF),nmea_support_checksum(out_msg));
            nmea_send_msg_to_uart(out_msg, index);
          }
        }
        else
        {
          break;
        }
      }
    }
  }
  ST_AGPS_TST_ephdb_release();
}

/********************************************//**
 * \brief   Execute set predicted ephemerides command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_saveephindb( tChar *cmd_par)
{
  ephemeris_raw_t ephemeris;
  tInt             length;
  tInt             sat_id;
  tChar            hexbuffer[200];
  tChar*           epointer = (tChar *)(&ephemeris);
  tUInt            field_count;
  tUInt            cnt;
  tChar            out_msg[256];
  tInt             index = 0;

  field_count = _clibs_sscanf( cmd_par, ",%d,%d,%s" ,&sat_id, &length, hexbuffer);

  if( field_count == 3)
  {
    /* translate hex buffer to binary */
    for( cnt = 0 ; cnt < length ; cnt++)
    {
      epointer[cnt] = nmea_support_hex2toint( hexbuffer + (cnt * 2));
    }

    if( ST_AGPS_TST_ephmgr_fill_eph(sat_id, &ephemeris))
    {
      gpOS_task_delay( gpOS_timer_ticks_per_sec() / 100);
      if( ST_AGPS_TST_ephmgr_fill_eph(sat_id, &ephemeris)) // in case of error retry
      {
        /* report error */
        index = _clibs_sprintf(out_msg, "%sERROR,%i", stagps_nmea_cmd_strings[STAGPS_NMEA_CMDID_SAVEEPHINDB], sat_id);

        GPS_DEBUG_MSG(("[nmea]gnss_set_ephemeris_params failed!\r\n"));
      }
    }
    else
    {
      index = _clibs_sprintf(out_msg, "%sOK,%i", stagps_nmea_cmd_strings[STAGPS_NMEA_CMDID_SAVEEPHINDB], sat_id);
    }
    index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
    nmea_send_msg_to_uart(out_msg, index);
  }
}

/********************************************//**
 * \brief   Invalidate real ephemerides in AGPS command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_clearephindb( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt field_count = 0;
  tInt sat_id;
  tInt index;

  field_count = _clibs_sscanf( cmd_par, ",%d", &sat_id);

  if(field_count == 1)
  {
      ST_AGPS_invalidate( sat_id, ST_AGPS_CLR_EPHDB);
      index = _clibs_sprintf(out_msg, "$PSTMINDBCLEAROK,%d",sat_id);
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMINDBCLEARERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute real ephemerides update in AGPS command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_realephupdstatus(tChar *cmd_par)
{
  tChar out_msg[256];
  tInt field_count = 0;
  tInt status;
  tInt index;

  field_count = _clibs_sscanf( cmd_par,",%d",&status);

  if(field_count == 1)
  {
      ST_AGPS_TST_ephmgr_set_real_ephemeris_update(status);
      index = _clibs_sprintf(out_msg, "$PSTMREPHUPONOFFOK,%d",status);
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMREPHUPONOFFERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute predicted ephemerides update in AGPS command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_predephupdstatus(tChar *cmd_par)
{
  tChar out_msg[256];
  tInt field_count = 0;
  tInt status;
  tInt index;

  field_count = _clibs_sscanf( cmd_par, ",%d", &status);

  if(field_count == 1)
  {
      ST_AGPS_TST_ephmgr_set_predicted_ephemeris_update(status);
      index = _clibs_sprintf(out_msg, "$PSTMPEPHUPONOFFOK,%d",status);
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMPEPHUPONOFFERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute polynomials dump command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_polydump( tChar *cmd_par)
{
  tChar out_msg[256];
  tU8 buffer[STAGPS_BUFEER_SIZE] = {0};
  tS32 buf_len = STAGPS_BUFEER_SIZE;
  tChar hex_buffer[STAGPS_STORAGE_LINE_SIZE + 4] = {0};
  tS32 sat_id;
  tS32 item;
  tS32 field_count = 0;
  tS32 i;
  tS32 index;
  tS32 index2;
  tS32 sat_idx;
  tS32 sat_idx_min;
  tS32 sat_idx_max;
  tULong seed_t0, poly_timestamp;
  tS32 error = 0;
  boolean_t found;

  field_count =  _clibs_sscanf( cmd_par, ",%d" ,&sat_id);

  if(field_count == 1)
  {
    index = _clibs_sprintf(out_msg, "$PSTMPOLYDUMPOK,%d,", sat_id);

    if (sat_id > 0)
    {
      sat_idx_min = gnss_sat_id_to_gnsslib_id(sat_id);
      sat_idx_max = sat_idx_min;
    }
    else
    {
      sat_idx_min = 0;
      sat_idx_max = STAGPS_LINKED_SAT_IDS - 1;
    }

    for (sat_idx = sat_idx_min; sat_idx <= sat_idx_max; sat_idx++)
    {
      index2 = 0;
      found = FALSE;

      buf_len = STAGPS_BUFEER_SIZE;
      if (ST_AGPS_TST_sagpsmgr_dump_seed(sat_idx, &seed_t0, buffer, &buf_len) == GNSS_NO_ERROR)//if(ST_AGPS_TST_sagpsmgr_dump_seed(sat_idx, &seed) == GNSS_NO_ERROR)
      {
        for (i = 0; i < buf_len; i++)
        {
          index2 += _clibs_sprintf(hex_buffer + index2, "%02x", buffer[i]);
        if (index2 >= STAGPS_STORAGE_LINE_SIZE)
        {
            GPS_DEBUG_MSG(("[st-agps/dump/seed] %d,%d,%d,%s*\r\n", sat_idx, gnss_stagps_gnsslib_id_to_sat_id(sat_idx), seed_t0, hex_buffer));//GPS_DEBUG_MSG(("[st-agps/dump/seed] %d,%d,%d,%s\r\n", sat_idx, gnss_gnsslib_id_to_sat_id(sat_idx), seed.rxn_seed.state.t0, seed_buffer));
          index2 = 0;
          _clibs_memset(hex_buffer, 0, STAGPS_STORAGE_LINE_SIZE + 2);
        }
        }

        index2 = 0;
        _clibs_memset(hex_buffer, 0, STAGPS_STORAGE_LINE_SIZE + 2);

        for (item = 0; item < STAGPS_POLYS_PER_SATELLITE; item++)
        {
          buf_len = STAGPS_BUFEER_SIZE;
          if (ST_AGPS_TST_sagpsmgr_dump_poly(sat_idx, item, &poly_timestamp, buffer, &buf_len) == GNSS_NO_ERROR)//if(ST_AGPS_TST_sagpsmgr_dump_poly(sat_idx, item, &poly) == GNSS_NO_ERROR)
          {
            found = TRUE;

            for (i = 0; i < buf_len; i++)
            {
              index2 += _clibs_sprintf(hex_buffer + index2, "%02x", buffer[i]);
            }

            if (index2 >= STAGPS_STORAGE_LINE_SIZE)
            {
              GPS_DEBUG_MSG(("[st-agps/dump/poly] %d,%d,%d,%s*\r\n", sat_idx, gnss_stagps_gnsslib_id_to_sat_id(sat_idx), poly_timestamp, hex_buffer));//GPS_DEBUG_MSG(("[st-agps/dump/poly] %d,%d,%d,%s\r\n", sat_idx, gnss_gnsslib_id_to_sat_id(sat_idx), poly.timestamp, poly_buffer));
              index2 = 0;
              _clibs_memset(hex_buffer, 0, STAGPS_STORAGE_LINE_SIZE + 2);
            }
          }
          else
          {
            error = gnss_stagps_gnsslib_id_to_sat_id(sat_idx);
            break;
          }
        }

        if (index2 > 0)
        {
          GPS_DEBUG_MSG(("[st-agps/dump/poly] %d,%d,%d,%s*\r\n", sat_idx, gnss_stagps_gnsslib_id_to_sat_id(sat_idx), poly_timestamp, hex_buffer));
        }
      }

      if (!found)
      {
        GPS_DEBUG_MSG(("[st-agps/dump/poly] %d,%d,%d,NOT_FOUND!\r\n", sat_idx, gnss_stagps_gnsslib_id_to_sat_id(sat_idx), poly_timestamp));
      }
    }

    if (error != 0)
    {
      index = _clibs_sprintf(out_msg, "$PSTMPOLYDUMPERROR,%d", gnss_stagps_gnsslib_id_to_sat_id(sat_idx));
    }
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMPOLYDUMPERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute load of polynomials from NMEA command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_polysave(tChar *cmd_par)
{
  tChar out_msg[256];
  tInt index;
  index = _clibs_sprintf(out_msg, "$PSTMPOLYSAVEERROR");
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute set poly manager status command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_setstatus_polymgr(tChar *cmd_par)
{
  tChar out_msg[256];
  tInt status;
  tInt field_count = 0;
  tInt index;

  field_count =  _clibs_sscanf(cmd_par, ",%d" ,&status);

  if(field_count == 1)
  {
    if (status == 1)
    {
      ST_AGPS_TST_sagpsmgr_start();
      index = _clibs_sprintf(out_msg, "$PSTMPOLSTARTED");
    }
    else
    {
      ST_AGPS_TST_sagpsmgr_suspend( ST_AGPS_SUSPEND_IMMEDIATELY);
      index = _clibs_sprintf(out_msg, "$PSTMPOLSUSPENDED");
    }
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMPOLONOFFERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute single real ephemeris invalidation command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_invalidateephem(tChar *cmd_par)
{
  tChar out_msg[256];
  tInt sat_id;
  tInt field_count = 0;
  tInt index;

  field_count =  _clibs_sscanf( cmd_par, ",%d" ,&sat_id);

  if(field_count == 1)
  {
    gnss_clear_sat_ephemeris(sat_id);
    index = _clibs_sprintf(out_msg, "$PSTMCLRSATEPHOK,%d",sat_id);
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMCLRSATEPHERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute colored bar status set command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_setbarcol( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt status;
  tInt field_count = 0;
  tInt index;

  field_count =  _clibs_sscanf( cmd_par, ",%d" ,&status);

  if(field_count == 1)
  {
    if (status == 0)
    {
      stagps_nmea_satbarcolor_status = FALSE;
      index = _clibs_sprintf(out_msg, "$PSTMBARCOLDISABLED");
    }
    else
    {
      stagps_nmea_satbarcolor_status = TRUE;
      index = _clibs_sprintf(out_msg, "$PSTMBARCOLENABLED");
    }
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMBARCOLERROR");
  }
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Updates the STAGPS library with seeds from the PGPS 7 seed file which is
 *          sent sat by sat with the nmea message string.
 *
 * \param   cmd_par   Command parameter containing the sat id, the seed for that single sat and the seed time
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_setsatseed( tChar *cmd_par)
{
  tChar out_msg[256];
  gnss_error_t ret = GNSS_NO_ERROR;
  tUInt prn, field_count;
  tChar seed[STAGPS_NMEA_PGPSSEEDMAXSIZE];
  tU8 seed_buf[STAGPS_NMEA_PGPSSEEDMAXSIZE / 2];
  tULong seed_t0;
  tULong seed_tcur;
  tLong seed_tau_gps, seed_tau_gps_dot;
  tUInt gps_utc_offset;
  tInt  len = 0, i, index;

  //GPS_DEBUG_MSG(("[nmea] setting pgps seed %s\r\n", msg));
  /* extract the seed prn */
  len = _clibs_strlen(&cmd_par[0]);

  if (len > STAGPS_NMEA_PGPSSEEDMAXSIZE)
  {
    ret = GNSS_ERROR;
  }
  else
  {
    for (i = len; (cmd_par[i] != '*') && (i > 0); i--)
    {
      cmd_par[i] = '\0';
    }
    cmd_par[i] = '\0';
    field_count = _clibs_sscanf(cmd_par, ",%d,%d,%d,%d,%d,%d,%s" , (tInt *)&prn, &seed_t0, &seed_tcur, &seed_tau_gps, &seed_tau_gps_dot, &gps_utc_offset, seed);
    if (field_count == 7)
    {
      if (gnss_stagps_sat_id_valid(prn) == FALSE)
      {
        GPS_DEBUG_MSG(("[nmea/pgps] seed prn (%d) in not valid - ST-AGNSS constellation is not enabled for that prn\r\n", prn));
      }
      else if ((prn != gnss_stagps_gnsslib_id_to_sat_id(stagps_plugin_handler->sat_idx_counter)) && (prn != MIN_GPS_SAT_ID))
      {
        GPS_DEBUG_MSG(("[nmea/pgps] error: seed prn in wrong order(expected %d, found %d)\r\n", gnss_stagps_gnsslib_id_to_sat_id(stagps_plugin_handler->sat_idx_counter), prn));
        ret = GNSS_ERROR;
      }
      else
      {
        if (prn == MIN_GPS_SAT_ID)
        {
          stagps_plugin_handler->sat_idx_counter = 0;
        }
        len = _clibs_strlen(&seed[0]);
        if ((len % 2) == 0)
        {
          //seed[len-4] = '\0';
          //GPS_DEBUG_MSG(("[nmea] prn %d, seed data: %s, seed t0 %d, seed t_cur %d, seed clock id %d\r\n", prn, seed, seed_t0, seed_tcur, seed_clkid));
          for (i = 0; i < len / 2; i++)
          {
            seed_buf[i] = nmea_support_hex2toint(&seed[i * 2]);
          }
        }
        else
        {
          GPS_DEBUG_MSG(("[nmea/pgps] error: seed format not correct\r\n"));
          ret = GNSS_ERROR;
        }
      }
    }
    else
    {
      GPS_DEBUG_MSG(("[nmea/pgps] error: wrong number of fields\r\n"));
      ret = GNSS_ERROR;
    }

    if (ret != GNSS_ERROR)
    {
      if ((len == 2) && (seed_buf[0] == 0))
      {
        //GPS_DEBUG_MSG(("[nmea] loading pgps seed empty for prn %d\r\n", prn));
      }
      else
      {
        //GPS_DEBUG_MSG(("[nmea] loading pgps seed from prn %d t0 %d\r\n", prn, seed_t0));
        if (pgps_load(prn, &seed_buf[0], seed_t0, seed_tcur, seed_tau_gps, seed_tau_gps_dot, gps_utc_offset))
        {
          GPS_DEBUG_MSG(("[nmea/pgps] seed for prn %d added, counter = %d\r\n", prn, stagps_plugin_handler->sat_idx_counter));
        }
        else
        {
          GPS_DEBUG_MSG(("[nmea/pgps] error loading seed for prn %d\r\n", prn));
          ret = GNSS_ERROR;
        }
      }
    }
  }

  if ((ret != GNSS_ERROR) || (gnss_stagps_sat_id_valid(prn) == FALSE))
  {
    stagps_plugin_handler->sat_idx_counter++;
    if (stagps_plugin_handler->sat_idx_counter > stagps_plugin_handler->sat_idx_pgps_max)
    {
      pgps_start();
      stagps_plugin_handler->sat_idx_counter = 0;
    }
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSSATSEEDOK");
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSSATSEEDERROR");
  }

  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Force PGPS propagation
 *
 * \param   cmd_par   Command parameter: NONE
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_pgpsprop( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt  index;

  pgps_start();
  stagps_plugin_handler->sat_idx_counter = 0;
  index = _clibs_sprintf(out_msg, "$PSTMSTAGPSFORCEPROPOK");

  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Reports next sat that will be set by setsatseed
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_getnextsat( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt  index;

  index = _clibs_sprintf(out_msg, "$PSTMSTAGPSSATSEEDNEXT,%d", gnss_stagps_gnsslib_id_to_sat_id(stagps_plugin_handler->sat_idx_counter));

  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Execute generate password command
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_exec_generatepwd( tChar *cmd_par)
{
  tChar out_msg[256];
  gnss_error_t ret = GNSS_NO_ERROR;
  tInt  len = 0, i, index;
  tUInt gps_seconds;
  tChar vendor_id[50];
  tChar device_id[50];
  tChar password[41];
  tULong time;
  tChar info[(50 * 2) + 2];
  tU8 field_count;

  /* extract the seed prn */
  len = _clibs_strlen(&cmd_par[0]);
  if (len > STAGPS_NMEA_PGPSSEEDMAXSIZE)
  {
    ret = GNSS_ERROR;
  }
  else
  {
    for (i = len; (cmd_par[i] != '*') && (i > 0); i--)
    {
      cmd_par[i] = '\0';
    }
    cmd_par[i] = '\0';

    field_count = _clibs_sscanf(cmd_par, ",%d,%s" , &time, info);

    if (field_count == 2)
    {
      gps_seconds = time;
      i = 0;
      while ((info[i] != '\0') && (info[i] != ','))
      {
        vendor_id[i] = info[i];
        i++;
      }
      vendor_id[i] = '\0';

      i++;
      index = i;
      while (info[i] != '\0')
      {
        device_id[i - index] = info[i];
        i++;
      }
      device_id[i - index] = '\0';
    }
    else
    {
      GPS_DEBUG_MSG(("[nmea][agps] error: wrong number of fields\r\n"));
      ret = GNSS_ERROR;
    }

    if (ret != GNSS_ERROR)
    {
      if (pgps_generate_pwd(vendor_id, device_id, gps_seconds, password) == FALSE)
      {
        ret = GNSS_ERROR;
      }
    }
  }

  if (ret != GNSS_ERROR)
  {
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSPASSRTN,%s", password);
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSPASSGENERROR");
  }

  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Force keplerization
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_keplerize(tChar * msg)
{
  extern void ST_AGPS_TST_keplerize(gnsslibid_t, ephemeris_raw_t*);
  //tChar out_msg[256];
  gnsslibid_t sat_idx;
  ephemeris_raw_t eph;

  GPS_DEBUG_MSG(("[st-agps/kepl] Keplerizing all sats!\r\n"));

  for (sat_idx = 0; sat_idx <= STAGPS_LINKED_SAT_IDS; sat_idx++)
  {
    ST_AGPS_TST_keplerize(sat_idx, &eph);
  }
  /*
  index = _clibs_sprintf(out_msg, "$PSTMERROR");
  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
  */
  return;
}

/********************************************//**
 * \brief   set ST-AGNSS constellation mask
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_cmdif_setconstmask ( tChar *cmd_par)
{
  tChar out_msg[256];
  tInt field_count;
  tUInt mask, stagps_mask = 0;
  //tInt sat_idx;
  tInt index;
  gnss_error_t ret = GNSS_NO_ERROR;

  //GPS_DEBUG_MSG(("[st-agps/const] Setting ST-AGNSS constellation mask!\r\n"));

  field_count=_clibs_sscanf( cmd_par,",%d",&mask);

  if( field_count == 1)
  {
    boolean_t suspended = ST_AGPS_suspended();

    if(!suspended)
    {
      ST_AGPS_suspend( ST_AGPS_SUSPEND_IMMEDIATELY);
    }

    //GPS_DEBUG_MSG(("[st-agps/const] Suspended ST-AGNSS constellation mask!\r\n"));

    if(mask & (1<<GNSS_SAT_TYPE_GPS))
    {
      stagps_mask |= STAGNSS_GPS_CONSTELLATION;
    }

    if (mask & (1<<GNSS_SAT_TYPE_GLONASS))
    {
      stagps_mask |= STAGNSS_GLO_CONSTELLATION;
    }

    if (mask & (1<<GNSS_SAT_TYPE_GALILEO))
    {
      stagps_mask |= STAGNSS_GAL_CONSTELLATION;
    }

    if (mask & (1<<GNSS_SAT_TYPE_COMPASS))
    {
      stagps_mask |= STAGNSS_COM_CONSTELLATION;
    }

    ret = ST_AGPS_set_constellation_mask( stagps_mask);

    if(!suspended)
    {
      ST_AGPS_start();
      ST_AGPS_start_initial_prediction();
    }

    //GPS_DEBUG_MSG(("[st-agps/const] Restarted ST-AGNSS constellation mask!\r\n"));
  }
  else
  {
    ret = GNSS_ERROR;
  }

  if (ret == GNSS_ERROR)
  {
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSSETCONSTMASKERROR");
    //GPS_DEBUG_MSG(("[st-agps/const] ERROR setting ST-AGNSS constellation mask!\r\n"));
  }
  else
  {
    index = _clibs_sprintf(out_msg, "$PSTMSTAGPSSETCONSTMASKOK,%d",mask);
    //GPS_DEBUG_MSG(("[st-agps/const] ST-AGNSS constellation mask has been set to %x\r\n", stagps_mask));
  }

  index += _clibs_sprintf(out_msg + index, "*%02x\r\n",nmea_support_checksum(out_msg));
  nmea_send_msg_to_uart(out_msg,index);
}

/********************************************//**
 * \brief   Send AGPS message
 *
 * \param   data_p void*
 * \return  None
 *
 ***********************************************/
static void stagps_nmea_outmsg_send_AGPS(void *data_p)
{
  /*{{{  decs*/
  tChar out_msg[256];
  tInt index=0;
  tInt sat_cnt[STAGPS_NUM_OF_LISTS] = {0};
  tU8 sat_list[STAGPS_NUM_OF_LISTS][STAGPS_MAX_LIST_LEN] = {0};
  tShort i, j, constellation_n, sat_n;
  tInt fix_status;
  raw_t *raw_data;
  raw_measure_list_t *raw_meas;
  tDouble hdop;
  tDouble pdop;
  tDouble vdop;
  tDouble gdop;

  /*}}}  */

  gnss_fix_get_dops_local(&pdop, &hdop, &vdop, &gdop,data_p);
  fix_status = gnss_fix_get_pos_status_local(data_p);
  raw_meas = gnss_fix_get_raw_measurements_local(data_p);
  raw_data = raw_meas->list;


  for(i = 0; i < TRK_CHANNELS_SUPPORTED; i++)
  {
    if (raw_meas->chans_used[i])
    {
      if ((gnss_sat_id_to_sat_type(raw_data[i].dsp.satid) == GNSS_SAT_TYPE_GPS) && (sat_cnt[0] < STAGPS_MAX_LIST_LEN))
      {
        pred_data_t pred_data;
        tInt age_days;

        gnss_get_ephemeris_predict_params(raw_data[i].dsp.satid,&pred_data);
        if ((pred_data.available != 0) && (stagps_nmea_satbarcolor_status == TRUE))
        {
          age_days = (tInt)(pred_data.age_h/24) + 1;
          sat_list[0][sat_cnt[0]] = raw_data[i].dsp.satid + (age_days * 32);
        }
        else
        {
          sat_list[0][sat_cnt[0]]  = raw_data[i].dsp.satid;
        }
        sat_cnt[0]++;
      }
      else if ((gnss_sat_id_to_sat_type(raw_data[i].dsp.satid) == GNSS_SAT_TYPE_GLONASS) && (sat_cnt[1] < STAGPS_MAX_LIST_LEN))
      {
        pred_data_t pred_data;
        tInt age_days;
        satid_t current_sat_id;

        gnss_get_ephemeris_predict_params(raw_data[i].dsp.satid,&pred_data);
        if (nmea_support_translate_satid(raw_data[i].dsp.satid,&current_sat_id) == GNSS_NO_ERROR)
        {
          if ((pred_data.available != 0) && (stagps_nmea_satbarcolor_status == TRUE))
          {
            age_days = (tInt)(pred_data.age_h/24) + 1;
            sat_list[1][sat_cnt[1]]  = current_sat_id + (age_days * 32);
          }
          else
          {
            sat_list[1][sat_cnt[1]]  = current_sat_id;
          }
          sat_cnt[1]++;
        }
      }
    }
  }

  for (constellation_n = 0; constellation_n < STAGPS_NUM_OF_LISTS; constellation_n++)
  {
    for (sat_n = 0; sat_n < STAGPS_MAX_LIST_LEN; sat_n += NMEA_GSV_MAX_SATS)
    {
      if (sat_list[constellation_n][sat_n] != 0)
      {
        switch (constellation_n)
        {
          case 0 :
            index = _clibs_sprintf(out_msg, "$PSTMAGPS");
            index += _clibs_sprintf(out_msg + index, ",A,%1d",fix_status);
            break;
          case 1 :
            index = _clibs_sprintf(out_msg, "$PSTMAGLO");
            index += _clibs_sprintf(out_msg + index, ",A,%1d",fix_status);
            break;
          default :
            break;
        }
        for(j = 0; j < NMEA_GSV_MAX_SATS; j++)
        {
          if (sat_list[constellation_n][j + sat_n] != 0)
          {
            index += _clibs_sprintf(out_msg + index, ",%02d", sat_list[constellation_n][j + sat_n]);
          }
          else
          {
            index += _clibs_sprintf(out_msg + index, ",");
          }
        }
        index += _clibs_sprintf(out_msg + index, ",%2.1f,%2.1f,%2.1f", pdop, hdop, vdop);
        index += _clibs_sprintf(out_msg + index, "*%02X", nmea_support_checksum(out_msg));
        index += _clibs_sprintf(out_msg + index, "\r\n");
        nmea_send_outmsg(out_msg, index, AGPS_NMEA_MSG);
      }
    }
  }
}

static tInt stagps_plugin_nmea_cmdif_parse( tChar *input_cmd_msg, tUInt cmd_size, tChar *cmd_par)
{
  tUInt cmd_id;

  if( nmea_support_cmdif_getid( input_cmd_msg, cmd_size, stagps_nmea_cmd_strings, STAGPS_NMEA_CMDID_NUMBER, &cmd_id) == NMEA_NOT_VALID)
  {
    return NMEA_NOT_VALID;
  }

  stagps_nmea_cmdif_exec_table[cmd_id]( cmd_par );

  return NMEA_OK;
}

/********************************************//**
 * \brief   Transmit AGPS related NMEA messages
 *
 * \param   param   NMEA parameters
 * \return  gpOS_SUCCESS
 *
 ***********************************************/
static gpOS_error_t stagps_plugin_nmea_outmsg_transmit( void *param)
{
  nmea_support_ext_params_t *nmea_params = (nmea_support_ext_params_t *)param;
  tUInt nmea_msg_list[2];

  nmea_msg_list[0] = nmea_params->msg_list[0];
  nmea_msg_list[1] = nmea_params->msg_list[1];

  if( nmea_msg_list_check(nmea_msg_list,AGPS_NMEA_MSG))
  {
    stagps_nmea_outmsg_send_AGPS( nmea_params->data_p);
  }

  return gpOS_SUCCESS;
}

static gpOS_error_t stagps_plugin_api( const gnssapp_plugins_cmd_t cmd, gnssapp_plugins_cmd_param_t *param)
{
  gpOS_error_t error = gpOS_SUCCESS;

  switch( cmd)
  {
    case GNSSAPP_PLUGINS_CMD_INIT:
      error = stagps_plugin_init( ( gpOS_partition_t *)param->data_ptr);
      break;

    case GNSSAPP_PLUGINS_CMD_GETVER:
      *((const tS8 **)param->data_ptr) = ST_AGPS_version();
      break;

    case GNSSAPP_PLUGINS_CMD_START:
      error = stagps_plugin_restart();
      break;

    case GNSSAPP_PLUGINS_CMD_SUSPEND:
      error = stagps_plugin_suspend();
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

gnssapp_plugins_handler_t stagps_gnssapp_plugin_handler =
{
  NULL,
  GNSSAPP_PLUGINS_ID_STAGPS,
  stagps_plugin_api,
  stagps_plugin_nmea_cmdif_parse,
  stagps_plugin_nmea_outmsg_transmit,
  #if defined( STBIN_LINKED )
  stbin_stagps_plugin_cmdif_parse,
  stbin_stagps_plugin_outmsg_transmit
  #else
  (gnssapp_plugins_stbin_cmdif_callback_t)NULL,
  (gnssapp_plugins_stbin_outmsg_callback_t)NULL
  #endif
};

#endif
