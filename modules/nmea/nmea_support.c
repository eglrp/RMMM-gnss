//!
//!   \file       nmea_support.c
//!   \brief      <i><b>NMEA support module, source file</b></i>
//!   \author     Many
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup NMEA
//!   \{
//!

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"
#include <math.h>

#include "typedefs.h"
#include "gpOS.h"

#include "gnss.h"
#include "gnss_api.h"
#include "gnss_debug.h"
#include "sw_config.h"
#include "gps_nvm.h"
#include "datum.h"

#include "nmea_support.h"
#include "gnssapp.h"
#include "platform.h"
#include "nmea.h"

#include "lld_rtc.h"
#include "lld_prcc_sta8090.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

extern const tInt gps_navigate_task_priority;

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define NMEA_SUPPORT_CMDMAXLENGTH     255

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static boolean_t            NMEA_first_timing_message = TRUE;
static boolean_t            NMEA_first_gga_message    = TRUE;
static gpOS_clock_t           NMEA_restart_cpu_time_event;
static boolean_t            NMEA_n_acq_done = FALSE;

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Adjusts the passed time of week (modulo 1 week)
 *
 * \param   t   the time of week
 * \return  the adjusted time of week
 *
 ***********************************************/
tDouble nmea_support_adjust_gps_time(tDouble t)
{
  if (t < 0)
  {
    t += SECS_PER_WEEK;
  }
  else if (t >= SECS_PER_WEEK)
  {
    t -= SECS_PER_WEEK;
  }

  return t;
}

/********************************************//**
 * \brief   Computes the checksum value for the NMEA string passed as
 *          an input. The string passed is supposed to start with the
 *          '$' starting delimiter and not end before the '*' checksum
 *          delimiter.
 *
 * \param   string  the string on which to compute the checksum
 * \return  the computed value
 *
 ***********************************************/
tChar nmea_support_checksum(const tChar *string)
{
  tInt index, checksum = 0;

  for(index = 1; string[index] != '\0'; index++)
  {
    checksum ^= string[index];
  }

  return checksum;
}

/********************************************//**
 * \brief   Verify string checksum
 *
 * \param   string    string to verify
 * \return  NMEA_OK if checksum is ok
 *
 ***********************************************/
nmea_error_t nmea_support_verify_checksum( const tChar *string)
{
  tInt str_checksum = 0;
  tInt local_checksum = 0;

  if( _clibs_strchr( string, '*') == NULL)
  {
    // No checksum to check
    return NMEA_OK;
  }

  // Discard $ character
  string++;

  while( *string != '*')
  {
    local_checksum ^= *string++;
  }

  if(_clibs_sscanf( string, "*%02x", &str_checksum) == 1)
  {
    if( local_checksum != str_checksum)
    {
      GPS_DEBUG_MSG(("[nmea] Command checksum error: found %02x, expected %02x\r\n", local_checksum, str_checksum));
      return NMEA_ERROR;
    }
  }

  return NMEA_OK;
}

/********************************************//**
 * \brief   Convert hex value to int
 *
 * \param   ch  char to convert
 * \return  Converted result
 *
 ***********************************************/
tU8 nmea_support_hextoint(const tChar ch)
{
  tInt chint = (tInt) ch;

  if ((chint >= (tInt)'0') && (chint <= (tInt)'9') )
    return (chint-(tInt)'0');

  if ((chint >= (tInt)'A') && (chint <= (tInt)'F') )
    return (10 + chint-(tInt)'A');

  if ((chint >= (tInt)'a') && (chint <= (tInt)'f') )
    return (10 + chint-(tInt)'a');

  return 0;
}

/********************************************//**
 * \brief   Convert two nibbles of a char to hex
 *
 * \param   twoch   character with nibbles to convert
 * \return  Converted result
 *
 ***********************************************/
tU8 nmea_support_hex2toint(const tChar* twoch)
{
  return ( (nmea_support_hextoint(twoch[0])<<4) + nmea_support_hextoint(twoch[1])  );
}

/********************************************//**
 * \brief       Extrapolate the user position after a given delay
 *              from a position fix
 *
 * \param[in]   pos         fix position
 * \param[in]   vel         fix velocity
 * \param[in]   delay       time delay since fix
 * \param[out]  extrap_pos  extrapolated position
 * \return      None
 *
 ***********************************************/
void nmea_support_extrapolate_pos( const position_t *pos, const velocity_t *vel, const tDouble delay, position_t *extrap_pos)
{
  extrap_pos->longitude = pos->longitude + (delay * vel->vel_east / (cos(pos->latitude*RADIANS) * NMEA_SUPPORT_DATUM_M_DEGREE));
  extrap_pos->latitude  = pos->latitude + (delay * vel->vel_north / NMEA_SUPPORT_DATUM_M_DEGREE);
  extrap_pos->height  = pos->height + (delay * vel->vel_vert);
}

/********************************************//**
 * \brief   Construct the real degrees into deg, min , min_fractions and 'N' or 'S'
 *
 * \param   real_degrees const tDouble
 * \param   plus_char const tChar
 * \param   minus_char const tChar
 * \param   decimal_places const tInt
 * \param   deg_int tInt*
 * \param   min_int tInt*
 * \param   min_frac_int tInt*
 * \param   sense_char tChar*
 * \return  None
 *
 ***********************************************/
void nmea_support_degrees_to_int( const tDouble real_degrees, const tChar plus_char, const tChar minus_char, const tInt decimal_places, tInt *deg_int, tInt *min_int, tInt *min_frac_int, tChar *sense_char)
{
  tInt i;
  tInt frac_scale = 1;
  tDouble half_round_value;
  tDouble abs_real_degrees;
  tDouble real_mins;

  for (i = 0; i < decimal_places; i++)
  {
    frac_scale *= 10;
  }

  half_round_value = 0.5 / (tDouble)(frac_scale * 60.0);

  if(real_degrees >= 0.0)
  {
        abs_real_degrees = real_degrees + half_round_value;
        *sense_char = plus_char;
  }
  else
  {
        abs_real_degrees = -real_degrees + half_round_value;
        *sense_char = minus_char;
  }


  *deg_int = (tInt)abs_real_degrees;
  real_mins = (abs_real_degrees - (tDouble)*deg_int) * 60.0;
  *min_int = (tInt)real_mins;
  *min_frac_int = (tInt)((real_mins- (tDouble)*min_int) * (tDouble)frac_scale);
}
/*}}}  */

/********************************************//**
 * \brief
 *
 * \param sat_id satid_t
 * \param translated_sat_id satid_t*
 * \return nmea_error_t
 *
 ***********************************************/
nmea_error_t nmea_support_translate_satid(satid_t sat_id, satid_t *translated_sat_id)
{
  nmea_error_t result = NMEA_OK;

  if(gnss_sat_id_to_sat_type(sat_id) == GNSS_SAT_TYPE_GLONASS)
  {
    tChar glonass_output_format;
    sw_config_get_param(CURRENT_CONFIG_DATA,GLONASS_OUTPUT_FORMAT_ID,&glonass_output_format);
    switch(glonass_output_format)
    {
      case GLONASS_OUTPUT_FORMAT_FREQ:
        *translated_sat_id = sat_id;
        result = NMEA_OK;
      break;

      case GLONASS_OUTPUT_FORMAT_SLOT:
        {
          tInt slot;
          if(gnss_glonass_get_satellite_slot(sat_id,&slot)==GNSS_NO_ERROR)
          {
            *translated_sat_id = 64+slot;
            result = NMEA_OK;
          }
          else
          {
            result = NMEA_ERROR;
          }
        }
      break;
    }
  }
  else if(gnss_sat_id_to_sat_type(sat_id) == GNSS_SAT_TYPE_SBAS)
  {
    *translated_sat_id = sat_id - 87;
    result = NMEA_OK;
  }
  else
  {
    *translated_sat_id = sat_id;
    result = NMEA_OK;
  }

  return result;
}

/********************************************//**
 * \brief
 *
 * \param type tChar
 * \return void
 *
 ***********************************************/
void nmea_support_initcoldstart( tU32 type)
{
  gpOS_task_delay(1000000);
  if (type & 0x1)
  {
    gnss_clear_all_almanacs();
  }

  /* set up bad ephemeris data */
  //nmea_cmdif_exec_warm_start();
  if (type & 0x2)
  {
    gnss_clear_all_ephems();
  }

  /* set up erroneous user position */
  /* the following values have been chosen such that the user position is invalidated */
  if (type & 0x4)
  {
    gnss_test_invalidate_user_pos();
    GPS_DEBUG_MSG(("invalid user position set ...\r\n"));
  }
  /* set up erroneous frequency */
  /*gnss_nmea_set_nco_value(FALSE, 0);
  GPS_DEBUG_MSG(("invalid nco frequency set ...\r\n"));*/

  /* set up an invalid rtc time */
  if (type & 0x8)
  {
    gnss_test_invalidate_rtc();
    GPS_DEBUG_MSG(("invalid rtc set ...\r\n"));
  }

  if (type & 0x10)
  {
    nvm_swap(TRUE);
  }

  if (type & 0x20)
  {
    gnss_test_invalidate_utc_params();
    GPS_DEBUG_MSG(("invalid UTC set ...\r\n"));
  }

  if (type & 0x40)
  {
    gnss_test_invalidate_iono_params();
    GPS_DEBUG_MSG(("invalid IONO set ...\r\n"));
  }
}

void nmea_support_swreset( void)
{

}

/********************************************//**
 * \brief   Get command info from command string
 *
 * \param[in]     incmdstr      Input command string
 * \param[out]    cmd_size_ptr  Pointer to command size
 * \param[out]    par_ptr       Pointer to parameters string
 * \return        NMEA_OK if a command is found
 *
 ***********************************************/
nmea_error_t nmea_support_cmdif_getcmdinfo( const tChar *incmdstr, tUInt *cmd_size_ptr, tChar **par_ptr)
{
  tUInt cmd_size = NMEA_SUPPORT_CMDMAXLENGTH;
  tChar *lupcharptr;

  // Check for parameters
  lupcharptr = _clibs_strchr( incmdstr, ',');

  if( lupcharptr == NULL)
  {
    // No parameters, check for CRC
    lupcharptr = _clibs_strchr( incmdstr, '*');

    if( lupcharptr == NULL)
    {
      register tUInt _r_len, _n_len;

      // No CRC, only command is present, check for \r and\or \n.
      lupcharptr = _clibs_strchr( incmdstr, '\r');
      _r_len = ( lupcharptr == NULL ) ? NMEA_SUPPORT_CMDMAXLENGTH : lupcharptr - incmdstr;

      lupcharptr = _clibs_strchr( incmdstr, '\n');
      _n_len = ( lupcharptr == NULL ) ? NMEA_SUPPORT_CMDMAXLENGTH : lupcharptr - incmdstr;

      cmd_size = ( _n_len < _r_len ) ? _n_len : _r_len;
    }
    else
    {
      // CRC found, set length
      cmd_size = lupcharptr - incmdstr;
    }
  }
  else
  {
    // Parameters found, set length
    cmd_size = lupcharptr - incmdstr;
  }

  if( cmd_size != NMEA_SUPPORT_CMDMAXLENGTH)
  {
    *cmd_size_ptr = cmd_size;
    *par_ptr = (tChar *)incmdstr + cmd_size;

    return NMEA_OK;
  }

  return NMEA_NOT_VALID;
}

/********************************************//**
 * \brief         Get command ID from command string
 *
 * \param[in]     cmd               Command string to check
 * \param[in]     cmd_size          Size of command string
 * \param[in]     cmd_string_table  String table to parse
 * \param[in]     max_idx           Max idx in subset
 * \param[out]    cmdid_ptr         Pointer to command ID
 * \return        NMEA_OK if ID is found
 *
 ***********************************************/
nmea_error_t nmea_support_cmdif_getid( const tChar *cmd, const tUInt cmd_size, const tChar * const *cmd_string_table, const tUInt max_idx, tUInt *cmdid_ptr)
{
  tUInt idx;

  for( idx = 0; idx < max_idx; idx++)
  {
    if( _clibs_strncmp( cmd, cmd_string_table[idx], _clibs_strlen(cmd_string_table[idx])) == 0)
    {
      *cmdid_ptr = idx;
      return NMEA_OK;
    }
  }

  return NMEA_NOT_VALID;
}

/********************************************//**
 * \brief   Execute procedures for a safe suspend and restart from NMEA
 *
 * \return  None
 *
 ***********************************************/
void nmea_support_restart( void)
{
  platform_gnss_suspend();

  // Wait 1sec to avoid spurious data on NMEA port.
  //gpOS_task_delay( gpOS_timer_ticks_per_sec());

  gnssapp_reset_startup_time();

  nmea_support_reset_nmea_restart_cpu_time();
  nmea_support_set_first_timing_msg_flag(TRUE);
  nmea_support_set_first_gga_msg_flag(TRUE);

  platform_gnss_restart();

  nmea_outmsg_transmit_short_header();
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
void nmea_support_set_first_timing_msg_flag( boolean_t value)
{
  NMEA_first_timing_message = value;
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
boolean_t nmea_support_get_first_timing_msg_flag( void)
{
  return NMEA_first_timing_message;
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
void nmea_support_set_first_gga_msg_flag( boolean_t value)
{
  NMEA_first_gga_message = value;
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
boolean_t nmea_support_get_first_gga_msg_flag( void)
{
  return NMEA_first_gga_message;

}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
boolean_t nmea_support_get_new_msg_event_flag( gpOS_clock_t cpu_time)
{
  boolean_t result = FALSE;
  gnssapp_startup_time_t * gnssapp_startup_time = NULL;
  gpOS_clock_t restart_cpu_time = NMEA_restart_cpu_time_event;

  gnssapp_startup_time     = gnssapp_get_startup_time();

  if ((boolean_t)gpOS_time_after( gnssapp_startup_time->gnssapp_init_end_cpu_time, gnssapp_startup_time->gnss_lib_set_timer_clock) != FALSE)
  {
    result = (boolean_t)gpOS_time_after( cpu_time, restart_cpu_time);
  }

  return result;
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
void nmea_support_reset_nmea_restart_cpu_time( void)
{
  NMEA_restart_cpu_time_event = gpOS_time_now();
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
void nmea_support_set_n_acq_done_flag( boolean_t value)
{
  NMEA_n_acq_done = value;
}

/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
boolean_t nmea_support_get_n_acq_done_flag( void)
{
  return NMEA_n_acq_done;
}


/********************************************//**
 * \brief
 *
 * \return  None
 *
 ***********************************************/
void nmea_support_enter_standby_mode( tUInt T)
{
  tU32 counter, alarm;

  if (T)
    {
      counter = LLD_RTC_GetDataRegister();
      alarm = counter + T;
      LLD_RTC_SetMatchRegister( alarm);

      LLD_RTC_EnableInterrupt();

    }

  /* Go in Stand By */
  LLD_PRCC_SetWakeupFromRTC( TRUE);
  //LLD_PRCC_SetWakeupFromSTDBYpin ( TRUE);
  LLD_PRCC_PeripheralClkDis( LLD_PRCC_PERIPHID_SRAM);
  LLD_PRCC_EnableWakeup();
  //LLD_PRCC_StandByEnable();
  LLD_PRCC_EnterStandby();
}

//!   \}

