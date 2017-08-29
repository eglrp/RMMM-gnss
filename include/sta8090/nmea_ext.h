//!
//!   \file       nmea_ext.h
//!   \brief      <i><b> NMEA extension module, header file</b></i>
//!   \author     Fulvio Boggia
//!   \authors    Many
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup NMEA
//!   \{
//!

#ifndef NMEA_EXT_H
#define NMEA_EXT_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gnss_api.h"
#include "nmea.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  NMEA_POWER_GNSS_NORMAL_MODE_1,  /* 0 */
  NMEA_POWER_GNSS_NORMAL_MODE_2,  /* 1 */
  NMEA_POWER_GPS_NORMAL_MODE,     /* 2 */
  NMEA_POWER_DUTY_CYCLE_MODE,     /* 3 */
  NMEA_POWER_RUN_MODE_1,          /* 4 */
  NMEA_POWER_RUN_MODE_2,          /* 5 */
  NMEA_POWER_WATCH_MODE,          /* 6 */
  NMEA_POWER_IDLE_MODE,           /* 7 */
  NMEA_POWER_STDBY_MODE,          /* 8 */
  NMEA_POWER_TRK_MODE,            /* 9 */
  NMEA_POWER_STOP_MODE,           /* 10 */
  NMEA_POWER_SMPS                 /* 11 */
} nmea_power_mode_cfg_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern nmea_error_t   nmea_ext_init                       ( gpOS_partition_t *part);
extern gpOS_error_t   nmea_ext_handle_output_msg          ( nmea_support_ext_params_t *param);
extern nmea_error_t   nmea_ext_cmdif_parse                ( tChar *input_cmd_msg, tUInt cmd_size, tChar *cmd_par);

#endif /* __NMEA_EXT_H__ */

//!   \}

