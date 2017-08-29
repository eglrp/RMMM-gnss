/*****************************************************************************
   FILE:          nmea_support.h
   PROJECT:       Cartesio
   SW PACKAGE:    NMEA frame module
------------------------------------------------------------------------------
   DESCRIPTION:   This module provides methods to handle a generic NMEA msg.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2006 STMicroelectronics, (CCD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      FB:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   08.07.04  |  FB  | Original version
*****************************************************************************/

#ifndef NMEA_SUPPORT_H
#define NMEA_SUPPORT_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define NMEA_SUPPORT_DATUM_WGS84_F    0.00335281066474
#define NMEA_SUPPORT_DATUM_WGS84_A    6378137.0
#define NMEA_SUPPORT_DATUM_M_DEGREE   (NMEA_SUPPORT_DATUM_WGS84_A * RADIANS)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  NMEA_NOT_VALID  = -2,
  NMEA_ERROR      = -1,
  NMEA_OK         = 0
} nmea_error_t;

typedef struct
{
  tUInt         msg_list[2];
  void *        data_p;
  tDouble       delay;
  gnss_time_t   utc_time;
} nmea_support_ext_params_t;

typedef struct
{
  satid_t       satid;
  tInt          azimuth;
  tInt          elevation;
  tDouble       CN0;
} nmea_support_sat_params_t;

typedef void (*nmea_support_cmdif_exec_t) ( tChar *);

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tDouble        nmea_support_adjust_gps_time              ( const tDouble);
extern tChar          nmea_support_checksum                     ( const tChar *);
extern nmea_error_t   nmea_support_verify_checksum              ( const tChar *string);
extern void           nmea_support_extrapolate_pos              ( const position_t *, const velocity_t *, const tDouble , position_t *);
extern void           nmea_support_degrees_to_int               ( const tDouble, const tChar, const tChar, const tInt, tInt *, tInt *, tInt *, tChar *);
extern tU8            nmea_support_hextoint                     ( const tChar ch);
extern tU8            nmea_support_hex2toint                    ( const tChar *twoch);
extern void           nmea_support_swreset                      ( void);
extern nmea_error_t   nmea_support_translate_satid              ( satid_t, satid_t *);
extern void           nmea_support_initcoldstart                ( tU32 type);
extern nmea_error_t   nmea_support_cmdif_getcmdinfo             ( const tChar *incmdstr, tUInt *cmd_size_ptr, tChar **par_ptr);
extern nmea_error_t   nmea_support_cmdif_getid                  ( const tChar *cmd, const tUInt cmd_size, const tChar * const *cmd_string_table, const tUInt max_idx, tUInt *cmdid_ptr);
extern void           nmea_support_restart                      ( void);
extern void           nmea_support_set_first_timing_msg_flag    ( boolean_t );
extern boolean_t      nmea_support_get_first_timing_msg_flag    ( void);
extern void           nmea_support_set_first_gga_msg_flag       ( boolean_t );
extern boolean_t      nmea_support_get_first_gga_msg_flag       ( void);
extern boolean_t      nmea_support_get_new_msg_event_flag       ( gpOS_clock_t cpu_time );
extern void           nmea_support_reset_nmea_restart_cpu_time  ( void);
extern void           nmea_support_set_n_acq_done_flag          ( boolean_t );
extern boolean_t      nmea_support_get_n_acq_done_flag          ( void );
extern void           nmea_support_enter_standby_mode           ( tUInt);

#endif /* __NMEA_SUPPORT_H__ */
