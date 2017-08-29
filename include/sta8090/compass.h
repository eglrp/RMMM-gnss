/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995-2012
                   All Rights Reserved

Source file name : compass.h
Description:       Compass Library

Date              Modification
13  Jan 12        Created

************************************************************************/
/*}}}  */

#ifndef COMPASS_H
#define COMPASS_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "mathconst.h"
#include "gnss_defs.h"
#include "gnss_const.h"
#include "compass_sat_id.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

//#define COMPASS_CORRECT_PARITY_ERRORS     //MUST BE NOT activated because wrong decoding observed when bit error on parity bit

// Compass Time
#define COMPASS_INIT_UTC_DELTA_TLS          14    // Leap Seconds at 2006 Jan. 1st UTC 00:00
#define COMPASS_INIT_WEEK                   1356  // GPS Week at 2006 Jan 1st UTC 00:00

#if !defined(GPS_TIME_VALID)
#define WEEK_VALID(week)                    ((((week) >= MIN_WEEK_N) && ((week) < MAX_WEEK_N))      ? TRUE:FALSE)
#define TOW_VALID(tow)                      ((((tow) >= 0.0) && ((tow) <= (tDouble)SECS_PER_WEEK))  ? TRUE:FALSE)
#define GPS_TIME_VALID(time)                (((WEEK_VALID((time).week_n) == TRUE) && (TOW_VALID((time).tow) == TRUE)) ? TRUE:FALSE)
#endif

// Ephemeris
#define COMPASS_IS_MANEUVER(urai)           (((urai)==15)?TRUE:FALSE)
#define COMPASS_IS_URAI_GOOD(urai)          (((urai)<15)?TRUE:FALSE)

#define COMPASS_EPH_FLAG_MASK_AVAIL         1
#define COMPASS_EPH_FLAG_MASK_HEALTH        2
#define COMPASS_EPH_FLAG_MASK_PREDICTED     4

// Packer
#define COMPASS_WORDS_PER_SUBFRAME          10
#define COMPASS_BITS_PER_WORD               30
#define COMPASS_MEO_TIME_OFFSET             6.0
#define COMPASS_GEO_TIME_OFFSET             0.6

// Subframe fields
#define COMPASS_EXTRACT_TOA(word)           (((word)[6] <<  3) & 0xFF000)
#define COMPASS_EXTRACT_FRAID(word)         (((word)[0] >> 12) & 0x7)
#define COMPASS_EXTRACT_PNUM1(word)         (((word)[1] >> 14) & 0xF)
#define COMPASS_EXTRACT_PNUM2(word)         (((word)[1] >> 13) & 0xF)
#define COMPASS_EXTRACT_PAGENUM(word)       (((word)[1] >> 10) & 0x7F)
#define COMPASS_EXTRACT_MEO_HEALTH(word)    (((word)[1] >> 17) & 0x1)
#define COMPASS_EXTRACT_GEO_HEALTH(word)    (((word)[1] >> 13) & 0x1)
#define COMPASS_EXTRACT_GEO_SAT_H2(word)    (((word)[1] >> 11) & 0x3)
#define COMPASS_EXTRACT_SOW(word)           ((((word)[0] << 8) & 0xFF000) | (((word)[1] >> 18) & 0xFFF))
#define COMPASS_EXTRACT_MEO_WEEK(word)      (((word)[2] >> 17) & 0x1FFF)
#define COMPASS_EXTRACT_GEO_WEEK(word)      (((word)[2] >> 13) & 0x1FFF)
#define COMPASS_EXTRACT_MEO_URAI(word)      (((word)[1] >>  8) & 0xF)
#define COMPASS_EXTRACT_GEO_URAI(word)      (((word)[2] >> 26) & 0xF)
#define COMPASS_EXTRACT_TOA_FRAID5(word)    ((((word)[6] >> 5) & 0xF8) | (((word)[7] >> 27) & 0x07))
#define COMPASS_EXTRACT_WNA_FRAID5(word)    (((word)[6] >> 13) & 0xFF)

// Orbits
#define COMPASS_PI                          3.1415926535898
#define COMPASS_C                           2.99792458E+8
#define COMPASS_U                           3.986004418E+14     /* earths universal gravitational param  */
#define COMPASS_ROOT_U                      19964980.38566      /* root of earths universal gravitational param  */
#define COMPASS_OMEGA_ZERO_DOT              7.2921150E-5        /* earths rotation rate radians/s           */
#define COMPASS_F                           (-4.442807309E-10)  /* = u/c, given p */

#define COMPASS_MAX_SAT_POS                 45000000.0
#define COMPASS_MIN_SAT_POS                 (-45000000.0)
#define COMPASS_MIN_SAT_VEL                 (-6000)
#define COMPASS_MAX_SAT_VEL                 6000
#define COMPASS_MIN_SAT_ACC                 (-10)
#define COMPASS_MAX_SAT_ACC                 10

#define COMPASS_MAX_ROOTA                   6600.0
#define COMPASS_MIN_ROOTA                   5000.0
#define COMPASS_MAX_ECCENTRICITY            0.03
#define COMPASS_MIN_ECCENTRICITY            0.0
#define COMPASS_MAX_INCLINATION             (60.0 * RADIANS)
#define COMPASS_MIN_INCLINATION             (0.0 * RADIANS)

// Diff and Integrity
#define COMPASS_DIFF_MAX_ID_NUM             18       // max. number of broadcast corrections
#define COMPASS_DIFF_UDREI_NOT_AVAILABLE    15
#define COMPASS_DIFF_DELTA_T_NOT_AVAILABLE  (0x1000) // -4096 two-complement on 13-bit
#define COMPASS_DIFF_DELTA_T_SCALING        10
#define COMPASS_DIFF_MAX_FAST_DATA_AGE      (2*60)   // [s]

#define COMPASS_Re                          6378.137
#define COMPASS_Hi                          375.0
#define COMPASS_Seventy                     (70.0*RADIANS)

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef tU16    compass_prn_t;
typedef tUInt   compass_word_t;

typedef struct
{
  compass_word_t  word[COMPASS_WORDS_PER_SUBFRAME];
} compass_subframe_data_t;

typedef struct compass_ephemeris_raw_tag
{
  tUInt   inclination         : 32;
  tUInt   eccentricity        : 32;
  tUInt   root_a              : 32;
  tUInt   mean_anomaly        : 32;
  tUInt   omega_zero          : 32;
  tUInt   perigee             : 32;

  tUInt   toe                 : 17;
  tUInt   time_group_delay    : 10;
  tUInt   aode                :  5;

  tUInt   omega_dot           : 24;
  tUInt   A0                  :  8;

  tUInt   af0                 : 24;
  tUInt   A1                  :  8;

  tUInt   sow                 : 20;
  tUInt   af2                 : 11;
  tUInt   is_geo              :  1;

  tUInt   af1                 : 22;
  tUInt   subframe_avail      : 10;

  tUInt   motion_difference   : 16;
  tUInt   A2                  :  8;
  tUInt   A3                  :  8;

  tUInt   crs                 : 18;
  tUInt   B2                  :  8;
  tUInt   urai                :  4;
  tUInt   ephems_n            :  2;

  tUInt   crc                 : 18;
  tUInt   B3                  :  8;
  tUInt   aodc                :  5;
  tUInt   spare0              :  1;
  //tUInt spare1                :  1;

  tUInt   cus                 : 18;
  tUInt   i_dot               : 14;

  tUInt   cuc                 : 18;
  tUInt   B0                  :  8;
  tUInt   spare1              :  6;
  //tUInt   spare2              :  6;

  tUInt   cis                 : 18;
  tUInt   B1                  :  8;
  tUInt   time_distance_h     :  6;
  //tUInt   spare3              :  6;

  tUInt   cic                 : 18;
  tUInt   nvm_reliable        :  1;
  tUInt   predicted           :  1;
  tUInt   age_h               : 10;
  tUInt   spare4              :  2;

  tUInt   toc                 : 17;
  tUInt   week                : 13;
  tUInt   available           :  1;
  tUInt   health              :  1;
} compass_ephemeris_raw_t;

typedef struct compass_almanac_raw_tag
{
  tUInt   prn                 :  8;
  tUInt   week                : 16;
  tUInt   toa                 :  8;

  tUInt   eccentricity        : 17;
  tUInt   af0                 : 11;
  tUInt   is_geo              :  1;
  tUInt   WNa_valid           :  1;
  tUInt   spare0              :  2;

  tUInt   omega_dot           : 17;
  tUInt   af1                 : 11;
  tUInt   spare1              :  4;

  tUInt   root_a              : 24;
  tUInt   spare2              :  8;

  tUInt   omega_zero          : 24;
  tUInt   spare3              :  8;

  tUInt   perigee             : 24;
  tUInt   spare4              :  8;

  tUInt   mean_anomaly        : 24;
  tUInt   spare5              :  8;

  tUInt   delta_i             : 16;
  tUInt   health              :  1;
  tUInt   available           :  1;
  tUInt   last_received_toa   :  8;
  tUInt   spare6              :  6;
} compass_almanac_raw_t;

typedef struct compass_iono_raw_tag
{
  tUInt   A0          :  8;
  tUInt   A1          :  8;
  tUInt   A2          :  8;
  tUInt   A3          :  8;
  tUInt   B0          :  8;
  tUInt   B1          :  8;
  tUInt   B2          :  8;
  tUInt   B3          :  8;
  tUInt   spare0      : 31;
  tUInt   available   :  1;
} compass_iono_raw_t;

typedef struct compass_utc_raw_tag
{
  tUInt   A0          : 32;
  tUInt   A1          : 24;
  tUInt   delta_tls   :  8;
  tUInt   delta_tlsf  :  8;
  tUInt   DN          :  8;
  tUInt   WNlsf       :  8;
  tUInt   spare0      :  7;
  tUInt   available   :  1;
} compass_utc_raw_t;

typedef struct compass_aux_fields_tag
{
  tUInt   is_geo      :  1;
  tUInt   FraID       :  3;
  tUInt   Pnum        :  7;
  tUInt   Pnum1       :  4;
  tUInt   Pnum2       :  4;
  tUInt   week        : 13;

  tUInt   sow         : 20;
  tUInt   sow_valid   :  1;
  tUInt   health      :  1;
  tUInt   urai        :  4;
  tUInt   SatH2       :  2;
  tUInt   spare0      :  4;

  tUInt   WNa         :  8;
  tUInt   toa         :  8;
  tUInt   health_idx  :  1;
  tUInt   health_upd  :  1;
  tUInt   spare1      : 14;

  tUInt   health_mask;
} compass_aux_fields_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* COMPASS_H */
