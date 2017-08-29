/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999
                   All Rights Reserved

Source file name : gnss_api.h
Author :           P D Bagnall

Header file for the GP6 Library interface.

Date        Modification                                    Initials
----        ------------                                    --------
01 Nov 96   Created                                         PDB

************************************************************************/

/*}}}  */

#ifndef GNSS_API_H
#define GNSS_API_H

/*{{{  includes*/

#include "gnss_os.h"
#include "gnss_defs.h"
#include "gnss_bsp_defs.h"
#include "gnss.h"

#include "compass.h"
#include "galileo.h"
/*}}}  */


#define GNSS_PPS_SAT_THRESHOLD_NOT_USED   0

#define GNSS_PPS_TIME_FILTER_OFF          1
#define GNSS_PPS_TIME_FILTER_ON           25

#define GNSS_DYNAMIC_VALIDITY_MASK  ((1<<GNSS_SAT_TYPE_GPS)|(1<<GNSS_SAT_TYPE_GLONASS)| \
                                     (1<<GNSS_SAT_TYPE_QZSS_L1_CA)|(1<<GNSS_SAT_TYPE_COMPASS)|(1<<GNSS_SAT_TYPE_GALILEO) |\
                                     (1<<GNSS_SAT_TYPE_L2C))


#define CLOCK_ACCL

#ifdef CLOCK_ACCL
#define N_VEL_STATE     5
#else
#define N_VEL_STATE     4
#endif

#define N_POS_STATE     5

/* Shift for the fix point conversion */
#define GNSS_SAT_POS_SHIFT                  (5U)
#define GNSS_PSEUDORANGE_SHIFT              (4U)
#define GNSS_SAT_VEL_SHIFT                  (14U)
#define GNSS_SAT_RANGE_RATE_CORRECT_SHIFT   (12U)   /*fp_s16_t*/
#define GNSS_SAT_RANGE_CORRECT_SHIFT        (10U)
#define GNSS_ATM_CORRECT_SHIFT              (6U)    /*fp_s16_t*/
#define GNSS_DIFF_RANGE_CORRECT_SHIFT       (7U)    /*fp_s16_t*/
#define GNSS_DIFF_RANGE_RATE_CORRECT_SHIFT  (12U)   /*fp_s16_t*/
#define GNSS_SEMIAXIS_E_SHIFT               (7U)    /*fp_s16_t*/
#define GNSS_SEMIAXIS_N_SHIFT               (7U)    /*fp_s16_t*/
#define GNSS_THETA_SHIFT                    (7U)    /*fp_s16_t*/
#define GNSS_CLOCK_OFFSET_SHIFT             (10U)
#define GNSS_GLONASS_PATH_DELAY_SHIFT       (7U)    /*fp_s16_t*/
#define GNSS_CLOCK_DRIFT_SHIFT              (10U)
#define GNSS_EHPE_SHIFT                     (1U)    /*fp_s16_t*/
#define GNSS_GEOID_MSL_SHIFT                (5U)    /*fp_s16_t*/
#define GNSS_POS_RES_SHIFT                  (14U)   /*fp_s32_t*/
#define GNSS_VEL_RES_SHIFT                  (8U)    /*fp_s16_t*/
#define GNSS_TOW_SHIFT                      (11U)
#define GNSS_FREQUENCY_SHIFT                (14U)
#define GNSS_USER_POS_SHIFT                 (8U)
#define GNSS_USER_VEL_SHIFT                 (21U)
#define GNSS_DOP_SHIFT                      (8U)    /*fp_s16_t*/
#define GNSS_POS_RMS_SHIFT                  (14U)   /*fp_s32_t*/
#define GNSS_VEL_RMS_SHIFT                  (8U)    /*fp_s16_t*/
#define GNSS_POS_COV_SHIFT                  (1U)    /*fp_s16_t*/
#define GNSS_VEL_COV_SHIFT                  (3U)    /*fp_s16_t*/

/* Maximum value of the constant below is 64 */
#define GNSS_DEFAULT_SAT_LIST_SIZE          16

/*{{{  typedefs*/
/*{{{  diff_mode_t*/
typedef enum gnss_state_s {
  GNSS_ENGINE_STATUS_UNINIT,
  GNSS_ENGINE_STATUS_LOADED,
  GNSS_ENGINE_STATUS_RUNNING,
  GNSS_ENGINE_STATUS_SUSPENDED
} gnss_engine_status_t;

/*{{{  operation_property_t*/
typedef enum
{
  GNSS_OPERATION_ORBIT_LIST_POPULATE   = 0,
  GNSS_OPERATION_LMS_SET_LS_ONLY       = 1,
  GNSS_OPERATION_POSITION_LOCK         = 2,
  GNSS_OPERATION_GLONASS_USE_MEASURES  = 10,
  GNSS_OPERATION_GLONASS_DELAY_LOCK    = 11
} operation_property_t;
/*}}}  */

/*{{{  diff_mode_t*/
typedef enum
{
  DIFF_MODE_ALWAYS = 0,
  DIFF_MODE_AUTO   = 1,
  DIFF_MODE_NEVER  = 2
} diff_mode_t;
/*}}}  */

typedef struct ext_fix_fp_tag
{
  ECEF_pos_fp_t  ECEF_pos;
  ECEF_vel_fp_t  ECEF_vel;
  ECEF_pos_fp_t  ECEF_pos_raw;
  ECEF_vel_fp_t  ECEF_vel_raw;
} ext_fix_fp_t;
/*}}}  */

 typedef struct math_fp_ellipse_s
{
	fp_s16_t semiaxis_e;
	fp_s16_t semiaxis_n;
	fp_s16_t theta;
} math_fp_ellipse_t;



/*{{{ residuals_t */
typedef struct residuals_fp_tag
{
  fp_s32_t pos_rms;
  fp_s32_t pos_rms_w_out_discarded;
  fp_s16_t vel_rms;
  fp_s16_t vel_rms_w_out_discarded;
} residual_fp_t;
/*}}} */

/*{{{  fix_init_t*/
typedef enum
{
  FULL = 0,
  REINIT= 1,
  BATTERY
} fix_init_t;
/*}}}  */

#define GNSS_DSP_DATA_MPATH_MAX_VALUE     3   // max. value of the mpath bit field

/*{{{  available_locked_t*/
typedef struct dsp_data_flags_tag {
  boolean_t available            : 1;
  boolean_t preamble_locked      : 1;
  tUInt     mpath                : 2;  // multi-path indicator
  tInt      state                : 5;
  tUInt     lli                  : 1;  // lli
  tUInt     selector             : 1;  // code/MF selector
} dsp_data_flags_t;
/*}}}  */


/*{{{  dsp_data_t*/
typedef struct dsp_data_tag {
  
  tInt                ave_phase_noise;
  fp_u32_t            pseudorange;        /* Change from double to fp_u32_t; scaling factor: 1<<4*/
  fp_s32_t            frequency;          /* Change from double to fp_s32_t; simple cast*/
  tInt                tracked_time;
  tU8                 signal_strength;
  tS8                 signal_strength_frac_part;
  satid_t             satid;
  dsp_data_flags_t    flags;
} dsp_data_t;
/*}}}  */

/*{{{  sat_data_t*/
typedef struct sat_data_tag {
  ECEF_pos_fp_t   sat_pos;
  ECEF_vel_fp_t   sat_vel;
  fp_s32_t        range_correction;       /* Change from double to fp_s32_t;   scaling factor: 1<<10   */
  fp_s16_t        atmospheric_correction; /* Change from double to fp_s16_t; scaling factor: 1<<5 */
  fp_s16_t        range_rate_correction;
  boolean_t       available;
} sat_data_t;
/*}}}  */

/*{{{  diff_data_t*/
typedef struct diff_data_tag {
  fp_s16_t            range_correction;             /*Scale factor of 1<<7*/
  fp_s16_t            range_rate_correction;        /*Scale factor of 1<<12*/
  diff_correction_t available;
} diff_data_t;

/*{{{  pred_data_t*/
typedef struct pred_data_tag {
  tUInt age_h              : 8;
  tUInt ephems_n           : 2;
  tUInt time_distance_h    : 6;
  tUInt available          : 1;
} pred_data_t;
/*}}}  */

/*{{{  raw_t*/
typedef struct raw_t_tag {
  dsp_data_t      dsp;
  sat_data_t      sat;
  diff_data_t     diff;
  pred_data_t     pred;
} raw_t;
/*}}}  */

/*{{{  raw_measure_list_t*/
typedef struct raw_measurement_list_tag
{
  tInt         list_size;
  boolean_t   chans_used[TRK_CHANNELS_SUPPORTED];
  boolean_t   chans_kf_used[TRK_CHANNELS_SUPPORTED];
  raw_t       list[TRK_CHANNELS_SUPPORTED];
} raw_measure_list_t;
/*}}}  */

#define DGPS_STATION_CHANNELS_SUPPORTED    24

/*{{{  dgps_station_dsp_data_t*/
typedef struct dgps_station_obs_tag {
  tU8 validity;
  satid_t sat_id;
  tDouble carrier_phase;
  tDouble pseudorange;
} dgps_station_obs_t;
/*}}}  */

/*{{{  dgps_station_sat_data_t*/
typedef struct dgps_station_sat_data_tag {
  boolean_t  available;
  ECEF_pos_t sat_pos;
  ECEF_vel_t sat_vel;
} dgps_station_sat_data_t;
/*}}}  */

/*{{{  dgps_station_raw_t*/
typedef struct dgps_station_raw_s {
  dgps_station_obs_t      obs;
  dgps_station_sat_data_t sat;
} dgps_station_raw_t;
/*}}}  */

/*{{{  gps_ephemeris_raw_t*/
typedef struct gps_ephemeris_raw_tag
{
  tUInt week                 : 16;
  tUInt toe                  : 16;

  tUInt toc                  : 16;
  tUInt iode1                : 8;
  tUInt iode2                : 8;

  tUInt iodc                 : 10;
  tUInt i_dot                : 14;
  tUInt spare1               : 8;

  tUInt omega_dot            : 24;
  tUInt ephems_n             : 2;
  tUInt time_distance_h      : 6;

  tUInt crs                  : 16;
  tUInt crc                  : 16;

  tUInt cus                  : 16;
  tUInt cuc                  : 16;

  tUInt cis                  : 16;
  tUInt cic                  : 16;

  tUInt motion_difference    : 16;
  tUInt age_h                : 10;
  tUInt spare3               : 6;

  tUInt inclination          : 32;
  tUInt eccentricity         : 32;
  tUInt root_a               : 32;
  tUInt mean_anomaly         : 32;
  tUInt omega_zero           : 32;
  tUInt perigee              : 32;

  tUInt time_group_delay     : 8;
  tUInt af2                  : 8;
  tUInt af1                  : 16;

  tUInt af0                  : 22;
  tUInt subframe1_available  : 1;
  tUInt subframe2_available  : 1;
  tUInt subframe3_available  : 1;
  tUInt available            : 1;
  tUInt health               : 1;
  tUInt predicted            : 1;
  tUInt accuracy             : 4;

} gps_ephemeris_raw_t;

/*}}}  */
/*{{{  glonass_ephemeris_raw_t*/

typedef struct glonass_ephemris_raw_tag
{
  tUInt week                  : 16;
  tUInt toe                   : 16;

  tUInt toe_lsb              : 4;
  tUInt NA                   : 11;
  tUInt tb                   : 7;
  tUInt M                    : 2;
  tUInt P1                   : 2;
  tUInt P3                   : 1;
  tUInt P2                   : 1;
  tUInt P4                   : 1;
  tUInt KP                   : 2;
  tUInt spare0               : 1;

  tUInt xn                   : 27;
  tUInt xn_dot_dot           : 5;
  tUInt xn_dot               : 24;
  tUInt n                    : 5;
  tUInt Bn                   : 3;

  tUInt yn                   : 27;
  tUInt yn_dot_dot           : 5;
  tUInt yn_dot               : 24;
  tUInt spare4               : 8;

  tUInt zn                   : 27;
  tUInt zn_dot_dot           : 5;
  tUInt zn_dot               : 24;
  tUInt ephems_n             : 2;
  tUInt time_distance_h      : 6;
  tUInt gamma_n              : 11;
  tUInt E_n                  : 5;
  tUInt freq_id              : 4;
  tUInt spare3               : 12;

  tUInt tau_n                : 22;
  tUInt age_h                : 10;

  tUInt tau_c                : 32;

  tUInt tau_GPS              : 22;
  tUInt spare5               : 10;

  tUInt NT                   : 11;
  tUInt N4                   : 5;
  tUInt tk                   : 12;
  tUInt FT                   : 4;

  tUInt spare7               : 32;

  tUInt m_available          : 5;
  tUInt nvm_reliable         : 1;
  tUInt spare8               : 26;

  tUInt spare9               : 25;
  tUInt available            : 1;
  tUInt health               : 1;
  tUInt predicted            : 1;
  tUInt spare10              : 4;
} glonass_ephemeris_raw_t;

/*}}}  */

/*{{{  ephemeris_raw_t*/
/*lint -estring(9018,*ephemeris_raw_t*) suppress declaration of symbol with union based type */
typedef union ephemris_raw_tag
{
  gps_ephemeris_raw_t       gps;
  glonass_ephemeris_raw_t   glonass;
  galileo_ephemeris_raw_t   galileo;
  compass_ephemeris_raw_t   compass;
} ephemeris_raw_t;

/*}}}  */

/*{{{  ST_AGPS_gps_ephemeris_raw_t*/
typedef struct ST_AGPS_gps_ephemeris_raw_tag
{
  tUInt week                 : 16;
  tUInt toe                  : 16;

  tUInt toc                  : 16;
  tUInt iode1                : 8;
  //tUInt iode2                : 8;
  //tUInt iodc                 : 10;
  //tUInt i_dot                : 14;
  tUInt spare0                : 8;

  //tUInt omega_dot            : 24;
  tUInt ephems_n             : 2;
  tUInt time_distance_h      : 6;
  tUInt time_group_delay     : 8;
  //tUInt crs                  : 16;
  //tUInt crc                  : 16;
  //tUInt cus                  : 16;
  //tUInt cuc                  : 16;
  //tUInt cis                  : 16;
  //tUInt cic                  : 16;
  tUInt motion_difference    : 16;

  //tUInt spare3               : 16;

  tUInt inclination          : 32;
  tUInt eccentricity         : 32;
  tUInt root_a               : 32;
  tUInt mean_anomaly         : 32;
  tUInt omega_zero           : 32;
  tUInt perigee              : 32;

  tUInt af2                  : 8;
  tUInt af0                  : 22;
  tUInt spare1               : 2;

  tUInt af1                  : 16;
  tUInt age_h                : 10;
  tUInt spare2               : 6;

  //tUInt subframe1_available  : 1;
  //tUInt subframe2_available  : 1;
  //tUInt subframe3_available  : 1;
  //tUInt available            : 1;
  //tUInt health               : 1;
  //tUInt predicted            : 1;
  //tUInt accuracy             : 4;
} ST_AGPS_gps_ephemeris_raw_t;
/*}}}  */

/*{{{  ST_AGPS_glonass_ephemeris_raw_t*/
typedef struct ST_AGPS_glonass_ephemris_raw_tag
{
  tUInt week                 : 16;
  tUInt toe                  : 16;

  tUInt toe_lsb              : 4;
  tUInt tau_GPS              : 22;
  tUInt spare1               : 6;

  //tUInt NA                   : 11;
  //tUInt tb                   : 7;
  //tUInt M                    : 2;
  //tUInt P1                   : 2;
  //tUInt P3                   : 1;
  //tUInt P2                   : 1;
  //tUInt P4                   : 1;
  //tUInt KP                   : 2;
  //tUInt spare0               : 1;

  tUInt xn                   : 27;
  tUInt xn_dot_dot           : 5;

  tUInt xn_dot               : 24;
  tUInt n                    : 5;
  tUInt Bn                   : 3;

  tUInt yn                   : 27;
  tUInt yn_dot_dot           : 5;

  tUInt yn_dot               : 24;
  tUInt spare4               : 8;

  tUInt zn                   : 27;
  tUInt zn_dot_dot           : 5;

  tUInt zn_dot               : 24;
  tUInt ephems_n             : 2;
  tUInt time_distance_h      : 6;

  tUInt gamma_n              : 11;
  tUInt E_n                  : 5;
  tUInt freq_id              : 4;
  tUInt spare3               : 12;

  tUInt tau_n                : 22;
  tUInt age_h                : 10;

  tUInt tau_c                : 32;

  //tUInt tau_GPS              : 22;
  //tUInt spare5               : 10;

  //tUInt NT                   : 11;
  //tUInt N4                   : 5;
  //tUInt tk                   : 12;
  //tUInt FT                   : 4;

  //tUInt spare7               : 32;

  //tUInt m_available          : 5;
  //tUInt nvm_reliable         : 1;
  //tUInt spare8               : 26;

  //tUInt spare9               : 25;
  //tUInt available            : 1;
  //tUInt health               : 1;
  //tUInt predicted            : 1;
  //tUInt spare10              : 4;
} ST_AGPS_glonass_ephemeris_raw_t;
/*}}}  */

/*{{{  ST_AGPS_galileo_ephemeris_raw_t*/
typedef struct ST_AGPS_galileo_ephemeris_raw_tag
{
  tUInt week                 : 16;
  tUInt motion_difference    : 16;

  tUInt toe                  : 14;
  tUInt toc                  : 14;
  tUInt available            : 1;
  tUInt health               : 1;
  tUInt ephems_n             : 2;

  tUInt af1                  : 21;
  tUInt age_h                : 10;
  tUInt spare0               : 1;

  tUInt iod_nav              : 10;
  tUInt af2                  : 6;
  tUInt spare1               : 16;

  //tUInt SISA                 : 8;

  //tUInt spare1               : 10;
  //tUInt BGD_E1_E5a           : 10;
  //tUInt BGD_E1_E5b           : 10;
  //tUInt E1BHS                : 2;

  tUInt inclination          : 32;
  tUInt eccentricity         : 32;
  tUInt root_a               : 32;
  tUInt mean_anomaly         : 32;
  tUInt omega_zero           : 32;
  tUInt perigee              : 32;

  //tUInt i_dot                : 14;

  //tUInt crs                  : 16;
  //tUInt crc                  : 16;
  //tUInt cus                  : 16;
  //tUInt cuc                  : 16;
  //tUInt cis                  : 16;
  //tUInt cic                  : 16;

  //tUInt omega_dot            : 24;
  //tUInt SVID                 : 6;
  //tUInt E1BDVS               : 1;
  //tUInt predicted            : 1;

  //tUInt word_available       : 5;

  tUInt af0                  : 31;
  tUInt spare2               : 1;
} ST_AGPS_galileo_ephemeris_raw_t;
/*}}}  */

/*{{{  compass_ephemeris_raw_t*/
typedef struct ST_AGPS_compass_ephemeris_raw_tag
{
  tUInt inclination       : 32;
  tUInt eccentricity      : 32;
  tUInt root_a            : 32;
  tUInt mean_anomaly      : 32;
  tUInt omega_zero        : 32;
  tUInt perigee           : 32;

  tUInt toe               : 17;
  tUInt aode              : 5;
  tUInt age_h             : 10;

  //tUInt time_group_delay  : 10;
  //tUInt omega_dot         : 24;
  //tUInt A0                : 8;

  tUInt af0               : 24;
  tUInt spare1            : 8;

  //tUInt time_group_delay  : 10;

  //tUInt A1                : 8;

  tUInt sow               : 20;
  tUInt af2               : 11;
  tUInt is_geo            : 1;

  tUInt af1               : 22;
  tUInt urai              : 4;
  tUInt aodc              : 5;
  tUInt spare2            : 1;

  //tUInt subframe_avail    : 10;

  tUInt motion_difference : 16;
  tUInt spare3            : 16;

  //tUInt A2                : 8;
  //tUInt A3                : 8;

  //tUInt crs               : 18;
  //tUInt B2                : 8;
  //tUInt spare0            : 2;

  //tUInt crc               : 18;
  //tUInt B3                : 8;
  //tUInt spare1            : 1;

  //tUInt cus               : 18;
  //tUInt i_dot             : 14;

  //tUInt cuc               : 18;
  //tUInt B0                : 8;
  //tUInt spare2            : 6;

  //tUInt cis               : 18;
  //tUInt B1                : 8;
  //tUInt spare3            : 6;

  //tUInt cic               : 18;
  //tUInt nvm_reliable      : 1;
  //tUInt spare4            : 13;

  tUInt toc               : 17;
  tUInt week              : 13;
  tUInt ephems_n          : 2;

  //tUInt available         : 1;
  //tUInt health            : 1;

} ST_AGPS_compass_ephemeris_raw_t;
/*}}}  */

/*{{{  ST_AGPS_ephemeris_raw_t*/
typedef union ST_AGPS_ephemris_raw_tag
{
  ST_AGPS_gps_ephemeris_raw_t     gps;
  ST_AGPS_glonass_ephemeris_raw_t glonass;
  ST_AGPS_galileo_ephemeris_raw_t galileo;
  ST_AGPS_compass_ephemeris_raw_t compass;
} ST_AGPS_ephemeris_raw_t;
/*}}}  */

/*{{{  glonass_m5_data_raw_t*/
typedef struct glonass_m5_data_raw_tag
{
  tUInt tau_c           : 32;
  tUInt NA              : 11;
  tUInt N4              : 5;
  tUInt spare0          : 16;
} glonass_m5_data_raw_t;
/*}}}  */

/*{{{  gps_almanac_raw_tag*/
typedef struct gps_almanac_raw_tag
{
  tUInt satid           : 8;
  tUInt week            : 16;
  tUInt toa             : 8;

  tUInt eccentricity    : 16;
  tUInt delta_i         : 16;

  tUInt omega_dot       : 16;
  tUInt spare0          : 16;

  tUInt root_a          : 24;
  tUInt spare1          : 8;

  tUInt omega_zero      : 24;
  tUInt spare2          : 8;

  tUInt perigee         : 24;
  tUInt spare3          : 8;

  tUInt mean_anomaly    : 24;
  tUInt spare4          : 8;

  tUInt af0             : 11;
  tUInt af1             : 11;
  tUInt health          : 1;
  tUInt available       : 1;
  tUInt spare5          : 8;

  tUInt spare6          : 32;
  tUInt spare7          : 32;
} gps_almanac_raw_t;

/*}}}  */

typedef struct glonass_almanac_raw_tag
{
  tUInt satid           : 8;
  tUInt week            : 16;
  tUInt spare0          : 8;

  tUInt toa             : 20;
  tUInt n_A             : 5;  // orbital slot
  tUInt H_n_A           : 5;  // channel
  tUInt M_n_A           : 2;  // 00=GLONASS 01=GLONASS-M

  tUInt tau_n_A         : 10;
  tUInt epsilon_n_A     : 15;
  tUInt spare1          : 7;

  tUInt t_lambda_n_A    : 21;
  tUInt spare2          : 11;

  tUInt lambda_n_A      : 21;
  tUInt spare3          : 11;

  tUInt delta_i_n_A     : 18;
  tUInt delta_T_n_dot_A : 7;
  tUInt spare4          : 7;

  tUInt delta_T_n_A     : 22;
  tUInt spare5          : 10;

  tUInt omega_n_A       : 16;
  tUInt health          : 1;
  tUInt available       : 1;
  tUInt spare6          : 14;

  tUInt tau_c           : 32;

  tUInt NA              : 11;
  tUInt N4              : 5;
  tUInt spare7          : 16;
} glonass_almanac_raw_t;

/*}}}  */
/*{{{  almanac_raw_t*/
/*lint -estring(9018,*almanac_raw_t*) suppress declaration of symbol with union based type */

typedef union almanac_raw_tag
{
  gps_almanac_raw_t       gps;
  glonass_almanac_raw_t   glo;
  galileo_almanac_raw_t   galileo;
  compass_almanac_raw_t   compass;
} almanac_raw_t;

/*}}}  */


/*{{{  utc_raw_t*/
typedef struct utc_raw_tag
{
  tUInt A0              : 32;
  tUInt A1              : 24;
  tUInt delta_tls       : 8;
  tUInt delta_tlsf      : 8;
  tUInt DN              : 8;
  tUInt tot             : 8;
  tUInt WNt             : 8;
  tUInt WNlsf           : 8;
  tUInt available       : 8;
  tUInt page18_offset   : 16;
} utc_raw_t;
/*}}}  */

/*{{{  glonass_utc_raw_t*/
typedef struct glonass_utc_raw_tag
{
  tUInt week_lsf    : 16;
  tUInt spare0      : 16;
  tUInt tow_lsf     : 32;
  tUInt B1          : 11;
  tUInt B2          : 10;
  tUInt KP          : 2;
  tUInt spare1      : 7;
  tUInt expired     : 1;
  tUInt available   : 1;
} glonass_utc_raw_t;
/*}}}  */

/*{{{  iono_raw_t*/
typedef struct iono_raw_tag
{
  tUInt A0          : 8;
  tUInt A1          : 8;
  tUInt A2          : 8;
  tUInt A3          : 8;
  tUInt B0          : 8;
  tUInt B1          : 8;
  tUInt B2          : 8;
  tUInt B3          : 8;
  tUInt available   : 8;
} iono_raw_t;
/*}}}  */

/*{{{  gnss_iono_raw_t*/
/*lint -estring(9018,*gnss_iono_raw_t*) suppress declaration of symbol with union based type */
typedef union gnss_iono_raw_tag
{
  iono_raw_t          gps;
  galileo_iono_raw_t  galileo;
  compass_iono_raw_t  compass;
} gnss_iono_raw_t;
/*}}}  */

/*{{{  tracker_data_t*/
typedef struct tracker_data_tag
{
  satid_t   sat_id;
  tInt      timer;
  tInt      state;
  tInt      frequency;
  tInt      ave_phase_noise;
  tInt      CN0;
  tInt      sync_lost;
} tracker_data_t;

/*}}}  */

typedef enum
{
  TRAIM_UNDER_ALARM = 0,
  TRAIM_OVER_ALARM  = 1,
  TRAIM_UNKNOWN     = 2
} traim_mode_t;

typedef struct traim_status_tag
{
  boolean_t             traim_valid;
  traim_mode_t          traim_solution;
  tShort                ave_error;             // nsec +/-32768 nsec
  satid_t               used_sat_id_table[TRK_CHANNELS_SUPPORTED];
  satid_t               removed_sat_id_table[TRK_CHANNELS_SUPPORTED];
  satid_t               used_sat_id;
  satid_t               removed_sat_id;
  tU8                   ref_second;            // 0 to 59
  //tInt                   traim_alarm;                        //TRK_CHANNELS_SUPPORTED bits set if alarm on.Lsb = index 0
  tShort                residual[TRK_CHANNELS_SUPPORTED];   // nsec +/-32768 nsec
  tDouble               mod_range[TRK_CHANNELS_SUPPORTED];
  tDouble               mod_time_data_received;
} traim_data_t;

/*{{{  pps_output_mode*/
typedef enum
{
  PPS_OUT_MODE_ALWAYS            = 0,
  PPS_OUT_MODE_ON_EVEN_SECONDS   = 1,
  PPS_OUT_MODE_ON_ODD_SECONDS    = 2
} pps_output_mode_t;
/*}}}  */

/*{{{  pps_reference_time*/
typedef enum
{
  PPS_TIME_REFERENCE_UTC                        = 0,
  PPS_TIME_REFERENCE_GPS_UTC                    = 1,
  PPS_TIME_REFERENCE_GLONASS_UTC                = 2,
  PPS_TIME_REFERENCE_UTC_SU                     = 3,
  PPS_TIME_REFERENCE_GPS_UTC_FROM_GLONASS_TIME  = 4,
  PPS_TIME_REFERENCE_COMPASS_UTC                = 5,
  PPS_TIME_REFERENCE_UTC_NTSC                   = 6
} pps_reference_time_t;
/*}}}  */

/*{{{  timing_data*/
typedef struct timing_data_s
{
  tU8                   used_sats;
  tU8                   fix_status;
  tU8                   constellation_mask;
  tU8                   elevation_mask;
  boolean_t             traim_enabled;
  tShort                traim_alarm;             // nsec +/-32768 nsec
  traim_data_t          traim_data;
} timing_data_t;
/*}}}  */

/*{{{  pps_data*/
typedef struct pps_data_s
{
  tDouble               rf_correction;  /*s*/
  tDouble               gps_rf_correction;  /*s*/
  tDouble               pulse_duration; /*s*/
  boolean_t             inverted_polarity;
  boolean_t             enabled;
  boolean_t             pps_valid;
  boolean_t             pps_synch_valid;
  tU8                   sat_threshold;
  fix_status_t          fix_condition;
  pps_output_mode_t     output_mode;
  gnss_sat_type_t       reference_constellation;
  pps_reference_time_t  reference_time;
  tShort                gps_utc_delta_time_s;
  tShort                gps_utc_delta_time_ns;
  tDouble               quantization_error;
  tDouble               pps_clk_freq_Hz;
  tDouble               tcxo_clk_freq_Hz;
  tU8                   pps_clk_setting;
  timing_data_t         applied_timing_data;
  tDouble               glonass_rf_correction;  /*s*/
  tShort                glonass_utc_delta_time_ns;

  tDouble               compass_rf_correction;  /*s*/
  tShort                compass_utc_delta_time_ns;
} pps_data_t;
/*}}}  */
typedef enum
{
  GNSS_LOWPWR_CYC_FULL_CONST,
  GNSS_LOWPWR_CYC_GPS_ONLY_REDUCED,
  GNSS_LOWPWR_CYC_EPH_REFRESH
} gnss_low_power_cyclic_state_t;

typedef enum
{
  GNSS_LOWPWR_PER_RESET,
  GNSS_LOWPWR_PER_SUSPENDED,
  GNSS_LOWPWR_PER_SUSPENDED_IMM_FIX,
  GNSS_LOWPWR_PER_FIX,
  GNSS_LOWPWR_PER_NO_FIX_ON_PERIOD,
  GNSS_LOWPWR_PER_NO_FIX_OFF_PERIOD,
  GNSS_LOWPWR_PER_EPH_REFRESH,
  GNSS_LOWPWR_PER_RTC_REFRESH
} gnss_low_power_periodic_state_t;

typedef struct gnss_low_power_cyclic_data_s
{
  gnss_low_power_cyclic_state_t state;
  tDouble                       ehpe;
  tDouble                       average_ehpe;
  tDouble                       average_ehpe_nmea;
  tU8                           counter_ehpe_IN;
  tU8                           counter_ehpe_OUT;
  tU8                           counter_NO_FIX_IN;
  tU8                           counter_NO_FIX_OUT;
  tShort                        average_NO_FIX;
  tShort                        sats_used;
  boolean_t                     reduced_type;
  boolean_t                     duty_cycle_on_off;
  tInt                          ms_off;
  boolean_t                     duty_cycle_state;
} gnss_low_power_cyclic_data_t;

typedef struct gnss_low_power_periodic_data_s
{
  gpOS_clock_t                    SavedTimerTicks;
  tU16                            NoFixCnt;
  tU8                             FixOnCnt;
  boolean_t                       NoFixCntSelector;
  boolean_t                       RTCTrimRequired;
} gnss_low_power_periodic_data_t;

typedef struct gnss_low_power_periodic_data_backup_s
{
  gnss_low_power_periodic_state_t state;
  gpOS_clock_t                    LastTransitionTime;
  gpOS_clock_t                    LastRTCTrimTime;
} gnss_low_power_periodic_data_backup_t;

typedef struct gnss_low_power_data_s
{
  boolean_t                      steady_state;
  boolean_t                      no_fix;
  boolean_t                      glonass_tow_referesh;
  tU8                            counter_glonass_eph_ON;
  gnss_sat_type_mask_t           eph_const_mask;
  tUInt                          eph_interval;
  boolean_t                      eph_long_refresh_timer;

  gnss_low_power_cyclic_data_t   cyclic;
  gnss_low_power_periodic_data_t periodic;
} gnss_low_power_data_t;



typedef struct sats_usage_tag {
  tU8 excluded_sats_doppl_raim                  : 1;
  tU8 excluded_sats_doppl_fde_soft              : 1;
  tU8 excluded_sats_doppl_fde_hard              : 1;
  tU8 excluded_sats_range_raim                  : 1;
  tU8 excluded_sats_range_fde_soft              : 1;
  tU8 excluded_sats_range_fde_hard              : 1;
  tU8 sats_used                                 : 1;
  tU8 sats_kf_used                              : 1;
} sats_usage_t;

typedef enum
{
  GNSS_NO_EXCLUSION=0U,
  GNSS_RAIM_EXCLUSION,
  GNSS_FDE_EXCLUSION
} gnss_exclusion_type_t;

/*{{{  dynamic_mng_data*/
typedef struct dynamic_mng_data_s
{
  tDouble    hd_acc_th;
  tDouble    ld_acc_th;
  tChar      hd_hysteresis_s;
  tChar      ld_stabilization_s;
} dynamic_mng_data_t;

typedef tVoid (*gnss_ephemeris_updated_callback_t)                  (satid_t sat_id);
typedef tVoid (*gnss_initvisiblesatslist_available_callback_t)      (void);

/*************************************************************************/
/* {{{{{{{{{{{{{{{{{{{{{{{    TYPES DEFs   }}}}}}}}}}}}}}}}}}}}}}}}}}}}}}*/
/*************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*{{{  exported functions*/
extern gnss_error_t         gnss_init( gnss_sat_type_mask_t, gnss_sat_type_mask_t);
extern gnss_error_t         gnss_init_p( gpOS_partition_t *, gnss_sat_type_mask_t, gnss_sat_type_mask_t,boolean_t,tUInt);
extern gnss_error_t         gnss_init_rtc( const rtc_switch_t, gnss_time_t*, gpOS_clock_t, tInt );
extern const tChar *        gnss_version( void );
extern const tChar *        gnss_supplier_id(void);
extern tUInt                gnss_get_cut_version( void);

#if defined(__STA2062__)
extern boolean_t            gnss_get_subsystem_version( tChar * );
extern tUInt                gnss_get_ekernel_version( void );
#endif

#if defined(__STA2064__)
extern boolean_t            gnss_get_subsystem_version( tChar * );
extern void                 gnss_set_cut_version( gnss_bsp_cut_t );
#endif

#if defined(__generic__)
extern boolean_t            gnss_get_subsystem_version( tChar * );
#endif

extern gnss_engine_status_t gnss_get_engine_status( tVoid);
extern gnss_error_t         gnss_start( void );
extern gnss_error_t         gnss_restart( void );
extern gnss_error_t         gnss_internal_restart( void );
extern gnss_error_t         gnss_suspend( void );
extern gnss_error_t         gnss_internal_suspend( void );
extern gnss_error_t         gnss_low_power_start(gnss_startup_mode_t);
extern void                 gnss_powerdown_request( void );
extern gnss_error_t         gnss_set_time( const tInt, const tInt, const tInt, const tInt, const tInt, const tInt, gnss_sat_type_t);
extern gnss_error_t         gnss_set_pos( const tDouble, const tDouble, const tDouble );
extern gnss_error_t         gnss_set_centre_freq( const tDouble );
extern gnss_error_t         gnss_set_freq_range( const tInt, const tInt );
extern tDouble              gnss_get_centre_freq( void );
extern tInt                 gnss_get_sat_iode( const satid_t );
extern sat_pos_type_t       gnss_get_sat_pos_type( const satid_t );
extern gnss_error_t         gnss_get_sat_az_el( const satid_t satid, tInt *az_p, tInt *el_p );
extern gnss_error_t         gnss_set_fix_rate( const tDouble );
extern tDouble              gnss_get_fix_rate( void );
extern boolean_t            gnss_get_time_valid( void );
extern gnss_error_t         gnss_enable_sat( const satid_t );
extern gnss_error_t         gnss_disable_sat( const satid_t );
extern void                 gnss_clear_all_ephems( void );
extern void                 gnss_clear_all_almanacs( void );
extern boolean_t            gnss_is_sat_enabled( const satid_t );
extern gnss_error_t         gnss_set_3D_dop_threshold( const tDouble, const tDouble, const tDouble, const tDouble );
extern void                 gnss_get_3D_dop_threshold( tDouble*, tDouble*, tDouble*, tDouble* );
extern gnss_error_t         gnss_set_2D_dop_threshold( const tDouble, const tDouble, const tDouble, const tDouble );
extern void                 gnss_get_2D_dop_threshold( tDouble*, tDouble*, tDouble*, tDouble* );
extern gnss_error_t         gnss_set_elevation_mask_angle( const tDouble );
extern gnss_error_t         gnss_set_elevation_mask_angle_positioning(const tDouble );
extern tDouble              gnss_get_elevation_mask_angle_positioning(void);
extern gnss_error_t         gnss_init_tracking_threshold( const tInt);
extern gnss_error_t         gnss_set_tracking_threshold( const tInt );
extern gnss_error_t         gnss_set_positioning_threshold(const tInt );
extern void                 gnss_set_sf_recovery_status(const boolean_t );

extern void                 gnss_init_2_5_ppm_support(const boolean_t );
extern void                 gnss_init_xtal_support(const boolean_t );
extern void                 gnss_set_2_5_ppm_support( const boolean_t );
extern boolean_t            gnss_get_2_5_ppm_support(void);

extern void                 gnss_acquisition_set_operational_mode( const gnss_sat_type_t, const tInt );

extern void                 gnss_tracker_events_on_debug_on_off(const boolean_t on_off);
extern void                 gnss_get_tracking_threshold( tInt *, tInt *, tInt * );
extern tDouble              gnss_get_elevation_mask_angle( void );
extern tInt                 gnss_get_sat_health( const satid_t );

extern void                 gnss_reset_diff_params( void );
extern gnss_error_t         gnss_set_diff_params( const satid_t, const tInt, const tDouble, const tDouble, const tDouble );
extern gnss_error_t         gnss_get_diff_params( const satid_t, tInt *, tDouble *, tDouble *, tDouble *, boolean_t * );
extern gnss_error_t         gnss_set_diff_mode( const diff_mode_t );
extern diff_mode_t          gnss_get_diff_mode( void );
extern gnss_error_t         gnss_diff_set_source_type( const gnss_diff_source_t );
extern gnss_error_t         gnss_diff_get_source_type( gnss_diff_source_t * );
extern gnss_error_t         gnss_diff_set_RTCM_flag( const boolean_t );
extern gnss_error_t         gnss_diff_get_RTCM_flag( boolean_t * );
extern boolean_t            gnss_diff_sbas_fast_correction_available( const satid_t );
extern boolean_t            gnss_diff_sbas_slow_correction_available( const satid_t );
extern boolean_t            gnss_diff_sbas_iono_correction_available( const satid_t );
extern boolean_t            gnss_diff_rtcm_correction_available( const satid_t );
extern gnss_error_t         gnss_set_sbas_diff_params( const satid_t, const tInt, const tDouble, const tDouble, const tDouble, const diff_correction_t );

extern gnss_error_t         gnss_waas_set_status( const satid_t, const gnss_waas_status_t );
extern gnss_error_t         gnss_waas_get_status( satid_t *, gnss_waas_status_t * );
extern gnss_error_t         gnss_waas_get_multi_ch_prn_and_status( const chanid_t, satid_t *, gnss_waas_status_t * );
extern gnss_error_t         gnss_waas_set_multi_ch_prn_and_status( const chanid_t, const satid_t, const gnss_waas_status_t );
extern gnss_error_t         gnss_waas_get_prn_to_decode( satid_t * );
extern gnss_error_t         gnss_waas_set_prn_to_decode( const satid_t );
extern boolean_t            gnss_waas_is_tracking( const tInt );

extern tDouble              gnss_get_utc_delta_time( void );
extern gnss_utc_delta_time_validity_t gnss_get_utc_delta_time_validity(void);
extern gnss_time_t          gnss_time_to_utc_time( const gnss_time_t , const gnss_sat_type_t );
extern gnss_error_t         gnss_get_ephemeris_params( const satid_t, ephemeris_raw_t *, boolean_t * );
extern gnss_error_t         gnss_set_ephemeris_params( const satid_t, const ephemeris_raw_t * );
extern tInt                 gnss_get_ephemeris_sizeof( const gnss_sat_type_t);
extern tInt                 gnss_get_almanac_sizeof( const gnss_sat_type_t);
extern void                 gnss_ephemeris_predicted_buffer_init( const ST_AGPS_ephemeris_raw_t * );
extern gnss_error_t         gnss_get_almanac_params( const satid_t, almanac_raw_t *, boolean_t * );
extern gnss_error_t         gnss_set_almanac_params( const satid_t, const almanac_raw_t * );
extern gnss_error_t         gnss_get_iono_params( gnss_iono_raw_t *, const gnss_sat_type_t );
extern gnss_error_t         gnss_set_iono_params( const gnss_iono_raw_t *, const gnss_sat_type_t );
extern gnss_error_t         gnss_get_utc_params( utc_raw_t * );
extern gnss_error_t         gnss_set_utc_params( const utc_raw_t * );
extern gnss_error_t         gnss_get_glonass_utc_params( glonass_utc_raw_t * );
extern gnss_error_t         gnss_set_glonass_utc_params( const glonass_utc_raw_t * );
extern gnss_error_t         gnss_get_utc_raw_data( utc_raw_t *);
extern gnss_error_t         gnss_clear_utc_raw_data( void);
extern gnss_error_t         gnss_invalidate_glonass_utc_params( void);

extern void                 gnss_get_sats_visible( visible_sats_data_t* );
extern void                 gnss_get_sats_visible_scaled(visible_sats_data_t *);
extern void                 gnss_update_sats_visible(void);
extern tDouble              gnss_get_sat_cn0( const satid_t );
extern tDouble              gnss_cpu_clock_rate_hi( void );

extern void                 gnss_fix_store( void );
extern void                 gnss_fix_read_claim( void );
extern void                 gnss_fix_read_release( void );
extern tDouble              gnss_fix_get_geoid_msl( void );
extern position_t*          gnss_fix_get_fil_pos( void );
extern velocity_t*          gnss_fix_get_fil_vel( void );
extern void                 gnss_fix_get_fil_pos_vel( position_t *, velocity_t * );
extern position_t*          gnss_fix_get_raw_pos( void );
extern velocity_t*          gnss_fix_get_raw_vel( void );
extern tUInt                gnss_fix_get_raw_pos_sats( void );
extern void                 gnss_fix_get_raw_pos_dops(tDouble *, tDouble *, tDouble *, tDouble *);
extern ECEF_pos_t*          gnss_fix_get_fil_ecef_pos( void );
extern ECEF_vel_t *         gnss_fix_get_fil_ecef_vel( void );
extern void                 gnss_fix_get_time( tInt *, tDouble *, gpOS_clock_t * );
extern void                 gnss_fix_get_time_data(tInt *, tDouble *, gpOS_clock_t *, master_timebase_t *);
extern gnss_sat_type_t      gnss_fix_get_time_sat_type(void);
extern time_validity_t      gnss_fix_get_time_validity( void );
extern void                 gnss_fix_get_stopped_duration( tInt * );
extern void                 gnss_fix_get_dops( tDouble *, tDouble *, tDouble *, tDouble * );
extern fix_status_t         gnss_fix_get_pos_status( void );
extern fix_status_t         gnss_fix_get_raw_pos_status( void );
extern diff_status_t        gnss_fix_get_diff_status( void );
extern tInt                 gnss_fix_get_num_sats_used( void );
extern tInt                 gnss_fix_get_num_sats_excluded( void );
extern pos_algo_t           gnss_fix_get_pos_algo( void );
extern boolean_t            gnss_fix_is_chan_used( const tInt );
extern raw_measure_list_t*  gnss_fix_get_raw_measurements( void );
extern tDouble              gnss_fix_get_position_rms_residual( void );
extern tDouble              gnss_fix_get_velocity_rms_residual( void );
extern void                 gnss_fix_get_position_residual( const tInt, tDouble *, boolean_t * );
extern void                 gnss_fix_get_position_residual_used( const tInt, tDouble *, boolean_t * );
extern void                 gnss_fix_get_velocity_residual( const tInt, tDouble  *, boolean_t * );
extern boolean_t            gnss_fix_get_excluded_sats_range(const tInt , gnss_exclusion_type_t *);
extern boolean_t            gnss_fix_get_excluded_sats_doppl(const tInt , gnss_exclusion_type_t *);
extern void                 gnss_fix_get_velocity_covariance( tDouble * N_cov, tDouble *E_cov, tDouble *V_cov );
extern void                 gnss_fix_get_position_covariance( tDouble * N_cov, tDouble *E_cov, tDouble *V_cov );
extern tDouble              gnss_fix_get_ehpe( void );
extern void                 gnss_fix_get_error_ellipse( math_ellipse_t * );
extern tDouble              gnss_fix_get_clock_drift( void );
extern tDouble              gnss_fix_get_clock_offset( void );
extern tDouble              gnss_fix_get_glonass_path_delay( void );

extern gnss_error_t         gnss_conv_vel_to_course_speed( const velocity_t *, tDouble *, tDouble * );
extern gnss_error_t         gnss_conv_vel_to_course_speed_3D( const velocity_t *, tDouble *, tDouble * );
extern gnss_error_t         gnss_get_date( const tInt, const tDouble, tInt *, tInt *, tInt* );
extern gnss_error_t         gnss_get_dayofyear( const tInt, const tDouble, tInt *, tInt* );
extern gnss_error_t         gnss_get_utc_time( const tDouble, tInt *, tInt *, tInt *, tInt * );
extern gnss_error_t         gnss_date_time_to_gps_time( const tInt, const tInt, const tInt, const tInt, const tInt, const tInt, gnss_time_t * );
extern boolean_t            gnss_date_time_valid( const tInt, const tInt, const tInt, const tInt, const tInt, const tInt );
extern gnss_error_t         gnss_get_sat_xyz_diff( const satid_t, tDouble *, tDouble *, tDouble * );
extern gnss_fde_status_t    gnss_get_fde_status( void );
extern void                 gnss_turn_stop_detection_on( boolean_t );
extern void                 gnss_turn_walking_mode_on( boolean_t );
extern void                 gnss_set_freq_ramp_mode( boolean_t );
extern boolean_t            gnss_get_stop_detection_status(void);
extern boolean_t            gnss_get_walking_mode_status(void);
extern void                 gnss_set_fde_status( gnss_fde_status_t );
extern time_validity_t      gnss_get_time_validity( void );
extern void                 gnss_rtc_set_alarm( tUInt );
extern void                 gnss_rtc_get_time( gnss_time_t *, gpOS_clock_t *, rtc_status_t *, time_validity_t * );
extern tInt                 gnss_get_noise_floor( void );
extern tInt                 gnss_get_noise_floor_raw(gnss_sat_type_t sat_type) ;
extern boolean_t            gnss_sat_tracking_check( const tInt );
extern gpOS_clock_t         gnss_get_tracker_time_now( void );
extern tUInt                gnss_get_error_status( void);

extern time_validity_t      gnss_time_get_validity    ( gnss_sat_type_t);
extern gnss_time_t          gnss_time_now             ( gnss_sat_type_t);
extern gnss_sat_type_t      gnss_time_get_master      ( void);
extern gnss_sat_type_t      gnss_time_get_aux         ( void);
extern time_validity_t      gnss_time_get_validity_raw( const gnss_sat_type_t);

extern master_timebase_t    gnss_time_to_mtb_time( const gnss_sat_type_t sat_type, const gnss_time_t *ref_gps_time );
extern gnss_time_t          gnss_time_then( const gpOS_clock_t, gnss_sat_type_t);
extern tow_t                gnss_time_lminus( const tow_t, const tow_t );
extern tow_t                gnss_time_lplus( const tow_t, const tDouble );
extern tDouble              gnss_time_minus( const gnss_time_t *, const gnss_time_t *);
extern gnss_time_t          gnss_time_plus( const gnss_time_t *, const tDouble );

extern void                 gnss_rtc_write( const gnss_time_reference_t, const time_validity_t, const boolean_t, const tDouble );
extern void                 gnss_rtc_read( gnss_time_t *, gpOS_clock_t *, rtc_status_t *, time_validity_t*, gnss_sat_type_t *, tDouble * );

extern gnss_error_t         gnss_test_set_operation( const operation_property_t, const tInt, const tInt, const tDouble * );

extern void                 gnss_ephemeris_broadcast_enable_sat( const satid_t );
extern void                 gnss_ephemeris_broadcast_disable_sat( const satid_t );
extern boolean_t            gnss_ephemeris_predicted( const satid_t );
extern gnss_error_t         gnss_get_ephemeris_predict_params( const satid_t, pred_data_t * );
extern void                 gnss_clear_sat_ephemeris( const satid_t );
extern tUInt                gnss_get_ephemeris_broadcast_onoff_status( void );
extern tUInt                gnss_get_multiconst_ephemeris_broadcast_onoff_status( gnss_sat_type_t );
extern boolean_t            gnss_get_sat_ephemeris_broadcast_onoff_status( const satid_t );

extern void                 gnss_set_tracker_jammer( tInt );
extern void                 gnss_set_rtt_tps( tDouble tps );

extern void                 gnss_test_set_user_pos( const tDouble, const tDouble, const tDouble );
extern void                 gnss_test_set_nco_value( const boolean_t, const nco_t );
extern void                 gnss_test_invalidate_rtc( void );
extern void                 gnss_test_invalidate_user_pos( void );
extern void                 gnss_test_invalidate_utc_params( void);
extern void                 gnss_test_invalidate_iono_params( void);
extern void                 gnss_test_set_nco_range(const nco_t, const nco_t);
extern void                 gnss_test_invalidate_nco_value(void);
extern void                 gnss_test_invalidate_nco_range(void);

extern void                 gnss_set_rf_test_mode_on_sat_n( const satid_t );
extern void                 gnss_reset_rf_test_mode( void );
extern boolean_t            gnss_rf_test_mode_del_sat_n (const satid_t test_sat_id);
extern void                 gnss_set_rf_test_mode_on_sat_n_cn0_trk_thr( satid_t, tU8);
extern void                 gnss_set_rf_test_mode_on_sat_n_cn0_trk_xtal_thr( satid_t test_sat_id, tU8 cn0_trk_threshold,tU8 cn0_xtal_nav_threshold);
extern gnss_error_t         gnss_set_constellation_mask( const gnss_sat_type_mask_t);
extern gnss_sat_type_mask_t gnss_get_constellation_mask( void);
extern gnss_error_t         gnss_set_constellation_usage_mask( const gnss_sat_type_mask_t);
extern gnss_sat_type_mask_t gnss_get_constellation_usage_mask( void);

extern gnss_error_t         gnss_init_tcxo_config( const tInt);
extern gnss_error_t         gnss_set_tcxo_config( const tInt);
extern tInt                 gnss_get_tcxo_config_selector( void);

extern void                 gnss_test_force_satellite_healthy_status(const satid_t, const tInt );
extern void                 gnss_test_get_forced_satellites_healthy_status(void);
extern void                 gnss_test_dsp_verification(tInt type, tInt satid, tInt par1,tInt par2,tInt par3,tInt par4,tInt par5);

extern gnss_error_t         gnss_pps_init( gpOS_partition_t *, const tDouble, const tDouble , const boolean_t );
extern void                 gnss_pps_enable_control( const boolean_t );
extern void                 gnss_pps_set_signal_on_off_status( const boolean_t);
extern boolean_t            gnss_pps_get_signal_on_off_status( void);
extern void                 gnss_pps_set_pulse_duration( const tDouble);
extern tDouble              gnss_pps_get_pulse_duration( void);
extern void                 gnss_pps_set_time_delay( const tDouble);
extern tDouble              gnss_pps_get_time_delay( void);
extern void                 gnss_pps_set_rf_compensation(gnss_sat_type_t, tDouble);
extern tDouble              gnss_pps_get_rf_compensation(gnss_sat_type_t );
extern void                 gnss_pps_set_polarity( const boolean_t);
extern boolean_t            gnss_pps_get_polarity( void);
extern tU8                  gnss_pps_get_used_sats( void);
extern fix_status_t         gnss_pps_get_fix_status( void);
extern void                 gnss_pps_set_sat_threshold( const tU8);
extern void                 gnss_pps_set_fix_condition( const fix_status_t );
extern void                 gnss_pps_set_elevation_mask( const tInt);
extern void                 gnss_pps_set_constellation_mask( const gnss_sat_type_mask_t);
extern void                 gnss_pps_set_reference_constellation( const gnss_sat_type_t );
extern void                 gnss_pps_set_reference_time(const pps_reference_time_t);

extern void                 gnss_pps_set_output_mode( const pps_output_mode_t );
extern tU8                  gnss_pps_get_sat_threshold( void);
extern fix_status_t         gnss_pps_get_fix_condition( void);
extern tInt                 gnss_pps_get_elevation_mask( void);
extern gnss_sat_type_mask_t gnss_pps_get_constellation_mask( void);
extern gnss_sat_type_t      gnss_pps_get_reference_constellation( void);
extern pps_reference_time_t gnss_pps_get_reference_time(void);
extern pps_output_mode_t    gnss_pps_get_output_mode( void);
extern boolean_t            gnss_pps_get_position_hold_status( void);
extern void                 gnss_pps_set_position_hold_status( const boolean_t status);
extern void                 gnss_pps_get_position_hold_ECEF_pos( tDouble *x, tDouble *y, tDouble *z);
extern void                 gnss_pps_get_position_hold_llh_pos( tDouble *lat,tDouble *lon,tDouble *h);
extern void                 gnss_pps_set_position_hold_ECEF_pos( const tDouble x, const tDouble y, const tDouble z);
extern void                 gnss_pps_set_position_hold_llh_pos( const tDouble lat,const tDouble lon,const tDouble h);
extern void                 gnss_pps_set_auto_hold_samples( const tUInt samples);
extern void                 gnss_pps_enable_traim ( const tDouble alarm);
extern void                 gnss_pps_disable_traim ( void);
extern void                 gnss_pps_get_traim_data( traim_data_t *traim_data);
extern void                 gnss_pps_set_cps_and_cpr_regs(tU16 cpsh, tU16 cpsl, tU16 cprh, tU16 cprl);
extern void                 gnss_pps_set_cps_regs(tU16 cpsh, tU16 cpsl);
extern void                 gnss_pps_set_cpr_regs(tU16 cprh, tU16 cprl);
extern void                 gnss_pps_set_params(const gnss_pps_params_t *);
extern void                 gnss_pps_signal_disable(void);
extern void                 gnss_pps_get_timing_data(timing_data_t *timing_data);
extern void                 gnss_pps_get_data(pps_data_t *pps_data);
extern void                 gnss_pps_set_clock_speed(tU8 clk);
extern tU8                  gnss_pps_get_clock_speed(void);
extern boolean_t            gnss_pps_started(void);
extern void                 gnss_pps_set_time_filter_feedback(const tInt );
extern tInt                 gnss_pps_get_time_filter_feedback(void);
extern void                 gnss_pps_get_mtb_timer(tUInt *, tUInt *);
extern void                 gnss_set_wls_runtime(const boolean_t, const tDouble, const tDouble);


/*old PPS API left here for backwar compatibility*/
extern void                 gnss_set_pps_pulse_duration( const tDouble);
extern tDouble              gnss_get_pps_pulse_duration( void);
extern void                 gnss_set_pps_rf_correction( tDouble);
extern tDouble              gnss_get_pps_rf_correction( void);
extern void                 gnss_set_pps_polarity( boolean_t);
extern boolean_t            gnss_get_pps_polarity( void);
/***********************/


extern gnss_error_t         gnss_init_high_dynamics_mode(const tInt );
extern gnss_error_t         gnss_set_high_dynamics_mode(const tInt );
extern tInt                 gnss_get_high_dynamics_mode(void);

extern gnss_error_t         gnss_set_multipath_mitigation_mode(const tInt );
extern tInt                 gnss_get_multipath_mitigation_mode(void);

extern void                 gnss_sat_database_lock(void);
extern void                 gnss_sat_database_unlock(void);

extern void                 gnss_notch_filter_enable          ( const gnss_sat_type_t , tInt, tInt);
extern void                 gnss_notch_filter_ext_enable      ( const gnss_sat_type_t , tInt , tInt , tShort , tShort , tInt ) ;  /*added kbw fine, gross, detection theshold*/
extern void                 gnss_notch_filter_disable         ( const gnss_sat_type_t);
extern gpOS_error_t         gnss_notch_filter_get_status      ( const gnss_sat_type_t , tInt *, tInt *, tInt *, tInt *, tShort *);

extern boolean_t            gnss_get_stagps_status    ( void );
extern void                 gnss_set_stagps_status    ( boolean_t );
extern void                 gnss_set_area_selection_limits(tDouble , tDouble );

extern tU8                  gnss_get_sat_list_size    ( void );
extern void                 gnss_set_sat_list_size    ( tU8 );

extern gnss_error_t         gnss_glonass_get_satellite_slot(const satid_t sat, tInt *slot);
extern void                 gnss_glonass_slot_map_recalc( satid_t *slot_map);
extern tInt                 gnss_glonass_get_tk(const satid_t sat_id);
extern satid_t              gnss_prn_to_satid(const gnss_sat_type_t sat_type, const tInt prn);

extern void                 gnss_set_rtc_rtt_mode(void);
extern void                 gnss_set_rtc_only_mode(void);
extern void                 gnss_rtc_data_invalidate(void);

extern boolean_t            gnss_get_position_hold_status(void);
extern void                 gnss_set_position_hold_status(boolean_t status);
extern void                 gnss_get_position_hold_ECEF_pos(tDouble *x, tDouble *y, tDouble *z);
extern void                 gnss_get_position_hold_llh_pos(tDouble *lat,tDouble *lon,tDouble *h);
extern void                 gnss_set_position_hold_ECEF_pos(tDouble x, tDouble y, tDouble z);
extern void                 gnss_set_position_hold_llh_pos(tDouble lat,tDouble lon,tDouble h);
extern gnss_error_t         gnss_enable_traim (const tDouble Alarm);
extern gnss_error_t         gnss_disable_traim (void);
extern void                 gnss_get_traim_data(traim_data_t *traim_data);

extern void                 gnss_fix_store_local( void *);
extern tDouble              gnss_fix_get_geoid_msl_local( void * );
extern position_t*          gnss_fix_get_fil_pos_local( void * );
extern velocity_t*          gnss_fix_get_fil_vel_local( void * );
extern void                 gnss_fix_get_fil_pos_vel_local( position_t *, velocity_t *, void * );
extern position_t*          gnss_fix_get_raw_pos_local( void * );
extern velocity_t*          gnss_fix_get_raw_vel_local( void * );
extern tUInt                gnss_fix_get_raw_pos_sats_local(void *);
extern void                 gnss_fix_get_raw_pos_dops_local(tDouble *, tDouble *, tDouble *, tDouble *,void *);
extern ECEF_pos_t*          gnss_fix_get_fil_ecef_pos_local( void *);
extern ECEF_vel_t *         gnss_fix_get_fil_ecef_vel_local(void *);
extern void                 gnss_fix_get_time_local( tInt *, tDouble *, gpOS_clock_t * , void * );
extern gnss_sat_type_t      gnss_fix_get_time_best_local( gnss_time_t *, gpOS_clock_t *, time_validity_t *, void* );
extern gnss_sat_type_t      gnss_fix_get_time_aux_local( gnss_time_t *, time_validity_t *, void* );
extern gnss_sat_type_t      gnss_fix_get_time_master_local( gnss_time_t *, time_validity_t *, void* );
extern void                 gnss_fix_get_time_data_local(tInt *, tDouble *, gpOS_clock_t *, master_timebase_t *,void* );
extern time_validity_t      gnss_fix_get_time_validity_local( void * );
extern void                 gnss_fix_get_stopped_duration_local( tInt * ,void * );
extern void                 gnss_fix_get_dops_local( tDouble *, tDouble *, tDouble *, tDouble * , void * );
extern fix_status_t         gnss_fix_get_pos_status_local( void * );
extern fix_status_t         gnss_fix_get_raw_pos_status_local( void * );
extern diff_status_t        gnss_fix_get_diff_status_local( void * );
extern tInt                 gnss_fix_get_num_sats_used_local( void * );
extern tInt                 gnss_fix_get_num_sats_excluded_local( void * );
extern pos_algo_t           gnss_fix_get_pos_algo_local( void * );
extern boolean_t            gnss_fix_is_chan_used_local( const tInt , void * );
extern raw_measure_list_t*  gnss_fix_get_raw_measurements_local( void * );
extern tDouble              gnss_fix_get_position_rms_residual_local( void * );
extern tDouble              gnss_fix_get_velocity_rms_residual_local( void * );
extern void                 gnss_fix_get_position_residual_local( const tInt, tDouble *, boolean_t * , void * );
extern void                 gnss_fix_get_position_residual_used_local( const tInt, tDouble *, boolean_t * , void * );
extern boolean_t            gnss_fix_get_excluded_sats_range_local(const tInt , gnss_exclusion_type_t *,void *);
extern boolean_t            gnss_fix_get_excluded_sats_doppl_local(const tInt , gnss_exclusion_type_t *,void *);
extern void                 gnss_fix_get_velocity_residual_local( const tInt, tDouble  *, boolean_t * , void * );
extern void                 gnss_fix_get_velocity_covariance_local( tDouble *, tDouble *, tDouble *, void * );
extern void                 gnss_fix_get_position_covariance_local( tDouble *, tDouble *, tDouble *, void * );
extern void                 gnss_fix_get_error_ellipse_local( math_ellipse_t *, void *);
extern void                 gnss_fix_get_position_all_covariance_local(tDouble *, tDouble *,tDouble *, tDouble *,tDouble *,tDouble *,void *);
extern void                 gnss_fix_get_velocity_all_covariance_local(tDouble *, tDouble *,tDouble *, tDouble *,tDouble *,tDouble *,void *);
extern tDouble*             gnss_fix_get_position_q_matrix_diag_local(void *);
extern tDouble*             gnss_fix_get_velocity_q_matrix_diag_local(void *);
extern tDouble              gnss_fix_get_ehpe_local( void * );
extern tDouble              gnss_fix_get_clock_drift_local( void * );
extern tDouble              gnss_fix_get_clock_offset_local( void * );
extern tDouble              gnss_fix_get_glonass_path_delay_local( void * );
extern gnss_error_t         gnss_fix_create_local_copy( gpOS_partition_t *, void * * );
extern gpOS_clock_t         gnss_fix_get_measure_requested_time_local(void *);
extern gpOS_clock_t         gnss_fix_get_measure_get_data_time_local(void *);
extern gpOS_clock_t         gnss_fix_get_fix_available_time_local(void *);
extern tU16                 gnss_fix_get_kf_config_status_local(void *);
extern gnss_sat_type_t      gnss_fix_get_time_sat_type_local(void* );

extern gpOS_clock_t         gnss_fix_get_measure_requested_time(void);
extern gpOS_clock_t         gnss_fix_get_measure_get_data_time(void);
extern gpOS_clock_t         gnss_fix_get_fix_available_time(void);
extern tU16                 gnss_fix_get_kf_config_status(void);

extern gnss_error_t         gnss_dynamic_set_constellation_mask( const gnss_sat_type_mask_t);
extern gnss_position_validity_t  gnss_get_user_state(void);

extern gnss_error_t         gnss_dynamic_set_top_n_bound( const tInt);

extern gnss_error_t         gnss_lms_set_config(gnss_lms_config_t *);
extern gnss_error_t         gnss_lms_get_config(gnss_lms_config_t *);

extern void                 gnss_low_power_set_config_params(gnss_low_power_cyclic_mode_t *, gnss_low_power_periodic_mode_t *);
extern void                 gnss_low_power_init_config_params(gnss_low_power_cyclic_mode_t *, gnss_low_power_periodic_mode_t *);
extern void                 gnss_low_power_init_prv_config_params(gnss_low_power_private_config_t *);
extern void                 gnss_low_power_get_config_params(gnss_low_power_cyclic_mode_t *, gnss_low_power_periodic_mode_t *);
extern void                 gnss_low_power_set_status(boolean_t);
extern boolean_t            gnss_low_power_get_status(void);
extern void                 gnss_low_power_get_data(gnss_low_power_data_t *);
extern void                 gnss_low_power_get_nvm_sat_valid( tUInt *nvm_num_valid_p);
extern void                 gnss_low_power_set_long_eph_refresh_timer( boolean_t long_timer);

#ifdef DUTY_CYCLE_ENABLE
extern gnss_error_t         gnss_duty_cycle_set_params( const tInt);
extern boolean_t            gnss_duty_cycle_get_state( void);
extern void                 gnss_duty_cycle_start( void);
extern void                 gnss_duty_cycle_stop( void);
#endif

extern void                 gnss_xtal_set_Beta_and_res_params(gnss_xtal_mgr_res_net_data_t );
extern void                 gnss_xtal_set_input_data_callback(tInt (* p)(void),tInt sel);
extern void                 gnss_xtal_mgr_support_set(tInt );
extern gnss_error_t         gnss_xtal_mgr_copy_state_and_cov_params(gnss_xtal_mgr_nvm_data_t *);
extern void                 gnss_xtal_mgr_set_state_and_cov_from_ext(gnss_xtal_mgr_nvm_data_t );
extern void                 gnss_xtal_mgr_set_Q_values(tDouble *);
extern void                 gnss_xtal_get_data(gnss_xtal_monitor_t *);
extern void                 gnss_xtal_mgr_set_compression_factor(const tInt , const tInt );
extern tDouble              gnss_xtal_mgr_get_compression_factor(void );

extern gpOS_clock_t         gnss_position_get_first_fix_timestamp(void);
extern void                 gnss_get_fix_config(gnss_fix_config_t *curr_config);
extern void                 gnss_set_fix_config(gnss_fix_config_t *new_config);
extern void                 gnss_set_min_max_week_number(tInt , tInt );
extern void                 gnss_get_min_max_week_number(tInt *, tInt *);
extern gnss_error_t         gnss_utc_gps_get_tau_g( const gnss_time_t *, tDouble *);
extern void                 gnss_set_gps_utc_delta_time_default(const tInt);
extern tInt                 gnss_get_gps_utc_delta_time_default(void);
extern void                 gnss_set_fast_CN0_mode(const boolean_t );
extern boolean_t            gnss_get_fast_CN0_mode_status(void);
extern gnss_error_t         gnss_set_high_acc_mode(const tInt, const tInt);
extern tInt                 gnss_get_high_acc_mode(void) ;
extern void                 gnss_set_rtc_calibration_mode(const boolean_t );

extern void                 gnss_eph_update_mask_copy_clear  ( tUInt *);
extern void                 gnss_alm_update_mask_copy_clear  ( tUInt *);
extern tUInt                gnss_flags_update_mask_get_clear ( const tUInt);
extern void                 gnss_flags_update_mask_reset     ( void);
extern void                 gnss_flags_update_mask_setbit    ( const tUInt bit);
extern void                 gnss_flags_update_mask_clearbit  ( const tUInt bit);

extern boolean_t            gnss_fix_get_galileo_ggto_local  ( void *,  tDouble*);
extern tUInt                gnss_galileo_get_iono_model      ( void);
extern gnss_error_t         gnss_galileo_set_iono_model      ( const tUInt);
extern tUInt                gnss_galileo_get_pilot_mode      ( void);
extern gnss_error_t         gnss_galileo_set_pilot_mode      ( const tUInt);
extern galileo_ggto_mode_t  gnss_galileo_get_ggto_mode       ( void);
extern gnss_error_t         gnss_galileo_set_ggto_mode       ( const galileo_ggto_mode_t);
extern boolean_t            gnss_galileo_get_ggto            ( const gnss_time_t *, tDouble *, const galileo_ggto_mode_t);
extern gnss_error_t         gnss_galileo_get_ggto_brdc_raw   ( galileo_ggto_raw_t *);
extern gnss_error_t         gnss_galileo_set_ggto_brdc_raw   ( const galileo_ggto_raw_t *);

extern void                 gnss_set_acq_enable(boolean_t acq_enable);

#if defined( DR_CODE_LINKED )
extern void                 gnss_api_install_dr_callback(void *);
#endif

#if defined( WAAS_LINKED )
extern void                 gnss_api_install_waas_callback(void *);
#endif

extern void                 gnss_set_ephemeris_updated_callback             ( const gnss_ephemeris_updated_callback_t cb);
extern void                 gnss_set_initvisiblesatslist_available_callback ( const gnss_initvisiblesatslist_available_callback_t cb);

extern tS32                 gnss_get_predicted_linked_sat_ids ( void);
extern void                 gnss_set_predicted_linked_sat_ids ( const tS32);
extern void                 gnss_set_predicted_satid_array    ( const gnsslibid_t, const satid_t);
extern satid_t              gnss_get_predicted_satid_array    ( const gnsslibid_t);
extern gnsslibid_t          gnss_stagps_sat_id_to_gnsslib_id  ( const satid_t);
extern satid_t              gnss_stagps_gnsslib_id_to_sat_id  ( const gnsslibid_t);
extern boolean_t            gnss_stagps_sat_id_valid          ( const satid_t);

extern void                 gnss_set_gnss_fix_invalid(const boolean_t);
extern boolean_t            gnss_get_gnss_fix_invalid(void);
extern void                 gnss_set_dynamic_mng_params(boolean_t, dynamic_mng_data_t *);

/*}}}  */

#ifdef __cplusplus
}
#endif
#endif /* GNSS_API_H */
