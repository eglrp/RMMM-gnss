/*{{{  COMMENT Standard Header*/
/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999
                   All Rights Reserved

Source file name : gp_defs.h
Author :           P D Bagnall

Contains the standard GPS definitions.

Date        Modification                                    Initials
----        ------------                                    --------
01 Nov 96   Created                                         PDB

************************************************************************/
/*}}}  */

#ifndef GNSS_DEFS_H
#define GNSS_DEFS_H

#include "typedefs.h"
#include "macros.h"
#include "fixpoint.h"
#include "gnss_os.h"

#define VISIBLE_MAX_NUM_OF_SATS     32
#define WAAS_BUFFER_SIZE            48

typedef enum gnss_error_e
{
  GNSS_NO_ERROR = 0,
  GNSS_ERROR
} RCIF_DATA_ALIGN gnss_error_t;

/*{{{  gnsslibid_t*/
typedef tU16 gnsslibid_t;
/*}}}  */

/*{{{  satid_t*/
typedef tU16 satid_t;
/*}}}  */

/*{{{  chanid_t*/
typedef tS8 chanid_t;
/*}}}  */

/*{{{  sigid_t*/
typedef tU16 sigid_t;
/*}}}  */

typedef enum gnss_sat_type_e {
  GNSS_SAT_TYPE_NOT_VALID = -1,
  GNSS_SAT_TYPE_GPS       = 0,
  GNSS_SAT_TYPE_GLONASS,
  GNSS_SAT_TYPE_QZSS_L1_CA,
  GNSS_SAT_TYPE_GALILEO,
  GNSS_SAT_TYPE_SBAS,
  GNSS_SAT_TYPE_QZSS_L1_SAIF,
  GNSS_SAT_TYPE_QZSS_L1C,
  GNSS_SAT_TYPE_COMPASS,
  GNSS_SAT_TYPE_PSEUDOLITE,
  GNSS_SAT_TYPE_L2C,
  GNSS_SAT_TYPE_END
} RCIF_DATA_ALIGN gnss_sat_type_t;

typedef tUInt gnss_sat_type_mask_t;  // one bit for each sat_type

/*{{{  nco_t*/
typedef tInt nco_t;
typedef tDouble nco_accurate_t;
/*}}}  */

/*{{{  rtc_switch_t*/
typedef enum rtc_switch_e
{
  RTC_SWITCH_OFF = 0,
  RTC_SWITCH_ON
} rtc_switch_t;
/*}}}  */

/*{{{  rtc_status_t*/
typedef enum rtc_status_e
{
  RTC_STATUS_INVALID,
  RTC_STATUS_STORED,
  RTC_STATUS_APPROXIMATE
} rtc_status_t;
/*}}}  */

/*{{{  visible_t*/

typedef struct visible_tag
{
  satid_t   satid;
  tInt      azimuth;
  tInt      elevation;
} visible_t;

/*}}}  */
/*{{{  visible_sats_data_t*/

typedef struct sat_visible_list_tag
{
  tInt        list_size;
  visible_t   list[VISIBLE_MAX_NUM_OF_SATS];
} visible_sats_data_t;

/*}}}  */
/*{{{  sat_dbase_all_sats_t*/

typedef struct sat_dbase_all_sats_tag
{
  tU8              info_valid;
  tU8              sv_health;
} sat_dbase_validity_t;

/*}}}  */
/*{{{  diff_status_t*/

typedef enum
{
  DIFF_STATUS_OFF = 0,
  DIFF_STATUS_ON  = 1
} diff_status_t;

/*}}}  */
/*{{{  fix_status_t*/

typedef enum
{
  NO_FIX = 1,
  FIX_2D = 2,
  FIX_3D = 3
} RCIF_DATA_ALIGN fix_status_t;

/*}}}  */

/*{{{  position_fp_t*/
typedef struct position_fp_tag
{
  fp_s32_t  latitude;
  fp_s32_t  longitude;
  fp_s32_t  height;
} position_fp_t;
/*}}}  */

/*{{{  position_t*/
typedef struct position_tag
{
  tDouble   latitude;
  tDouble   longitude;
  tDouble   height;
} position_t;
/*}}}  */

/*{{{  utc_ls_case_t*/
typedef enum {
  UTC_LS_FUTURE  = 0,  // leap-second is in the future (will happen)
  UTC_LS_PAST    = 1,  // leap-second is in the past (already happened)
  UTC_LS_MIDDLE  = 2   // leap-second is in the gray time interval
} utc_ls_case_t;
/*}}}  */

/*{{{  dynamic_mode_t*/
typedef enum {
    LOW_DYNAMICS   = 0, /*low dynamic mode, legacy value*/
    HIGH_DYNAMICS  = 1, /*high dynamic mode, legacy value*/
    MILD_DYNAMICS  = 2, /*LD same step response , 10Hz update mode*/
    AUTO_DYNAMICS  = 3
} trk_dynamics_t;
/*}}}  */


/*{{{  velocity_t*/
typedef struct velocity_tag
{
  tDouble   vel_north;
  tDouble   vel_east;
  tDouble   vel_vert;
} velocity_t;
/*}}}  */
/*{{{  pos_algo_t*/
typedef enum pos_algo_e
{
  NONE,
  LMS_ALGO,
  KF_ALGO
} pos_algo_t;
/*}}}  */
/*{{{  gnss_waas_symbols_t*/
typedef struct gps_waas_symbols_s
{
  tU8   even_packet;
  tU8   odd_packet;
} gnss_waas_symbols_t;
/*}}}  */
/*{{{  waas_status_t*/
typedef enum gps_waas_status_e
{ WAAS_STATUS_OFF,
  WAAS_STATUS_ON
} RCIF_DATA_ALIGN gnss_waas_status_t;
/*}}}  */
/*{{{  dops_t*/

typedef struct dops_tag
{
  tDouble   pdop;
  tDouble   vdop;
  tDouble   hdop;
  tDouble   gdop;
} dops_t;
/*}}}  */

 /*{{{  dops_fp_t*/
typedef struct dops_fp_tag
{
  fp_s16_t  pdop;    /*scaling factor *1<<8*/
  fp_s16_t  vdop;    /*scaling factor *1<<8*/
  fp_s16_t  hdop;    /*scaling factor *1<<8*/
  fp_s16_t  gdop;    /*scaling factor *1<<8*/
} dops_fp_t;
/*}}}  */

/*{{{  fda_status_t*/
typedef enum gnss_fda_status_e
{
  FDA_STATUS_OFF,
  FDA_STATUS_ON
} gnss_fda_status_t;
/*}}}  */
/*{{{  fde_status_t*/
typedef enum gnss_fde_status_e
{
  FDE_STATUS_OFF,
  FDE_STATUS_ON
} gnss_fde_status_t;
/*}}}  */
/*{{{  ads_status_t*/
typedef enum gnss_ads_status_e
{
  ADS_STATUS_OFF,
  ADS_STATUS_ON
} gnss_ads_status_t;
/*}}}  */
/*{{{  gnss_diff_source_t*/
typedef enum gps_diff_source_e
{
  DIFF_SOURCE_NONE,
  DIFF_SOURCE_WAAS,
  DIFF_SOURCE_RTCM,
  DIFF_SOURCE_AUTO
} gnss_diff_source_t;
/*}}}  */
/*{{{  gnss_diff_source_t*/
typedef enum gps_diff_correction_e
{
  DIFF_CORRECTION_NONE  = 0U,
  DIFF_CORRECTION_RTCM  = 1U,
  DIFF_CORRECTION_IONO  = 2U,
  DIFF_CORRECTION_SLOW  = 4U,
  DIFF_CORRECTION_FAST  = 8U
} diff_correction_t;
/*}}}  */
/*{{{  tracker_sat_position_type_t*/
typedef enum sat_pos_type_s
{
  SAT_POSTYPE_EMPTY     = 0,
  SAT_POSTYPE_EPHEMERIS = 1,
  SAT_POSTYPE_ALMANAC   = 2,
  SAT_POSTYPE_UNHEALTHY = 3
} RCIF_DATA_ALIGN sat_pos_type_t;
/*}}}  */

/*{{{  time_validity_t*/
typedef enum time_validity_e
{
  NO_TIME,
  FLASH_TIME,
  USER_TIME,
  USER_RTC_TIME,
  RTC_TIME,
  RTC_TIME_ACCURATE,
  APPROX_TIME,
  ACCURATE_TIME = 0x8,
  POSITION_TIME,
  EPHEMERIS_TIME
} RCIF_DATA_ALIGN time_validity_t;
/*}}}  */

/*{{{  fix_time_info_t*/
typedef struct fix_time_info_tag {
  tU8 fix_time_validity                         : 4;
  tU8 fix_time_best_sat_type                    : 4;
} fix_time_info_t;
/*}}}  */


/*{{{  master_timebase_t*/
typedef struct master_timebase_s
{
  tInt    ms;
  tInt    timestamp;
} master_timebase_t;
/*}}}  */
/*{{{  tow_t*/
typedef tDouble tow_t;
/*}}}  */
/*{{{  gnss_time_t*/
typedef struct gnss_time_tag
{
  tInt    week_n;
  tow_t   tow;
} gnss_time_t;
/*}}}  */

/*{{{  gnss_time_fp_t*/
typedef struct gnss_time_fp_tag
{
  tInt      week_n;
  fp_s32_t  tow;
} gnss_time_fp_t;
/*}}}  */

/*{{{  gnss_time_reference_t*/
typedef struct gps_time_reference_s
{
  gnss_time_t         gps_time;
  gpOS_clock_t        cpu_time;
  tDouble             ticks_per_sec;
  gnss_time_t         last_ephemeris_time;
  gnss_time_t         mtb_gps_time;
  master_timebase_t   mtb_time;
  tDouble             mtb_rate;
  gnss_sat_type_t     sat_type;
  time_validity_t     time_validity;
} gnss_time_reference_t;

/*{{{  gnss_time_reference_data_t*/
typedef struct gnss_time_reference_data_s
{
  gnss_time_reference_t gps_time_reference;
  gnss_time_reference_t glonass_time_reference;
  gnss_time_reference_t galileo_time_reference;
  gnss_time_reference_t compass_time_reference;
}gnss_time_reference_data_t;

/*{{{  gnss_pps_params_t*/
typedef struct gnss_pps_params_s
{
  boolean_t     pps_enabled;
  tU16          pps_cpsh;
  tU16          pps_cprh;
  tUInt         pps_cpsl;
  tUInt         pps_cprl;
  tUInt         pps_cpl_step;
  tU16          pps_cph_step;
  gpOS_clock_t    rising_edge_cpu_time;
  gpOS_clock_t    pulse_duration_ticks;
} gnss_pps_params_t;
/*}}}  */

/*{{{  gnss_lms_config_t*/
typedef struct gnss_lms_config_s
{
  tDouble       position_residual_thr;
  tDouble       position_residual_thr_after_raim;
  tDouble       raim_min_angle_separation;
  tDouble       raim_thr_coeff_1;
  tDouble       min_tracked_time;
  tDouble       glonass_path_delay_init_value;
  boolean_t     enable_2D_fixes;
  boolean_t     lock_glonass_path_delay;
  tInt          min_sat_gnss_fix;
  tInt          min_sat_single_const_fix;
  boolean_t     check_residual_hdop_product;
} gnss_lms_config_t;
/*}}}  */

/*{{{  gnss_low_power_cyclic_mode_t*/
typedef struct gnss_low_power_cyclic_mode_s
{
  tU8       ehpe_threshold; //meters max 255m
  tU8       N_sats_reduced;
  gnss_sat_type_mask_t const_mask_init;
  boolean_t reduced_type;
  boolean_t duty_cycle_on_off;
  tShort    ms_off;
} gnss_low_power_cyclic_mode_t;
/*}}}  */

/*{{{  gnss_low_power_periodic_mode_t*/
typedef struct gnss_low_power_periodic_mode_s
{
  tU32      fix_period;
  tU8       fix_on_time;
  tU8       EPH_refresh;

  tU8       RTC_refresh;
  tU8       NoFixTimeout;
  tU16      NoFixOffTime;

  boolean_t periodic_mode;
} gnss_low_power_periodic_mode_t;
/*}}}  */

/*{{{  gnss_low_power_default_config_t*/
typedef struct gnss_low_power_private_config_s
{
  tUShort   gnss_eph_refresh_interval;
  tU8       timer_scan_glonass_eph;
  tU8       eph_limit;

  tU8       ehpe_timer_in;
  tU8       ehpe_timer_out;

  tU16      NoFixTimeout2;

} gnss_low_power_private_config_t;
/*}}}  */

/*{{{  gnss_low_power_config_t*/
typedef struct gnss_low_power_config_s
{
  gnss_low_power_private_config_t    private_config;

  gnss_low_power_cyclic_mode_t       cyclic;
  gnss_low_power_periodic_mode_t     periodic;
} gnss_low_power_config_t;
/*}}}  */

/*{{{  gnss_low_power_modes_t*/
typedef struct gnss_low_power_modes_s
{
  gnss_low_power_cyclic_mode_t       cyclic;
  gnss_low_power_periodic_mode_t     periodic;
} gnss_low_power_modes_t;
/*}}}  */



typedef enum gnss_startup_mode_e {
  GNSS_STARTUP_POWER_ON,   /* Classic power on */
  GNSS_STARTUP_WAKEUP_RTC, /* Standby wake-up by RTC */
  GNSS_STARTUP_WAKEUP_PIN  /* Standby wake-up by PIN */
} gnss_startup_mode_t;

/*{{{  gnss_position_validity_t*/
typedef enum gnss_position_validity_e
{
  GNSS_POSITION_UNKNOWN,
  GNSS_POSITION_VALID
} gnss_position_validity_t;
/*}}}  */

/*{{{  gnss_utc_delta_time_validity_t*/
typedef enum gnss_utc_delta_time_validity_e
{
  UTC_GPS_OFFEST_NOT_VALID,
  UTC_GPS_OFFEST_STORED,
  UTC_GPS_OFFEST_VALID
} gnss_utc_delta_time_validity_t;
/*}}}  */

/*{{{  gnss_xtal_monitor_t*/
typedef struct gnss_xtal_monitor_s
{
 tDouble    T;
 tDouble    filtered_ramp_rate;
 tDouble    raw_ramp_rate;
 tDouble    nav_xtal_ADC_rate;
 tDouble    nav_xtal_adc_stored_value;
 boolean_t  kf_ramp_rate_status;
 boolean_t  acq_reactivation_status;
}
gnss_xtal_monitor_t;

typedef struct gnss_xtal_mgr_nvm_data_tag
{
  tDouble       coeff_val[4];
  tDouble       variance_val[16];
  tInt          ext_set;
  gnss_time_t   time_info;
} gnss_xtal_mgr_nvm_data_t;

typedef struct gnss_xtal_mgr_res_net_data_tag
{
  tInt    B;
  tInt    RTo;
  tInt    Rs;
} gnss_xtal_mgr_res_net_data_t;

/*{{{  gnss_fix_config_t*/
typedef struct gnss_fix_config_s
{
  boolean_t   few_sats_pos_estimation_enable;
  tU8         pos_estimation_sat_th;
} gnss_fix_config_t;
/*}}}  */

/*{{{  acq_ctrl_flags_t*/
typedef struct acq_ctrl_flags_tag
{
  tUInt   pilot_unit_0    : 1;   // 0=data   1=pilot
  tUInt   pilot_unit_1    : 1;
  tUInt   mm_unit_0       : 2;   // 0=IQ-mix 1=power
  tUInt   mm_unit_1       : 2;
  tUInt   multi_chan      : 1;
  tUInt   valid           : 1;   // 1=valid 0=no
  tUInt   coh_int_time_0  : 4;
  tUInt   coh_int_time_1  : 4;
  tUInt   n_steps_0       : 12;
  tUInt   spare0          : 4;

  tUInt   nco_step_0      : 10;
  tUInt   nco_step_1      : 10;
  tUInt   n_steps_1       : 12;

  tInt    bandwidth_0;
  tInt    bandwidth_1;

} acq_ctrl_flags_t;
/*}}}  */

/*{{{  gnss_flags_update_bit_t*/
typedef enum gnss_flags_update_bit_e
{
  FLAGS_UPDATE_BIT_IONO_GPS     = 0,
  FLAGS_UPDATE_BIT_IONO_GALILEO = 1,
  FLAGS_UPDATE_BIT_IONO_COMPASS = 2,
  FLAGS_UPDATE_BIT_GALILEO_GGTO = 3

} gnss_flags_update_bit_t;
/*}}}  */

/*{{{  enum gnss_prn_mod_t*/
typedef enum gnss_prn_mod_e
{
  GNSS_PRN_MOD_UNKNOWN = 0,
  GNSS_PRN_MOD_1_MS,
  GNSS_PRN_MOD_2_MS,
  GNSS_PRN_MOD_4_MS,
  GNSS_PRN_MOD_10_MS,
  GNSS_PRN_MOD_20_MS,
  GNSS_PRN_MOD_100_MS,
  GNSS_PRN_MOD_200_MS,
  GNSS_PRN_MOD_600_MS,
  GNSS_PRN_MOD_1_S,
  GNSS_PRN_MOD_2_S,
  GNSS_PRN_MOD_END
} gnss_prn_mod_t;
/*}}}  */

/*{{{  gnss_prn_div_t*/
typedef union
{
  struct
  {
    tU16 prn_mod    : 8;  // modulus gnss_prn_mod_e
    tS16 div_cnt    : 8;  // div count
  } BIT;
  tU16 REG;
} gnss_prn_div_t;
/*}}}  */

/*{{{  gnss_prn_data_t*/
typedef struct gnss_prn_data_tag
{
  tInt            prn;            // [m] mod prn_mod
  tShort          prn_accuracy;   // [chip]
  gnss_prn_div_t  prn_div;

} gnss_prn_data_t;
/*}}}  */

/*{{{  ECEF_coord_t*/
typedef struct ECEF_coord_tag {
  tDouble   x;
  tDouble   y;
  tDouble   z;
} ECEF_coord_t;
/*}}}  */

/*{{{  ECEF_pos_t*/
#define ECEF_pos_t ECEF_coord_t
/*}}}  */

/*{{{  ECEF_vel_t*/
#define ECEF_vel_t ECEF_coord_t
/*}}}  */

/*{{{  ECEF_pos_t*/
typedef struct ECEF_coord_fp_tag {
  fp_s32_t x;  /* Change from double to fp_s32_t; scaling factor: 1<<5*/
  fp_s32_t y;  /* Change from double to fp_s32_t; scaling factor: 1<<5*/
  fp_s32_t z;  /* Change from double to fp_s32_t; scaling factor: 1<<5*/
} ECEF_coord_fp_t;
/*}}}  */

/*{{{  ECEF_vel_t*/
typedef ECEF_coord_fp_t ECEF_pos_fp_t;
/*}}}  */

/*{{{  ECEF_vel_t*/
typedef ECEF_coord_fp_t ECEF_vel_fp_t;
/*}}}  */


/* JPH Merge WAAS Back End process in Navigate process */
#define WAAS_FRAME_SIZE         32

typedef enum
{
  NAV_SET_POS_CMD,
  NAV_SET_TIME_CMD,
  NAV_SET_FREQ_CMD,
  NAV_START_CMD,
  NAV_RESTART_CMD,
  NAV_SUSPEND_CMD,
  NAV_SET_FREQ_RANGE_CMD,
  NAV_SET_LMS_CFG_CMD,
  NAV_SET_LOW_POW_CFG_CMD,
  NAV_INIT_LOW_POW_CFG_CMD,
  NAV_INIT_LOW_POW_PRIVATE_CFG_CMD,
  NAV_SET_LOW_POW_LONG_EPH_TIMER_CMD,
  NAV_WAKEUP_CMD
} nav_msg_type_e;


typedef struct nav_msg_set_time_tag
{
  gnss_time_t time;
  gnss_sat_type_t sat_type;
} nav_msg_set_time_t;

typedef struct nav_msg_set_freq_tag
{
  nco_t nco;
} nav_msg_set_nco_t;

typedef struct nav_msg_set_freq_range_tag
{
  nco_t max_nco;
  nco_t min_nco;
} nav_msg_set_nco_range_t;

typedef struct nav_msg_set_pos_tag
{
  tDouble lat;
  tDouble lon;
  tDouble height;
} nav_msg_set_pos_t;

typedef struct nav_msg_tag
{
  nav_msg_type_e type;
  union
  {
    nav_msg_set_pos_t       measure_data;
    nav_msg_set_time_t      time_data;
    nav_msg_set_nco_t       nco_data;
    nav_msg_set_nco_range_t nco_range_data;
    gnss_lms_config_t       lms_config;
    gnss_low_power_modes_t  nav_low_power_config_modes;
    gnss_low_power_private_config_t  nav_low_power_config_setup;
    gnss_startup_mode_t     startup_mode;
    boolean_t               nav_low_power_long_eph;
  } data;
} nav_message_t;

typedef struct
{
  satid_t       waas_sat_id;
  gpOS_clock_t  next_correction_timeout;
  boolean_t     waas_be_running;
  boolean_t     waas_compute_correction;
  gnss_time_t   diff_mode_gnss_time;
  gnss_time_t   no_diff_mode_gnss_time;
} nav_diff_correction_info_t;

typedef struct
{
  position_t    pos;
  fix_status_t  fix_status;
  diff_status_t diff_status;
} nav_diff_correction_gnss_fix_info_t;

typedef enum
{
  NAV_CMD,
  WAAS_CMD
} cmd_type_e;

typedef enum waas_be_cmd_e {
  WAAS_BE_SET_ON,
  WAAS_BE_SET_OFF,
  WAAS_BE_SET_SAT,
  WAAS_BE_SET_MULTI_CH_PRN,
  WAAS_BE_SET_PRN_TO_DECODE,
  WAAS_BE_AUTO_SEARCH_ON,
  WAAS_BE_AUTO_SEARCH_OFF
} waas_be_cmd_t;

typedef struct waas_be_multi_ch_s {
  tUChar channel;
  tUChar sat;
  gnss_waas_status_t status;
}waas_be_multi_ch_t;

typedef struct waas_be_msg_s {
  waas_be_cmd_t command;
  union {
    tUChar                sat;
    waas_be_multi_ch_t    multi_ch;
  } param;
} waas_be_msg_t;


typedef struct amq_msg_tag
{
  cmd_type_e type;
  union
  {
    nav_message_t       nav_message;
    waas_be_msg_t       waas_message;
  } data;
} amq_message_t;


#endif /* GNSS_DEFS_H */
