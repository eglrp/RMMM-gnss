/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999
                   All Rights Reserved

Source file name : gp_core.h
Author :           P D Bagnall

Header file for the GP6 Library interface.

Date        Modification                                    Initials
----        ------------                                    --------
01 Nov 96   Created                                         PDB

************************************************************************/

/*}}}  */

#ifndef GP_CORE_H
#define GP_CORE_H

/*{{{  includes*/

#include "gnss_api.h"
/*}}}  */

#define gps_error_t               gnss_error_t
#define GPS_ERROR                 GNSS_ERROR
#define GPS_NO_ERROR              GNSS_NO_ERROR

#define gps_time_t                gnss_time_t
#define gps_waas_status_t         gnss_waas_status_t
#define gps_diff_source_t         gnss_diff_source_t
#define gps_ads_status_t          gnss_ads_status_t
#define gps_fda_status_t          gnss_fda_status_t
#define gps_fde_status_t          gnss_fde_status_t
#define gps_time_reference_t      gnss_time_reference_t
#define gps_time_then(cpu_time)   gnss_time_then(cpu_time, GNSS_SAT_TYPE_GPS)

extern const tChar *    gpslib_ver;
extern const tChar *    gpslib_supplier_id;

/*{{{  exported functions*/
extern gps_error_t     gps_init_rtc    (const rtc_switch_t, gps_time_t*, gpOS_clock_t, tInt);
extern gps_error_t     gps_init        (void);
extern const tChar *   gps_version     (void);

#if defined(__STA2062__)
extern boolean_t       gps_get_subsystem_version ( tChar *);
extern tUInt           gps_get_ekernel_version   ( void);
#endif

#if defined(__STA2064__)
extern boolean_t       gps_get_subsystem_version ( tChar *);
extern void            gps_set_cut_version       ( tracker_cut_version_t);
#endif

#if defined(__generic__)
extern boolean_t       gps_get_subsystem_version ( tChar *);
#endif

extern gps_error_t     gps_start                     ( void);
extern gps_error_t     gps_restart                   ( void);
extern gps_error_t     gps_suspend                   ( void);
extern void            gps_powerdown_request         ( void);
extern gps_error_t     gps_set_time                  ( const tInt, const tInt, const tInt, const tInt, const tInt, const tInt);
extern gps_error_t     gps_set_pos                   ( const tDouble, const tDouble, const tDouble);
extern gps_error_t     gps_set_centre_freq           ( const tDouble);
extern gps_error_t     gps_set_freq_range            ( const tInt, const tInt);
extern tDouble         gps_get_centre_freq           ( void);
extern tInt            gps_get_sat_iode              ( const satid_t);
extern sat_pos_type_t  gps_get_sat_pos_type          ( const satid_t);
extern gps_error_t     gps_get_sat_az_el             ( const satid_t satid, tInt *az_p, tInt *el_p);
extern gps_error_t     gps_set_fix_rate              ( const tDouble);
extern tDouble         gps_get_fix_rate              ( void);
extern boolean_t       gps_get_time_valid            ( void);
extern gps_error_t     gps_enable_sat                ( const satid_t);
extern gps_error_t     gps_disable_sat               ( const satid_t);
extern void            gps_clear_all_ephems          ( void);
extern void            gps_clear_all_almanacs        ( void);
extern boolean_t       gps_is_sat_enabled            ( const satid_t);
extern gps_error_t     gps_set_3D_dop_threshold      ( const tDouble, const tDouble, const tDouble, const tDouble);
extern void            gps_get_3D_dop_threshold      ( tDouble*, tDouble*, tDouble*, tDouble*);
extern gps_error_t     gps_set_2D_dop_threshold      ( const tDouble, const tDouble, const tDouble, const tDouble);
extern void            gps_get_2D_dop_threshold      ( tDouble*, tDouble*, tDouble*, tDouble*);
extern gps_error_t     gps_set_elevation_mask_angle  ( const tDouble);
extern gps_error_t     gps_set_tracking_threshold    ( const tInt);
extern void            gps_set_sf_recovery_status    ( const boolean_t);
extern void            gps_set_2_5_ppm_support       ( const boolean_t);
extern void            gps_get_tracking_threshold    ( tInt *,tInt *,tInt *);
extern tDouble         gps_get_elevation_mask_angle  ( void);
extern tInt            gps_get_sat_health            ( const satid_t);

extern void            gps_reset_diff_params                   ( void);
extern gps_error_t     gps_set_diff_params                     ( const satid_t, const tInt, const tDouble, const tDouble, const tDouble);
extern gps_error_t     gps_get_diff_params                     ( const satid_t, tInt *, tDouble *, tDouble *, tDouble *, boolean_t *);
extern gps_error_t     gps_set_diff_mode                       ( const diff_mode_t);
extern diff_mode_t     gps_get_diff_mode                       ( void);
extern gps_error_t     gps_diff_set_source_type                ( const gps_diff_source_t);
extern gps_error_t     gps_diff_get_source_type                ( gps_diff_source_t *);
extern gps_error_t     gps_diff_set_RTCM_flag                  ( const boolean_t);
extern gps_error_t     gps_diff_get_RTCM_flag                  ( boolean_t *);
extern boolean_t       gps_diff_sbas_fast_correction_available ( const satid_t);
extern boolean_t       gps_diff_sbas_slow_correction_available ( const satid_t);
extern boolean_t       gps_diff_sbas_iono_correction_available ( const satid_t);
extern boolean_t       gps_diff_rtcm_correction_available      ( const satid_t);
extern gps_error_t     gps_set_sbas_diff_params                ( const satid_t, const tInt, const tDouble, const tDouble, const tDouble,const diff_correction_t);

extern gps_error_t     gps_waas_set_status                   ( const satid_t, const gps_waas_status_t);
extern gps_error_t     gps_waas_get_status                   ( satid_t *, gps_waas_status_t *);
extern gps_error_t     gps_waas_get_multi_ch_prn_and_status  ( const chanid_t, satid_t *, gps_waas_status_t *);
extern gps_error_t     gps_waas_set_multi_ch_prn_and_status  ( const chanid_t, const satid_t, const gps_waas_status_t);
extern gps_error_t     gps_waas_get_prn_to_decode            ( satid_t *);
extern gps_error_t     gps_waas_set_prn_to_decode            ( const satid_t );
extern boolean_t       gps_waas_is_tracking                  ( const tInt);

extern tDouble         gps_get_utc_delta_time              ( void);
extern gps_error_t     gps_get_ephemeris_params            ( const satid_t, ephemeris_raw_t *, boolean_t *);
extern gps_error_t     gps_set_ephemeris_params            ( const satid_t, const ephemeris_raw_t *);
extern void            gps_ephemeris_predicted_buffer_init ( const ST_AGPS_ephemeris_raw_t *);
extern gps_error_t     gps_get_almanac_params              ( const satid_t, almanac_raw_t *, boolean_t *);
extern gps_error_t     gps_set_almanac_params              ( const satid_t, const almanac_raw_t *);
extern gps_error_t     gps_get_iono_params                 ( iono_raw_t *);
extern gps_error_t     gps_set_iono_params                 ( const iono_raw_t *);
extern gps_error_t     gps_get_utc_params                  ( utc_raw_t *);
extern gps_error_t     gps_set_utc_params                  ( const utc_raw_t *);
extern void            gps_get_sats_visible                ( visible_sats_data_t*);
extern tDouble         gps_get_sat_cn0                     ( const satid_t);
extern tDouble         gps_cpu_clock_rate_hi               ( void);

extern void                 gps_fix_store                       ( void);
extern void                 gps_fix_read_claim                  ( void);
extern void                 gps_fix_read_release                ( void);
extern tDouble              gps_fix_get_geoid_msl               ( void);
extern position_t*          gps_fix_get_fil_pos                 ( void);
extern velocity_t*          gps_fix_get_fil_vel                 ( void);
extern void                 gps_fix_get_time                    ( tInt *, tDouble *, gpOS_clock_t *);
extern time_validity_t      gps_fix_get_time_validity           ( void);
extern void                 gps_fix_get_stopped_duration        ( tInt *);
extern void                 gps_fix_get_dops                    ( tDouble *, tDouble *, tDouble *,tDouble *);
extern fix_status_t         gps_fix_get_pos_status              ( void);
extern diff_status_t        gps_fix_get_diff_status             ( void);
extern tInt                 gps_fix_get_num_sats_used           ( void);
extern pos_algo_t           gps_fix_get_pos_algo                ( void);
extern boolean_t            gps_fix_is_chan_used                ( const tInt);
extern raw_measure_list_t*  gps_fix_get_raw_measurements        ( void);
extern tDouble              gps_fix_get_position_rms_residual   ( void);
extern tDouble              gps_fix_get_velocity_rms_residual   ( void);
extern void                 gps_fix_get_position_residual       ( const tInt, tDouble *, boolean_t *);
extern void                 gps_fix_get_velocity_residual       ( const tInt, tDouble  *, boolean_t *);
extern void                 gps_fix_get_velocity_covariance     ( tDouble * N_cov, tDouble *E_cov, tDouble *V_cov);
extern void                 gps_fix_get_position_covariance     ( tDouble * N_cov, tDouble *E_cov, tDouble *V_cov);
extern tDouble              gps_fix_get_clock_drift             ( void);

extern gps_error_t      gps_conv_vel_to_course_speed        ( const velocity_t *, tDouble *, tDouble *);
extern gps_error_t      gps_conv_vel_to_course_speed_3D     ( const velocity_t *, tDouble *, tDouble *);
extern gps_error_t      gps_get_date                        ( const tInt, const tDouble, tInt *, tInt *, tInt*);
extern gps_error_t      gps_get_utc_time                    ( const tDouble, tInt *, tInt *, tInt *, tInt *);
extern gps_error_t      gps_date_time_to_gps_time           ( const tInt, const tInt, const tInt, const tInt, const tInt, const tInt, gps_time_t *);
extern boolean_t        gps_date_time_valid                 ( const tInt, const tInt, const tInt, const tInt, const tInt, const tInt);
extern gps_error_t      gps_get_sat_xyz_diff                ( const satid_t, tDouble *, tDouble *, tDouble *);
extern gps_fde_status_t gps_get_fde_status                  ( void);
extern void             gps_turn_stop_detection_on          ( boolean_t);
extern void             gps_turn_walking_mode_on            ( boolean_t);
extern gps_ads_status_t gps_get_ads_status                  ( void);
extern void             gps_set_fde_status                  ( gps_fde_status_t);
extern void             gps_set_ads_status                  ( gps_ads_status_t);
extern time_validity_t  gps_get_time_validity               ( void);
extern void             gps_rtc_set_alarm                   ( tUInt);
extern void             gps_rtc_get_time                    ( gps_time_t *, gpOS_clock_t *, rtc_status_t *, time_validity_t *);
extern tInt             gps_get_noise_floor                 ( void);
extern boolean_t        gps_sat_tracking_check              ( const tInt);
extern gpOS_clock_t     gps_get_tracker_time_now            ( void);

extern tow_t            gps_time_lminus                     ( const tow_t, const tow_t);
extern tow_t            gps_time_lplus                      ( const tow_t, const tDouble);
extern tow_t            gps_time_minus                      ( const gps_time_t *, const gps_time_t *);
extern gps_time_t       gps_time_plus                       ( const gps_time_t *, const tDouble);

extern void             gps_rtc_write                       ( const gps_time_reference_t, const time_validity_t, const boolean_t);
extern void             gps_rtc_read                        ( gps_time_t *, gpOS_clock_t *, rtc_status_t *, time_validity_t*);

extern gps_error_t      gps_test_set_operation              ( const operation_property_t, const tInt, const tInt, const tDouble *);

extern void             gps_ephemeris_broadcast_enable_sat            (const satid_t);
extern void             gps_ephemeris_broadcast_disable_sat           (const satid_t);
extern boolean_t        gps_ephemeris_predicted                       (const satid_t);
extern gps_error_t      gps_get_ephemeris_predict_params              (const satid_t,pred_data_t *);
extern void             gps_clear_sat_ephemeris                       (const satid_t);
extern tUInt            gps_get_ephemeris_broadcast_onoff_status      (void);
extern boolean_t        gps_get_sat_ephemeris_broadcast_onoff_status  (const satid_t);

extern void             gps_set_tracker_jammer  ( tInt);
extern void             gps_set_rtt_tps         ( tDouble tps);

extern void             gps_test_set_user_pos         ( const tDouble, const tDouble, const tDouble);
extern void             gps_test_set_nco_value        ( const boolean_t, const nco_t);
extern void             gps_test_invalidate_rtc       ( void);
extern void             gps_test_invalidate_user_pos  ( void);
extern void             gps_test_invalidate_utc_params( void);

extern void             gps_set_rf_test_mode_on_sat_n ( const satid_t);
extern void             gps_reset_rf_test_mode        ( void);
extern void             gps_set_rf_test_mode_on_sat_n_cn0_trk_thr( satid_t, tU8);

/*}}}  */

#endif
