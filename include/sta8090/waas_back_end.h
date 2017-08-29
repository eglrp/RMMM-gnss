/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999,2004,2006
                   All Rights Reserved

Source file name : waas_back_end.h
Author :           Dante Di Domenico
Description:       Header of main file of the back-end process
                   sub-component


Date              Modification                            Initials
1  Oct 04         Created                                 DDD
1  Oct 06         Modifications caused by different       DDD
                  naming in the file of the waas project

************************************************************************/
/*}}}  */

#ifndef WAAS_BACK_END_H
#define WAAS_BACK_END_H
#include "waas_defs.h"

#define WAAS_CORRECTION_INTERVAL  (30 * NAV_CPU_TICKS_PER_SECOND)

extern gpOS_task_t *     waas_back_end_get_task_wptr       ( void);
extern void         waas_back_end_init                ( void);
extern gnss_error_t waas_back_end_start               ( gpOS_partition_t *);
extern void         waas_back_end_set_sat             ( const tUChar);
extern void         waas_back_end_on                  ( void);
extern void         waas_back_end_off                 ( void);
extern void         waas_back_end_set_multi_ch_prn    ( const tUChar, const tUChar, const gnss_waas_status_t);
extern void         waas_back_end_set_prn_to_decode   ( const tUChar waas_satellite);
extern gnss_error_t waas_back_end_get_time2acq_flag   ( tUChar *);
extern void         waas_back_end_get_current_waas_id ( satid_t *);
extern void         waas_back_end_set_current_waas_id ( satid_t );
extern gnss_error_t waas_back_end_frame_ready         ( satid_t, tUChar *, gnss_time_t);
extern boolean_t    waas_back_end_suspend_done        ( void);
extern void         waas_back_end_auto_search_on      ( void);
extern void         waas_back_end_auto_search_off     ( void);
extern void         waas_back_end_autosearch_set_mask ( tUInt);
extern gnss_error_t waas_back_end_autosearch_get_last_decoded_sat(satid_t *);
extern void         waas_back_end_autosearch_set_timeout(tUInt ,tUInt ,tUInt ,tUInt);
extern void waas_cmd_service( const waas_be_msg_t *, nav_diff_correction_info_t* );
extern void waas_be_compute_waas_correction(satid_t, position_t*, visible_sats_data_t *);
extern void waas_be_decode_waas_correction(satid_t, tUChar *, tUInt);
extern void waas_be_run_auto_search(nav_diff_correction_info_t*, const nav_diff_correction_gnss_fix_info_t* );
#endif
