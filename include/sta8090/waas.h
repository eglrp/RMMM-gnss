/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999,2004,2006
                   All Rights Reserved

Source file name : waas.h
Author :           Dante Di Domenico
Description:       Header of main file of the waas library used in the
                   gps library

Date              Modification                            Initials
1  Oct 04         Created                                 DDD
16 Oct 06         Revision with new automaic              DDD
                  search function

************************************************************************

Here is the PRN/Satellite ID information for WAAS and EGNOS:

      Inmarsat          PRN

The following PRNs have been allocated to the WAAS system:
        AOR-W           122
        POR             134

The following PRNs have been allocated to the EGNOS system:
        AOR-E           120
       Artemis          124
        IOR-W           126
        IOR-E           131

The following PRNs have been allocated to the MSAS system:
       MTSAT-1          129
       MTSAT-2          137
********************************************************************/

#ifndef WAAS_H
#define WAAS_H

/*****************************************************************************
   includes
*****************************************************************************/
/*{{{  includes*/
#include "gnss_defs.h"
#include "waas_defs.h"
#include "waas_back_end.h"
/*}}}  */

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*{{{  Constant definitions for SBAS */
#define WAAS_CORRECTION_SCALE   10.0
#define SLOT_PER_SERVICE        8
/*}}}  */

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef void          (*nav_waas_be_compute_waas_correction_t)  ( satid_t, position_t *pos, visible_sats_data_t * );
typedef void          (*nav_waas_be_run_auto_search_t)          ( nav_diff_correction_info_t*, nav_diff_correction_gnss_fix_info_t* );
typedef void          (*nav_waas_cmd_service_t)                 ( const waas_be_msg_t *, nav_diff_correction_info_t* );
typedef gnss_error_t  (*nav_waas_fe_subframe_init_t)            ( gpOS_partition_t * );
typedef gnss_error_t  (*nav_waas_fe_subframe_decode_t)          ( const tVoid *);

typedef struct waas_callback_s
{
  nav_waas_be_compute_waas_correction_t   nav_waas_be_compute_waas_correction_cb;
  nav_waas_be_run_auto_search_t           nav_waas_be_run_auto_search_cb;
  nav_waas_cmd_service_t                  nav_waas_cmd_service_cb;
  nav_waas_fe_subframe_init_t             nav_waas_fe_subframe_init_cb;
  nav_waas_fe_subframe_decode_t           nav_waas_fe_subframe_decode_cb;
} waas_callback_t;

/*{{{  waas_msg_dump_message_t*/
typedef struct waas_msg_dump_message_s
{
  satid_t       sat_id;
  gnss_time_t   waas_frame_time;
  tUChar        buf[WAAS_FRAME_SIZE];
} waas_msg_dump_message_t;
/*}}}  */

/*****************************************************************************
   exported variables
*****************************************************************************/

extern const tChar *      waaslib_ver;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern const tChar *      waas_version                ( void);
extern gnss_error_t       waas_init                   ( void);
extern gnss_error_t       waas_init_p                 ( gpOS_partition_t *part);
extern void               waas_set_status             ( const gnss_waas_status_t new_waas_status);
extern gnss_waas_status_t waas_get_status             ( void);
extern boolean_t          waas_suspend_done           ( void);
extern void               waas_set_satellite          ( const satid_t new_satellite);
extern satid_t            waas_get_satellite          ( void);
extern void               waas_set_multi_ch           ( const chanid_t waas_channel, const satid_t waas_satellite, const gnss_waas_status_t waas_status);
extern gnss_error_t       waas_get_multi_ch           ( const chanid_t waas_channel, satid_t *waas_satellite, gnss_waas_status_t *waas_status);
extern void               waas_set_prn_to_decode      ( const satid_t waas_satellite);
extern gnss_error_t       waas_get_prn_to_decode      ( satid_t *waas_prn_to_decode_ptr);
extern void               waas_sat_pos_update         ( void);
extern tInt               waas_sat_get_elevation      ( const satid_t sat_id);
extern tInt               waas_sat_get_azimuth        ( const satid_t sat_id);
extern gnss_error_t       waas_sat_lon_get            ( const satid_t sat_id, tDouble *sbas_longitude_sat);
extern tInt               waas_prn_to_slot            ( const satid_t satid, tInt *slot2prn_db);
extern void               waas_get_sat_params         ( tSatData *waas_sat_db, tInt *slot2prn_ext);
extern void               waas_get_sat_correction     ( satid_t satid, tGpsSatCorrection *sat_correction);
extern void               waas_get_geo_params         ( satid_t satid, tGeoSatParam *params);
extern gnss_error_t       waas_automatic_search       ( satid_t *sbas_id_searched);
extern gnss_error_t       waas_set_sbas_list          ( tSbasInfo *sbas_list, tInt service_number);
extern void               waas_set_integrity_check    ( boolean_t new_int_check);
extern gnss_error_t       waas_get_time2acq_flag      ( tUChar *time2acq);

extern void               waas_enable_backup                ( void);
extern void               waas_disable_backup               ( void);
extern void               waas_enable_periodic_data_backup  ( boolean_t type);
extern void               waas_disable_periodic_backup      ( void);
extern void               waas_enable_periodic_backup       ( void);
extern boolean_t          waas_get_backup_status            ( void);
extern void               waas_execute_data_backup          ( void);
extern boolean_t          waas_get_periodic_backup_status   ( void);

extern gnss_error_t       waas_insert_sat                       ( tUInt prn, tDouble longitude,tSbasService service );
extern void               waas_remove_sat                       ( tUInt prn,tSbasService service );
extern void               waas_enable_autosearch                ( void);
extern void               waas_disable_autosearch               ( void);
extern boolean_t          waas_check_sat_limit                  ( const satid_t sat_id, tInt user_longitude);
extern void               waas_autosearch_set_sat_enabled_mask  ( tUInt waas_sat_enabled_mask);
extern void               waas_invalidate_data_correction       ( void);
extern gnss_error_t       waas_autosearch_get_last_decoded_sat  ( satid_t *waas_id);
extern void               waas_autosearch_set_timeout           ( tUInt t_next_sat, tUInt t_next_session, tUInt t_decoding, tUInt t_differential);

extern void               waas_install_callback                 ( void) ;

extern void               waas_msg_dump_enqueue       ( const satid_t sat_id, const void *buf, const gnss_time_t waas_frame_time);
extern void *             waas_msg_get_dump_msg       ( void);
extern void               waas_msg_dump_msg_release   ( waas_msg_dump_message_t *message);


#ifdef __cplusplus
}
#endif
#endif
