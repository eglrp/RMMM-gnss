//!
//!   \file       nmea.h
//!   \brief      <i><b> NMEA module, header file</b></i>
//!   \author     Fulvio Boggia
//!   \authors    Many
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup NMEA
//!   \{
//!

#ifndef NMEA_H
#define NMEA_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gnss_api.h"
#include "nmea_support.h"

#undef      NMEA_FRONTEND_SUPPORT
#undef      NMEA_REMOTE_FRONTEND_SUPPORT
#undef      NMEA_ADC_SUPPORT
#undef      NMEA_ANTENNA_SENSING_SUPPORT
#undef      NMEA_NOTCH_SUPPORT
#undef      NMEA_SQI_DATASTORAGE
#undef      NMEA_BINIMG_SUPPORT

#if defined( __STA2062__)
#define     NMEA_FRONTEND_SUPPORT
#endif

#if defined( __STA2064__) || defined( __generic__)
#define     NMEA_FRONTEND_SUPPORT
#if defined( RCIF_OVER_UART)
#define     NMEA_REMOTE_FRONTEND_SUPPORT
#define     NMEA_NOTCH_SUPPORT
#endif
#endif

#if defined( __STA8088__)
#define     NMEA_FRONTEND_SUPPORT
#define     NMEA_ADC_SUPPORT
#define     NMEA_ANTENNA_SENSING_SUPPORT
#define     NMEA_NOTCH_SUPPORT
#define     NMEA_BINIMG_SUPPORT
#if defined( NVM_SQI )
#define     NMEA_SQI_DATASTORAGE
#endif
#endif

#if defined( __STA8090__)
#define     NMEA_FRONTEND_SUPPORT
#define     NMEA_ADC_SUPPORT
#define     NMEA_ANTENNA_SENSING_SUPPORT
#define     NMEA_NOTCH_SUPPORT
#define     NMEA_BINIMG_SUPPORT
#if defined( NVM_SQI )
#define     NMEA_SQI_DATASTORAGE
#endif
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define GNS_NMEA_MSG                  0U
#define GGA_NMEA_MSG                  1U
#define GSA_NMEA_MSG                  2U
#define GST_NMEA_MSG                  3U
#define VTG_NMEA_MSG                  4U
#define NOISE_NMEA_MSG                5U
#define RMC_NMEA_MSG                  6U
#define RF_NMEA_MSG                   7U
#define TG_NMEA_MSG                   8U
#define TS_NMEA_MSG                   9U
#define PA_NMEA_MSG                   10U
#define SAT_NMEA_MSG                  11U
#define RES_NMEA_MSG                  12U
#define TIM_NMEA_MSG                  13U
#define WAAS_NMEA_MSG                 14U
#define DIFF_NMEA_MSG                 15U
#define SBAS_NMEA_MSG                 17U
#define RFTEST_NMEA_MSG               18U
#define GSV_NMEA_MSG                  19U
#define GLL_NMEA_MSG                  20U
#define PPS_NMEA_MSG                  21U
#define TEST_NMEA_MSG                 22U
#define CPU_USAGE_NMEA_MSG            23U
#define ZDA_NMEA_MSG	                24U
#define TRAIM_NMEA_MSG                25U
#define POSHOLD_NMEA_MSG              26U
#define KFCOV_NMEA_MSG                27U
#define AGPS_NMEA_MSG                 28U
#define LOWPOWER_NMEA_MSG             29U
#define NOTCH_NMEA_MSG                30U
#define TM_NMEA_MSG                   31U
#define PV_NMEA_MSG                   32U
#define PVQ_NMEA_MSG                  33U
#define UTC_NMEA_MSG                  34U
#define ADC_NMEA_MSG                  35U
#define ANT_NMEA_MSG                  36U
#define XTAL_NMEA_MSG                 37U
#define DR_NMEA_MSG                   38U
#define DTM_NMEA_MSG                  39U
#define EPHEM_NMEA_MSG                40U
#define ALM_NMEA_MSG                  41U
#define IONO_NMEA_MSG                 42U
#define GGTO_NMEA_MSG                 43U
#define BIAS_NMEA_MSG                 44U
#define GBS_NMEA_MSG                  45U
#define PVRAW_NMEA_MSG                46U
#define SBAS_NMEA_M                   47U
#define FEDATA_NMEA_MSG               48U

#define DR_RAW_SEN_SAMPLES_NMEA_MSG   54U
#define DR_CALIBRATION_NMEA_MSG       55U
#define DR_GPS_SAMPLE_NMEA_MSG        56U
#define DR_DEBUG_NMEA_MSG             57U
#define DR_STATE_NMEA_MSG             58U
#define DR_NVM_ACCESS_NMEA_MSG        59U
#define DR_SENSORS_NMEA_MSG           60U
#define DR_SLOPE_NMEA_MSG             61U
#define DR_COVARIANCE_NMEA_MSG        62U

#define NMEA_GSV_MAX_SATS             12
#define NMEA_OUT_MSG_BUFFER_SIZE      256

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  NMEA_TXMODE_ON_UTC_SECOND  =  0,
  NMEA_TXMODE_AFTER_FIX      =  1
} nmea_transmit_mode_t;

typedef tU32          (*nmea_inout_t)                           ( tChar *, tU32, gpOS_clock_t *);
typedef enum
{
  NMEA_INTERNAL_IF_MODE = 0,
  NMEA_EXTERNAL_IF_MODE = 1
}nmea_if_mode_t;

typedef void          (*nmea_fix_pos_vel_callback_t)            ( position_t *, tDouble *, tDouble *, tDouble);
typedef tInt          (*nmea_fix_pos_status_callback_t)         ( void);
typedef void          (*nmea_fix_data_update_callback_t)        ( void);
typedef void          (*nmea_fix_data_lock_callback_t)          ( void);
typedef void          (*nmea_fix_data_unlock_callback_t)        ( void);
typedef void          (*nmea_cmdif_msg_forward_callback_t)      ( tChar *, tInt);
typedef gpOS_error_t  (*nmea_get_external_sat_data_callback_t)  ( tInt, tInt *, tInt *, tInt*);
typedef void          (*nmea_external_cmdif_callback_t)         ( void);
typedef void          (*nmea_external_outmsg_callback_t)        ( void);
typedef boolean_t     (*nmea_fix_data_extrapolate_callback_t)   ( gpOS_clock_t *);

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern nmea_error_t NMEA_start                        ( const tInt, const nmea_transmit_mode_t, const tDouble, nmea_inout_t, nmea_inout_t);
extern nmea_error_t NMEA_start_p                      ( gpOS_partition_t *, const tInt, const nmea_transmit_mode_t, const tDouble, nmea_inout_t, nmea_inout_t);
extern nmea_error_t NMEA_second_input_start_p         ( gpOS_partition_t *, nmea_inout_t );
extern void         nmea_outmsg_transmit_short_header ( void);
extern void         nmea_execute_cpuusage_check       ( void);
extern void         nmea_send_msg_to_uart             ( const tChar *, const tInt );
extern void         nmea_send_outmsg                  ( const tChar *, const tInt, tUInt );
extern boolean_t    nmea_msg_list_check               ( const tUInt *, tUInt );
extern void         nmea_outmsg_set_contellation      ( const gnss_sat_type_mask_t );

extern void         nmea_set_fix_event_type                 ( tInt );
extern void         nmea_set_fix_pos_vel_callback           ( nmea_fix_pos_vel_callback_t );
extern void         nmea_set_fix_pos_status_callback        ( nmea_fix_pos_status_callback_t );
extern void         nmea_set_fix_data_update_callback       ( nmea_fix_data_update_callback_t );
extern void         nmea_set_fix_data_lock_callback         ( nmea_fix_data_lock_callback_t );
extern void         nmea_set_fix_data_unlock_callback       ( nmea_fix_data_unlock_callback_t );
extern void         nmea_set_get_external_sat_data_callback ( nmea_get_external_sat_data_callback_t func_ptr);
extern void         nmea_set_cmdif_msg_forward_callback     ( nmea_cmdif_msg_forward_callback_t );
extern void         nmea_rtcm_input_chan_enable             ( tInt );
extern void         nmea_set_external_cmdif_callback        ( nmea_external_cmdif_callback_t );
extern void         nmea_set_external_outmsg_callback       ( nmea_external_outmsg_callback_t );
extern void         nmea_set_if_mode                        ( nmea_if_mode_t );
extern void         nmea_set_if_mode_ext                    ( const nmea_if_mode_t, const nmea_if_mode_t);
extern void         nmea_set_fix_data_extrapolate_callback  ( nmea_fix_data_extrapolate_callback_t );
#endif  /* __NMEA_H__ */

//!   \}

