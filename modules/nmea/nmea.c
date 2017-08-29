//!
//!   \file       nmea.c
//!   \brief      <i><b>NMEA module, source file</b></i>
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

#include <math.h>
#include "clibs.h"
#include "macros.h"

#include "gpOS.h"
#include "gnss.h"
#include "gnss_defs.h"
#include "gnss_api.h"
#include "gnss_debug.h"
#include "gps_nvm.h"
#include "gnss_events.h"
#include "gnssapp.h"
#include "gnssapp_plugins.h"
#include "rtc.h"
#include "nmea.h"
#include "nmea_support.h"
#include "sw_config.h"
#include "platform.h"

#if defined( NMEA_FRONTEND_SUPPORT )
#include "frontend.h"
#endif

#if defined( NMEA_REMOTE_FRONTEND_SUPPORT)
#include "remote_fe.h"
#endif

#include "svc_mcu.h"
#include "svc_pwr.h"
#include "svc_ver.h"
#include "in_out.h"

#if defined( RTCM_LINKED )
#include "dgps.h"
#endif

#if defined( RTC_ERROR_TEST )
#include "gps_rtc.h"
#endif

#if defined( NMEA_SQI_DATASTORAGE )
#include "svc_sqi.h"
#endif // defined

#if defined( NMEA_ANTENNA_SENSING_SUPPORT)
#include "antenna_sensing.h"
#endif

#if defined( NMEA_ADC_SUPPORT )
#include "svc_adc.h"
#endif

#include "datum.h"

#include "gnss_msg.h"
#include "gnss_msg_queue.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

extern const tInt     gnss_navigate_task_priority;

extern gpOS_clock_t   nmea_outmsg_delaytonextfix;
extern tInt           nmea_outmsg_GGA_posdigit;
extern tInt           nmea_outmsg_RMC_posdigit;
extern tInt           nmea_outmsg_speed_digits;
extern tInt           nmea_outmsg_course_digits;

extern const tInt     nmea_outmsg2_task_priority;

extern const tInt     nmea_cmdif_task_priority;

extern tUInt  NMEA_msg_list[2];
extern tUInt  NMEA_msg_list_1[2];
extern tUInt  NMEA_msg_list_2[2];
extern tUInt  NMEA_on_debug_msg_list[2];
extern tUInt  NMEA_on_debug_msg_list_1[2];
extern tUInt  NMEA_on_debug_msg_list_2[2];
extern tU8    NMEA_msg_list_scaling;
extern tU8    NMEA_msg_list_scaling_1;
extern tU8    NMEA_msg_list_scaling_2;
extern tU8    nmea_on_debug_setting;
extern tU8    nmea_msg_timestamp_enabled;

extern tU8    nmea_port;

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define NMEA_OEM_ID "STM"

#define NMEA_UTC_SECOND_PREEMPT_TIME  250000

#define NMEA_OUTMSG_TASK_WS_SIZE  4160 //(4096)
#define NMEA_CMDIF_TASK_WS_SIZE   4096 //(3584)
#define NMEA_TASK_OUT2_WS_SIZE    4096 //(3584)

#define NMEA_TYPE_NORMAL        0
#define NMEA_TYPE_DEBUG         1
#define NMEA_TYPE_POS_DEBUG     2

#define MAX_NMEA_MSG_LENGTH 512

#define MS_TO_KNOTS   (3600.0 / 1852.0)
#define MS_TO_KPH     (3600.0 / 1000.0)

#if defined( NMEA_SQI_DATASTORAGE)
#define NMEA_SQIDS_BUFFER_SIZE            32U
#define NMEA_SQIDS_USERREGIONOFFSET       0x180000U
#define NMEA_SQIDS_USERREGIONSIZE         0x80000U
#endif

#if defined( NMEA_ADC_SUPPORT )
#if defined ( __STA8088__ )
#define SVC_ADC_AVERG_MIN                 256
#endif
#if defined ( __STA8090__ )
#define SVC_ADC_AVERG_MIN                 1
#define SVC_ADC_AVERG_MAX                 4
#endif
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  NMEA_READER_STATUS_START,
  NMEA_READER_STATUS_READING
} nmea_reader_status_t;

typedef enum
{
  NMEA_CMDID_INIT_GPS,
  NMEA_CMDID_INIT_TIME,
  NMEA_CMDID_INIT_FREQ,
  NMEA_CMDID_CLR_EPHEMS,
  NMEA_CMDID_CLR_ALMANACS,
  NMEA_CMDID_TOG_RMCMSG,
  NMEA_CMDID_INIT_COLD,
  NMEA_CMDID_INIT_WARM,
  NMEA_CMDID_INIT_HOT,

  NMEA_CMDID_DUMP_EPHEMS,
  NMEA_CMDID_LOAD_EPHEMS,
  NMEA_CMDID_NMEA_ON_OFF,
  NMEA_CMDID_GPS_RESET,
  NMEA_CMDID_SOFT_RESET,
  NMEA_CMDID_DUMP_ALMANAC,
  NMEA_CMDID_LOAD_ALMANAC,
  NMEA_CMDID_TIME_INV,
  NMEA_CMDID_GPS_SUSPEND,
  NMEA_CMDID_GPS_RESUME,
  NMEA_CMDID_SET_RANGE,
  NMEA_CMDID_NVM_SWAP,
  NMEA_CMDID_RF_TEST_ON,
  NMEA_CMDID_RF_TEST_OFF,
  NMEA_CMDID_SAVE_ALMANAC,
  NMEA_CMDID_SAVE_EPHEMS,
  NMEA_CMDID_GET_SW_VER,
  NMEA_CMDID_GPS_FWUPGRADE,
  NMEA_CMDID_NMEA_CONFIG,
  NMEA_CMDID_FIX2D_ON_OFF,
  NMEA_CMDID_GET_RTC_TIME,
  NMEA_CMDID_TASK_CHECK,
  NMEA_CMDID_STACK_CHECK_RESET,
  NMEA_CMDID_HEAP_CHECK,
  NMEA_CMDID_SET_ALGO_ON_OFF,
  NMEA_CMDID_GET_ALGO_STATUS,
  NMEA_CMDID_NVM_ITEM_INV,
  NMEA_CMDID_SET_GPS_POS,
  NMEA_CMDID_RTC_TEST_ON,
  NMEA_CMDID_TRK_INTERFERE_TEST,
  NMEA_CMDID_RTC_ERROR_TEST_CMD,
  NMEA_CMDID_SET_DIFF_SOURCE,
  NMEA_CMDID_SET_FIX_RATE,
  NMEA_CMDID_STOP_DETECTION_ON_OFF,
  NMEA_CMDID_WALKING_MODE_ON_OFF,
  NMEA_CMDID_SW_CONFIG_SET_PAR,
  NMEA_CMDID_SW_CONFIG_GET_PAR,
  NMEA_CMDID_SW_CONFIG_SAVE_PAR,
  NMEA_CMDID_SW_CONFIG_RESTORE_PAR,
  NMEA_CMDID_SW_CONFIG_GET_BLOCK,
  NMEA_CMDID_RTC_WRITE,
  NMEA_CMDID_FE_DUMP,
  NMEA_CMDID_FE_WRITE,
  NMEA_CMDID_RF_TEST_ADD,
  NMEA_CMDID_RF_TEST_DEL,
  NMEA_CMDID_DATUM_SELECT,
  NMEA_CMDID_DATUM_SET_PARAM,
  NMEA_CMDID_DEBUG_ON_OFF,
  NMEA_CMDID_NOTCH_ENABLE,
  NMEA_CMDID_ENABLE_DISABLE_POSITION_HOLD,
  NMEA_CMDID_SET_CONSTELLATION_MASK,
  NMEA_CMDID_PPS_IF_CMD,
  NMEA_CMDID_LOWPOWER_CMD,
  NMEA_CMDID_FORCESTANDBY_CMD,
  NMEA_CMDID_NMEA_REQUEST_CMD,
  NMEA_CMDID_STBIN_CMD,

#if defined( NMEA_SQI_DATASTORAGE )
  NMEA_CMDID_SQI_DATA_ERASE,
  NMEA_CMDID_SQI_DATA_SECTOR_ERASE,
  NMEA_CMDID_SQI_DATA_GET,
  NMEA_CMDID_SQI_DATA_SET,
#endif

  NMEA_CMDID_DUMP_IONO_PARAMS,
  NMEA_CMDID_SAVE_IONO_PARAMS,

  NMEA_CMDID_GALILEO,
  NMEA_CMDID_GALILEO_GGTO,
  NMEA_CMDID_GALILEO_DUMP_GGTO,
  NMEA_CMDID_SWCONFIG_WRITE_BLOCK,
  NMEA_CMDID_NUMBER
} nmea_cmdid_t;

// PPS interface related type
typedef enum
{
  NMEA_PPSIF_CMDMODE_GET = 1,
  NMEA_PPSIF_CMDMODE_SET = 2
} nmea_ppsif_cmdmode_t;

typedef enum
{
  NMEA_PPSIF_CMDID_ON_OFF                   = 1,
  NMEA_PPSIF_CMDID_OUT_MODE                 = 2,
  NMEA_PPSIF_CMDID_REFERENCE_CONSTELLATION  = 3,
  NMEA_PPSIF_CMDID_PULSE_DELAY              = 4,
  NMEA_PPSIF_CMDID_PULSE_DURATION           = 5,
  NMEA_PPSIF_CMDID_PULSE_POLARITY           = 6,
  NMEA_PPSIF_CMDID_PULSE_DATA               = 7,
  NMEA_PPSIF_CMDID_FIX_CONDITION            = 8,
  NMEA_PPSIF_CMDID_SAT_TRHESHOLD            = 9,
  NMEA_PPSIF_CMDID_ELEVATION_MASK           = 10,
  NMEA_PPSIF_CMDID_COSTELLATION_MASK        = 11,
  NMEA_PPSIF_CMDID_TIMING_DATA              = 12,
  NMEA_PPSIF_CMDID_POSITION_HOLD_DATA       = 13,
  NMEA_PPSIF_CMDID_AUTO_HOLD_SAMPLES        = 14,
  NMEA_PPSIF_CMDID_TRAIM                    = 15,
  NMEA_PPSIF_CMDID_TRAIM_USED               = 16,
  NMEA_PPSIF_CMDID_TRAIM_RES                = 17,
  NMEA_PPSIF_CMDID_TRAIM_REMOVED            = 18,
  NMEA_PPSIF_CMDID_REFERENCE_TIME           = 19,
  NMEA_PPSIF_CMDID_CONSTELLATION_RF_DELAY   = 20
} nmea_ppsif_cmdid_t;


/* To store data common to several NMEA output messages */
typedef struct nmea_outmsg_common_data
{
  // gnss_get_utc_time()
  tInt          hours;
  tInt          mins;
  tInt          secs;
  tInt          msecs;

  // nmea_fix_get_pos_status()
  tInt          fix_type;

  // gnss_fix_get_diff_status_local()
  diff_status_t diff_status;

  // gnss_get_date()
  tInt          year;
  tInt          month;
  tInt          day;

  // nmea_fix_get_position_velocity()
  position_t    extrap_pos;
  tDouble       speed;
  tDouble       course;

  // gnss_fix_get_geoid_msl_local()
  tDouble       geoid_msl;

  // gnss_fix_get_dops_local();
  tDouble       pdop;
  tDouble       hdop;
  tDouble       vdop;
  tDouble       gdop;

  // gnss_fix_get_num_sats_used_local()
  tInt          num_sats_used;
  tInt          num_sats_excluded;

} nmea_outmsg_common_data_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static gnss_events_synch_handler_t *nmea_outmsg_synchdlr_ptr;
static gnss_events_event_id_t nmea_outmsg_syncevent_id = GNSS_EVENTID_FIXREADY;

static gpOS_semaphore_t *   nmea_outmsg_access;
static gpOS_semaphore_t *   nmea_cmdif_semaphore;
static boolean_t            nmea_outmsg_enabled = TRUE;
static gpOS_task_t *        nmea_outmsg_task;
static boolean_t            nmea_outmsg_startmsg = TRUE;
static tChar                nmea_outmsg_talker_id;
static void *               nmea_outmsg_fixdata_ptr;
static nmea_transmit_mode_t nmea_outmsg_transmit_mode;
static gpOS_clock_t         nmea_outmsg_gga_timestamp;
static gpOS_clock_t         nmea_outmsg_first_gga_timestamp;
static gpOS_clock_t         nmea_outmsg_header_timestamp;

static gnss_sat_type_mask_t nmea_outmsg_const_mask;
static tInt                 nmea_outmsg_GSA_constellations_enabled;
static tInt                 nmea_outmsg_GNS_constellations_enabled;
static tInt                 nmea_outmsg_GNS_constellations_used;

static gnss_events_synch_handler_t *nmea_outmsg2_synchdlr_ptr;
static gpOS_task_t *        nmea_outmsg2_task;

static boolean_t            nmea_cmdif_eco_enabled = TRUE;
static boolean_t            nmea_cmdif_TTFF_msg_enabled = TRUE;
static boolean_t            nmea_cmdif_SAT_excl_present =TRUE;
static gpOS_task_t *        nmea_cmdif_task;
static gpOS_task_t *        nmea_cmdif2_task;
static gpOS_task_t **       nmea_rtcm_input_task = NULL;

static boolean_t            NMEA_initialized = FALSE;
static nmea_inout_t         nmea_ioport_read;
static nmea_inout_t         nmea_ioport_write;
static tInt                 nmea_baud_rate;

static satid_t              nmea_rftest_satid;

#if defined( NMEA_SQI_DATASTORAGE)
static tU8                  nmea_sqids_databuf[NMEA_SQIDS_BUFFER_SIZE];
static tU32                 nmea_sqids_databuf_size = NMEA_SQIDS_BUFFER_SIZE;
#endif

#if defined( NMEA_ADC_SUPPORT )
tU8                         adc_chan_to_read[8];
boolean_t                   adc_chan_read_mode_ON;
#endif

//#if defined( NMEA_ANTENNA_SENSING_SUPPORT )
//antenna_sensing_mode_t      adc_antenna_sensing_ON_OFF //adc_antenna_sensing_read_mode_ON;
//#endif

static nmea_if_mode_t       nmea_if_mode_cmdif  = NMEA_INTERNAL_IF_MODE;
static nmea_if_mode_t       nmea_if_mode_outmsg = NMEA_INTERNAL_IF_MODE;

static nmea_fix_pos_vel_callback_t         nmea_fix_pos_vel_callback         = NULL;
static nmea_fix_pos_status_callback_t      nmea_fix_pos_status_callback      = NULL;
static nmea_fix_data_update_callback_t     nmea_fix_data_update_callback     = NULL;
static nmea_fix_data_lock_callback_t       nmea_fix_data_lock_callback       = NULL;
static nmea_fix_data_unlock_callback_t     nmea_fix_data_unlock_callback     = NULL;

static nmea_cmdif_msg_forward_callback_t   nmea_cmdif_msg_forward_callback   = NULL;
static nmea_external_cmdif_callback_t      nmea_external_cmdif_callback      = NULL;
static nmea_external_outmsg_callback_t     nmea_external_outmsg_callback     = NULL;
static nmea_fix_data_extrapolate_callback_t  nmea_fix_data_extrapolate_callback = NULL;

const static tChar * const nmea_cmd_strings[] =
{
  "$PSTMINITGPS",             // NMEA_CMDID_INIT_GPS,
  "$PSTMINITTIME",            // NMEA_CMDID_INIT_TIME,
  "$PSTMINITFRQ",             // NMEA_CMDID_INIT_FREQ,
  "$PSTMCLREPHS",             // NMEA_CMDID_CLR_EPHEMS,
  "$PSTMCLRALMS",             // NMEA_CMDID_CLR_ALMANACS,
  "$PSTMRMC",                 // NMEA_CMDID_TOG_RMCMSG,
  "$PSTMCOLD",                // NMEA_CMDID_INIT_COLD,
  "$PSTMWARM",                // NMEA_CMDID_INIT_WARM,
  "$PSTMHOT",                 // NMEA_CMDID_INIT_HOT,

  "$PSTMDUMPEPHEMS",          // NMEA_CMDID_DUMP_EPHEMS,
  "$PSTMLOADEPHEMS",          // NMEA_CMDID_LOAD_EPHEMS,
  "$PSTMNMEAONOFF",           // NMEA_CMDID_NMEA_ON_OFF,
  "$PSTMGPSRESET",            // NMEA_CMDID_GPS_RESET,
  "$PSTMSRR",                 // NMEA_CMDID_SOFT_RESET,
  "$PSTMDUMPALMANAC",         // NMEA_CMDID_DUMP_ALMANAC,
  "$PSTMLOADALM",             // NMEA_CMDID_LOAD_ALMANAC,
  "$PSTMTIMEINV",             // NMEA_CMDID_TIME_INV,
  "$PSTMGPSSUSPEND",          // NMEA_CMDID_GPS_SUSPEND,
  "$PSTMGPSRESTART",          // NMEA_CMDID_GPS_RESUME,
  "$PSTMSETRANGE",            // NMEA_CMDID_SET_RANGE,
  "$PSTMNVMSWAP",             // NMEA_CMDID_NVM_SWAP,
  "$PSTMRFTESTON",            // NMEA_CMDID_RF_TEST_ON,
  "$PSTMRFTESTOFF",           // NMEA_CMDID_RF_TEST_OFF,
  "$PSTMALMANAC",             // NMEA_CMDID_SAVE_ALMANAC,
  "$PSTMEPHEM",               // NMEA_CMDID_SAVE_EPHEMS,
  "$PSTMGETSWVER",            // NMEA_CMDID_GET_SW_VER,
  "$PSTMFWUPGRADE",           // NMEA_CMDID_GPS_FWUPGRADE,
  "$PSTMNMEACONFIG",          // NMEA_CMDID_NMEA_CONFIG,
  "$PSTM2DFIXONOFF",          // NMEA_CMDID_FIX2D_ON_OFF,
  "$PSTMGETRTCTIME",          // NMEA_CMDID_GET_RTC_TIME,
  "$PSTMTASKCHECK",           // NMEA_CMDID_TASK_CHECK,
  "$PSTMSTACKURES",           // NMEA_CMDID_STACK_CHECK_RESET,
  "$PSTMHEAPCHECK",           // NMEA_CMDID_HEAP_CHECK,
  "$PSTMSETALGO",             // NMEA_CMDID_SET_ALGO_ON_OFF,
  "$PSTMGETALGO",             // NMEA_CMDID_GET_ALGO_STATUS,
  "$PSTMNVMITEMINV",          // NMEA_CMDID_NVM_ITEM_INV,
  "$PSTMSETPOS",              // NMEA_CMDID_SET_GPS_POS,
  "$PSTMRTCTEST",             // NMEA_CMDID_RTC_TEST_ON,
  "$PSTMTRKJAMMER",           // NMEA_CMDID_TRK_INTERFERE_TEST,
  "$PSTMRTCERRTEST",          // NMEA_CMDID_RTC_ERROR_TEST_CMD,
  "$PSTMSETDIFFSOURCE",       // NMEA_CMDID_SET_DIFF_SOURCE,
  "$PSTMSETFIXRATE",          // NMEA_CMDID_SET_FIX_RATE,
  "$PSTMSTOPDETECTIONONOFF",  // NMEA_CMDID_STOP_DETECTION_ON_OFF,
  "$PSTMWALKINGONOFF",        // NMEA_CMDID_WALKING_MODE_ON_OFF,
  "$PSTMSETPAR",              // NMEA_CMDID_SW_CONFIG_SET_PAR,
  "$PSTMGETPAR",              // NMEA_CMDID_SW_CONFIG_GET_PAR,
  "$PSTMSAVEPAR",             // NMEA_CMDID_SW_CONFIG_SAVE_PAR,
  "$PSTMRESTOREPAR",          // NMEA_CMDID_SW_CONFIG_RESTORE_PAR,
  "$PSTMGETCONFIG",           // NMEA_CMDID_SW_CONFIG_GET_BLOCK,
  "$PSTMRTCWRITE",            // NMEA_CMDID_RTC_WRITE,
  "$PSTMFEDUMP",              // NMEA_CMDID_FE_DUMP,
  "$PSTMFEWRITE",             // NMEA_CMDID_FE_WRITE,
  "$PSTMRFSATADD",            // NMEA_CMDID_RF_TEST_ADD,
  "$PSTMRFSATDEL",            // NMEA_CMDID_RF_TEST_DEL,
  "$PSTMDATUMSELECT",         // NMEA_CMDID_DATUM_SELECT,
  "$PSTMDATUMSETPARAM",       // NMEA_CMDID_DATUM_SET_PARAM,
  "$PSTMDEBUGONOFF",          // NMEA_CMDID_DEBUG_ON_OFF,
  "$PSTMNOTCH",               // NMEA_CMDID_NOTCH_ENABLE,
  "$PSTMENABLEPOSITIONHOLD",  // NMEA_CMDID_ENABLE_DISABLE_POSITION_HOLD,
  "$PSTMSETCONSTMASK",        // NMEA_CMDID_SET_CONSTELLATION_MASK,
  "$PSTMPPS",                 // NMEA_CMDID_PPS_IF_CMD,
  "$PSTMLOWPOWERONOFF",       // NMEA_CMDID_LOWPOWER_CMD,
  "$PSTMFORCESTANDBY",        // NMEA_CMDID_FORCESTANDBY_CMD,
  "$PSTMNMEAREQUEST",         // NMEA_CMDID_NMEA_REQUEST_CMD,
  "$PSTMSTBIN",               // NMEA_CMDID_STBIN_CMD,
#if defined( NMEA_SQI_DATASTORAGE )
  "$PSTMSQIERASE",            // NMEA_CMDID_SQI_DATA_ERASE,
  "$PSTMSQISECTORERASE",      // NMEA_CMDID_SQI_DATA_SECTOR_ERASE,
  "$PSTMSQIGET",              // NMEA_CMDID_SQI_DATA_GET,
  "$PSTMSQISET",              // NMEA_CMDID_SQI_DATA_SET,
#endif                        //

  "$PSTMDUMPIONO",            // NMEA_CMDID_DUMP_IONO_PARAMS,
  "$PSTMIONOPARAMS",          // NMEA_CMDID_SAVE_IONO_PARAMS,

  "$PSTMGALILEO",             // NMEA_CMDID_GALILEO,
  "$PSTMGALILEOGGTO",         // NMEA_CMDID_GALILEO_GGTO,
  "$PSTMGALILEODUMPGGTO",     // NMEA_CMDID_GALILEO_DUMP_GGTO,
  "$PSTMSETSWCONFIG",        // NMEA_CMDID_SWCONFIG_WRITE_BLOCK
  "$PSTMDRNVMSAVE",          //NMEA_CMDID_DR_NVM_SAVE
  NULL                       // NMEA_CMDID_NUMBER
};

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

// Output messages related procedures
static gpOS_task_exit_status_t  nmea_outmsg_process( gpOS_task_param_t );
static void                   nmea_outmsg_transmit_after_fix( void );
static void                   nmea_outmsg_transmit_on_utc_sec( void );
static void                   nmea_outmsg_transmit( const tUInt *, void *, const tDouble, const gnss_time_t );
static void                   nmea_outmsg_transmit_configured( const tUInt *, void *, const tDouble, const gnss_time_t );
static void                   nmea_outmsg_transmit_full_header( void );

static gpOS_task_exit_status_t nmea_outmsg2_process( void * );
static void               nmea_outmsg2_transmit_after_fix( void );

static void     nmea_outmsg_get_common_data( void *, nmea_outmsg_common_data_t *, const tDouble, const gnss_time_t );
static void     nmea_outmsg_send_RMC( const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_GGA( const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_VTG( void *, const tDouble, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_GST( void *, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_GSA( void *, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_GBS( void *, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_GNS( void *, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_DTM( void *, const tDouble, const nmea_outmsg_common_data_t *);
static void     nmea_outmsg_send_GSV( void * );
static void     nmea_outmsg_send_PA( void * );
static void     nmea_outmsg_send_RF( void * );
static void     nmea_outmsg_send_SAT( void * );
static void     nmea_outmsg_send_TG( void *, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_TS( void * );
static void     nmea_outmsg_send_PRES( void * );
static void     nmea_outmsg_send_VRES( void * );
static void     nmea_outmsg_send_TIM( void * );
static void     nmea_outmsg_send_GLL( const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_ZDA( const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_TESTRF( void * );
static void     nmea_outmsg_send_KFCOV( void * );
static void     nmea_outmsg_send_PPSDATA( void );
static void     nmea_outmsg_send_POSHOLD( void );
static void     nmea_outmsg_send_TRAIM( void );
static void     nmea_outmsg_send_NOTCH( void );
static void     nmea_outmsg_send_TM( void * );
static void     nmea_outmsg_send_NOISE( void * );
static void     nmea_outmsg_send_CPUUSAGE( void );
static void     nmea_outmsg_send_LOWPOWERSTATUS( void );
static void     nmea_outmsg_send_XTALSTATUS( void );
#if defined( NMEA_ADC_SUPPORT )
static void     nmea_outmsg_send_ADCDATA( void );
#endif
static void     nmea_outmsg_send_ANTENNASENSING_STATUS( void );
static void     nmea_outmsg_send_PV( void *, const nmea_outmsg_common_data_t * );
static void     nmea_outmsg_send_PVQ( void *, const tDouble, const gnss_time_t );
static void     nmea_outmsg_send_UTC( const nmea_outmsg_common_data_t *, const gnss_time_t );
static void     nmea_outmsg_send_PVRAW( void *, const tDouble, const gnss_time_t );
static void     nmea_outmsg_send_TTFF( void );
static void     nmea_outmsg_send_EPHEM( void );
static void     nmea_outmsg_send_ALM( void );
static void     nmea_outmsg_send_BIASDATA( void * );
static void     nmea_outmsg_send_IONO( void );
static void     nmea_outmsg_send_FEDATA( void );

static void     nmea_outmsg_send_GGTO( void );

static void     nmea_get_next_utc_clock( const tInt, const tDouble, const gpOS_clock_t, tDouble *, gpOS_clock_t * );

// Command related procedures
static gpOS_task_exit_status_t  nmea_cmdif_process( gpOS_task_param_t );
static void                   nmea_cmdif_extended_read( nmea_inout_t, tChar * );
static nmea_error_t           nmea_cmdif_parse( const tChar *cmd, const tUInt cmd_size, tChar *cmd_par );

static void         nmea_cmdif_exec_initgps( tChar * );
static void         nmea_cmdif_exec_inittime( tChar * );
static void         nmea_cmdif_exec_initfreq( tChar * );
static void         nmea_cmdif_exec_clrephems( tChar * );
static void         nmea_cmdif_exec_clralmanacs( tChar * );
static void         nmea_cmdif_exec_toggleRMCmsg( tChar * );
static void         nmea_cmdif_exec_initcold( tChar * );
static void         nmea_cmdif_exec_initwarm( tChar * );
static void         nmea_cmdif_exec_inithot( tChar * );
static void         nmea_cmdif_exec_loadephems( tChar * );
static void         nmea_cmdif_exec_dumpephems( tChar * );
static void         nmea_cmdif_exec_saveephems( tChar * );
static void         nmea_cmdif_exec_gpsreset( tChar * );
static void         nmea_cmdif_exec_softreset( tChar * );
static void         nmea_cmdif_exec_loadalms( tChar * );
static void         nmea_cmdif_exec_savealms( tChar * );
static void         nmea_cmdif_exec_dumpalms( tChar * );
static void         nmea_cmdif_exec_setrange( tChar * );
static void         nmea_cmdif_exec_rfteston( tChar * );
static void         nmea_cmdif_exec_rftestoff( tChar * );
static void         nmea_cmdif_exec_rftestadd( tChar * );
static void         nmea_cmdif_exec_rftestdel( tChar * );
static void         nmea_cmdif_exec_get_swver( tChar * );
static void         nmea_cmdif_exec_fwupgrade( tChar * );
static void         nmea_cmdif_exec_nmeacfg( tChar * );
static void         nmea_cmdif_exec_set2dfix( tChar * );
static void         nmea_cmdif_exec_getrtctime( tChar * );
static void         nmea_cmdif_exec_taskscheck( tChar * );
static void         nmea_cmdif_exec_checkmem( tChar * );
static void         nmea_cmdif_exec_resetstackusage( tChar * );
static void         nmea_cmdif_exec_getalgostatus( tChar * );
static void         nmea_cmdif_exec_setalgostatus( tChar * );
static void         nmea_cmdif_exec_nvm_invalidate( tChar * );
static void         nmea_cmdif_exec_jammer_test( tChar * );
static void         nmea_cmdif_exec_set_gnss_pos( tChar * );
static void         nmea_cmdif_exec_setstopdet( tChar * );
static void         nmea_cmdif_exec_setwalkingmode( tChar * );
static void         nmea_cmdif_exec_setrtctest( tChar * );
static void         nmea_cmdif_exec_rtcerrortest( tChar * );

static void         nmea_cmdif_exec_fedump( tChar * );
static void         nmea_cmdif_exec_fewrite( tChar * );

static void         nmea_cmdif_exec_setdiffsource( tChar * );
static void         nmea_cmdif_exec_setfixrate( tChar * );

static void         nmea_cmdif_exec_swcfg_setpar( tChar * );
static void         nmea_cmdif_exec_swcfg_getpar( tChar * );
static void         nmea_cmdif_exec_swcfg_savepar( tChar * );
static void         nmea_cmdif_exec_swcfg_restorepar( tChar * );

static void         nmea_cmdif_exec_swcfg_getblock( tChar * );

static void         nmea_cmdif_exec_rtcwrite( tChar * );
static void         nmea_cmdif_exec_datum_select( tChar * );
static void         nmea_cmdif_exec_datum_set_param( tChar * );

static void         nmea_cmdif_exec_setnmea( tChar * );
static void         nmea_cmdif_exec_setdebug( tChar * );
static void         nmea_cmdif_exec_setposhold( tChar * );
static void         nmea_cmdif_exec_setconstmask( tChar * );
static void         nmea_cmdif_exec_ppsif( tChar * );
static void         nmea_cmdif_exec_notch( tChar * );

static void         nmea_cmdif_exec_gpssuspend( tChar * );
extern void         nmea_cmdif_exec_gpsrestart( tChar * );
static void         nmea_cmdif_exec_timeinvalidate( tChar * );
static void         nmea_cmdif_exec_nvmswap( tChar * );

#if defined( NMEA_SQI_DATASTORAGE)
static void         nmea_cmdif_exec_sqids_erase( tChar * );
static void         nmea_cmdif_exec_sqids_sector_erase( tChar * );
static void         nmea_cmdif_exec_sqids_get( tChar * );
static void         nmea_cmdif_exec_sqids_set( tChar * );
#endif

static void         nmea_cmdif_exec_dump_iono_params( tChar * );
static void         nmea_cmdif_exec_save_iono_params( tChar * );

static void         nmea_cmdif_exec_galileo( tChar * );
static void         nmea_cmdif_exec_galileo_ggto( tChar * );
static void         nmea_cmdif_exec_galileo_dump_ggto( tChar * );

static void         nmea_cmdif_exec_write_swconfig_data_block( tChar * );

// Internal function
static nmea_error_t nmea_cmdif_send_ephemeris( const tInt );
static nmea_error_t nmea_cmdif_send_almanac( const tInt );
static void         nmea_cmdif_send_iono_params( const gnss_sat_type_t );

static void         nmea_send_galilo_ggto( void );

#if defined( NMEA_NOTCH_SUPPORT )
static tInt         nmea_cmdif_notch_parse_params( tChar*, tInt*, tInt*, tInt*, tShort* ,tShort*,tInt*);
#endif

static void         nmea_cmdif_exec_nmea_request( tChar * );
static void         nmea_cmdif_exec_stbin( tChar * );

static void         nmea_outmsg_send_GSV_for_constellation( void *, gnss_sat_type_mask_t, const visible_sats_data_t *, tChar * );
static void         nmea_outmsg_transmit_swconfig( void );

static void         nmea_swconfig_senddatablock( tInt , tChar * );

static void         nmea_cmdif_exec_setlowpower( tChar * );
static void         nmea_cmdif_exec_force_standby( tChar * );

static void         nmea_dynamic_set_application_on_off( const gnss_sat_type_mask_t mask);

static const nmea_support_cmdif_exec_t nmea_cmdif_exec_table[] =
{
  nmea_cmdif_exec_initgps,            // NMEA_CMDID_INIT_GPS,
  nmea_cmdif_exec_inittime,           // NMEA_CMDID_INIT_TIME,
  nmea_cmdif_exec_initfreq,           // NMEA_CMDID_INIT_FREQ,
  nmea_cmdif_exec_clrephems,          // NMEA_CMDID_CLR_EPHEMS,
  nmea_cmdif_exec_clralmanacs,        // NMEA_CMDID_CLR_ALMANACS,
  nmea_cmdif_exec_toggleRMCmsg,       // NMEA_CMDID_TOG_RMCMSG,
  nmea_cmdif_exec_initcold,           // NMEA_CMDID_INIT_COLD,
  nmea_cmdif_exec_initwarm,           // NMEA_CMDID_INIT_WARM,
  nmea_cmdif_exec_inithot,            // NMEA_CMDID_INIT_HOT,

  nmea_cmdif_exec_dumpephems,         // NMEA_CMDID_DUMP_EPHEMS,
  nmea_cmdif_exec_loadephems,         // NMEA_CMDID_LOAD_EPHEMS,
  nmea_cmdif_exec_setnmea,            // NMEA_CMDID_NMEA_ON_OFF,
  nmea_cmdif_exec_gpsreset,           // NMEA_CMDID_GPS_RESET,
  nmea_cmdif_exec_softreset,          // NMEA_CMDID_SOFT_RESET,
  nmea_cmdif_exec_dumpalms,           // NMEA_CMDID_DUMP_ALMANAC,
  nmea_cmdif_exec_loadalms,           // NMEA_CMDID_LOAD_ALMANAC,
  nmea_cmdif_exec_timeinvalidate,     // NMEA_CMDID_TIME_INV,
  nmea_cmdif_exec_gpssuspend,         // NMEA_CMDID_GPS_SUSPEND,
  nmea_cmdif_exec_gpsrestart,         // NMEA_CMDID_GPS_RESUME,
  nmea_cmdif_exec_setrange,           // NMEA_CMDID_SET_RANGE,
  nmea_cmdif_exec_nvmswap,            // NMEA_CMDID_NVM_SWAP,
  nmea_cmdif_exec_rfteston,           // NMEA_CMDID_RF_TEST_ON,
  nmea_cmdif_exec_rftestoff,          // NMEA_CMDID_RF_TEST_OFF,
  nmea_cmdif_exec_savealms,           // NMEA_CMDID_SAVE_ALMANAC,
  nmea_cmdif_exec_saveephems,         // NMEA_CMDID_SAVE_EPHEMS,
  nmea_cmdif_exec_get_swver,          // NMEA_CMDID_GET_SW_VER,
  nmea_cmdif_exec_fwupgrade,          // NMEA_CMDID_GPS_FWUPGRADE,
  nmea_cmdif_exec_nmeacfg,            // NMEA_CMDID_NMEA_CONFIG,
  nmea_cmdif_exec_set2dfix,           // NMEA_CMDID_FIX2D_ON_OFF,
  nmea_cmdif_exec_getrtctime,         // NMEA_CMDID_GET_RTC_TIME,
  nmea_cmdif_exec_taskscheck,         // NMEA_CMDID_TASK_CHECK,
  nmea_cmdif_exec_resetstackusage,    // NMEA_CMDID_STACK_CHECK_RESET,
  nmea_cmdif_exec_checkmem,           // NMEA_CMDID_HEAP_CHECK,
  nmea_cmdif_exec_setalgostatus,      // NMEA_CMDID_SET_ALGO_ON_OFF,
  nmea_cmdif_exec_getalgostatus,      // NMEA_CMDID_GET_ALGO_STATUS,
  nmea_cmdif_exec_nvm_invalidate,     // NMEA_CMDID_NVM_ITEM_INV,
  nmea_cmdif_exec_set_gnss_pos,       // NMEA_CMDID_SET_GPS_POS,
  nmea_cmdif_exec_setrtctest,         // NMEA_CMDID_RTC_TEST_ON,
  nmea_cmdif_exec_jammer_test,        // NMEA_CMDID_TRK_INTERFERE_TEST,
  nmea_cmdif_exec_rtcerrortest,       // NMEA_CMDID_RTC_ERROR_TEST_CMD,
  nmea_cmdif_exec_setdiffsource,      // NMEA_CMDID_SET_DIFF_SOURCE,
  nmea_cmdif_exec_setfixrate,         // NMEA_CMDID_SET_FIX_RATE,
  nmea_cmdif_exec_setstopdet,         // NMEA_CMDID_STOP_DETECTION_ON_OFF,
  nmea_cmdif_exec_setwalkingmode,     // NMEA_CMDID_WALKING_MODE_ON_OFF,
  nmea_cmdif_exec_swcfg_setpar,       // NMEA_CMDID_SW_CONFIG_SET_PAR,
  nmea_cmdif_exec_swcfg_getpar,       // NMEA_CMDID_SW_CONFIG_GET_PAR,
  nmea_cmdif_exec_swcfg_savepar,      // NMEA_CMDID_SW_CONFIG_SAVE_PAR,
  nmea_cmdif_exec_swcfg_restorepar,   // NMEA_CMDID_SW_CONFIG_RESTORE_PAR,
  nmea_cmdif_exec_swcfg_getblock,     // NMEA_CMDID_SW_CONFIG_GET_BLOCK,
  nmea_cmdif_exec_rtcwrite,           // NMEA_CMDID_RTC_WRITE,
  nmea_cmdif_exec_fedump,             // NMEA_CMDID_FE_DUMP,
  nmea_cmdif_exec_fewrite,            // NMEA_CMDID_FE_WRITE,
  nmea_cmdif_exec_rftestadd,          // NMEA_CMDID_RF_TEST_ADD,
  nmea_cmdif_exec_rftestdel,          // NMEA_CMDID_RF_TEST_DEL,
  nmea_cmdif_exec_datum_select,       // NMEA_CMDID_DATUM_SELECT,
  nmea_cmdif_exec_datum_set_param,    // NMEA_CMDID_DATUM_SET_PARAM,
  nmea_cmdif_exec_setdebug,           // NMEA_CMDID_DEBUG_ON_OFF,
  nmea_cmdif_exec_notch,              // NMEA_CMDID_NOTCH_ENABLE,
  nmea_cmdif_exec_setposhold,         // NMEA_CMDID_ENABLE_DISABLE_POSITION_HOLD,
  nmea_cmdif_exec_setconstmask,       // NMEA_CMDID_SET_CONSTELLATION_MASK,
  nmea_cmdif_exec_ppsif,              // NMEA_CMDID_PPS_IF_CMD,
  nmea_cmdif_exec_setlowpower,        // NMEA_CMDID_LOWPOWER_CMD,
  nmea_cmdif_exec_force_standby,      // NMEA_CMDID_FORCESTANDBY_CMD,
  nmea_cmdif_exec_nmea_request,       // NMEA_CMDID_NMEA_REQUEST_CMD,
  nmea_cmdif_exec_stbin,              // NMEA_CMDID_STBIN_CMD,
#if defined( NMEA_SQI_DATASTORAGE )
  nmea_cmdif_exec_sqids_erase,        //NMEA_CMDID_SQI_DATA_ERASE,
  nmea_cmdif_exec_sqids_sector_erase, //NMEA_CMDID_SQI_DATA_SECTOR_ERASE,
  nmea_cmdif_exec_sqids_get,          //NMEA_CMDID_SQI_DATA_GET,
  nmea_cmdif_exec_sqids_set,          //NMEA_CMDID_SQI_DATA_SET,
#endif

  nmea_cmdif_exec_dump_iono_params,   // NMEA_CMDID_DUMP_IONO_PARAMS,
  nmea_cmdif_exec_save_iono_params,   // NMEA_CMDID_SAVE_IONO_PARAMS,

  nmea_cmdif_exec_galileo,            // NMEA_CMDID_GALILEO
  nmea_cmdif_exec_galileo_ggto,       // NMEA_CMDID_GALILEO_GGTO
  nmea_cmdif_exec_galileo_dump_ggto,  // NMEA_CMDID_GALILEO_DUMP_GGTO
  nmea_cmdif_exec_write_swconfig_data_block, // NMEA_CMDID_SWCONFIG_WRITE_BLOCK
  NULL                                //NMEA_CMDID_NUMBER
};

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

void nmea_set_fix_pos_vel_callback( nmea_fix_pos_vel_callback_t func_ptr )
{
  nmea_fix_pos_vel_callback = func_ptr;
}

void nmea_set_fix_pos_status_callback( nmea_fix_pos_status_callback_t func_ptr )
{
  nmea_fix_pos_status_callback = func_ptr;
}

void nmea_set_fix_data_update_callback( nmea_fix_data_update_callback_t func_ptr )
{
  nmea_fix_data_update_callback = func_ptr;
}

void nmea_set_fix_data_lock_callback( nmea_fix_data_lock_callback_t func_ptr )
{
  nmea_fix_data_lock_callback = func_ptr;
}

void nmea_set_fix_data_unlock_callback( nmea_fix_data_unlock_callback_t func_ptr )
{
  nmea_fix_data_unlock_callback = func_ptr;
}

void nmea_set_cmdif_msg_forward_callback( nmea_cmdif_msg_forward_callback_t func_ptr )
{
  nmea_cmdif_msg_forward_callback = func_ptr;
}

void nmea_rtcm_input_chan_enable( tInt input_channel )
{
  if ( input_channel != 0 )
  {
    nmea_rtcm_input_task = &nmea_cmdif2_task;
  }
  else
  {
    nmea_rtcm_input_task = &nmea_cmdif_task;
  }
}

void nmea_set_external_cmdif_callback( nmea_external_cmdif_callback_t func_ptr )
{
  nmea_external_cmdif_callback = func_ptr;
}

void nmea_set_external_outmsg_callback( nmea_external_outmsg_callback_t func_ptr )
{
  nmea_external_outmsg_callback = func_ptr;
}

void nmea_set_fix_data_extrapolate_callback(nmea_fix_data_extrapolate_callback_t func_ptr)
{
  nmea_fix_data_extrapolate_callback = func_ptr;
}

void nmea_set_if_mode( nmea_if_mode_t if_mode )
{
  nmea_if_mode_cmdif  = if_mode;
  nmea_if_mode_outmsg = if_mode;
}

void nmea_set_if_mode_ext( const nmea_if_mode_t if_mode_cmdif, const nmea_if_mode_t if_mode_outmsg)
{
  nmea_if_mode_cmdif  = if_mode_cmdif;
  nmea_if_mode_outmsg = if_mode_outmsg;
}

static void nmea_fix_get_position_velocity( position_t *extrap_pos, tDouble *course, tDouble *speed, tDouble delay, void *data_p )
{
  if ( nmea_fix_pos_vel_callback != NULL )
  {
    nmea_fix_pos_vel_callback( extrap_pos, course, speed, delay );
  }
  else
  {
    position_t pos;
    velocity_t vel;
    gnss_fix_get_fil_pos_vel_local(&pos, &vel, data_p);

    if (delay == 0.0)
    {
      extrap_pos->latitude  = pos.latitude;
      extrap_pos->longitude = pos.longitude;
      extrap_pos->height    = pos.height;
    }
    else
    {
      nmea_support_extrapolate_pos( &pos, &vel, delay, extrap_pos );
    }
    gnss_conv_vel_to_course_speed( &vel, course, speed );
  }
}

static tInt nmea_fix_get_pos_status( void *data_p )
{
  tInt result;
  result = ( tInt )gnss_fix_get_pos_status_local( data_p );

  if ( result == ( tInt )NO_FIX )
  {
    if ( nmea_fix_pos_status_callback != NULL )
    {
      result = ( tInt )nmea_fix_pos_status_callback();
    }
  }

  return result;
}

void nmea_set_fix_event_type( tInt fix_event_type )
{
  nmea_outmsg_syncevent_id = ( gnss_events_event_id_t )fix_event_type;
}

static void nmea_fix_data_update( void *data_p )
{
  if ( nmea_fix_data_update_callback != NULL )
  {
    nmea_fix_data_update_callback();
  }
  else
  {
    gnss_fix_store_local( data_p );
  }
}

static void nmea_fix_data_lock( void )
{
  if ( nmea_fix_data_lock_callback != NULL )
  {
    nmea_fix_data_lock_callback();
  }

  gnss_fix_read_claim();
}

static void nmea_fix_data_unlock( void )
{
  gnss_fix_read_release();

  if ( nmea_fix_data_unlock_callback != NULL )
  {
    nmea_fix_data_unlock_callback();
  }
}

boolean_t nmea_msg_list_check( const tUInt *nmea_msg_list, tUInt message_type_id )
{
  if ( nmea_msg_list[( tInt )( message_type_id / 32 )] & ( 1 << ( message_type_id % 32 ) ) )
  {
    return TRUE;
  }

  return FALSE;
}

static boolean_t nmea_fix_data_extrapolate(gpOS_clock_t *fix_time)
{
  if(nmea_fix_data_extrapolate_callback != NULL)
  {
    nmea_fix_data_extrapolate_callback(fix_time);
    return( TRUE );
  }

  *fix_time = 0;

  return( FALSE );
}

/********************************************//**
 * \brief   Allocates and runs two processes which send
 *          and receive NMEA format messages via the rs232 port.
 *
 * \param   nmea_msg_list         List of messages to show
 * \param   nmea_outmsg_transmit_mode    transmit mode
 * \param   requested_sleep_time  ?
 * \param   read_fun              Commands input procedure
 * \param   write_fun             Output procedure
 * \return  NMEA_ERROR if something went wrong
 *
 ***********************************************/
nmea_error_t NMEA_start( const tInt nmea_msg_list, const nmea_transmit_mode_t nmea_outmsg_transmit_mode, const tDouble requested_sleep_time, nmea_inout_t read_fun, nmea_inout_t write_fun )
{
  return NMEA_start_p( NULL, nmea_msg_list, nmea_outmsg_transmit_mode, requested_sleep_time, read_fun, write_fun );
}

/********************************************//**
 * \brief   Allocates and runs two processes which send
 *          and receive NMEA format messages via the rs232 port.
 *
 * \param   part                  Partition to use to start NMEA
 * \param   nmea_msg_list         List of messages to show
 * \param   nmea_outmsg_transmit_mode    transmit mode
 * \param   requested_sleep_time  ?
 * \param   read_fun              Commands input procedure
 * \param   write_fun             Output procedure
 * \return  NMEA_ERROR if something went wrong
 *
 ***********************************************/
nmea_error_t NMEA_start_p( gpOS_partition_t *part, const tInt nmea_msg_list, const nmea_transmit_mode_t transmit_mode, const tDouble requested_sleep_time, nmea_inout_t read_fun, nmea_inout_t write_fun )
{
  tInt nmea_outmsg_task_priority;

  if ( NMEA_initialized == FALSE )
  {
    NMEA_msg_list[0]      = nmea_msg_list;
    nmea_outmsg_transmit_mode = transmit_mode;

    /* Register Nmea to the AMQ */
    if ( gnss_msg_queue_Nmea_register() == GNSS_ERROR )
    {
      ERROR_MSG( "[nmea][start] Error regsitering to AMQ\r\n" );
      return ( NMEA_ERROR );
    }

    if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ) )
    {
      nmea_outmsg2_synchdlr_ptr = gnss_events_synch_handler_create();

      if ( nmea_outmsg2_synchdlr_ptr == GNSS_EVENTS_SYNCHANDLER_NONE )
      {
        return ( NMEA_ERROR );
      }

      nmea_outmsg_task_priority = gnss_navigate_task_priority - 1;
    }
    else
    {
      nmea_outmsg_task_priority = nmea_cmdif_task_priority;
    }

    nmea_outmsg_synchdlr_ptr = gnss_events_synch_handler_create();

    if ( nmea_outmsg_synchdlr_ptr == GNSS_EVENTS_SYNCHANDLER_NONE )
    {
      return ( NMEA_ERROR );
    }

    nmea_cmdif_eco_enabled = ( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, NMEA_CMDIF_ECO_ON_OFF_SWITCH );
    nmea_cmdif_TTFF_msg_enabled = ( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, NMEA_TTFF_MSG_ON_OFF_SWITCH );
    nmea_cmdif_SAT_excl_present = ( boolean_t )sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, SAT_EXCL_PRESENT_GSA_GGA );
    sw_config_get_param( CURRENT_CONFIG_DATA, NMEA_TALKER_ID, &nmea_outmsg_talker_id );
    nmea_ioport_read = read_fun;
    nmea_ioport_write = write_fun;
    nmea_outmsg_access = gpOS_semaphore_create_p( SEM_FIFO, part, 1 );
    nmea_cmdif_semaphore = gpOS_semaphore_create_p( SEM_FIFO, part, 1 );
    nmea_outmsg_task = gpOS_task_create_p( part, nmea_outmsg_process, NULL, NMEA_OUTMSG_TASK_WS_SIZE, nmea_outmsg_task_priority, "NMEA output Process", gpOS_TASK_FLAGS_ACTIVE );
    nmea_cmdif_task = gpOS_task_create_p( part, nmea_cmdif_process, ( void * )nmea_ioport_read, NMEA_CMDIF_TASK_WS_SIZE, nmea_cmdif_task_priority + 1, "NMEA input Process", gpOS_TASK_FLAGS_ACTIVE );

    if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ) || ( NMEA_on_debug_msg_list_2[0] != 0 ) || ( NMEA_on_debug_msg_list_2[1] != 0 ) )
    {
      gnss_fix_create_local_copy( (gpOS_partition_t *)part, &nmea_outmsg_fixdata_ptr );
      nmea_outmsg2_task = gpOS_task_create_p( part, nmea_outmsg2_process, NULL, NMEA_TASK_OUT2_WS_SIZE, nmea_outmsg2_task_priority, "NMEA msg output Process 2", gpOS_TASK_FLAGS_ACTIVE );

      if ( ( nmea_outmsg2_task == NULL ) || ( nmea_outmsg_fixdata_ptr == NULL ) )
      {
        gpOS_task_delete( nmea_outmsg2_task );
        ERROR_MSG( "[nmea][start] Error creating OS items.\r\n" );
        return NMEA_ERROR;
      }
    }

    if (
      ( nmea_outmsg_access == NULL ) ||
      ( nmea_cmdif_task == NULL ) ||
      ( nmea_outmsg_task == NULL )
    )
    {
      gpOS_task_delete( nmea_outmsg_task );
      gpOS_task_delete( nmea_cmdif_task );
      gpOS_semaphore_delete( nmea_outmsg_access );
      gpOS_task_delete( nmea_outmsg2_task );
      ERROR_MSG( "[nmea][start] Error creating OS items.\r\n" );
      return NMEA_ERROR;
    }

    if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ))
    {
      gnss_events_install( nmea_outmsg_syncevent_id, nmea_outmsg2_synchdlr_ptr );
    }

    {
      tUInt datum_code = 0;

      if(GNSS_NO_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, DATUM_ID, &datum_code ))
      {
        datum_init(datum_code);
      }
    }

    nmea_support_set_first_timing_msg_flag( TRUE );
    nmea_support_set_first_gga_msg_flag( TRUE );
    NMEA_initialized = TRUE;

    return ( NMEA_OK );
  }

  return ( NMEA_ERROR );
}


nmea_error_t NMEA_second_input_start_p( gpOS_partition_t *part, nmea_inout_t read_fun )
{
  nmea_cmdif2_task = gpOS_task_create_p( part, nmea_cmdif_process, ( void * )read_fun, NMEA_CMDIF_TASK_WS_SIZE, nmea_cmdif_task_priority + 1, "NMEA input 2 Process", gpOS_TASK_FLAGS_ACTIVE );

  if ( nmea_cmdif2_task == NULL )
  {
    gpOS_task_delete( nmea_cmdif2_task );
    return ( NMEA_ERROR );
  }

  return ( NMEA_OK );
}

static void nmea_cmdif_msg_forward( tChar *msg, tInt len )
{
  if ( nmea_cmdif_msg_forward_callback != NULL )
  {
    if ( *nmea_rtcm_input_task != NULL )
    {
      if ( *nmea_rtcm_input_task == gpOS_task_get_id() )
      {
        nmea_cmdif_msg_forward_callback( msg, len );
      }
    }
  }
}

static void nmea_cmdif_send_eco( tChar *msg, tInt len )
{
  if ( nmea_cmdif_eco_enabled == TRUE )
  {
    nmea_send_msg_to_uart( msg, len );
  }
}

/********************************************//**
 * \brief   Process that handles commands coming from input
 *          source
 *
 * \param   p   Task parameter (NULL)
 * \return  None
 *
 ***********************************************/
static gpOS_task_exit_status_t nmea_cmdif_process( gpOS_task_param_t p )
{
  boolean_t exit_flag = FALSE;

  while ( exit_flag == FALSE )
  {
    if ( nmea_if_mode_cmdif == NMEA_EXTERNAL_IF_MODE )
    {
      if ( nmea_external_cmdif_callback != NULL )
      {
        nmea_external_cmdif_callback();
      }
      else
      {
        nmea_if_mode_cmdif = NMEA_INTERNAL_IF_MODE;
      }
    }

    if ( nmea_if_mode_cmdif == NMEA_INTERNAL_IF_MODE )
    {
      tChar msg[MAX_NMEA_MSG_LENGTH];
      nmea_inout_t read_fun = ( nmea_inout_t )p;
      nmea_cmdif_extended_read( read_fun, msg );
      gpOS_semaphore_wait( nmea_cmdif_semaphore );

      if ( nmea_support_verify_checksum( msg ) == NMEA_OK )
      {
        tUInt cmd_size = 0;
        tChar *par_ptr = NULL;

        if ( nmea_support_cmdif_getcmdinfo( msg, &cmd_size, &par_ptr ) == NMEA_OK )
        {
          if ( gnssapp_plugins_nmea_cmdif_parse( msg, cmd_size, par_ptr ) == NMEA_NOT_VALID )
          {
            if ( nmea_cmdif_parse( msg, cmd_size, par_ptr ) == NMEA_NOT_VALID )
            {
              nmea_cmdif_msg_forward( msg, _clibs_strlen( msg ) - 1 );
            }
            else
            {
              nmea_cmdif_send_eco( msg, _clibs_strlen( msg ) );
            }
          }
          else
          {
            nmea_cmdif_send_eco( msg, _clibs_strlen( msg ) );
          }
        }
        else
        {
          nmea_cmdif_send_eco( msg, _clibs_strlen( msg ) );
        }
      }
      else
      {
        nmea_cmdif_send_eco( msg, _clibs_strlen( msg ) );
      }

      gpOS_semaphore_signal( nmea_cmdif_semaphore );
    }
  }

  // should never reach this
  return -1;
}

/********************************************//**
 * \brief   Reads a command from the input stream consisting
 *          of first byte of '$' and ending with CR or LF.
 *
 * \param   cmd   Command string pointer
 * \return  void
 *
 ***********************************************/
static void nmea_cmdif_extended_read( nmea_inout_t read_fun, tChar *cmd )
{
  /*{{{  decs*/
  boolean_t msg_rdy = FALSE;
  static boolean_t prev_msg_state = ( boolean_t )NMEA_READER_STATUS_START;
  boolean_t msg_state = ( boolean_t )NMEA_READER_STATUS_START;
  tInt  msg_length = 0;
  tChar *msg_ptr = cmd;
  tChar *msg_start_ptr = cmd;
  /*}}}  */

  while ( !msg_rdy )
  {
    tChar ch;

    /* Ask for a character */
    if( read_fun( &ch, 1, gpOS_TIMEOUT_INFINITY) > 0)
    {
      /* Parse character */
      switch ( msg_state )
      {
        case NMEA_READER_STATUS_START:
          if ( ch == '$' )
          {
            msg_state = ( boolean_t )NMEA_READER_STATUS_READING;
            msg_ptr = cmd;
            *msg_ptr = ch;
            msg_length = 1;
            msg_ptr++;
          }
          else
          {
            if ( !( ( ( prev_msg_state == NMEA_READER_STATUS_READING ) && ( ch == '\n' ) ) || ( ( prev_msg_state == NMEA_READER_STATUS_READING ) && ( ch == '\r' ) ) ) )
            {
              nmea_cmdif_msg_forward( &ch, 1 );
            }
          }

          prev_msg_state = ( boolean_t )NMEA_READER_STATUS_START;
          break;

        case NMEA_READER_STATUS_READING:
          *msg_ptr = ch;
          msg_ptr++;
          msg_length++;

          if ( ( ch == '\n' ) || ( ch == '\r' ) )
          {
            msg_rdy = TRUE;
            *msg_ptr = '\n';
            msg_ptr++;
            msg_length++;
            *msg_ptr = '\0';
          }

          if ( msg_length >= MAX_NMEA_MSG_LENGTH )
          {
            msg_state = ( boolean_t )NMEA_READER_STATUS_START;
            nmea_cmdif_msg_forward( msg_start_ptr, msg_length );
          }

          *msg_ptr = '\0';
          prev_msg_state = ( boolean_t )NMEA_READER_STATUS_READING;
          break;
      }
    }
  }
}

/********************************************//**
 * \brief   Extract the data from the init GPS command
 *
 * \param   msg     Parameters string
 * \return  NMEA_ERROR if the command data does not meet expected criteria
 *
 ***********************************************/
static void nmea_cmdif_exec_initgps( tChar *msg )
{
  /*{{{  decs*/
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index = 0;
  tInt field_count = 0;
  tDouble lat_val, lat_deg, lat_min;
  tDouble lon_val, lon_deg, lon_min;
  tDouble height_val;
  tInt year_val, month_val, day_val;
  tInt hours_val, mins_val, secs_val;
  tChar lat_sense, lon_sense;
  gnss_error_t error = GNSS_NO_ERROR;
  /*}}}  */
  field_count = _clibs_sscanf( msg, ",%2lf%6lf,%1c,%3lf%6lf,%1c,%5lf,%2d,%2d,%4d,%2d,%2d,%2d",
                               &lat_deg, &lat_min, &lat_sense, &lon_deg, &lon_min, &lon_sense, &height_val,
                               &day_val, &month_val, &year_val, &hours_val, &mins_val, &secs_val
                             );

  if ( field_count == 13 )
  {
    if ( ( lat_deg > 90.0 )  || ( lat_deg < 0.0 ) ||
         ( lat_min >= 60.0 ) || ( lat_min < 0.0 ) ||
         ( lon_deg > 180.0 ) || ( lon_deg < 0.0 ) ||
         ( lon_min >= 60.0 ) || ( lon_min < 0.0 ) ||
         ( height_val > 10000.0 ) || ( height_val < 0.0 ) ||
         ( ( lat_sense != 'N' ) && ( lat_sense != 'S' ) ) ||
         ( ( lon_sense != 'W' ) && ( lon_sense != 'E' ) )
       )
    {
      error = GNSS_ERROR;
    }

    lat_val = lat_deg + (lat_min / 60.0);
    lon_val = lon_deg + (lon_min / 60.0);

    if ( lat_sense == 'S' )
    {
      lat_val = -lat_val;
    }

    if ( lon_sense == 'W' )
    {
      lon_val = -lon_val;
    }

    if ( ( lat_val > 90.0 ) || ( lat_val < ( -90.0 ) ) )
    {
      error = GNSS_ERROR;
    }

    if ( ( lon_val > 180.0 ) || ( lon_val < ( -180.0 ) ) )
    {
      error = GNSS_ERROR;
    }

    if ( !gnss_date_time_valid( year_val, month_val, day_val, hours_val, mins_val, secs_val ) )
    {
      error = GNSS_ERROR;
    }

    error = gnss_set_pos( lat_val, lon_val, height_val );

    if ( error == GNSS_NO_ERROR )
    {
      error = gnss_set_time( year_val, month_val, day_val, hours_val, mins_val, secs_val, GNSS_SAT_TYPE_GPS );
    }
  }
  else
  {
    error = GNSS_ERROR;
  }

  if ( error == GNSS_NO_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMINITGPSOK" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMINITGPSERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
  return;
}

/********************************************//**
 * \brief   Extract the data from the init time command
 *
 * \param   msg     Parameters string
 * \return  NMEA_ERROR if the command data does not meet expected criteria
 *
 ***********************************************/
static void nmea_cmdif_exec_inittime( tChar *msg )
{
  /*{{{  decs*/
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index = 0;
  tInt field_count = 0;
  tInt year_val, month_val, day_val;
  tInt hours_val, mins_val, secs_val;
  gnss_error_t error = GNSS_NO_ERROR;
  /*}}}  */
  field_count = _clibs_sscanf( msg, ",%2d,%2d,%4d,%2d,%2d,%2d",
                               &day_val, &month_val, &year_val, &hours_val, &mins_val, &secs_val
                             );

  if ( field_count == 6 )
  {
    if ( !gnss_date_time_valid( year_val, month_val, day_val, hours_val, mins_val, secs_val ) )
    {
      error = GNSS_ERROR;
    }
    else
    {
      error = gnss_set_time( year_val, month_val, day_val, hours_val, mins_val, secs_val, GNSS_SAT_TYPE_GPS );
    }
  }
  else
  {
    error = GNSS_ERROR;
  }

  if ( error == GNSS_NO_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMINITTIMEOK" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMINITTIMEERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
  return;
}

/********************************************//**
 * \brief   Clear all ephemerides
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_clrephems( tChar *cmd_par )
{
  gnss_clear_all_ephems();
}

/********************************************//**
 * \brief   Clear all almanacs
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_clralmanacs( tChar *cmd_par )
{
  gnss_clear_all_almanacs();
}

/********************************************//**
 * \brief   Toggle RMC message output
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_toggleRMCmsg( tChar *cmd_par )
{
  NMEA_msg_list[0] ^= ( 1 << RMC_NMEA_MSG ); // this is left only for backward compatibility
}

/********************************************//**
 * \brief   Execute RTC invalidation command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_timeinvalidate( tChar *cmd_par )
{
  gnss_test_invalidate_rtc();
}

/********************************************//**
 * \brief   Execute cold start command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_initcold( tChar *cmd_par )
{
  tInt field_count = 0;
  tU32 type;
  boolean_t lowpower_status = FALSE;

  GPS_DEBUG_MSG( ( "\r\n\n\n\n\nInitialising Cold Start....\r\n" ) );

  svc_pwr_get_lowpower_allowed( &lowpower_status);
  svc_pwr_set_lowpower_allowed( FALSE);

  gnssapp_suspend();
  field_count = _clibs_sscanf( cmd_par, ",%u", &type );

  if ( field_count == 1 )
  {
    nmea_support_initcoldstart( type );
  }
  else
  {
    tU32 config_type;
    sw_config_get_param( CURRENT_CONFIG_DATA, COLD_START_TYPE_ID, &config_type );
    nmea_support_initcoldstart( config_type );
  }

  nmea_support_restart();
  gnssapp_restart();
  svc_pwr_set_lowpower_allowed( lowpower_status);
}

/********************************************//**
 * \brief   Execute warm start command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_initwarm( tChar *cmd_par )
{
  tInt field_count = 0;
  tInt delta;
  boolean_t lowpower_status = FALSE;

  GPS_DEBUG_MSG( ( "\r\n\n\n\n\nInitialising Warm Start...\r\n" ) );
  field_count = _clibs_sscanf( cmd_par, ",%d", &delta );

  svc_pwr_get_lowpower_allowed( &lowpower_status);
  svc_pwr_set_lowpower_allowed( FALSE);

  gnssapp_suspend();
  // specific actions for warm start
  gnss_clear_all_ephems();

  if ( field_count == 1 )
  {
    tDouble time_error;
    gnss_time_t gnss_time_result;
    gpOS_clock_t cpu_time_result;
    rtc_status_t rtc_status;
    time_validity_t stored_time_validity;
    gnss_time_reference_t gnss_time_reference_copy;
    tInt err = delta;
    gnss_sat_type_t sat_type;
    tDouble utc_delta_time;
    //gnssapp_suspend();
    gpOS_interrupt_lock();
    gnss_rtc_read( &gnss_time_result, &cpu_time_result, &rtc_status, &stored_time_validity, &sat_type, &utc_delta_time );
    time_error = ( tDouble )err / 1000;
    gnss_time_result = gnss_time_plus( &gnss_time_result, time_error );
    gnss_time_reference_copy.gps_time = gnss_time_result;
    gnss_time_reference_copy.sat_type = sat_type;
    gnss_time_reference_copy.cpu_time = cpu_time_result;

    if ( abs_int(err) < 50/*ms*/ )
    {
      gnss_rtc_write( gnss_time_reference_copy, ACCURATE_TIME, TRUE, utc_delta_time );
    }
    else
    {
      gnss_rtc_write( gnss_time_reference_copy, RTC_TIME, TRUE, utc_delta_time );
    }

    gpOS_interrupt_unlock();
  }

  //nvm_swap(TRUE);
  nmea_support_restart();
  gnssapp_restart();
  svc_pwr_set_lowpower_allowed( lowpower_status);
}

/********************************************//**
 * \brief   Execute hot start command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_inithot( tChar *cmd_par )
{
  boolean_t lowpower_status = FALSE;
  GPS_DEBUG_MSG( ( "\r\n\n\n\n\nInitialising Hot Start...\r\n" ) );
  svc_pwr_get_lowpower_allowed( &lowpower_status);
  svc_pwr_set_lowpower_allowed( FALSE);
  gnssapp_suspend();
  nmea_support_restart();
  gnssapp_restart();
  svc_pwr_set_lowpower_allowed( lowpower_status);
}

/********************************************//**
 * \brief   Execute change frequency command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_initfreq( tChar *msg )
{
  tInt field_count;
  tInt freq_val;
  field_count = _clibs_sscanf( msg, ",%6d", &freq_val );

  if ( field_count == 1 )
  {
    if ( ( freq_val > 132000 ) || ( freq_val < ( -132000 ) ) )
    {
      return;
    }

    gnss_set_centre_freq( ( tDouble ) freq_val );
  }

  return;
}

/********************************************//**
 * \brief   Parse the command and call the appropiate
 *          GPS library func associated to it
 *
 * \param   cmd   The command to be parsed
 * \return  void
 *
 ***********************************************/
static nmea_error_t nmea_cmdif_parse( const tChar *cmd, const tUInt cmd_size, tChar *cmd_par )
{
  tUInt cmdid;
  gpOS_clock_t t1;
  t1 = gpOS_time_now();

  if ( nmea_support_cmdif_getid( cmd, cmd_size, nmea_cmd_strings, NMEA_CMDID_NUMBER, ( tUInt * )&cmdid ) == NMEA_NOT_VALID )
  {
    return NMEA_NOT_VALID;
  }

  GPS_DEBUG_MSG( ( "NMEA Parsing Time %d\r\n", gpOS_time_minus( gpOS_time_now(), t1 ) ) );
  nmea_cmdif_exec_table[cmdid]( cmd_par );
  return NMEA_OK;
}

/********************************************//**
 * \brief   A parallel process which reads navigation data from
 *          the gps core and produces position and velocity
 *          output messages to the serial port.
 *          Note that this process should never terminate.
 *
 * \param   p   Task parameter (NULL)
 * \return  None
 *
 ***********************************************/
static gpOS_task_exit_status_t nmea_outmsg_process( gpOS_task_param_t p )
{
  boolean_t exit_flag = FALSE;

  while ( exit_flag == FALSE )
  {
    if ( nmea_if_mode_outmsg == NMEA_EXTERNAL_IF_MODE )
    {
      if ( nmea_external_outmsg_callback != NULL )
      {
        nmea_external_outmsg_callback();
      }
      else
      {
        nmea_if_mode_outmsg = NMEA_INTERNAL_IF_MODE;
      }
    }

    if ( nmea_if_mode_outmsg == NMEA_INTERNAL_IF_MODE )
    {
      gnss_events_install( nmea_outmsg_syncevent_id, nmea_outmsg_synchdlr_ptr );
      nmea_outmsg_transmit_short_header();

      while ( nmea_if_mode_outmsg == NMEA_INTERNAL_IF_MODE )
      {
        if ( nmea_outmsg_transmit_mode == NMEA_TXMODE_AFTER_FIX )
        {
          nmea_outmsg_transmit_after_fix();
        }
        else
        {
          nmea_outmsg_transmit_on_utc_sec();
        }
      }

      gnss_events_uninstall( nmea_outmsg_syncevent_id, nmea_outmsg_synchdlr_ptr );
    }
  }

  // should never reach this
  return -1;
}

/********************************************//**
 * \brief   Message transmission loop for UTC second based messages
 *          (type A, B or C): the messages are transmitted on UTC
 *          sec. boundaries
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_transmit_on_utc_sec( void )
{
  tDouble delta_t;
  tInt week_n;
  gpOS_clock_t fix_clock, utc_clock;
  tDouble tow, utc_tow;
  gnss_time_t utc_time;
  tUInt msg_list[2];
  gnss_events_wait( nmea_outmsg_syncevent_id, nmea_outmsg_synchdlr_ptr );
  nmea_fix_data_update( NULL );
  nmea_fix_data_lock();
  gnss_fix_get_time( &week_n, &tow, &fix_clock );
  nmea_get_next_utc_clock( week_n, tow, fix_clock, &utc_tow, &utc_clock );
  nmea_fix_data_unlock();
  gpOS_task_delay_until( utc_clock + 1 * gpOS_timer_ticks_per_sec() - NMEA_UTC_SECOND_PREEMPT_TIME );

  while ( nmea_if_mode_outmsg == NMEA_INTERNAL_IF_MODE )
  {
    nmea_fix_data_lock();
    gnss_fix_get_time( &week_n, &tow, &fix_clock );
    nmea_get_next_utc_clock( week_n, tow, fix_clock, &utc_tow, &utc_clock );
    /* calculate the time to extrapolate the fix forward by */
    delta_t = gpOS_time_minus( utc_clock, fix_clock ) / gnss_cpu_clock_rate_hi();
    utc_time.tow = utc_tow;
    utc_time.week_n = week_n;
    msg_list[0] = NMEA_msg_list[0] | NMEA_on_debug_msg_list[0];
    msg_list[1] = NMEA_msg_list[1] | NMEA_on_debug_msg_list[1];
    nmea_outmsg_transmit( msg_list, NULL, delta_t, utc_time );
    nmea_fix_data_unlock();
    gpOS_task_delay_until( utc_clock + gpOS_timer_ticks_per_sec() - NMEA_UTC_SECOND_PREEMPT_TIME );
  }
}

/********************************************//**
 * \brief   Message transmission loop for fix based messages
 *          (type D): the message is transmitted as soon as a new
 *          fix is available
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_transmit_after_fix( void )
{
  tInt week_n;
  gpOS_clock_t fix_clock;
  gpOS_clock_t ext_fix_clock = 0; //extrapolated fix clock
  tDouble tow;
  gnss_time_t gnss_time = {0};
  gnss_time_t utc_time;
  tInt fix_rate_ms;
  tInt NMEA_msg_list_period_ms = 0;
  tInt NMEA_msg_list_1_period_ms = 0;
  tInt NMEA_msg_list_event_counter = NMEA_msg_list_scaling;
  tInt NMEA_msg_list_1_event_counter = NMEA_msg_list_scaling_1;
  tUInt msg_list[2];
  fix_rate_ms = 1000 * gnss_get_fix_rate();
  NMEA_msg_list_period_ms = ( tInt )( fix_rate_ms * NMEA_msg_list_scaling );
  NMEA_msg_list_1_period_ms = ( tInt )( fix_rate_ms * NMEA_msg_list_scaling_1 );

  while ( nmea_if_mode_outmsg == NMEA_INTERNAL_IF_MODE )
  {
    gnss_events_wait( nmea_outmsg_syncevent_id, nmea_outmsg_synchdlr_ptr );
    NMEA_msg_list_event_counter--;
    NMEA_msg_list_1_event_counter--;

    if ( ( NMEA_msg_list_event_counter == 0 ) || ( NMEA_msg_list_1_event_counter == 0 ) )
    {
      boolean_t extr_status = FALSE;

      nmea_fix_data_update( NULL );
      nmea_fix_data_lock();
      gnss_fix_get_time_local( &week_n, &tow, &fix_clock, NULL );
      gnss_time.week_n = week_n;
      gnss_time.tow = tow;
      utc_time = gnss_time_to_utc_time( gnss_time, gnss_fix_get_time_sat_type_local( NULL ) );

      extr_status = nmea_fix_data_extrapolate(&ext_fix_clock);

      if( extr_status == TRUE)
      {
        GPS_DEBUG_MSG(( "[nmea] Fix Time: GNSS %d, Extr %d\r\n", fix_clock, ext_fix_clock));
      }

      { // synchronization checking
        tDouble tow_frac;
        tInt tow_frac_int;
        tInt synch_error;
        tow_frac = ( tow + 0.05 ); // rounding to nearest 100ms
        tow_frac = ( tow_frac - ( tInt )tow_frac ) * 10;
        tow_frac_int = ( tInt )tow_frac * 100;

        if ( NMEA_msg_list_event_counter == 0 )
        {
          msg_list[0] = NMEA_msg_list[0] | NMEA_on_debug_msg_list[0];
          msg_list[1] = NMEA_msg_list[1] | NMEA_on_debug_msg_list[1];

          if( ext_fix_clock == 0)
          {
            nmea_outmsg_transmit( msg_list, NULL, 0, utc_time );
          }
          synch_error = 0;

          if ( fix_rate_ms < 1000 )
          {
            synch_error = ( tow_frac_int % NMEA_msg_list_period_ms ) / fix_rate_ms;

            if ( synch_error > (NMEA_msg_list_scaling / 2) )
            {
              synch_error = synch_error - NMEA_msg_list_scaling;
            }
          }

          NMEA_msg_list_event_counter = NMEA_msg_list_scaling - synch_error;
        }

        if ( NMEA_msg_list_1_event_counter == 0 )
        {
          msg_list[0] = NMEA_msg_list_1[0] | NMEA_on_debug_msg_list_1[0];
          msg_list[1] = NMEA_msg_list_1[1] | NMEA_on_debug_msg_list_1[1];

          if( ext_fix_clock != 0)
          {
            if( gpOS_time_after( ext_fix_clock, fix_clock))
            {
              GPS_DEBUG_MSG(( "[nmea] Extr to Fix Time: %d\r\n", gpOS_time_minus(ext_fix_clock,fix_clock)));

              /* Correct fix time with extrapolated time */
              utc_time.tow += ((tDouble)gpOS_time_minus(ext_fix_clock,fix_clock))/gpOS_timer_ticks_per_sec();
            }
          }

          if( (extr_status == FALSE) || (ext_fix_clock != 0))
          {
            nmea_outmsg_transmit( msg_list, NULL, 0, utc_time );
          }

          synch_error = 0;

          if ( fix_rate_ms < 1000 )
          {
            synch_error = ( tow_frac_int % NMEA_msg_list_1_period_ms ) / fix_rate_ms;

            if ( synch_error > (NMEA_msg_list_scaling_1 / 2) )
            {
              synch_error = synch_error - NMEA_msg_list_scaling_1;
            }
          }

          NMEA_msg_list_1_event_counter = NMEA_msg_list_scaling_1 - synch_error;
        }
      }
      nmea_fix_data_unlock();
    }
  }
}

/********************************************//**
 * \brief   transmits the selected type of message and an echo of
 *          any received commands if available
 *
 * \param   msg_list    Message list
 * \param   data_p      Fix data structure
 * \param   delta_t     the delay (in secs) of the current UTC second from the last fix
 * \param   utc_time    the current utc second (time of week)
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_transmit( const tUInt *msg_list, void * data_p, const tDouble delta_t, const gnss_time_t utc_time )
{
  if((msg_list[0] == 0) && (msg_list[1] == 0))
  {
    return;
  }

  nmea_outmsg_transmit_configured( msg_list, data_p, delta_t, utc_time );
}

/********************************************//**
 * \brief   Transmits a proprietary header (GPS S/W version and date)
 *
 * \return  None
 *
 ***********************************************/
void nmea_outmsg_transmit_short_header( void )
{
  tChar header[100];
  tInt index;

  if ( sw_config_get_software_switch_status( ST_HEADERS_SWITCH ) )
  {
    index = _clibs_sprintf( header, "$PSTMVER,%s", gnss_version() );
  }
  else
  {
    sw_config_t *sw_config_ptr;
    sw_config_get_current( &sw_config_ptr );
    index = _clibs_sprintf( header, "$GPTXT,%s", sw_config_ptr->text_message );
  }

  index += _clibs_sprintf( &header[index], "*%02X\r\n", nmea_support_checksum( header ) );
  nmea_outmsg_header_timestamp = gpOS_time_now();
  nmea_send_msg_to_uart( header, index );
}

/********************************************//**
 * \brief   Transmits a proprietary header (App_OS S/W version and date)
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_transmit_full_header( void )
{
  tInt index;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  out_msg[0] = 0;

  if ( sw_config_get_software_switch_status( ST_HEADERS_SWITCH ) )
  {
    index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gpOS_version() );

    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gnssapp_version() );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

#if defined( NMEA_BINIMG_SUPPORT )
    index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gnssapp_binimg_version() );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
#endif

    index = _clibs_sprintf( out_msg, "$PSTMVER,%s_%x", "SWCFG", gnssapp_swcfg_version() );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    {
      tInt id;

      for ( id = 0; id < GNSSAPP_PLUGINS_NUMBER; id++ )
      {
        const tChar *ver_ptr = NULL;
        gnssapp_plugins_cmd_param_t gpparam;
        gpparam.data_ptr = ( void * )&ver_ptr;
        gnssapp_plugins_cmd( ( gnssapp_plugins_id_t )id, GNSSAPP_PLUGINS_CMD_GETVER, &gpparam );

        if ( ver_ptr != NULL )
        {
          index = _clibs_sprintf( out_msg, "$PSTMVER,%s", ver_ptr );
          index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
          nmea_send_msg_to_uart( out_msg, index );
        }
      }
    }

    index = _clibs_sprintf( out_msg, "$PSTMVER,%s_%08x", svc_mcu_getprodname(), svc_mcu_getromver() );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    index = _clibs_sprintf( out_msg, "$GPTXT,%s", gnss_supplier_id() );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );

    if ( sw_config_get_software_switch_status( CONFIG_TEXT_AS_HEADER_SWITCH ) )
    {
      sw_config_t *sw_config_ptr;
      sw_config_get_current( &sw_config_ptr );
      index = _clibs_sprintf( out_msg, "$GPTXT,%s", sw_config_ptr->text_message );
      index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
      nmea_send_msg_to_uart( out_msg, index );
    }
  }
}

/********************************************//**
 * \brief   Transmits all configuration data block
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_transmit_swconfig( void )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  if ( sw_config_get_software_switch_status( CONFIG_TEXT_AS_HEADER_SWITCH ) )
  {
    nmea_swconfig_senddatablock( CURRENT_CONFIG_DATA, out_msg );
  }
}

/********************************************//**
 * \brief   Schedules the navigation data to be output on this pass.
 *
 * \param   nmea_msg_list Message list
 * \param   data_p        Fix data structure
 * \param   delay         the delay (in secs) of the current UTC second from the last fix
 * \param   utc_time      the current utc second (time of week)
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_transmit_configured( const tUInt *nmea_msg_list, void * data_p, const tDouble delay, const gnss_time_t utc_time )
{
  nmea_outmsg_common_data_t  nmea_common_data;
  nmea_outmsg_common_data_t *nmea_common_data_p = &nmea_common_data;
  static gpOS_clock_t prev_FWConfig_print_cpu_time = 0;
  gpOS_clock_t current_cpu_time = gpOS_time_now();
  static boolean_t FWConfig_print_twice = FALSE;

  /* Print the SWCONFIG at statup or after 10 mins */
  if (  ( ( nmea_outmsg_startmsg == TRUE ) && ( svc_pwr_StartupMode() == SVC_PWR_STARTUP_POWER_ON ) )  ||
        ( gpOS_time_minus( current_cpu_time , prev_FWConfig_print_cpu_time ) >= (NAV_CPU_TICKS_PER_SECOND * 600)) && (FWConfig_print_twice == FALSE) )
  {
    nmea_outmsg_transmit_full_header();
    nmea_outmsg_transmit_swconfig();

    /* If the FWCONFIG print has been done once */
    if ( nmea_outmsg_startmsg == FALSE )
    {
      FWConfig_print_twice = TRUE;
    }

    nmea_outmsg_startmsg = FALSE;
    prev_FWConfig_print_cpu_time = current_cpu_time;
  }

  // First get data common to several NMEA output messages
  nmea_outmsg_get_common_data( data_p, nmea_common_data_p, delay, utc_time );

  // Then call procedures for each NMEA message

  if ( nmea_msg_list_check( nmea_msg_list, DTM_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_DTM( data_p, delay, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, RMC_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_RMC( nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GGA_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GGA( nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GNS_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GNS( data_p, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, VTG_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_VTG( data_p, delay, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GST_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GST( data_p, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GBS_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GBS( data_p, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GSA_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GSA( data_p, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GSV_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GSV( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, GLL_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GLL( nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, ZDA_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_ZDA( nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, RF_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_RF( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, RES_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_PRES( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, RES_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_VRES( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, TG_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_TG( data_p, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, TS_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_TS( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, PA_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_PA( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, SAT_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_SAT( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, TIM_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_TIM( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, RFTEST_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_TESTRF( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, KFCOV_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_KFCOV( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, PPS_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_PPSDATA();
  }

  if ( nmea_msg_list_check( nmea_msg_list, POSHOLD_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_POSHOLD();
  }

  if ( nmea_msg_list_check( nmea_msg_list, TRAIM_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_TRAIM();
  }

#if defined( NMEA_NOTCH_SUPPORT )
  if ( nmea_msg_list_check( nmea_msg_list, NOTCH_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_NOTCH();
  }
#endif

  if ( nmea_msg_list_check( nmea_msg_list, TM_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_TM( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, NOISE_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_NOISE( data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, LOWPOWER_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_LOWPOWERSTATUS();
  }

  if ( nmea_msg_list_check( nmea_msg_list, PV_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_PV( data_p, nmea_common_data_p );
  }

  if ( nmea_msg_list_check( nmea_msg_list, PVRAW_NMEA_MSG ) == TRUE )
  {
    nmea_outmsg_send_PVRAW( data_p, delay, utc_time );
  }

  if ( nmea_msg_list_check( nmea_msg_list, PVQ_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_PVQ( data_p, delay, utc_time );
  }

  if ( nmea_msg_list_check( nmea_msg_list, UTC_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_UTC( nmea_common_data_p, utc_time );
  }

  if ( nmea_msg_list_check( nmea_msg_list, XTAL_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_XTALSTATUS();
  }

#if defined( NMEA_ADC_SUPPORT )
  if ( nmea_msg_list_check( nmea_msg_list, ADC_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_ADCDATA();
  }
#endif

#if defined( NMEA_ANTENNA_SENSING_SUPPORT )
  if ( nmea_msg_list_check( nmea_msg_list, ANT_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_ANTENNASENSING_STATUS();
  }
#endif

  {
    nmea_support_ext_params_t params;
    params.msg_list[0]   = nmea_msg_list[0];
    params.msg_list[1]   = nmea_msg_list[1];
    params.data_p     = data_p;
    params.delay      = delay;
    params.utc_time   = utc_time;
    gnssapp_plugins_nmea_outmsg_transmit( &params );
  }
  {
    tInt week;
    gpOS_clock_t cpu_time;
    tDouble tow;
    gnss_fix_get_time_local( &week, &tow, &cpu_time, data_p );

    if ( ( nmea_cmdif_TTFF_msg_enabled == TRUE ) && ( nmea_support_get_new_msg_event_flag( cpu_time ) == TRUE ) )
    {
      tInt fix_type;
      fix_type = nmea_fix_get_pos_status( data_p );

      if ( ( nmea_support_get_first_timing_msg_flag() == TRUE ) && ( fix_type != NO_FIX ) )
      {
        nmea_outmsg_send_TTFF();
        nmea_support_set_first_timing_msg_flag( FALSE );
      }
    }
  }


  if ( nmea_msg_list_check( nmea_msg_list, EPHEM_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_EPHEM();
  }

  if ( nmea_msg_list_check( nmea_msg_list, ALM_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_ALM();
  }

  if ( nmea_msg_list_check( nmea_msg_list, IONO_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_IONO();
  }

  if ( nmea_msg_list_check( nmea_msg_list, GGTO_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_GGTO();
  }

  if ( nmea_msg_list_check( nmea_msg_list, BIAS_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_BIASDATA(data_p);
  }

  if ( nmea_msg_list_check( nmea_msg_list, FEDATA_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_FEDATA();
  }
  else
  {
    /*not needed*/
  }

  /* this is kept as last message of the nmea msg set */
  if ( nmea_msg_list_check( nmea_msg_list, CPU_USAGE_NMEA_MSG ) == TRUE)
  {
    nmea_outmsg_send_CPUUSAGE();
  }
}

/*}}}  */
/*{{{  nmea_send_msg_to_uart()*/
/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_send_msg_to_uart

Description      : Transmits a message to the uart

Parameters       : tChar * msg    -- The message to be sent to the uart
                                   tInt  length   -- The number of characters in the message.

Return           : None.

Globals Accessed : nav_msg_uart_handle
******************************************************************************/

/*}}}  */
void nmea_send_msg_to_uart( const tChar *msg, const tInt length ) // to be used for command responses
{
  tInt len;
  tChar *ptr;

  if ( NMEA_initialized == TRUE )
  {
    ptr = (tChar *)msg;
    len = length;

    if(nmea_msg_timestamp_enabled != 0)
    {
      if(length < (NMEA_OUT_MSG_BUFFER_SIZE - 11))
      {
        len = length - 2;
        len += _clibs_sprintf(&ptr[len], ",%010u\r\n",gpOS_time_now());
      }
    }

    if ( nmea_on_debug_setting & SWCFG_DUAL_NMEA_PORT_ENABLE )
    {
      gpOS_task_t *current_task_id;
      current_task_id = gpOS_task_get_id();
      gpOS_semaphore_wait( nmea_outmsg_access );

      if ( current_task_id == nmea_cmdif_task )
      {
        nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
      }
      else if ( current_task_id == nmea_cmdif2_task )
      {
        gnss_debug_send_msg_to_uart( ( tChar * )msg, len );
      }
      else
      {
        nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
      }

      gpOS_semaphore_signal( nmea_outmsg_access );
    }
    else
    {
      gpOS_semaphore_wait( nmea_outmsg_access );
      nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
      gpOS_semaphore_signal( nmea_outmsg_access );
    }
  }
}

void nmea_send_outmsg( const tChar *msg, const tInt length, tUInt msg_type ) // to be used for output messages
{
  tInt len;
  tChar *ptr;

  ptr = (tChar *)msg;
  len = length;

  if(nmea_msg_timestamp_enabled != 0)
  {
    if(length < (NMEA_OUT_MSG_BUFFER_SIZE - 11))
    {
      len = length - 5;
      len += _clibs_sprintf(&ptr[len], ",%010u",gpOS_time_now());

      len += _clibs_sprintf(&ptr[len], "*%02X", nmea_support_checksum(ptr));
      len += _clibs_sprintf(&ptr[len], "\r\n");
    }
  }

  if ( nmea_on_debug_setting & SWCFG_DUAL_NMEA_PORT_ENABLE )
  {
    gpOS_task_t *current_task_id;
    current_task_id = gpOS_task_get_id();
    gpOS_semaphore_wait( nmea_outmsg_access );

    if ( current_task_id == nmea_outmsg2_task )
    {
      if ( ( nmea_on_debug_setting & SWCFG_NMEA_OUTPUT_ENABLE ) && ( nmea_msg_list_check( NMEA_msg_list_2, msg_type ) ) )
      {
        nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
      }

      if ( ( nmea_on_debug_setting & SWCFG_NMEA_OUTPUT_ON_DEBUG_ENABLE ) && ( nmea_msg_list_check( NMEA_on_debug_msg_list_2, msg_type ) ) )
      {
        gnss_debug_send_msg_to_uart( ( tChar * )msg, len );
      }
    }
    else if ( current_task_id == nmea_outmsg_task )
    {
      if ( ( nmea_on_debug_setting & SWCFG_NMEA_OUTPUT_ENABLE ) && ( ( nmea_msg_list_check( NMEA_msg_list, msg_type ) ) || ( nmea_msg_list_check( NMEA_msg_list_1, msg_type ) ) ) )
      {
        nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
      }

      if ( ( nmea_on_debug_setting & SWCFG_NMEA_OUTPUT_ON_DEBUG_ENABLE ) && ( ( nmea_msg_list_check( NMEA_on_debug_msg_list, msg_type ) ) || ( nmea_msg_list_check( NMEA_on_debug_msg_list_1, msg_type ) ) ) )
      {
        gnss_debug_send_msg_to_uart( ( tChar * )msg, len );
      }
    }
    else if ( current_task_id == nmea_cmdif_task )
    {
      nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
    }
    else if ( current_task_id == nmea_cmdif2_task )
    {
      gnss_debug_send_msg_to_uart( ( tChar * )msg, len );
    }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

    gpOS_semaphore_signal( nmea_outmsg_access );
  }
  else
  {
    gpOS_semaphore_wait( nmea_outmsg_access );
    nmea_ioport_write( ( tChar * )msg, len, gpOS_TIMEOUT_INFINITY);
    gpOS_semaphore_signal( nmea_outmsg_access );
  }

}

/********************************************//**
 * \brief   Computes the closest future UTC second boundary.
 *
 * \param   week_n      week number of gps time for last fix
 * \param   tow const   time of week of gps time for last fix
 * \param   fix_clock   cpu clock for last fix
 * \param   utc_tow     next utc second time (return parameter)
 * \param   utc_clock   related clock (return parameter)
 * \return  None
 *
 ***********************************************/
static void nmea_get_next_utc_clock( const tInt week_n, const tDouble tow, const gpOS_clock_t fix_clock, tDouble *utc_tow, gpOS_clock_t *utc_clock )
{
  gpOS_clock_t curr_clock;
  tDouble fix_utc_time;
  tDouble time_diff;
  tDouble next_utc_second;
  tDouble clock_rate;
  tDouble delta_time_to_utc;
  curr_clock = gpOS_time_now();
  clock_rate = gnss_cpu_clock_rate_hi();

  if ( gnss_time_get_master() == GNSS_SAT_TYPE_GLONASS )
  {
    delta_time_to_utc = ( tDouble ) 3.0 * SECS_PER_HOUR;
  }
  else
  {
    delta_time_to_utc =  gnss_get_utc_delta_time() ;
  }

  fix_utc_time = nmea_support_adjust_gps_time( tow - delta_time_to_utc );
  time_diff       = ( tDouble )( gpOS_time_minus( curr_clock, fix_clock ) ) / clock_rate;
  next_utc_second = ( tDouble )( tInt )nmea_support_adjust_gps_time( fix_utc_time + time_diff + 1.0 );
  *utc_clock = gpOS_time_plus( curr_clock, ( gpOS_clock_t )( tInt )( nmea_support_adjust_gps_time( next_utc_second - fix_utc_time - time_diff ) * clock_rate ) );
  *utc_tow = nmea_support_adjust_gps_time( next_utc_second );
}

/********************************************//**
 * \brief   Execute GPS engine suspension
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
extern void nmea_cmdif_exec_gpssuspend( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
  {
    gnssapp_suspend();
    platform_gnss_suspend();
  }
  index = _clibs_sprintf( out_msg, "$PSTMGPSSUSPENDED" );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Execute GPS engine suspension
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
extern void nmea_cmdif_exec_gpsrestart( tChar *cmd_par )
{
  if( GNSS_ENGINE_STATUS_SUSPENDED == gnss_get_engine_status())
  {
    platform_gnss_restart();
    gnssapp_restart();
  }
}


/********************************************//**
 * \brief    Get data common to several NMEA output messages
 *
 * \param   data_p              Fix data structure
 * \param   nmea_common_data_p  Common NMEA output data structure
 * \param   delay               Time delay since last fix
 * \param   utc_time            Current utc time
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_get_common_data( void *data_p, nmea_outmsg_common_data_t *nmea_common_data_p, const tDouble delay, const gnss_time_t utc_time )
{
  gnss_get_utc_time( utc_time.tow, &nmea_common_data_p->hours, &nmea_common_data_p->mins, &nmea_common_data_p->secs, &nmea_common_data_p->msecs );

  nmea_common_data_p->fix_type = nmea_fix_get_pos_status( data_p );

  nmea_common_data_p->diff_status = gnss_fix_get_diff_status_local( data_p );

  gnss_get_date( utc_time.week_n, utc_time.tow, &nmea_common_data_p->year, &nmea_common_data_p->month, &nmea_common_data_p->day );

  nmea_fix_get_position_velocity( &nmea_common_data_p->extrap_pos, &nmea_common_data_p->course, &nmea_common_data_p->speed, delay, data_p );

  if(nmea_common_data_p->course > 359.95)
  {
    nmea_common_data_p->course = 0.0;
  }

  nmea_common_data_p->geoid_msl = gnss_fix_get_geoid_msl_local( data_p );

  {
    ECEF_pos_t temp_ecef;
    datum_convert_position_wgs84_to_local_datum( &temp_ecef, &nmea_common_data_p->extrap_pos, &nmea_common_data_p->geoid_msl );
  }

  gnss_fix_get_dops_local( &nmea_common_data_p->pdop, &nmea_common_data_p->hdop, &nmea_common_data_p->vdop, &nmea_common_data_p->gdop, data_p );

  nmea_common_data_p->num_sats_used = gnss_fix_get_num_sats_used_local( data_p );

  nmea_common_data_p->num_sats_excluded = gnss_fix_get_num_sats_excluded_local( data_p );

  nmea_outmsg_set_contellation(gnss_get_constellation_mask());
}

/********************************************//**
 * \brief   Transmits a RMC type message
 *
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_RMC( const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  /*{{{  decs*/
  tInt index     = 0;
  tInt    lat_deg;
  tInt    lat_min;
  tInt    lat_min_frac;
  tInt    lon_deg;
  tInt    lon_min;
  tInt    lon_min_frac;
  tChar   lat_sense_ch;
  tChar   lon_sense_ch;
  tChar fix_valid_ch;
  tChar format[64];
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tChar mode_indicator_ch = 'N'; /*data not valid*/
  /*}}}  */

  if ( ( nmea_common_data_p->fix_type == ( tInt )FIX_2D ) || ( nmea_common_data_p->fix_type == ( tInt )FIX_3D ) )
  {
    fix_valid_ch = 'A';

    if ( nmea_common_data_p->diff_status == DIFF_STATUS_ON )
    {
      mode_indicator_ch = 'D'; /* available with diff corrections */
    }
    else
    {
      mode_indicator_ch = 'A'; /* available*/
    }
  }
  else
  {
    fix_valid_ch = 'V';
    mode_indicator_ch = 'N'; /*data not valid*/

    if ( nmea_common_data_p->fix_type > ( tInt )NO_FIX )
    {
      mode_indicator_ch = 'E'; /* esimated(dead reckoning) */
    }
  }

  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.latitude, 'N', 'S', nmea_outmsg_RMC_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.longitude, 'E', 'W', nmea_outmsg_RMC_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
  index += _clibs_sprintf( out_msg, "$G%cRMC", nmea_outmsg_talker_id );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d,%c", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs, fix_valid_ch );
  _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_RMC_posdigit, nmea_outmsg_RMC_posdigit );
  index += _clibs_sprintf( &out_msg[index], format,
                           lat_deg, lat_min, lat_min_frac, lat_sense_ch,
                           lon_deg, lon_min, lon_min_frac, lon_sense_ch
                         );

  _clibs_sprintf( format, ",%%03.%df,%%03.%df", nmea_outmsg_speed_digits, nmea_outmsg_course_digits);
  index += _clibs_sprintf( &out_msg[index], format, nmea_common_data_p->speed * MS_TO_KNOTS, nmea_common_data_p->course );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d", nmea_common_data_p->day, nmea_common_data_p->month, nmea_common_data_p->year % 100 );

  if ( sw_config_get_software_switch_status( NMEA_301_FORMAT_SWITCH ) )
  {
    index += _clibs_sprintf( &out_msg[index], ",,,%c", mode_indicator_ch );
  }
  else
  {
    index += _clibs_sprintf( &out_msg[index], ",0.0,W" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, RMC_NMEA_MSG );
}

static void nmea_outmsg_send_UTC( const nmea_outmsg_common_data_t *nmea_common_data_p, const gnss_time_t utc_time )
{
  tInt   index = 0;
  tChar  out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  index = _clibs_sprintf( out_msg, "$PSTMUTC" );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%04d", nmea_common_data_p->day, nmea_common_data_p->month, nmea_common_data_p->year );
  index += _clibs_sprintf( &out_msg[index], ",%u", (utc_time.week_n * SECS_PER_WEEK) + ( tInt )( utc_time.tow + 0.5 ) );
  index += _clibs_sprintf( &out_msg[index], ",%02d,%0d", ( tInt )gnss_get_utc_delta_time(), gnss_get_utc_delta_time_validity() );
  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, UTC_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a DTM type message where position has configurable decimal places
 *
 * \param   nmea_outmsg_common_data_t    Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_DTM( void *data_p, const tDouble delay, const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  /*{{{  decs*/
  tInt     index = 0;
  tInt     lat_deg;
  tInt     lat_min;
  tInt     lat_min_frac;
  tChar    lat_sense;
  tInt     lon_deg;
  tInt     lon_min;
  tInt     lon_min_frac;
  tChar    lon_sense;
  tDouble  speed;
  tDouble  course;
  tDouble  delta_h;
  position_t wgs84_pos;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tChar format[64];
  datum_code_t datum_code;

  nmea_fix_get_position_velocity( &wgs84_pos, &course, &speed, delay, data_p );
  nmea_support_degrees_to_int( (nmea_common_data_p->extrap_pos.latitude - wgs84_pos.latitude), 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense );
  nmea_support_degrees_to_int( (nmea_common_data_p->extrap_pos.longitude - wgs84_pos.longitude), 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense );

  if(lat_deg > 0)
  {
    lat_min += lat_deg * 60;
  }

  if(lon_deg > 0)
  {
    lon_min += lon_deg * 60;
  }

  delta_h = nmea_common_data_p->extrap_pos.height - wgs84_pos.height;

  datum_code = datum_get_current_code();

  index += _clibs_sprintf( out_msg, "$G%cDTM", nmea_outmsg_talker_id );

  switch(datum_code)
  {
    case DATUM_DEFAULT:
    case DATUM_CODE_WGS84:
      index += _clibs_sprintf( &out_msg[index], ",W84,");
    break;

    case DATUM_CODE_PZ90:
    case DATUM_CODE_PZ90_2:
    case DATUM_CODE_PZ90_11:
      index += _clibs_sprintf( &out_msg[index], ",P90,%03d",datum_code);
    break;

    case DATUM_CODE_USER_D:
      index += _clibs_sprintf( &out_msg[index], ",999,");
    break;

    default:
      index += _clibs_sprintf( &out_msg[index], ",IHO,%03d",datum_code);
    break;
  }

  _clibs_sprintf( format, ",%%03d.%%0%dd,%%c,%%03d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
  index += _clibs_sprintf( &out_msg[index], format,lat_min, lat_min_frac, lat_sense,lon_min, lon_min_frac, lon_sense);
  index += _clibs_sprintf( &out_msg[index], ",%lf,W84",delta_h);
  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );


  nmea_send_outmsg( out_msg, index, DTM_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a GGA type message where position has configurable decimal places
 *
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GGA( const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  /*{{{  decs*/
  tInt     index     = 0;
  tInt     lat_deg;
  tInt     lat_min;
  tInt     lat_min_frac;
  tInt     lon_deg;
  tInt     lon_min;
  tInt     lon_min_frac;
  tChar    lat_sense_ch;
  tChar    lon_sense_ch;
  tChar    format[64];
  tInt     fix_type_gga = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt     num_sats_used;

  fix_type_gga = nmea_common_data_p->fix_type;

  if ( ( nmea_common_data_p->fix_type == ( tInt )FIX_2D ) || ( nmea_common_data_p->fix_type == ( tInt )FIX_3D ) )
  {
    if ( nmea_common_data_p->diff_status == DIFF_STATUS_ON )
    {
      fix_type_gga = 2; /* available with diff corrections */
    }
    else
    {
      fix_type_gga = 1; /* available*/
    }
  }
  else if ( nmea_common_data_p->fix_type == ( tInt )NO_FIX )
  {
    fix_type_gga = 0;
  }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.latitude, 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.longitude, 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
  index += _clibs_sprintf( out_msg, "$G%cGGA", nmea_outmsg_talker_id );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
  _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
  //index += _clibs_sprintf(&out_msg[index], ",%02d%02d.%05d,%c,%03d%02d.%05d,%c",
  index += _clibs_sprintf( &out_msg[index], format,
                           lat_deg, lat_min, lat_min_frac, lat_sense_ch,
                           lon_deg, lon_min, lon_min_frac, lon_sense_ch
                         );
  if  (nmea_cmdif_SAT_excl_present == TRUE)
  {
    num_sats_used = nmea_common_data_p->num_sats_used;
  }
  else
  {
    /* remove the number of excluded sats */
     num_sats_used = nmea_common_data_p->num_sats_used - nmea_common_data_p->num_sats_excluded;
  }

  index += _clibs_sprintf( &out_msg[index], ",%01d,%02d,%01.1f,%06.2f,M,%01.1f,M,,",
                           fix_type_gga, num_sats_used, nmea_common_data_p->hdop, nmea_common_data_p->extrap_pos.height, nmea_common_data_p->geoid_msl
                         );
  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );

  if ( nmea_support_get_first_gga_msg_flag() == TRUE )
  {
    nmea_outmsg_first_gga_timestamp = gpOS_time_now();
    nmea_support_set_first_gga_msg_flag( FALSE );
  }

  nmea_outmsg_gga_timestamp = gpOS_time_now();

  nmea_send_outmsg( out_msg, index, GGA_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a ERR type message containing
 *          information from the positioning algorithm.
 *
 * \param   data_p  Fix data structure
 * \return  void
 *
 ***********************************************/
static void nmea_outmsg_send_PRES( void *data_p )
{
  tInt index = 0;
  tDouble rms_residual;
  chanid_t chan_id;
  tInt sat_cnt = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  //raw_measure_list_t *raw_data;
  rms_residual = gnss_fix_get_position_rms_residual_local( data_p );
  //raw_data = gnss_fix_get_raw_measurements();
  index += _clibs_sprintf( out_msg, "$PSTMPRES" );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf", rms_residual );

  for ( chan_id = 0; chan_id < TRK_CHANNELS_SUPPORTED; chan_id++ )
  {
    tDouble residual;
    boolean_t residual_valid;
    gnss_fix_get_position_residual_local( chan_id, &residual, &residual_valid, data_p );

    if ( residual_valid )
    {
      index += _clibs_sprintf( &out_msg[index], ",%.1lf", residual );
      sat_cnt++;
    }
  }

  for ( chan_id = 0; chan_id < (TRK_CHANNELS_SUPPORTED - sat_cnt); chan_id++ )
  {
    index += _clibs_sprintf( &out_msg[index], "," );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, RES_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a ERR type message containing
 *          information from the positioning algorithm.
 *
 * \param   data_p  Fix data structure
 * \return void
 *
 ***********************************************/
static void nmea_outmsg_send_VRES( void *data_p )
{
  tInt index     = 0;
  tDouble rms_residual;
  chanid_t chan_id;
  tInt sat_cnt = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  //raw_measure_list_t *raw_data;
  rms_residual = gnss_fix_get_velocity_rms_residual_local( data_p );
  //raw_data = gnss_fix_get_raw_measurements();
  index += _clibs_sprintf( out_msg, "$PSTMVRES" );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf", rms_residual );

  for ( chan_id = 0; chan_id < TRK_CHANNELS_SUPPORTED; chan_id++ )
  {
    tDouble residual;
    boolean_t residual_valid;
    gnss_fix_get_velocity_residual_local( chan_id, &residual, &residual_valid, data_p );

    if ( residual_valid )
    {
      index += _clibs_sprintf( &out_msg[index], ",%.1lf", residual );
      sat_cnt++;
    }
  }

  for ( chan_id = 0; chan_id < (TRK_CHANNELS_SUPPORTED - sat_cnt); chan_id++ )
  {
    index += _clibs_sprintf( &out_msg[index], "," );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, RES_NMEA_MSG );
}

#if defined( NMEA_NOTCH_SUPPORT )
/********************************************//**
 * \brief   Transmits information about the NOTCH status
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_NOTCH( void )
{
  tInt index     = 0;
  tInt kfreq_now, lock_en, pwr, ovfs;
  tShort mode;
  tInt kfreq_now_Hz_gps, kfreq_now_Hz_gln;
  tInt kfreq_temp_gps, kfreq_temp_gln;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  index += _clibs_sprintf( out_msg, "$PSTMNOTCHSTATUS" );
  gnss_notch_filter_get_status( GNSS_SAT_TYPE_GPS, &kfreq_now, &lock_en, &pwr, &ovfs, &mode );

  if ( mode == 0 )
  {
    index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d", 0, 0, 0, 0, mode );
  }
  else
  {
    tInt max_pos = 2048 ;
    tDouble output_limit = 4096;
    
    kfreq_temp_gps = (kfreq_now > max_pos) ? - ( output_limit - kfreq_now ) : kfreq_now;  /* get sign from uint*/

    kfreq_now_Hz_gps = ( tInt )( ( acos( ( -1 * kfreq_temp_gps * ( tDouble )( 1 / (tDouble)max_pos) ) ) / ( tDouble )( ( 2 * PI ) ) ) * ( 16 * 1023000 ) );

    index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d", kfreq_now_Hz_gps, lock_en, pwr, ovfs, mode );
  }

  gnss_notch_filter_get_status( GNSS_SAT_TYPE_GLONASS, &kfreq_now, &lock_en, &pwr, &ovfs, &mode );

  if ( mode == 0 )
  {
    index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d", 0, 0, 0, 0, mode );
  }
  else
  {
    tInt max_pos = 2048 ;
    tInt output_limit = 4096;

    kfreq_temp_gln = (kfreq_now > max_pos) ? - ( output_limit - kfreq_now ) : kfreq_now;  /* get sign from uint*/

    kfreq_now_Hz_gln = ( tInt )( ( acos( ( -1 * kfreq_temp_gln * ( tDouble )( 1 / (tDouble)max_pos ) ) ) / ( tDouble )( ( 2 * PI ) ) ) * ( 32 * 1023000 ) );

    index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d", kfreq_now_Hz_gln, lock_en, pwr, ovfs, mode );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, NOTCH_NMEA_MSG );
}
/*}}}  */
#endif

/********************************************//**
 * \brief   Transmits a PA type message containing information
 *          from the positioning algorithm.
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_PA( void * data_p )
{
  /*{{{  decs*/
  tInt           index     = 0;
  tInt           stopped_duration;
  pos_algo_t    pos_algo;
  tChar          algo_str[2];
  tChar          out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  /*}}}  */
  gnss_fix_get_stopped_duration( &stopped_duration );
  pos_algo = gnss_fix_get_pos_algo_local( data_p );

  if ( pos_algo == NONE )
  {
    _clibs_sprintf( algo_str, "  " );
  }
  else if ( pos_algo == LMS_ALGO )
  {
    _clibs_sprintf( algo_str, "LS" );
  }
  else if ( pos_algo == KF_ALGO )
  {
    _clibs_sprintf( algo_str, "KF" );
  }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

  index += _clibs_sprintf( out_msg, "$PSTMPA" );
  index += _clibs_sprintf( &out_msg[index], ",%2s,%02d", algo_str, stopped_duration );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, PA_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a VTG type message
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_VTG( void *data_p, const tDouble delay, const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  tInt index     = 0;
  tDouble course;
  tDouble speed;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tChar format[64];
  position_t extrap_pos;
  tChar mode_indicator_ch = 'N'; /*data not valid*/

  if ( ( nmea_common_data_p->fix_type == ( tInt )FIX_2D ) || ( nmea_common_data_p->fix_type == ( tInt )FIX_3D ) )
  {
    if ( nmea_common_data_p->diff_status == DIFF_STATUS_ON )
    {
      mode_indicator_ch = 'D'; /* available with diff corrections */
    }
    else
    {
      mode_indicator_ch = 'A'; /* available*/
    }
  }
  else
  {
    mode_indicator_ch = 'N'; /*data not valid*/

    if ( nmea_common_data_p->fix_type > ( tInt )NO_FIX )
    {
      mode_indicator_ch = 'E'; /* esimated(dead reckoning) */
    }
  }

  if (delay != 0.0)
  {
    nmea_fix_get_position_velocity( &extrap_pos, &course, &speed, 0.0, data_p );

    if(course > 359.95)
    {
       course = 0.0;
    }
  }
  else
  {
    course = nmea_common_data_p->course;
    speed  = nmea_common_data_p->speed;
  }

  index += _clibs_sprintf( out_msg, "$G%cVTG", nmea_outmsg_talker_id );

  _clibs_sprintf( format, ",%%03.%df,T,,M,%%03.%df,N,%%03.%df,K", nmea_outmsg_course_digits, nmea_outmsg_speed_digits, nmea_outmsg_speed_digits);
   index += _clibs_sprintf( &out_msg[index], format,course,speed * MS_TO_KNOTS, speed * MS_TO_KPH);

  if ( sw_config_get_software_switch_status( NMEA_301_FORMAT_SWITCH ) )
  {
    index += _clibs_sprintf( &out_msg[index], ",%c", mode_indicator_ch );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, VTG_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a GST type message
 *
 * \param   data_p    Fix data structure
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GST( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  tInt    index = 0;
  tDouble ehpe, n_cov, e_cov, v_cov, major_axis, minor_axis, angle;
  math_ellipse_t ellipse;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  ehpe = gnss_fix_get_ehpe_local( data_p );
  gnss_fix_get_position_covariance_local( &n_cov, &e_cov, &v_cov, data_p );
  gnss_fix_get_error_ellipse_local( &ellipse, data_p );

  if ( ellipse.semiaxis_n >= ellipse.semiaxis_e )
  {
    major_axis = ellipse.semiaxis_n;
    minor_axis = ellipse.semiaxis_e;
    angle = 0 - ellipse.theta;
  }
  else
  {
    major_axis = ellipse.semiaxis_e;
    minor_axis = ellipse.semiaxis_n;
    angle = 90 - ellipse.theta;
  }

  /* wrap angle*/
  if ( angle > 180 )
  {
    angle -= 360;
  }
  else if ( angle < ( -180 ) )
  {
    angle += 360;
  }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

  index += _clibs_sprintf( out_msg, "$G%cGST", nmea_outmsg_talker_id );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf", ehpe );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf,%.1lf,%.1lf", major_axis, minor_axis, angle );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf,%.1lf,%.1lf", sqrt( n_cov ), sqrt( e_cov ), sqrt( v_cov ) );
  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, GST_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a GSA type message for specific satellite
 *
 * \param   data_p      Fix data structure
 * \param   nmea_common_data    Common NMEA output data structure
 * \param   sat_type    Satellite type
 * \param   talker_id
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GSA_for_constellation( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p, gnss_sat_type_t sat_type, tChar *talker_id )
{
  /*{{{  decs*/
  tInt index, sat_cnt = 0, msg_count = 0;
  tInt i, j, start_index = 0;
  raw_t *raw_data;
  raw_measure_list_t *raw_meas;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  fix_status_t gnss_fix_status;
  /*}}}  */
  raw_meas = gnss_fix_get_raw_measurements_local( data_p );
  raw_data = raw_meas->list;

  gnss_fix_status = gnss_fix_get_pos_status_local( data_p );

  while ( start_index < TRK_CHANNELS_SUPPORTED )
  {
    index = _clibs_sprintf( out_msg, "$%sGSA", talker_id );
    index += _clibs_sprintf( &out_msg[index], ",A,%1d", gnss_fix_status);
    sat_cnt = 0;

    for ( i = start_index; ( i < TRK_CHANNELS_SUPPORTED ) && ( sat_cnt < NMEA_GSV_MAX_SATS ); i++ )
    {
      satid_t current_sat_id;

      boolean_t current_sat_excluded = FALSE;
      gnss_exclusion_type_t exclusion_type;

      if  ( nmea_cmdif_SAT_excl_present == FALSE )
      {
        current_sat_excluded = (gnss_fix_get_excluded_sats_range_local(i, &exclusion_type,data_p)==TRUE) || (gnss_fix_get_excluded_sats_doppl_local(i, &exclusion_type,data_p)==TRUE);
      }


      if ( ( raw_meas->chans_used[i] ) && ( sat_type == gnss_sat_id_to_sat_type( raw_data[i].dsp.satid ) ) && (current_sat_excluded == FALSE) )
      {
        if ( nmea_support_translate_satid( raw_data[i].dsp.satid, &current_sat_id ) == GNSS_NO_ERROR )
        {
          index += _clibs_sprintf( &out_msg[index], ",%02d", current_sat_id );
          sat_cnt++;
        }
      }
    }

    start_index = i;

    if ( !( ( msg_count > 0 ) && ( sat_cnt == 0 ) ) )
    {
      for ( j = sat_cnt; j < NMEA_GSV_MAX_SATS; j++ )
      {
        index += _clibs_sprintf( &out_msg[index], "," );
      }

      index += _clibs_sprintf( &out_msg[index], ",%2.1f,%2.1f,%2.1f", nmea_common_data_p->pdop, nmea_common_data_p->hdop, nmea_common_data_p->vdop );
      index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
      index += _clibs_sprintf( &out_msg[index], "\r\n" );
      nmea_send_outmsg( out_msg, index, GSA_NMEA_MSG );
      msg_count++;
    }
  }
}

/********************************************//**
 * \brief   Transmits a GSA type message this contains
 *          information on the satellites used in the fix.
 *
 * \param   data_p      Fix data structure
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GSA( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p )
  {

  if ( ( nmea_outmsg_GSA_constellations_enabled > 1 ) || sw_config_get_software_switch_status( NMEA_GNGSA_ENABLE ) )
  {
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS ) )
    {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_GPS, "GN" );
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS ) )
      {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_GLONASS, "GN" );
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
    {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_QZSS_L1_CA, "GN" );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO))
    {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_GALILEO, "GN" );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS))
      {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_COMPASS, "GN" );
    }
  }
  else
  {
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS ) )
    {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_GPS, "GP" );
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS ) )
      {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_GLONASS, "GL" );
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
    {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_QZSS_L1_CA, "QZ" );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO))
      {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_GALILEO, "GA" );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS))
    {
      nmea_outmsg_send_GSA_for_constellation( data_p, nmea_common_data_p, GNSS_SAT_TYPE_COMPASS, "BD" );
    }

  }
}

/********************************************//**
 * \brief   Transmits a GBS type message this contains
 *          information on GNSS Satellite Fault Detection.
 *
 * \param   data_p      Fix data structure
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GBS( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  /*{{{  decs*/
  tInt index     = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  chanid_t chan_id;
  gnss_exclusion_type_t exclusion_type;
  tDouble mlfs_fde_range_residual =0.0, mlfs_raim_range_residual =0.0;
  chanid_t mlfs_fde_chan_id=0, mlfs_raim_chan_id=0;
  tDouble range_residual = 0.0;
  boolean_t residual_valid;
  raw_t *raw_data;
  raw_measure_list_t *raw_meas;
  satid_t current_sat_id;
  tDouble n_cov, e_cov, v_cov;

  raw_meas = gnss_fix_get_raw_measurements_local( data_p );
  raw_data = raw_meas->list;

  index += _clibs_sprintf( out_msg, "$G%cGBS", nmea_outmsg_talker_id );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );

  /* If there is no fix or the number of sats used is < 4*/
  if ( ( nmea_common_data_p->fix_type == ( tInt )NO_FIX )  ||   (raw_meas->list_size < 4)  )
  {
    index += _clibs_sprintf( &out_msg[index], ",,,,,,," );
  }
  else
  /* If there is a fix */
  {
    /* search for Sats excluded */
    for ( chan_id = 0; chan_id < TRK_CHANNELS_SUPPORTED ; chan_id++ )
    {
      /* If the sat is excluded because of FDE or RAIM */
      if ( gnss_fix_get_excluded_sats_range_local((tInt)chan_id, &exclusion_type, data_p) == TRUE )
      {
        /* get the range residual associated */
        gnss_fix_get_position_residual_used_local( (tInt)chan_id, &range_residual, &residual_valid, data_p );

        if ( residual_valid == TRUE )
        {
          switch (exclusion_type)
          {
            case GNSS_FDE_EXCLUSION:
            {
              if ( fabs(range_residual) > fabs(mlfs_fde_range_residual))
              {
                /* keep the chan id with the biggest residual */
                mlfs_fde_range_residual = range_residual;
                mlfs_fde_chan_id = chan_id;
              }
            }
            break;

            case GNSS_RAIM_EXCLUSION:
            {
              if ( fabs(range_residual) > fabs(mlfs_raim_range_residual))
              {
                /* keep the chan id with the biggest residual */
                mlfs_raim_range_residual = range_residual;
                mlfs_raim_chan_id = chan_id;
              }
            }
            break;

           case GNSS_NO_EXCLUSION:
           default:{}
            break;
          }
        }
      }
    }

    /* Print Expected Error in lat, lon, altitude */
    gnss_fix_get_position_covariance_local( &n_cov, &e_cov, &v_cov, data_p );
    index += _clibs_sprintf( &out_msg[index], ",%.1lf,%.1lf,%.1lf", sqrt( n_cov ), sqrt( e_cov ), sqrt( v_cov ) );

    /* if a Most likely failed satellite has been found with FDE */
    if (mlfs_fde_range_residual != 0.0)
    {
      if ( nmea_support_translate_satid( raw_data[mlfs_fde_chan_id].dsp.satid, &current_sat_id ) == NMEA_OK )
      {
        /* Print the PRN of MLFS */
        index += _clibs_sprintf( &out_msg[index], ",%02d", current_sat_id );
        /* "Probability of missed detection for most likely failed satellite" not supported*/
        index += _clibs_sprintf( &out_msg[index], "," );
        /* Print the residual */
        index += _clibs_sprintf( &out_msg[index], ",%.1lf", mlfs_fde_range_residual );
        /* "MLFS bias std dev" not supported*/
        index += _clibs_sprintf( &out_msg[index], "," );
      }
      else
      {
        index += _clibs_sprintf( &out_msg[index], ",,,," );
      }
    }
    else
      /* if a Most likely failed satellite has been found with RAIM */
      if (mlfs_raim_range_residual != 0.0)
      {

        if ( nmea_support_translate_satid( raw_data[mlfs_raim_chan_id].dsp.satid, &current_sat_id ) == NMEA_OK )
        {
          /* Print the PRN of MLFS */
          index += _clibs_sprintf( &out_msg[index], ",%02d", current_sat_id );
          /* "Probability of missed detection for most likely failed satellite" not supported*/
          index += _clibs_sprintf( &out_msg[index], "," );
          /* Print the residual */
          index += _clibs_sprintf( &out_msg[index], ",%.1lf", mlfs_raim_range_residual );
          /* "MLFS bias std dev" not supported*/
          index += _clibs_sprintf( &out_msg[index], "," );
        }
        else
        {
          index += _clibs_sprintf( &out_msg[index], ",,,," );
        }
      }
      /* No Failed satellite */
      else
      {
        index += _clibs_sprintf( &out_msg[index], ",,,," );
      }
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf(&out_msg[index], "\r\n");
  nmea_send_outmsg( out_msg, index, GBS_NMEA_MSG );

}

/********************************************//**
 * \brief   Send information about CPU usage
 *
 * \return void
 *
 ***********************************************/
static void nmea_outmsg_send_CPUUSAGE( void )
{
  tInt index;
  tInt clk_source, clk_speed;
  tUInt cpu_usage;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  cpu_usage = svc_mcu_getcpuusage();

  platform_get_clock_configuration_status( &clk_source, &clk_speed);

  if( LLD_PRCC_ARMCLKSRC_192f0 == (LLD_PRCC_ARMClkSrcTy)clk_source)
  {
    clk_speed = ((192 * 1023) / 1000) / (clk_speed + 1);
    index = _clibs_sprintf( out_msg, "$PSTMCPU,%3.2lf,%d,%d", ( tDouble )cpu_usage / 100.0, -1, clk_speed );
  }
  else if( LLD_PRCC_ARMCLKSRC_TCXO == (LLD_PRCC_ARMClkSrcTy)clk_source)
  {
    index = _clibs_sprintf( out_msg, "$PSTMCPU,%3.2lf,%d,%d", ( tDouble )cpu_usage / 100.0, -1, platform_get_tcxo_speed()/1000000);
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMCPU,%3.2lf,%d,%d", ( tDouble )cpu_usage / 100.0, -1, -1 );
  }
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, CPU_USAGE_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a GSV type message for specific constellation
 *
 * \param   data_p                  Fix data structure
 * \param   sat_constellation_mask  Constellation mask
 * \param   sats_visible            Visible sats
 * \param   talker_id               Talker Id
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GSV_for_constellation( void *data_p, gnss_sat_type_mask_t sat_constellation_mask, const visible_sats_data_t *sats_visible, tChar *talker_id )
{
  tInt           j;
  satid_t        sat_id;
  satid_t        translated_sat_id;
  gnss_sat_type_t sat_type;
  tInt           sats_used  = 0;
  tInt           chan_index;
  satid_t        tracker_sats[VISIBLE_MAX_NUM_OF_SATS + TRK_CHANNELS_SUPPORTED];
  tInt           tracker_sats_elevation[VISIBLE_MAX_NUM_OF_SATS + TRK_CHANNELS_SUPPORTED];
  tInt           tracker_sats_azimut[VISIBLE_MAX_NUM_OF_SATS + TRK_CHANNELS_SUPPORTED];
  tInt           tracker_sats_CN0[VISIBLE_MAX_NUM_OF_SATS + TRK_CHANNELS_SUPPORTED];
  boolean_t      visible_sat_found = FALSE;
  boolean_t      external_sat_found = FALSE;
  tInt           satellite_CN0 = 0;
  tChar          out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  raw_t          *raw_data;

  raw_data = gnss_fix_get_raw_measurements_local( data_p )->list;

  for ( j = 0; j < sats_visible->list_size; j++ )
  {
    if ( MCR_ISBITSET( sat_constellation_mask, gnss_sat_id_to_sat_type( sats_visible->list[j].satid ) ) )
    {
      tracker_sats[sats_used]           = sats_visible->list[j].satid;
      tracker_sats_CN0[sats_used]       = 0;
      tracker_sats_elevation[sats_used] = ( tInt )sats_visible->list[j].elevation;
      tracker_sats_azimut[sats_used]    = ( tInt )sats_visible->list[j].azimuth;
      sats_used++;
    }
  }

  /* fills sats CN0s and add tracked sats not in the list */
  for ( chan_index = 0; chan_index < (tInt)TRK_CHANNELS_SUPPORTED; chan_index++ )
  {
    satellite_CN0 = (tInt)raw_data[chan_index].dsp.signal_strength;

    if ( raw_data[chan_index].dsp.flags.state > 2)
    {
      sat_id = raw_data[chan_index].dsp.satid;
      sat_type = gnss_sat_id_to_sat_type( sat_id );

      visible_sat_found = FALSE;
      /* Checks if the tracked satellite is in the satellite visible list*/
      for ( j = 0; ( j < sats_used ) && ( visible_sat_found == FALSE ); )
      {
        if ( ( sat_id == tracker_sats[j] ) && MCR_ISBITSET( sat_constellation_mask, sat_type) )
        {
          visible_sat_found = TRUE;
        }
        else
        {
          j++;
        }
      }

      /* if so, update power */
      if ( visible_sat_found == TRUE )
      {
        tracker_sats_CN0[j] = satellite_CN0;
      }
      else if ( MCR_ISBITSET( sat_constellation_mask, GNSS_SAT_TYPE_SBAS ) )
      {
        nmea_support_sat_params_t   sbas_sat_params;
        gnssapp_plugins_cmd_param_t gpparam;
        sbas_sat_params.satid = sat_id;
        gpparam.id = 0;
        gpparam.data_ptr = &sbas_sat_params;

        if ( gnssapp_plugins_cmd( GNSSAPP_PLUGINS_ID_WAAS, GNSSAPP_PLUGINS_CMD_CUSTOM, &gpparam ) == gpOS_SUCCESS )
        {
          external_sat_found = TRUE;

          if ( sats_used < NMEA_GSV_MAX_SATS )
          {
            sats_used++;
          }

          tracker_sats[sats_used - 1]           = sat_id;
          tracker_sats_CN0[sats_used - 1]       = (tInt)sbas_sat_params.CN0;
          tracker_sats_elevation[sats_used - 1] = sbas_sat_params.elevation;
          tracker_sats_azimut[sats_used - 1]    = sbas_sat_params.azimuth;
        }
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

      if ( ( visible_sat_found == FALSE ) && ( external_sat_found == FALSE ) && ( satellite_CN0 > 0 ) )
      {
        if ( MCR_ISBITSET( sat_constellation_mask, sat_type) )
        {
          nmea_error_t error;

          error = nmea_support_translate_satid( sat_id, &translated_sat_id );

          if( error == GNSS_NO_ERROR )
          {
            tracker_sats[sats_used]           = sat_id;
            tracker_sats_CN0[sats_used]       = satellite_CN0;
            tracker_sats_elevation[sats_used] = 0;
            tracker_sats_azimut[sats_used]    = 0;
            sats_used++;
          }
        }
      }
    }
  }

  {
    tInt tot_num_msg;
    tInt msg_num;
    tInt sat_count = 0;
    tInt gsv_sats_used = sats_used;

    /* Calculate how many sats have been used and how many messages this will need*/
    tot_num_msg = ( ( gsv_sats_used - 1 ) / 4 ) + 1;

    for ( msg_num = 1; sat_count < gsv_sats_used; msg_num++ )
    {
      tInt i;
      tInt index = 0;
      tInt delivered_sats = 0;
      tChar satCN0_str[3];
      index = _clibs_sprintf( out_msg, "$%sGSV,%1d,%1d,%02d", talker_id, tot_num_msg, msg_num, gsv_sats_used );

      for ( i = 0; ( delivered_sats < 4 ) && ( ( i + sat_count ) < gsv_sats_used ); i++ )
      {
        if ( nmea_support_translate_satid( ( satid_t )tracker_sats[i + sat_count], &translated_sat_id ) == GNSS_NO_ERROR )
        {
          if ( tracker_sats_CN0[i + sat_count] > 0 )
          {
            _clibs_sprintf( satCN0_str, "%02d", tracker_sats_CN0[i + sat_count] );
          }
          else
          {
            satCN0_str[0] = '\0';
          }

          index += _clibs_sprintf( &out_msg[index], ",%02d,%02d,%03d,%s", translated_sat_id, tracker_sats_elevation[i + sat_count], tracker_sats_azimut[i + sat_count], satCN0_str );
          delivered_sats++;
        }
      }

      sat_count += i;

      for ( j = i; j < 4; j++ )
      {
        index += _clibs_sprintf( &out_msg[index], ",,,," );
      }

      index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
      index += _clibs_sprintf( &out_msg[index], "\r\n" );
      nmea_send_outmsg( out_msg, index, GSV_NMEA_MSG );
    }
  }
}

/********************************************//**
 * \brief   Transmits a GSV type message
 *
 * \param   data_p      Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GSV( void *data_p )
{
  gnss_sat_type_mask_t sat_constellation_mask = 0;

  visible_sats_data_t sats_visible;
  gnss_get_sats_visible( &sats_visible );

  if ( sw_config_get_software_switch_status( WAAS_SAT_ON_GSV ) != 0U )
  {
    sat_constellation_mask = ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_SBAS );
  }

  if ( sw_config_get_software_switch_status( NMEA_GNGSV_ENABLE ) != 0U )
  {
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS ) )
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_GPS );
    }
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS ) )
    {
      sat_constellation_mask |= ((tUInt)1 << (tUInt)GNSS_SAT_TYPE_GLONASS );
    }
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_QZSS_L1_CA );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO))
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_GALILEO );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS))
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_COMPASS );
    }

    nmea_outmsg_send_GSV_for_constellation( data_p, sat_constellation_mask, &sats_visible, "GN" );
  }
  else
  {
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS ) )
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_GPS );
      nmea_outmsg_send_GSV_for_constellation( data_p, sat_constellation_mask, &sats_visible, "GP" );
      sat_constellation_mask = 0;
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS ) )
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_GLONASS );
      nmea_outmsg_send_GSV_for_constellation( data_p, sat_constellation_mask, &sats_visible, "GL" );
      sat_constellation_mask = 0;
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_QZSS_L1_CA );
      nmea_outmsg_send_GSV_for_constellation( data_p, sat_constellation_mask, &sats_visible, "QZ" );
      sat_constellation_mask = 0;
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO))
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_GALILEO );
      nmea_outmsg_send_GSV_for_constellation( data_p, sat_constellation_mask, &sats_visible, "GA" );
      sat_constellation_mask = 0;
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS))
    {
      sat_constellation_mask |= ( (tUInt)1 << (tUInt)GNSS_SAT_TYPE_COMPASS );
      nmea_outmsg_send_GSV_for_constellation( data_p, sat_constellation_mask, &sats_visible, "BD" );
      sat_constellation_mask = 0;
    }
  }
}

/********************************************//**
 * \brief   Transmits a GLL type message
 *
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  void
 *
 ***********************************************/
static void nmea_outmsg_send_GLL( const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  /*{{{  decs*/
  tInt   index     = 0;
  tInt   lat_deg;
  tInt   lat_min;
  tInt   lat_min_frac;
  tInt   lon_deg;
  tInt   lon_min;
  tInt   lon_min_frac;
  tChar  lat_sense_ch;
  tChar  lon_sense_ch;
  tChar  fix_type_ch;
  tChar  format[64];
  tChar  out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tChar  mode_indicator_ch = 'N'; /*data not valid*/

  if ( ( nmea_common_data_p->fix_type == FIX_2D ) || ( nmea_common_data_p->fix_type == FIX_3D ) )
  {
    fix_type_ch = 'A'; /*data valid*/

    if ( nmea_common_data_p->diff_status == DIFF_STATUS_ON )
    {
      mode_indicator_ch = 'D'; /* Differential mode */
    }
    else
    {
      mode_indicator_ch = 'A'; /* Autonomous mode*/
    }
  }
  else
  {
    fix_type_ch = 'V'; /* data invalid */
    mode_indicator_ch = 'N'; /*data not valid*/

    if ( nmea_common_data_p->fix_type > ( tInt )NO_FIX )
    {
      mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
    }
  }

  /*}}}   */
  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.latitude, 'N', 'S', nmea_outmsg_RMC_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.longitude, 'E', 'W', nmea_outmsg_RMC_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
  index += _clibs_sprintf( out_msg, "$G%cGLL", nmea_outmsg_talker_id );
  _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_RMC_posdigit, nmea_outmsg_RMC_posdigit );
  //index += _clibs_sprintf(&out_msg[index], ",%02d%02d.%03d,%c,%03d%02d.%03d,%c",
  index += _clibs_sprintf( &out_msg[index], format,
                           lat_deg, lat_min, lat_min_frac, lat_sense_ch,
                           lon_deg, lon_min, lon_min_frac, lon_sense_ch
                         );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
  index += _clibs_sprintf( &out_msg[index], ",%c", fix_type_ch );

  if ( sw_config_get_software_switch_status( NMEA_301_FORMAT_SWITCH ) )
  {
    index += _clibs_sprintf( &out_msg[index], ",%c", mode_indicator_ch );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, GLL_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmit the ZDA message
 *
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_ZDA( const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  tInt index     = 0;
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  index += _clibs_sprintf( out_msg, "$G%cZDA", nmea_outmsg_talker_id );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%02d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
  index += _clibs_sprintf( &out_msg[index], ",%02d,%02d,%04d", nmea_common_data_p->day, nmea_common_data_p->month, nmea_common_data_p->year );
  index += _clibs_sprintf( &out_msg[index], ",00,00" );
  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, ZDA_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a message which contains average phase noise values.
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_RF( void *data_p )
{
  /*{{{  decs*/
  tInt               j = 0;
  tInt               message_index        = 0;
  tInt               sats_printed_already = 0;
  tInt               no_of_messages       = 0;
  raw_measure_list_t *raw;
  tInt               chan_index = 0;
  tInt               sats_used  = 0;
  tChar              out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  /*}}}  */

  raw = gnss_fix_get_raw_measurements_local( data_p );

  /* Calculate how many sats have been used and how many messages this will need*/
  for ( chan_index = 0; chan_index < TRK_CHANNELS_SUPPORTED; chan_index++ )
  {
    if (raw->list[chan_index].dsp.flags.available == TRUE)
    {
      sats_used++;
    }
  }

  no_of_messages = ( ( sats_used - 1 ) / 3 ) + 1;
  /* Print out messages in blocks of four */
  chan_index           = 0;
  sats_printed_already = 0;

  for ( message_index = 1; message_index <= no_of_messages; message_index++ )
  {
    tInt sats_printed_this_message = 0;
    tInt index = 0;
    index += _clibs_sprintf( &out_msg[index], "$PSTMRF,%1d,%1d,%02d", no_of_messages, message_index, sats_used );

    /* Print the next four messages */
    do
    {
      if (raw->list[chan_index].dsp.flags.available == TRUE)
      {
        index += _clibs_sprintf( &out_msg[index], ",%02d,%05d,%06d,%02d",
                                 raw->list[chan_index].dsp.satid,
                                 raw->list[chan_index].dsp.ave_phase_noise,
                                 (tInt)MCR_FP32_QM_TO_DOUBLE(raw->list[chan_index].dsp.frequency,GNSS_FREQUENCY_SHIFT),
                                 raw->list[chan_index].dsp.signal_strength
                               );

        sats_printed_this_message++;
      }

      chan_index++;
    } while ( ( sats_printed_this_message < 3 ) && ( ( sats_printed_this_message + sats_printed_already ) < sats_used ) );

    sats_printed_already += sats_printed_this_message;

    /* Add further commas where no more satellites are available */
    for ( j = sats_printed_this_message; j < 3; j++ )
    {
      index += _clibs_sprintf( &out_msg[index], ",,,," );
    }

    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_outmsg( out_msg, index, RF_NMEA_MSG );
  }
}

/********************************************//**
 * \brief   Transmits a message which contains information on
 *          the sat position and range and velocity measurement
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_SAT( void *data_p )
{
  /*{{{  decs*/
  tInt index     = 0;
  tInt msg_num   = 0;
  raw_measure_list_t *raw_meas;
  raw_t *raw_data;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  /*}}}  */
  raw_meas = gnss_fix_get_raw_measurements_local( data_p );
  raw_data = raw_meas->list;

  for ( msg_num = 0; msg_num < TRK_CHANNELS_SUPPORTED; msg_num++ )
  {
    if ( raw_meas->chans_used[msg_num] )
    {
      index = 0;
      index += _clibs_sprintf( out_msg, "$PSTMSAT" );
      index += _clibs_sprintf( &out_msg[index], ",%02d", raw_data[msg_num].dsp.satid );
      index += _clibs_sprintf( &out_msg[index], ",%08.0f", MCR_FPU32_QM_TO_DOUBLE(raw_data[msg_num].dsp.pseudorange,GNSS_PSEUDORANGE_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%04.0f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].dsp.frequency,GNSS_FREQUENCY_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%08.0f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_pos.x,GNSS_SAT_POS_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%08.0f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_pos.y,GNSS_SAT_POS_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%08.0f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_pos.z,GNSS_SAT_POS_SHIFT));
      index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
      nmea_send_outmsg( out_msg, index, SAT_NMEA_MSG );
    }
  }
}

/********************************************//**
 * \brief   Transmits a message which contains information on
 *          the time of fix and other global (non sat-specific)
 *          parameters for a fix
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TG( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  /*{{{  decs*/
  tInt index     = 0, num_sats_used;
  //tInt week;
  gpOS_clock_t cpu_time;
  //tDouble tow;
  //time_validity_t time_validity;
  tDouble nco;
  tU16 kf_config_status = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_sat_type_t time_best_sat_type;
  gnss_sat_type_t time_aux_sat_type;
  gnss_sat_type_t time_master_sat_type;
  gnss_time_t time_best;
  gnss_time_t time_aux;
  gnss_time_t time_master;
  time_validity_t time_best_validity;
  time_validity_t time_aux_validity;
  time_validity_t time_master_validity;
  time_best_sat_type   = gnss_fix_get_time_best_local( &time_best, &cpu_time, &time_best_validity, data_p );
  time_master_sat_type = gnss_fix_get_time_master_local( &time_master, &time_master_validity, data_p );
  time_aux_sat_type    = gnss_fix_get_time_aux_local( &time_aux, &time_aux_validity, data_p );
  /* if the Sats excluded are present in NMEA */
  if (nmea_cmdif_SAT_excl_present == TRUE)
  {
    num_sats_used = nmea_common_data_p->num_sats_used;
  }
  else
  {
    /* Remove the number of excluded sats */
    num_sats_used = nmea_common_data_p->num_sats_used - nmea_common_data_p->num_sats_excluded;
  }
  nco = gnss_get_centre_freq();
  kf_config_status = gnss_fix_get_kf_config_status();
  index = 0;
  index += _clibs_sprintf( out_msg, "$PSTMTG" );
  index += _clibs_sprintf( &out_msg[index], ",%d", time_best.week_n );
  index += _clibs_sprintf( &out_msg[index], ",%.4f", time_best.tow );
  index += _clibs_sprintf( &out_msg[index], ",%d", num_sats_used );
  index += _clibs_sprintf( &out_msg[index], ",%d", cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%d", time_best_validity );
  index += _clibs_sprintf( &out_msg[index], ",%.4lf", nco );

  index += _clibs_sprintf( &out_msg[index], ",%04x", kf_config_status );

  index += _clibs_sprintf( &out_msg[index], ",%d", gnss_get_constellation_mask() );
  index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d", time_best_sat_type, time_master_sat_type, time_aux_sat_type );
  index += _clibs_sprintf( &out_msg[index], ",%d", time_master.week_n );
  index += _clibs_sprintf( &out_msg[index], ",%.4f", time_master.tow );
  index += _clibs_sprintf( &out_msg[index], ",%d", time_master_validity );
  index += _clibs_sprintf( &out_msg[index], ",%d", time_aux.week_n );
  index += _clibs_sprintf( &out_msg[index], ",%.4f", time_aux.tow );
  index += _clibs_sprintf( &out_msg[index], ",%d", time_aux_validity );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TG_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a message which contains information
 *          on sat related parameters for a fix
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TS( void *data_p )
{
  /*{{{  decs*/
  tInt index     = 0;
  tInt msg_num   = 0;
  raw_t *raw_data;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt nmea_ext_options = 0;
  tInt nmea_ext_TS_format;
  tInt dsp_available_flag;
  tInt plf_flags = 0;
  satid_t sat_id;
  /*}}}  */
  sw_config_get_param( CURRENT_CONFIG_DATA, NMEA_EXT_OPTIONS_ID, &nmea_ext_options );
  nmea_ext_TS_format = ( nmea_ext_options & 1 );
  raw_data = gnss_fix_get_raw_measurements_local( data_p )->list;

  for ( msg_num = 0; msg_num < TRK_CHANNELS_SUPPORTED; msg_num++ )
  {
    if (raw_data[msg_num].dsp.flags.available || raw_data[msg_num].sat.available || raw_data[msg_num].diff.available)
    /* data output even if not used */
    {
      dsp_available_flag = (tInt)raw_data[msg_num].dsp.flags.available;

      if ( nmea_ext_TS_format )
      {
        dsp_available_flag |= 0x8;
      }

      sat_id = (satid_t)raw_data[msg_num].dsp.satid;  // no translation in case of error... keep GLONASS freq_id


      index = 0;
      index += _clibs_sprintf( out_msg, "$PSTMTS" );
      index += _clibs_sprintf( &out_msg[index], ",%01d", dsp_available_flag );
      index += _clibs_sprintf( &out_msg[index], ",%02d", sat_id );
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FPU32_QM_TO_DOUBLE(raw_data[msg_num].dsp.pseudorange,GNSS_PSEUDORANGE_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].dsp.frequency,GNSS_FREQUENCY_SHIFT));


      if ( nmea_ext_TS_format )
      {
        index += _clibs_sprintf( &out_msg[index], ",0.0" );
      }

      plf_flags = raw_data[msg_num].dsp.flags.preamble_locked;
      index += _clibs_sprintf( &out_msg[index], ",%02d", plf_flags);
      index += _clibs_sprintf( &out_msg[index], ",%02d", raw_data[msg_num].dsp.signal_strength );
      index += _clibs_sprintf( &out_msg[index], ",%d", raw_data[msg_num].dsp.tracked_time );
      index += _clibs_sprintf( &out_msg[index], ",%01d", raw_data[msg_num].sat.available );
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_pos.x,GNSS_SAT_POS_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_pos.y,GNSS_SAT_POS_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_pos.z,GNSS_SAT_POS_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_vel.x,GNSS_SAT_VEL_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_vel.y,GNSS_SAT_VEL_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.sat_vel.z,GNSS_SAT_VEL_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP32_QM_TO_DOUBLE(raw_data[msg_num].sat.range_correction,GNSS_SAT_RANGE_CORRECT_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP16_QM_TO_DOUBLE(raw_data[msg_num].sat.atmospheric_correction,GNSS_ATM_CORRECT_SHIFT));
      if ( nmea_ext_TS_format )
      {
        index += _clibs_sprintf( &out_msg[index], ",%.0f", MCR_FP16_QM_TO_DOUBLE(raw_data[msg_num].sat.range_rate_correction,GNSS_SAT_RANGE_RATE_CORRECT_SHIFT) * 1000.0);
      }

      index += _clibs_sprintf( &out_msg[index], ",%01d", raw_data[msg_num].diff.available );
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP16_QM_TO_DOUBLE(raw_data[msg_num].diff.range_correction,GNSS_DIFF_RANGE_CORRECT_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%.2f", MCR_FP16_QM_TO_DOUBLE(raw_data[msg_num].diff.range_rate_correction,GNSS_DIFF_RANGE_RATE_CORRECT_SHIFT));
      index += _clibs_sprintf( &out_msg[index], ",%d", raw_data[msg_num].pred.available );
      index += _clibs_sprintf( &out_msg[index], ",%d", raw_data[msg_num].pred.age_h );
      index += _clibs_sprintf( &out_msg[index], ",%d", raw_data[msg_num].pred.ephems_n );
      index += _clibs_sprintf( &out_msg[index], ",%d", raw_data[msg_num].pred.time_distance_h );
      index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
      nmea_send_outmsg( out_msg, index, TS_NMEA_MSG );
    }
  }
}

/********************************************//**
 * \brief   Transmits a ERR type message containing
 *          information from the positioning algorithm.
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TIM( void *data_p )
{
  tInt index     = 0;
  time_validity_t time_validity;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  index += _clibs_sprintf( out_msg, "$PSTMTIM" );
  time_validity = gnss_fix_get_time_validity_local( data_p );

  if ( ( time_validity == RTC_TIME ) || ( time_validity == RTC_TIME_ACCURATE ) )
  {
    index += _clibs_sprintf( &out_msg[index], ",RTC" );
  }
  else if ( ( time_validity == APPROX_TIME ) || ( time_validity == EPHEMERIS_TIME ) || ( time_validity == POSITION_TIME ) )
  {
    index += _clibs_sprintf( &out_msg[index], ",VALID" );
  }
  else
  {
    index += _clibs_sprintf( &out_msg[index], ",INVALID" );
  }

  index += _clibs_sprintf( &out_msg[index], ",%u", gpOS_time_now());
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TIM_NMEA_MSG );
}

/********************************************//**
 * \brief   Send RF test message
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TESTRF( void *data_p )
{
  tInt     index;
  chanid_t chan_id;
  tInt     cn0         = 0;
  tInt     freq        = 0;
  tInt     phase_noise = 0;
  satid_t  sat_id;
  tChar    out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  raw_t    *raw_data;
  sat_id = nmea_rftest_satid;

  raw_data = gnss_fix_get_raw_measurements_local( data_p )->list;

  for ( chan_id = 0; chan_id < (chanid_t)TRK_CHANNELS_SUPPORTED; chan_id++ )
  {
    if ( sat_id == raw_data[chan_id].dsp.satid )
    {
      if ( gnss_sat_tracking_check( raw_data[chan_id].dsp.flags.state ) == TRUE )
      {
        cn0         = (tInt)raw_data[chan_id].dsp.signal_strength;
        freq        = (tInt)MCR_FP32_QM_TO_DOUBLE(raw_data[chan_id].dsp.frequency, GNSS_FREQUENCY_SHIFT);
        phase_noise = raw_data[chan_id].dsp.ave_phase_noise;
      }

      break;
    }
  }

  index = _clibs_sprintf( out_msg, "$PSTMTESTRF,%02d,%05d,%05d,%02d", sat_id, freq, phase_noise, cn0 );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, RFTEST_NMEA_MSG );
}

/********************************************//**
 * \brief   Send KFCOV message
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_KFCOV( void *data_p )
{
  tInt index;
  tDouble N, E, V, H;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_fix_get_position_covariance_local( &N, &E, &V, data_p );
  H = sqrt( N + E );
  index = _clibs_sprintf( out_msg, "$PSTMKFCOV,%.1f,%.1f,%.1f,%.1f", H, N, E, V );
  gnss_fix_get_velocity_covariance_local( &N, &E, &V, data_p );
  H = sqrt( N + E );
  index += _clibs_sprintf( &out_msg[index], ",%.1f,%.1f,%.1f,%.1f", H, N, E, V );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, KFCOV_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a message which contains information
 *          on the Pulse Per Second Status
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_PPSDATA( void )
{
  tInt    index     = 0;
  pps_data_t pps_data = {0};
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_pps_get_data( &pps_data );
  index = _clibs_sprintf( out_msg, "$PSTMPPSDATA,%d,%d,%d,%d,%d,%d,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.3e,%.2lf,%.2lf",
                          pps_data.enabled, pps_data.pps_valid, pps_data.pps_synch_valid,
                          pps_data.output_mode,
                          pps_data.reference_time,
                          pps_data.reference_constellation,
                          pps_data.pulse_duration,
                          ( tInt )( pps_data.rf_correction * 1E9 ),
                          ( tInt )( pps_data.gps_rf_correction * 1E9 ),
                          ( tInt )( pps_data.glonass_rf_correction * 1E9 ),
                          pps_data.inverted_polarity,
                          pps_data.fix_condition,
                          pps_data.sat_threshold,
                          pps_data.applied_timing_data.elevation_mask,
                          pps_data.applied_timing_data.constellation_mask,
                          pps_data.applied_timing_data.traim_data.ref_second,
                          pps_data.applied_timing_data.fix_status,
                          pps_data.applied_timing_data.used_sats,
                          pps_data.gps_utc_delta_time_s,
                          pps_data.gps_utc_delta_time_ns,
                          pps_data.glonass_utc_delta_time_ns,
                          pps_data.quantization_error,
                          pps_data.pps_clk_freq_Hz,
                          pps_data.tcxo_clk_freq_Hz
                        );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, PPS_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a message which contains information
 *          on the Position Hold algorithm
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_POSHOLD( void )
{
  tInt    index     = 0;
  tInt    lat_deg;
  tInt    lat_min;
  tInt    lat_min_frac;
  tInt    lon_deg;
  tInt    lon_min;
  tInt    lon_min_frac;
  tChar   lat_sense_ch;
  tChar   lon_sense_ch;
  tInt    valid;
  position_t pos = {0};
  tChar   format[64];
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_pps_get_position_hold_llh_pos( &pos.latitude, &pos.longitude, &pos.height );

  if ( gnss_pps_get_position_hold_status() )
  {
    valid = 1;
  }
  else
  {
    valid = 0;
  }

  nmea_support_degrees_to_int( pos.latitude, 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
  nmea_support_degrees_to_int( pos.longitude, 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
  _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
  index = _clibs_sprintf( out_msg, "$PSTMPOSHOLD" );
  index += _clibs_sprintf( &out_msg[index], ",%d", valid );
  index += _clibs_sprintf( &out_msg[index], format,
                           lat_deg, lat_min, lat_min_frac, lat_sense_ch,
                           lon_deg, lon_min, lon_min_frac, lon_sense_ch
                         );
  index += _clibs_sprintf( &out_msg[index], ",%06.2f", pos.height );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, POSHOLD_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a message which contains information
 *          on the TRAIM algorithm
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TRAIM( void )
{
  tInt    index     = 0;
  timing_data_t timing_data = {0};
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_pps_get_timing_data( &timing_data );
  index = _clibs_sprintf( out_msg, "$PSTMTRAIMSTATUS,%d,%d,%d,%d,%d,%d,%d", timing_data.traim_enabled, timing_data.traim_data.traim_solution, timing_data.traim_alarm, timing_data.traim_data.ave_error, timing_data.traim_data.used_sat_id, timing_data.traim_data.removed_sat_id, timing_data.traim_data.ref_second );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TRAIM_NMEA_MSG );
  index = _clibs_sprintf( out_msg, "$PSTMTRAIMUSED,%d,%d", timing_data.traim_enabled, timing_data.traim_data.used_sat_id );
  {
    tInt i;

    for ( i = 0; i < timing_data.traim_data.used_sat_id; i++ )
    {
      index += _clibs_sprintf( &out_msg[index], ",%d", timing_data.traim_data.used_sat_id_table[i] );
    }
  }
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TRAIM_NMEA_MSG );
  index = _clibs_sprintf( out_msg, "$PSTMTRAIMRES,%d,%d", timing_data.traim_enabled, timing_data.traim_data.used_sat_id );
  {
    tInt i;

    for ( i = 0; i < timing_data.traim_data.used_sat_id; i++ )
    {
      index += _clibs_sprintf( &out_msg[index], ",%d", timing_data.traim_data.residual[i] );
    }
  }
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TRAIM_NMEA_MSG );
  index = _clibs_sprintf( out_msg, "$PSTMTRAIMREMOVED,%d,%d", timing_data.traim_enabled, timing_data.traim_data.removed_sat_id );
  {
    tInt i;

    for ( i = 0; i < timing_data.traim_data.removed_sat_id; i++ )
    {
      index += _clibs_sprintf( &out_msg[index], ",%d", timing_data.traim_data.removed_sat_id_table[i] );
    }
  }
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TRAIM_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a GNS type message for specific constellation
 *
 * \param   data_p    Fix data structure
 * \param   nmea_common_data    Common NMEA output data structure
 * \param   talker_id
 * \return  void
 *
 ***********************************************/
static void nmea_outmsg_send_GNS_for_constellation( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p, tChar *talker_id )
{
  /*{{{  decs*/
  tInt index     = 0;
  tInt    lat_deg;
  tInt    lat_min;
  tInt    lat_min_frac;
  tInt    lon_deg;
  tInt    lon_min;
  tInt    lon_min_frac;
  tChar   lat_sense_ch;
  tChar   lon_sense_ch;
  tChar    format[64];
  raw_t *raw_data;
  raw_measure_list_t *raw_meas;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tChar gps_mode_indicator_ch = 'N';
  tChar gln_mode_indicator_ch = 'N';
  tChar gal_mode_indicator_ch = 'N';
  tChar qzs_mode_indicator_ch = 'N';
  tChar bds_mode_indicator_ch = 'N';
  tChar mode_indicator_ch     = 'N';
  tInt num_sats_used=0;

  if((talker_id != NULL) && (nmea_common_data_p != NULL))
  {
    if ( _clibs_strncmp(talker_id,"GN",2) == 0)
    {
      raw_meas = gnss_fix_get_raw_measurements_local( data_p );
      raw_data = raw_meas->list;

      if ( ( nmea_common_data_p->fix_type == ( tInt )FIX_2D ) || ( nmea_common_data_p->fix_type == ( tInt )FIX_3D ) )
      {
        tUInt i;

        for ( i = 0; i < TRK_CHANNELS_SUPPORTED ; i++ )
        {
          boolean_t current_sat_excluded = FALSE;
          gnss_exclusion_type_t exclusion_type;

          if ( nmea_cmdif_SAT_excl_present == FALSE )
          {
            if ( (gnss_fix_get_excluded_sats_range_local((tInt)i, &exclusion_type,data_p)==TRUE) )
            {
              current_sat_excluded = TRUE;
            }

            if ( (gnss_fix_get_excluded_sats_doppl_local((tInt)i, &exclusion_type,data_p)==TRUE) )
            {
              current_sat_excluded = TRUE;
            }
          }
          if (( raw_meas->chans_used[i] == TRUE) && (current_sat_excluded == FALSE))
          {
            gnss_sat_type_t sat_type = gnss_sat_id_to_sat_type( raw_data[i].dsp.satid );

            if ( MCR_ISBITSET( nmea_outmsg_const_mask, sat_type ))
            {
              mode_indicator_ch = 'A';

              if ( nmea_common_data_p->diff_status == DIFF_STATUS_ON )
              {
                if ( raw_data[i].diff.available != DIFF_CORRECTION_NONE )
                {
                  mode_indicator_ch = 'D';
                }
              }
              switch (sat_type)
              {
                case GNSS_SAT_TYPE_GPS:
                  gps_mode_indicator_ch = mode_indicator_ch;
                  break;

                case GNSS_SAT_TYPE_GLONASS:
                  gln_mode_indicator_ch = mode_indicator_ch;
                  break;

                case GNSS_SAT_TYPE_QZSS_L1_CA:
                  qzs_mode_indicator_ch = mode_indicator_ch;
                  break;

                case GNSS_SAT_TYPE_GALILEO:
                  gal_mode_indicator_ch = mode_indicator_ch;
                  break;

                case GNSS_SAT_TYPE_COMPASS:
                  bds_mode_indicator_ch = mode_indicator_ch;
                  break;

                default:
                /* do nothing */
                  break;

              }
            }
          }
        }
      }
      else
      {
        gps_mode_indicator_ch = 'N';
        gln_mode_indicator_ch = 'N';
        gal_mode_indicator_ch = 'N';
        qzs_mode_indicator_ch = 'N';
        bds_mode_indicator_ch = 'N';

        if ( nmea_common_data_p->fix_type > ( tInt )NO_FIX )
        {
          if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS ) )
          {
            gps_mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
          }
          if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS ) )
          {
            gln_mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
          }
          if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO ) )
          {
            gal_mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
          }
          if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1C ) )
          {
            qzs_mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
          }
          if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS ) )
          {
            bds_mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
          }
        }
      }
    }
    else
    {
      if ( ( nmea_common_data_p->fix_type == ( tInt )FIX_2D ) || ( nmea_common_data_p->fix_type == ( tInt )FIX_3D ) )
      {
        mode_indicator_ch = 'A';

        if ( nmea_common_data_p->diff_status == DIFF_STATUS_ON )
        {
          mode_indicator_ch = 'D';
        }
      }
      else
      {
        if ( nmea_common_data_p->fix_type > ( tInt )NO_FIX )
        {
          mode_indicator_ch = 'E'; /* esimated(dead reckoning) mode */
        }
      }
    }
    nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.latitude, 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
    nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.longitude, 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
    index += _clibs_sprintf( out_msg, "$%sGNS", talker_id );
    index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
    _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
    //index += _clibs_sprintf(&out_msg[index], ",%02d%02d.%03d,%c,%03d%02d.%03d,%c",
    index += _clibs_sprintf( &out_msg[index], format,
                             lat_deg, lat_min, lat_min_frac, lat_sense_ch,
                             lon_deg, lon_min, lon_min_frac, lon_sense_ch );

    if ( _clibs_strncmp(talker_id,"GN",2) == 0)
    {
      index += _clibs_sprintf( &out_msg[index], ",%c%c%c%c%c", gps_mode_indicator_ch, gln_mode_indicator_ch, gal_mode_indicator_ch, qzs_mode_indicator_ch, bds_mode_indicator_ch );
    }
    else
    {
      index += _clibs_sprintf( &out_msg[index], ",%c", mode_indicator_ch);
    }

    /* if the Sats excluded are present in NMEA */
    if (nmea_cmdif_SAT_excl_present == TRUE)
    {
      num_sats_used = nmea_common_data_p->num_sats_used;
    }
    else
    {
      /* Remove the number of excluded sats */
      num_sats_used = nmea_common_data_p->num_sats_used - nmea_common_data_p->num_sats_excluded;
    }

    index += _clibs_sprintf( &out_msg[index], ",%02d,%01.1f,%06.1f,%01.1f,,",
                             num_sats_used, nmea_common_data_p->hdop, nmea_common_data_p->extrap_pos.height, nmea_common_data_p->geoid_msl );
    index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
    index += _clibs_sprintf( &out_msg[index], "\r\n" );
    nmea_send_outmsg( out_msg, index, GNS_NMEA_MSG );
  }
}

/********************************************//**
 * \brief   Transmits a GNS type message
 *
 *
 * \param   data_p      Fix data structure
 * \param   nmea_common_data    Common NMEA output data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_GNS( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p )
{

  if ((sw_config_get_software_switch_status( NMEA_GNGSA_ENABLE )==1U) || ((nmea_outmsg_GNS_constellations_enabled > 1 )  || (nmea_outmsg_GNS_constellations_used >1 )))
  {
    nmea_outmsg_send_GNS_for_constellation( data_p, nmea_common_data_p, "GN" );
  }
  else
  {
    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS ) )
    {
      nmea_outmsg_send_GNS_for_constellation( data_p, nmea_common_data_p, "GP" );
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS ) )
      {
      nmea_outmsg_send_GNS_for_constellation( data_p, nmea_common_data_p, "GL" );
    }

    if ( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
    {
      nmea_outmsg_send_GNS_for_constellation( data_p, nmea_common_data_p, "QZ" );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO))
      {
      nmea_outmsg_send_GNS_for_constellation( data_p, nmea_common_data_p, "GA" );
    }

    if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS))
    {
      nmea_outmsg_send_GNS_for_constellation( data_p, nmea_common_data_p, "BD" );
    }
  }
}


#if defined( NMEA_ADC_SUPPORT )
/********************************************//**
 * \brief   Transmits information about the ADC channel required data
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_ADCDATA( void )
{
  if ( adc_chan_read_mode_ON )
  {
    tInt index     = 0;
    tU32 adc_data[8]; // = 0;
    tInt i;
    tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
    index += _clibs_sprintf( out_msg, "$PSTMADCDATA" );

    for ( i = 0; i < 8; i++ )
    {
      adc_data[i] = 0;
    }

    for ( i = 0; i < 8; i++ )
    {
      if ( adc_chan_to_read[i] == 1 )
      {
      #if defined ( __STA8088__ )
        svc_adc_read( i, SVC_ADC_AVERG_MIN, &adc_data[i], NULL );
      #endif
      #if defined ( __STA8090__ )
        svc_adc_read( i, SVC_ADC_AVERG_MAX, &adc_data[i], NULL );
      #endif
        index += _clibs_sprintf( &out_msg[index], ",%d", adc_data[i] );
      }
      else
      {
        index += _clibs_sprintf( &out_msg[index], "," );
      }
    }

    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}
/*}}}  */
#endif

#if defined( NMEA_ANTENNA_SENSING_SUPPORT )
static void nmea_outmsg_send_ANTENNASENSING_STATUS( void )
{
  //if ( adc_antenna_sensing_ON_OFF /*adc_antenna_sensing_read_mode_ON*/ )
  if( antenna_sensing_get_mode() == ANTENNA_SENSING_MODE_ON)
  {
    static antenna_status_t curr_status, prev_status = ANTENNA_SENSING_STATUS_UNINIT;
    tInt index     = 0;
    tUInt adc_antenna_sensing_cfg_params = 0;
    tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

    antenna_sensing_update_status();

    curr_status = antenna_get_status();

    sw_config_get_param( CURRENT_CONFIG_DATA, ADC_ANTENNA_SENSING_CFG_PARAMS_ID, &adc_antenna_sensing_cfg_params );

    if ( ( curr_status != prev_status ) || ( ( adc_antenna_sensing_cfg_params & 0x4 ) != 0 ) )
    {
      prev_status = curr_status;
      index += _clibs_sprintf( out_msg, "$PSTMANTENNASTATUS,%d", antenna_get_status() );
      index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
      nmea_send_msg_to_uart( out_msg, index );
    }
  }
}
#endif

/********************************************//**
 * \brief   Dump ephemeris for a given satellite
 *
 * \param   sat_id  ID of satellite to dump
 * \return  NMEA_ERROR if ephemeris is not accessible
 *
 ***********************************************/
static nmea_error_t nmea_cmdif_send_ephemeris( const tInt sat_id )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  ephemeris_raw_t ephemeris;
  boolean_t eph_available = FALSE;
  tU8* epointer = ( tU8* )( &ephemeris );
  tInt index;
  tInt index2;

  if ( gnss_get_ephemeris_params( sat_id, &ephemeris, &eph_available ) == GNSS_ERROR )
  {
    GPS_DEBUG_MSG( ( "[nmea]gnss_get_ephemeris_params failed!\r\n" ) );
  }

  if ( eph_available == TRUE )
  {
    tInt eph_sizeof = gnss_get_ephemeris_sizeof( gnss_sat_id_to_sat_type( sat_id ) );
    index = _clibs_sprintf( out_msg, "$P%sEPHEM,%i,%i,", NMEA_OEM_ID, sat_id, eph_sizeof );

    for ( index2 = 0; index2 < eph_sizeof; index2++ )
    {
      index += _clibs_sprintf( &out_msg[index], "%02x", epointer[index2] );
    }

    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_outmsg( out_msg, index , EPHEM_NMEA_MSG);
  }
  else
  {
    /*flash_debug("ephemeris not available for selected satellite\r\n");  silent*/
  }

  return NMEA_OK;
}

/********************************************//**
 * \brief   Execute GPS reset command
 *
 * \param   cmd_par   Parameters string
 * \return  void
 *
 ***********************************************/
static void nmea_cmdif_exec_gpsreset( tChar *cmd_par )
{
  boolean_t lowpower_status = FALSE;
  svc_pwr_get_lowpower_allowed( &lowpower_status);
  svc_pwr_set_lowpower_allowed( FALSE);
  if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
  {
    gnssapp_suspend();
    platform_gnss_suspend();
  }
  platform_gnss_restart();
  gnssapp_restart();
  svc_pwr_set_lowpower_allowed( lowpower_status);
}

/********************************************//**
 * \brief   Execute soft reset command
 *
 * \param   cmd_par   Parameters string
 * \return  void
 *
 ***********************************************/
static void nmea_cmdif_exec_softreset( tChar *cmd_par )
{
  svc_pwr_set_lowpower_allowed(FALSE);
  if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
  {
    gnssapp_suspend();
    platform_gnss_suspend();
  }
  svc_mcu_sw_reset();
}

/********************************************//**
 * \brief   Execute NVM swap command
 *
 * \param   cmd_par   Parameters string
 * \return  void
 *
 ***********************************************/
static void nmea_cmdif_exec_nvmswap( tChar *cmd_par )
{
  nvm_swap( TRUE );
}

/********************************************//**
 * \brief   Execute clear all ephemerides command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_loadephems( tChar *cmd_par )
{
  GPS_DEBUG_MSG( ( "\r\n[nmea]Clearing all ephemerides\r\n" ) );
  gnss_clear_all_ephems();
  nvm_swap( TRUE );
  /*GPS_DEBUG_MSG(("%s\r\n", msg));*/
}

/********************************************//**
 * \brief   Load ephemerides from NMEA input
 *
 * \param   cmd_par   Parameters string
 * \return  void
 *
 ***********************************************/
static void nmea_cmdif_exec_saveephems( tChar *cmd_par )
{
  tChar           out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt            index;
  ephemeris_raw_t ephemeris;
  tInt            length;
  tInt            sat_id;
  tChar           hexbuffer[200];
  tU8 *           eph_ptr = (tU8 *)&ephemeris;
  tUInt           field_count;
  tUInt           cnt;
  tBool           error_flag = FALSE;

  GPS_DEBUG_MSG( ( "[nmea]entering load ephem\r\n" ) );
  field_count =  _clibs_sscanf( cmd_par, ",%d,%d,%s", &sat_id, &length, hexbuffer );

  if ( field_count == 3 )
  {
    _clibs_memset( &ephemeris, 0, sizeof( ephemeris_raw_t ) );

    /* translate hex buffer to binary */
    for ( cnt = 0 ; cnt < length ; cnt++ )
    {
      eph_ptr[cnt] = nmea_support_hex2toint( hexbuffer + ( cnt * 2 ) );
    }

    if ( gnss_set_ephemeris_params( sat_id, &ephemeris ) )
    {
      /* report error */
      GPS_DEBUG_MSG( ( "[nmea]gnss_set_ephemeris_params failed!\r\n" ) );
      error_flag = TRUE;
    }
  }
  else
  {
    error_flag = TRUE;
  }

  if (error_flag == TRUE)
  {
    index = _clibs_sprintf( out_msg, "$PSTMEPHEMERROR" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMEPHEMOK" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Dump ephemerides for all satellites
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_dumpephems( tChar *cmd_par )
{
  tInt sat;
  gnss_sat_type_mask_t const_mask_supported = gnss_get_constellation_mask();
  GPS_DEBUG_MSG( ( "[nmea]start of dumpephems \r\n" ) );
  //gpOS_task_delay( 1000000 );

  for ( sat = MIN_GPS_SAT_ID; sat <= MAX_GPS_SAT_ID ; sat ++ )
  {
    nmea_cmdif_send_ephemeris( sat );
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_GLONASS ) )
  {
    for ( sat = MIN_GLONASS_SAT_ID; sat <= MAX_GLONASS_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_ephemeris( sat );
    }
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_QZSS_L1_CA ) )
  {
    for ( sat = MIN_QZSS_L1_CA_SAT_ID; sat <= MAX_QZSS_L1_CA_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_ephemeris( sat );
    }
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_GALILEO ) )
  {
    for ( sat = MIN_GALILEO_SAT_ID; sat <= MAX_GALILEO_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_ephemeris( sat );
    }
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_COMPASS ) )
  {
    for ( sat = MIN_COMPASS_SAT_ID; sat <= MAX_COMPASS_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_ephemeris( sat );
    }
  }

  GPS_DEBUG_MSG( ( "[nmea]end of dumpephems \r\n" ) );
}

/********************************************//**
 * \brief   Load almanacs from NMEA input
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_savealms( tChar *cmd_par )
{
  tChar           out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt            index;
  almanac_raw_t   almanac;
  tInt            length, sat_id;
  tChar           hexbuffer[200];
  tU8 *           apointer = (tU8 *)&almanac;
  tUInt           field_count;
  tUInt           cnt;
  tBool           error_flag = FALSE;

  GPS_DEBUG_MSG( ( "[nmea]entering load alamanac\r\n" ) );
  field_count = _clibs_sscanf( cmd_par, ",%d,%d,%s", &sat_id, &length, hexbuffer );

  if ( field_count == 3 )
  {
    /* translate hex buffer to binary */
    for ( cnt = 0 ; cnt < length ; cnt++ )
    {
      apointer[cnt] = nmea_support_hex2toint( hexbuffer + ( cnt * 2 ) );
    }

    if ( gnss_set_almanac_params( sat_id, &almanac ) == GNSS_ERROR )
    {
      /* report error */
      GPS_DEBUG_MSG( ( "[nmea]gnss_set_almanac_params failed!\r\n" ) );
      error_flag = TRUE;
    }
  }
  else
  {
    error_flag = TRUE;
  }

  if (error_flag == TRUE)
  {
    index = _clibs_sprintf( out_msg, "$PSTMALMANACERROR" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMALMANACOK" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );

}

/********************************************//**
 * \brief   Clear all almanacs
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_loadalms( tChar *cmd_par )
{
  GPS_DEBUG_MSG( ( "\r\n[nmea]Save Almanacs Received\r\n" ) );
  gnss_clear_all_almanacs();
  nvm_swap( TRUE );
}

/********************************************//**
 * \brief   Dumps almanacs for all satellites
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_dumpalms( tChar *cmd_par )
{
  tInt sat;
  gnss_sat_type_mask_t const_mask_supported = gnss_get_constellation_mask();
  GPS_DEBUG_MSG( ( "[nmea]start of dumpalms \r\n" ) );
  //gpOS_task_delay( 1000000 );

  for ( sat = MIN_GPS_SAT_ID; sat <= MAX_GPS_SAT_ID ; sat ++ )
  {
    nmea_cmdif_send_almanac( sat );
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_GLONASS ) )
  {
    for ( sat = MIN_GLONASS_SAT_ID; sat <= MAX_GLONASS_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_almanac( sat );
    }
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_QZSS_L1_CA ) )
  {
    for ( sat = MIN_QZSS_L1_CA_SAT_ID; sat <= MAX_QZSS_L1_CA_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_almanac( sat );
    }
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_GALILEO ) )
  {
    for ( sat = MIN_GALILEO_SAT_ID; sat <= MAX_GALILEO_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_almanac( sat );
    }
  }

  if ( MCR_ISBITSET( const_mask_supported, GNSS_SAT_TYPE_COMPASS ) )
  {
    for ( sat = MIN_COMPASS_SAT_ID; sat <= MAX_COMPASS_SAT_ID ; sat ++ )
    {
      nmea_cmdif_send_almanac( sat );
    }
  }

  GPS_DEBUG_MSG( ( "[nmea]end of dumpalms \r\n" ) );
}

/********************************************//**
 * \brief   Dump almanac for a given satellite
 *
 * \param   sat_id  Satellite to dump
 * \return  NMEA_ERROR if almanac is not accessible
 *
 ***********************************************/
static nmea_error_t nmea_cmdif_send_almanac( const tInt sat_id )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  almanac_raw_t almanac;
  boolean_t alm_available = FALSE;
  tU8* apointer = ( tU8* )( &almanac ) ;
  tInt index, index2;

  if ( gnss_get_almanac_params( sat_id, &almanac, &alm_available ) == GNSS_ERROR )
  {
    GPS_DEBUG_MSG( ( "[nmea]gnss_get_almanac_params failed!\r\n" ) );
  }

  if ( alm_available == TRUE )
  {
    tInt alm_sizeof = gnss_get_almanac_sizeof( gnss_sat_id_to_sat_type( sat_id ) );
    index = _clibs_sprintf( out_msg, "$P%sALMANAC,%i,%i,", NMEA_OEM_ID, sat_id, alm_sizeof );

    for ( index2 = 0; index2 < alm_sizeof; index2++ )
    {
      index += _clibs_sprintf( &out_msg[index], "%02x", apointer[index2] );
    }

    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_outmsg( out_msg, index, ALM_NMEA_MSG);
  }
  else
  {
    /*flash_debug("ephemeris not available for selected satellite\r\n");  silent*/
  }

  return NMEA_OK;
}

/********************************************//**
 * \brief   Set frequency range
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setrange( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt min, max;
  field_count = _clibs_sscanf( cmd_par, ",%d,%d", &min, &max );

  if ( field_count == 2 )
  {
    if ( gnss_set_freq_range( max, min ) == GNSS_NO_ERROR )
    {
      index = _clibs_sprintf( out_msg, "$PSTMSETRANGEOK" );
    }
    else
    {
      index = _clibs_sprintf( out_msg, "$PSTMSETRANGEERROR" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETRANGEERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Execute RF test on command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_rfteston( tChar *cmd_par )
{
  if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
  {
    gnssapp_suspend();
    platform_gnss_suspend();
  }
  platform_gnss_restart();
  gnss_reset_rf_test_mode();
  nmea_cmdif_exec_rftestadd( cmd_par );
  gnssapp_restart();
  NMEA_msg_list[0] |= ( (tUInt)1 << RFTEST_NMEA_MSG );
}

/********************************************//**
 * \brief   Execute RF test off command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_rftestoff( tChar *cmd_par )
{
  gnss_reset_rf_test_mode();
  NMEA_msg_list[0] &= ~( (tUInt)1 << RFTEST_NMEA_MSG );
  if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
  {
    gnssapp_suspend();
    platform_gnss_suspend();
  }
  platform_gnss_restart();
  gnssapp_restart();
}

/********************************************//**
 * \brief   Execute RF test add command
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_rftestadd( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt sat_id = 0;
  tInt cn0_trk_threshold = 1; /*dB*/
  tInt xtal_rf_nav_usage_dB_thr   = 40; /*dB*/
  field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &sat_id, &cn0_trk_threshold, &xtal_rf_nav_usage_dB_thr );

  if ( field_count == 3 )
  {
    gnss_set_rf_test_mode_on_sat_n_cn0_trk_xtal_thr( sat_id, cn0_trk_threshold , xtal_rf_nav_usage_dB_thr );
    nmea_rftest_satid = sat_id;
  }

  field_count = _clibs_sscanf( cmd_par, ",%d,%d", &sat_id, &cn0_trk_threshold );

  if ( field_count == 2 )
  {
    gnss_set_rf_test_mode_on_sat_n_cn0_trk_thr( sat_id, cn0_trk_threshold );
    nmea_rftest_satid = sat_id;
  }
  else if ( field_count == 1 )
  {
    gnss_set_rf_test_mode_on_sat_n( sat_id );
    nmea_rftest_satid = sat_id;
  }
  else
  {
    gnss_set_rf_test_mode_on_sat_n( 16 );
    nmea_rftest_satid = 16;
  }

  index = _clibs_sprintf( out_msg, "$PSTMRFTESTON" );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Delete satellite for RF test
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_rftestdel( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt sat_id;
  field_count = _clibs_sscanf( cmd_par, ",%d", &sat_id );

  if ( field_count == 1 )
  {
    gnss_rf_test_mode_del_sat_n( sat_id );
  }

  index = _clibs_sprintf( out_msg, "$PSTMRFSATDEL" );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Displays version for SW modules
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_get_swver( tChar *cmd_par )
/*lint -e{904}: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
/*lint -e{9090}: unconditional break missing from switch case [MISRA 2012 Rule 16.3, required] */
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt field_count = 0;
  tInt type;
  tInt index = 0;
  field_count = _clibs_sscanf( cmd_par, ",%d", &type );

  if ( field_count == 1 )
  {
    switch ( type )
    {
      case 0:
        index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gnss_version() );
        break;

      case 1:
        index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gpOS_version() );
        break;

      case 2:
        index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gnssapp_version() );
        break;

      case 4:
        {
          const tChar *ver_ptr = NULL;
          gnssapp_plugins_cmd_param_t gpparam;
          gpparam.data_ptr = ( void * )&ver_ptr;
          gnssapp_plugins_cmd( GNSSAPP_PLUGINS_ID_WAAS, GNSSAPP_PLUGINS_CMD_GETVER, &gpparam );

          if ( ver_ptr != NULL )
          {
            index = _clibs_sprintf( out_msg, "$PSTMVER,%s", ver_ptr );
          }
          else
          {
            index = _clibs_sprintf( out_msg, "$PSTMVERERROR" );
          }
        }
        break;

#if defined( NMEA_BINIMG_SUPPORT )
      case 6:
        index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gnssapp_binimg_version() );
        break;
#endif

      case 7:
        index = _clibs_sprintf( out_msg, "$PSTMVER,%s_%x", svc_mcu_getprodname(), svc_mcu_getromver() );
        break;

      case 8:
        {
          const tChar *ver_ptr = NULL;
          gnssapp_plugins_cmd_param_t gpparam;
          gpparam.data_ptr = ( void * )&ver_ptr;
          gnssapp_plugins_cmd( GNSSAPP_PLUGINS_ID_STAGPS, GNSSAPP_PLUGINS_CMD_GETVER, &gpparam );

          if ( ver_ptr != NULL )
          {
            index = _clibs_sprintf( out_msg, "$PSTMVER,%s", ver_ptr );
          }
          else
          {
            index = _clibs_sprintf( out_msg, "$PSTMVERERROR" );
          }
        }
        break;

      case 9:
        {
          const tChar *ver_ptr = NULL;
          gnssapp_plugins_cmd_param_t gpparam;
          gpparam.data_ptr = ( void * )&ver_ptr;
          gnssapp_plugins_cmd( GNSSAPP_PLUGINS_ID_RTCM, GNSSAPP_PLUGINS_CMD_GETVER, &gpparam );

          if ( ver_ptr != NULL )
          {
            index = _clibs_sprintf( out_msg, "$PSTMVER,%s", ver_ptr );
          }
          else
          {
            index = _clibs_sprintf( out_msg, "$PSTMVERERROR" );
          }
        }
        break;

      case 10:
        {
          const tChar *ver_ptr = NULL;
          gnssapp_plugins_cmd_param_t gpparam;
          gpparam.data_ptr = ( void * )&ver_ptr;
          gnssapp_plugins_cmd( GNSSAPP_PLUGINS_ID_DR, GNSSAPP_PLUGINS_CMD_GETVER, &gpparam );

          if ( ver_ptr != NULL )
          {
            index = _clibs_sprintf( out_msg, "$PSTMVER,%s", ver_ptr );
          }
          else
          {
            index = _clibs_sprintf( out_msg, "$PSTMVERERROR" );
          }
        }
        break;

      case 11:
        {
          index = _clibs_sprintf( out_msg, "$PSTMVER,%s_%x", "SWCFG", gnssapp_swcfg_version() );
        }
        break;

      case 254:
        {
          nmea_outmsg_transmit_swconfig();
          return;
        }

      case 255:
        {
          nmea_outmsg_transmit_short_header();
          nmea_outmsg_transmit_full_header();
          return;
        }

      default:
        index = _clibs_sprintf( out_msg, "$PSTMVERERROR" );
        break;
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMVER,%s", gnss_version() );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Executes FW upgrade
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_fwupgrade( tChar *cmd_par )
{
  #if defined( NMEA_BINIMG_SUPPORT )
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;

  gnssapp_suspend();
  GPS_DEBUG_MSG( ( "\r\nGPS_FW_UPGRADE Received\r\n" ) );
  index = _clibs_sprintf( out_msg, "$PSTMFWUPGRADEOK\r\n" );
  nmea_send_msg_to_uart( out_msg, index );
  gpOS_task_delay( gpOS_timer_ticks_per_sec() / 10 );
  gpOS_interrupt_lock();

  platform_fwupgrade();
  #endif
}

/********************************************//**
 * \brief   Extract the data from the NMEA CONFIG command
 *
 * \param   msg                 Parameters string
 * \param   baud_rate           The port baud rate
 * \param   nmea_msg_list       The list of nmea sentences
 * \param   nmea_outmsg_transmit_mode  NMEA_TXMODE_ON_UTC_SECOND or NMEA_TXMODE_AFTER_FIX
 * \return  NMEA_ERROR if the command data does not meet expected criteria
 *
 ***********************************************/
static nmea_error_t nmea_cmdif_nmeacfg_parse_params( tChar *msg, tInt *baud_rate, tUInt *nmea_msg_list, nmea_transmit_mode_t *transmit_mode_ptr )
{
  tInt field_count = 0;
  tUInt baud_rate_val;
  tInt nmea_msg_list_val;
  tInt nmea_transmit_mode_int;
  nmea_transmit_mode_t nmea_transmit_mode_val;
  field_count = _clibs_sscanf( msg, ",%i,%d,%d",
                               &baud_rate_val, &nmea_msg_list_val, &nmea_transmit_mode_int
                             );

  if ( nmea_transmit_mode_int == 0 )
  {
    nmea_transmit_mode_val = NMEA_TXMODE_ON_UTC_SECOND;
  }
  else
  {
    nmea_transmit_mode_val = NMEA_TXMODE_AFTER_FIX;
  }

  if ( field_count == 3 )
  {
    if ( ( baud_rate_val >= 4800 ) && ( baud_rate_val <= 115200 ) )
    {
        *baud_rate = baud_rate_val;
        *transmit_mode_ptr = nmea_transmit_mode_val;

        if ( nmea_msg_list_val != 0 )
        {
          nmea_msg_list[0] = nmea_msg_list_val;
        }

        return ( NMEA_OK );
    }
  }

  return ( NMEA_ERROR );
}

/********************************************//**
 * \brief   Change NMEA configuration
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_nmeacfg( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index = 0;

  if ( nmea_cmdif_nmeacfg_parse_params( cmd_par, &nmea_baud_rate, &NMEA_msg_list[0], &nmea_outmsg_transmit_mode ) == NMEA_OK )
  {
    gpOS_semaphore_wait( nmea_outmsg_access );
    //nmea_uart_init( nmea_baud_rate);
    gpOS_semaphore_signal( nmea_outmsg_access );
  }
  else
  {
    index = _clibs_sprintf( &out_msg[index], "$PSTMNMEACONFIGERROR*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}

/********************************************//**
 * \brief   Set 2D fix status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_set2dfix( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index = 0;
  tInt field_count = 0;
  tInt param;
  extern boolean_t gnss_fix_allow_large_2D_move;
  field_count = _clibs_sscanf( cmd_par, ",%d", &param );

  if ( field_count == 1 )
  {
    if ( param != 0 )
    {
      gnss_fix_allow_large_2D_move = TRUE;
      index = _clibs_sprintf( out_msg, "$PSTM2DFIXENABLED" );
    }
    else
    {
      gnss_fix_allow_large_2D_move = FALSE;
      index = _clibs_sprintf( out_msg, "$PSTM2DFIXDISABLED" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTM2DFIXONOFFERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Displays RTC time
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_getrtctime( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  gnss_time_t gnss_time;
  gpOS_clock_t cpu_time;
  rtc_status_t rtc_status;
  time_validity_t time_validity;
  tInt    year;
  tInt    month;
  tInt    day;
  tInt    hours;
  tInt    mins;
  tInt    secs;
  tInt    msecs;
  gnss_rtc_get_time( &gnss_time, &cpu_time, &rtc_status, &time_validity );
  gnss_get_utc_time( gnss_time.tow, &hours, &mins, &secs, &msecs );
  gnss_get_date( gnss_time.week_n, gnss_time.tow, &year, &month, &day );
  index = _clibs_sprintf( out_msg, "$PSTMRTCTIME,%02d%02d%02d.%03d,%02d%02d%02d,%d,%d", hours, mins, secs, msecs, day, month, year % 100, rtc_status, time_validity );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Displays informations on executing tasks
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_taskscheck( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gpOS_task_t*  task;
  tSize   size;
  tSize   used;
  tDouble   used_percentage;
  tInt index;
  task = gpOS_task_get_head();

  while ( task != NULL )
  {
    /* determine stack unused */
    used = gpOS_task_get_stack_used( task );
    size = gpOS_task_get_stack_size( task );
    used_percentage = ( ( tDouble )used / ( tDouble )size ) * 100;
    index = _clibs_sprintf( out_msg, "$PSTMTASKCHECK,%s,%d,0x%08x,0x%08x,%d,%d,%4.1f",
                            gpOS_task_get_name( task ),
                            gpOS_task_get_priority( task ),
                            ( tUInt )gpOS_task_get_stack_base( task ),
                            ( tUInt )gpOS_task_get_stack_ptr( task ),
                            size,
                            used,
                            used_percentage
                          );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
    task = gpOS_task_get_next( task );
  }
}

/********************************************//**
 * \brief   Displays information about memory partitions
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_checkmem( tChar *cmd_par )
{
  tInt index;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gpOS_partition_t* next_part = NULL;
  index = _clibs_sprintf( out_msg, "$PSTMHEAPCHECK,%d,%d", gpOS_memory_getheapsize(), gpOS_memory_getheapfree() );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );

  do
  {
    next_part = gpOS_memory_getnextpartition( next_part );

    if ( next_part != NULL )
    {
      index = _clibs_sprintf( out_msg, "$PSTMHEAPCHECK,%d,%d", gpOS_memory_getheapsize_p( next_part ), gpOS_memory_getheapsize_p( next_part ) );
      index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
      nmea_send_msg_to_uart( out_msg, index );
    }
  }
  while ( next_part != NULL );
}

/********************************************//**
 * \brief   Reset free stack space for each task
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_resetstackusage( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gpOS_task_t*  task;
  tInt index;
  tU8 *stack_ptr;
  tU8 *stack_start_ptr;
  tU8 *stack_curr_ptr;
  task = gpOS_task_get_head();

  while ( task != NULL )
  {
    stack_curr_ptr = (tU8 *)gpOS_task_get_stack_ptr( task);

    for( stack_ptr = gpOS_task_get_stack_base( task ); ( *stack_ptr != 0xaa ) && ( stack_ptr < stack_curr_ptr ); stack_ptr++ );

    stack_start_ptr = stack_ptr + 1;
    /* determine stack unused */
    gpOS_interrupt_lock();

    for ( stack_ptr = stack_start_ptr; stack_ptr < stack_curr_ptr ; stack_ptr++ )
    {
      *stack_ptr = 0xaa;
    }

    gpOS_interrupt_unlock();
    task = gpOS_task_get_next( task );
  }

  index = _clibs_sprintf( out_msg, "$PSTMSTACKUSAGERESETOK" );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set RTC test status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setrtctest( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt field_count = 0, index;
  tInt status;
  extern void rtc_test_on_off( boolean_t );
  field_count = _clibs_sscanf( cmd_par, ",%1d", &status );

  if ( field_count == 1 )
  {
    if ( status != 0 )
    {
      rtc_test_on_off( TRUE );
    }
    else
    {
      rtc_test_on_off( FALSE );
    }

    index = _clibs_sprintf( out_msg, "$PSTMRTCTESTOK,%d", status );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMRTCTESTERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set algo status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setalgostatus( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tUInt algo_type;
  tUInt algo_status;
  tInt field_count = 0, index = 0;
  field_count = _clibs_sscanf( cmd_par, ",%1d,%1d", &algo_type, &algo_status );

  if ( field_count == 2 )
  {
    switch ( algo_type )
    {
      case 0:/*FDA*/
        break;

      case 1:/*FDE*/
        if ( algo_status != 0 )
        {
          gnss_set_fde_status( FDE_STATUS_ON );
        }
        else
        {
          gnss_set_fde_status( FDE_STATUS_OFF );
        }

        break;

      case 2:/*ADS*/
        break;
    }

    index = _clibs_sprintf( out_msg, "$PSTMSETALGOOK,%d,%d", algo_type, algo_status );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETALGOERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Get algo status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_getalgostatus( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tUInt algo_type;
  tUInt algo_status = 0;
  tInt field_count = 0, index;
  field_count = _clibs_sscanf( cmd_par, ",%1d", &algo_type );

  if ( field_count == 1 )
  {
    switch ( algo_type )
    {
      case 0:/*FDA*/
        break;

      case 1:/*FDE*/
        algo_status = ( tUInt )gnss_get_fde_status();
        break;

      case 2:/*ADS*/
        break;
    }

    index = _clibs_sprintf( out_msg, "$PSTMGETALGOOK,%d,%d", algo_type, algo_status );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMGETALGOERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Invalidate an item in NVM
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_nvm_invalidate( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt field_count = 0;
  tInt item_ID;
  tInt dir_ID;
  tInt index;
  field_count = _clibs_sscanf( cmd_par, ",%d,%d", &dir_ID, &item_ID );

  if ( field_count == 2 )
  {
    if ( nvm_set_item_invalid( dir_ID, item_ID ) == NVM_NO_ERROR )
    {
      index = _clibs_sprintf( out_msg, "$PSTMNVMITEMINVOK,%d,%d", dir_ID, item_ID );
    }
    else
    {
      index = _clibs_sprintf( out_msg, "$PSTMNVMITEMINVERROR" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMNVMITEMINVERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Change position
 *
 * \param   cmd_par   Parameters string
 * \return  NMEA_ERROR if the command data does not meet expected criteria
 *
 ***********************************************/
void nmea_cmdif_exec_set_gnss_pos( tChar *cmd_par )
{
  tInt field_count = 0;
  tDouble lat_val, lat_deg, lat_min;
  tDouble lon_val, lon_deg, lon_min;
  tDouble height_val;
  tChar lat_sense, lon_sense;
  /*}}}  */
  field_count = _clibs_sscanf( cmd_par, ",%2lf%6lf,%1c,%3lf%6lf,%1c,%5lf",
                               &lat_deg, &lat_min, &lat_sense, &lon_deg, &lon_min, &lon_sense, &height_val
                             );

  if ( field_count == 7 )
  {
    if ( ( lat_deg > 90.0 )  || ( lat_deg < 0.0 ) ||
         ( lat_min >= 60.0 ) || ( lat_min < 0.0 ) ||
         ( lon_deg > 180.0 ) || ( lon_deg < 0.0 ) ||
         ( lon_min >= 60.0 ) || ( lon_min < 0.0 ) ||
         ( height_val > 10000.0 ) || ( height_val < 0.0 ) ||
         ( ( lat_sense != 'N' ) && ( lat_sense != 'S' ) ) ||
         ( ( lon_sense != 'W' ) && ( lon_sense != 'E' ) )
       )
    {
      return;
    }

    lat_val = lat_deg + (lat_min / 60.0);
    lon_val = lon_deg + (lon_min / 60.0);

    if ( lat_sense == 'S' )
    {
      lat_val = -lat_val;
    }

    if ( lon_sense == 'W' )
    {
      lon_val = -lon_val;
    }

    if ( ( lat_val > 90.0 ) || ( lat_val < ( -90.0 ) ) )
    {
      return;
    }

    if ( ( lon_val > 180.0 ) || ( lon_val < ( -180.0 ) ) )
    {
      return;
    }

    gnss_test_set_user_pos( lat_val * RADIANS, lon_val * RADIANS, height_val );
  }
}

/********************************************//**
 * \brief   Execute jammer test
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_jammer_test( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt value;
  field_count = _clibs_sscanf( cmd_par, ",%d", &value );

  if ( field_count == 1 )
  {
    gnss_set_tracker_jammer( value );
    index = _clibs_sprintf( out_msg, "$PSTMTRKJAMMEROK,%d", value );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMTRKJAMMERERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Execute RTC error test
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_rtcerrortest( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt err, accuracy;
  tDouble time_error;
  gnss_time_t gnss_time_result;
  gpOS_clock_t cpu_time_result;
  rtc_status_t rtc_status;
  time_validity_t stored_time_validity;
  gnss_time_reference_t gnss_time_reference_copy;
  field_count = _clibs_sscanf( cmd_par, ",%d,%d", &err/*ms*/, &accuracy/*ms*/ );

  if ( field_count == 2 )
  {
    gnss_sat_type_t sat_type;
    tDouble utc_delta_time;
    gnssapp_suspend();
    gpOS_interrupt_lock();
    gnss_rtc_read( &gnss_time_result, &cpu_time_result, &rtc_status, &stored_time_validity, &sat_type, &utc_delta_time );
    time_error = ( tDouble )err / 1000;
    gnss_time_result = gnss_time_plus( &gnss_time_result, time_error );
    gnss_time_reference_copy.gps_time = gnss_time_result;
    gnss_time_reference_copy.sat_type = sat_type;
    gnss_time_reference_copy.cpu_time = cpu_time_result;

    if ( accuracy < 50/*ms*/ )
    {
      gnss_rtc_write( gnss_time_reference_copy, ACCURATE_TIME, TRUE, utc_delta_time );
    }
    else
    {
      gnss_rtc_write( gnss_time_reference_copy, RTC_TIME, TRUE, utc_delta_time );
    }

    gpOS_interrupt_unlock();
    GPS_DEBUG_MSG( ( "[rtc][deliberate error]: %d ms\r\n", err ) );
    index = _clibs_sprintf( out_msg, "$PSTMRTCERRTESTOK,%d,%d,%d,%lf,%lf", err, accuracy, gnss_time_reference_copy.gps_time.week_n, gnss_time_reference_copy.gps_time.tow, time_error );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
    nmea_support_restart();
    gnssapp_restart();
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMRTCERRTESTERROR,%d,%d", err, accuracy );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}

/********************************************//**
 * \brief   Set source for differential parameters
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setdiffsource( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt value;
  field_count = _clibs_sscanf( cmd_par, ",%d", &value );

  if ( field_count == 1 )
  {
    switch ( value )
    {
      case 0:
        gnss_diff_set_source_type( DIFF_SOURCE_NONE );
        break;

      case 1:
        gnss_diff_set_source_type( DIFF_SOURCE_WAAS );
        break;

      case 2:
        gnss_diff_set_source_type( DIFF_SOURCE_RTCM );
        break;

      case 3:
        gnss_diff_set_source_type( DIFF_SOURCE_AUTO );
        break;

      default:
        break;
    }

    index = _clibs_sprintf( out_msg, "$PSTMSETDIFFSOURCEOK,%d", value );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETDIFFSOURCEERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set fix rate for GNSS library
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setfixrate( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tDouble value;
  field_count = _clibs_sscanf( cmd_par, ",%lf", &value );

  if ( field_count == 1 )
  {
    gnss_set_fix_rate( value );
    index = _clibs_sprintf( out_msg, "$PSTMSETFIXRATEOK,%.2lf", value );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETFIXRATEERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set stop detection status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setstopdet( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt value;
  field_count = _clibs_sscanf( cmd_par, ",%d", &value );

  if ( field_count == 1 )
  {
    if ( value == 1 )
    {
      gnss_turn_stop_detection_on( TRUE );
      index = _clibs_sprintf( out_msg, "$PSTMSTOPDETECTIONON" );
    }
    else
    {
      gnss_turn_stop_detection_on( FALSE );
      index = _clibs_sprintf( out_msg, "$PSTMSTOPDETECTIONOFF" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSTOPDETECTIONFFERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set walking mode status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setwalkingmode( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count = 0;
  tInt value;
  field_count = _clibs_sscanf( cmd_par, ",%d", &value );

  if ( field_count == 1 )
  {
    if ( value != 0 )
    {
      gnss_turn_walking_mode_on( TRUE );
      index = _clibs_sprintf( out_msg, "$PSTMWALKINGON" );
    }
    else
    {
      gnss_turn_walking_mode_on( FALSE );
      index = _clibs_sprintf( out_msg, "$PSTMWALKINGOFF" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMWALKINGERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set position hold status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setposhold( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tDouble lat, lat_deg, lat_min;
  tDouble lon, lon_deg, lon_min;
  tChar lat_sense, lon_sense;
  tInt field_count = 0;
  tDouble height_val;
  tInt result = 0;
  tInt enable_disable = 0;
  field_count = _clibs_sscanf( cmd_par, ",%d,%2lf%10lf,%1c,%3lf%10lf,%1c,%10lf", &enable_disable,
                               &lat_deg, &lat_min, &lat_sense,
                               &lon_deg, &lon_min, &lon_sense, &height_val
                             );

  if ( field_count == 8 )
  {
    if ( enable_disable != 0 )
    {
      if (
        ( lat_deg > 90.0 )       || ( lat_deg < 0.0 )     ||
        ( lat_min >= 60.0 )      || ( lat_min < 0.0 )     ||
        ( lon_deg > 180.0 )      || ( lon_deg < 0.0 )     ||
        ( lon_min >= 60.0 )      || ( lon_min < 0.0 )     ||
        ( height_val > 10000.0 ) || ( height_val < -1500.0 )  ||
        ( ( lat_sense != 'N' )    && ( lat_sense != 'S' ) ) ||
        ( ( lon_sense != 'W' )    && ( lon_sense != 'E' ) )
      )
      {
        result = 1;
      }

      lat = lat_deg + (lat_min / 60.0);
      lon = lon_deg + (lon_min / 60.0);

      if ( lat_sense == 'S' )
      {
        lat = -lat;
      }

      if ( lon_sense == 'W' )
      {
        lon = -lon;
      }

      if ( ( lat > 90.0 ) || ( lat < ( -90.0 ) ) )
      {
        result = 1;
      }

      if ( ( lon > 180.0 ) || ( lon < ( -180.0 ) ) )
      {
        result = 1;
      }

      if ( !result )
      {
        gnss_pps_set_position_hold_llh_pos( lat, lon, height_val );
        gnss_pps_set_position_hold_status( TRUE );
        index = _clibs_sprintf( out_msg, "$PSTMPOSITIONHOLDENABLED" );
      }
    }
    else
    {
      gnss_pps_set_position_hold_status( FALSE );
      index = _clibs_sprintf( out_msg, "$PSTMPOSITIONHOLDDISABLED" );
    }
  }
  else
  {
    result = 1;
  }

  if ( result != 0 )
  {
    index = _clibs_sprintf( out_msg, "$PSTMENABLEPOSITIONHOLDERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set parameter in SW configuration block
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_swcfg_setpar( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  gnss_error_t err = GNSS_NO_ERROR;
  tInt config_type;
  tInt config_section;
  tInt config_param_id;
  tInt field_count;
  field_count = _clibs_sscanf( cmd_par, ",%1d%1d%2d", &config_type, &config_section, &config_param_id );

  if ( field_count == 3 )
  {
    cmd_par += 5;

    if ( config_section != 0 )
    {
      switch ( config_section )
      {
        case 1:
          {
            tInt value_int;
            tU8 value;
            field_count = _clibs_sscanf( cmd_par, ",%x", &value_int );
            value = value_int;

            if ( field_count == 1 )
            {
              sw_config_set_param( ( 100 + config_param_id ), &value, 0 );
            }
            else
            {
              err = GNSS_ERROR;
            }
          }
          break;

        case 2:
          {
            tInt value;
            tInt mode = 0;
            field_count = _clibs_sscanf( cmd_par, ",%x,%d", &value, &mode );

            if ( ( field_count == 1 ) || ( field_count == 2 ) )
            {
              sw_config_set_param( ( 200 + config_param_id ), &value, mode );
            }
            else
            {
              err = GNSS_ERROR;
            }
          }
          break;

        case 3:
          {
            tDouble value;
            field_count = _clibs_sscanf( cmd_par, ",%lf", &value );

            if ( field_count == 1 )
            {
              sw_config_set_param( ( 300 + config_param_id ), &value, 0 );
            }
            else
            {
              err = GNSS_ERROR;
            }
          }
          break;

        case 4:
          {
            tU32 p, h, v, g;
            sw_config_dops_t value;
            field_count = _clibs_sscanf( cmd_par, ",%u,%u,%u,%u", &p, &h, &v, &g );

            if ( field_count == 4 )
            {
              value.pdop = (tU8)p;
              value.hdop = (tU8)h;
              value.vdop = (tU8)v;
              value.gdop = (tU8)g;
              sw_config_set_param( ( 400 + config_param_id ), &value, 0 );
            }
            else
            {
              err = GNSS_ERROR;
            }
          }
          break;

        case 5:
          {
            if( _clibs_strchr( cmd_par, '*') == NULL)
            { // No checksum
              cmd_par[_clibs_strlen(cmd_par)-2]='\0';
            }
            else
            { // With checksum
              cmd_par[_clibs_strlen(cmd_par)-5]='\0';
            }
            sw_config_set_param( ( 500 + config_param_id ), ( cmd_par + 1 ), 0 );
          }
          break;

        default:
          {
#if defined( DR_CODE_LINKED )
            tInt value;
            tInt mode = 0;
            field_count = _clibs_sscanf( cmd_par, ",%x,%d", &value, &mode );
            if ( ( field_count == 1 ) || ( field_count == 2 ) )
            {
              err = sw_config_set_param( ( 600 + config_param_id ), &value, mode );
            }
            else
            {
              err = GNSS_ERROR;
            }
#else
            err = GNSS_ERROR;
#endif
          }
          break;
      }
    }
  }
  else
  {
    err = GNSS_ERROR;
  }

  if ( err == GNSS_NO_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETPAROK,%1d%1d%02d", config_type, config_section, config_param_id );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETPARERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief
 *
 * \param config_type tInt
 * \param config_section tInt
 * \param config_param_id tInt
 * \return void
 *
 ***********************************************/
static void nmea_swconfig_dumpsingleitem( tInt config_type, tInt config_section, tInt config_param_id )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt err = GNSS_NO_ERROR;

  if ( config_section != 0 )
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETPAR,%1d%1d%02d", config_type, config_section, config_param_id );

    switch ( config_section )
    {
      case 1:
        {
          tUInt value;

          if ( sw_config_get_param( config_type, ( 100 + config_param_id ), &value ) == GNSS_NO_ERROR )
          {
            index += _clibs_sprintf( &out_msg[index], ",0x%02x", ( 0xFF & value ) );
          }
          else
          {
            err = GNSS_ERROR;
          }
        }
        break;

      case 2:
        {
          tInt value;

          if ( sw_config_get_param( config_type, ( 200 + config_param_id ), &value ) == GNSS_NO_ERROR )
          {
            index += _clibs_sprintf( &out_msg[index], ",0x%08x", value );
          }
          else
          {
            err = GNSS_ERROR;
          }
        }
        break;

      case 3:
        {
          tDouble value;

          if ( sw_config_get_param( config_type, ( 300 + config_param_id ), &value ) == GNSS_NO_ERROR )
          {
            index += _clibs_sprintf( &out_msg[index], ",%e", value );
          }
          else
          {
            err = GNSS_ERROR;
          }
        }
        break;

      case 4:
        {
          sw_config_dops_t value;

          if ( sw_config_get_param( config_type, ( 400 + config_param_id ), &value ) == GNSS_NO_ERROR )
          {
            index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d", value.pdop, value.hdop, value.vdop, value.gdop );
          }
          else
          {
            err = GNSS_ERROR;
          }
        }
        break;

      case 5:
        {
          index += _clibs_sprintf( &out_msg[index], "," );

          if ( sw_config_get_param( config_type, ( 500 + config_param_id ), ( tChar * )( &out_msg[index] ) ) == GNSS_NO_ERROR )
          {
            index = _clibs_strlen( out_msg );
          }
          else
          {
            err = GNSS_ERROR;
          }
        }
        break;

      default:
        {
#if defined( DR_CODE_LINKED )
          tInt value;

          if ( sw_config_get_param( config_type, ( 600 + config_param_id ), &value ) == GNSS_NO_ERROR )
          {
            index += _clibs_sprintf( &out_msg[index], ",0x%08x", value );
          }
          else
          {
            err = GNSS_ERROR;
          }

#else
          err = GNSS_ERROR;
#endif
        }
        break;
    }
  }
  else
  {
    err = GNSS_ERROR;
  }

  if ( err == GNSS_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMGETPARERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief
 *
 * \param config_type tInt
 * \param config_section tInt
 * \param items tInt
 * \return void
 *
 ***********************************************/
static void nmea_swconfig_dumpallitems( tInt config_type, tInt config_section, tInt items )
{
  tInt i;

  if ( items > 100 )
  {
    items = 100;
  }

  for ( i = 0; i < items; i++ )
  {
    nmea_swconfig_dumpsingleitem( config_type, config_section, i );
  }
}

/********************************************//**
 * \brief   Get a parameter from SW config block
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_swcfg_getpar( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt config_type;
  tInt config_section;
  tInt config_param_id;
  tInt field_count;
  field_count = _clibs_sscanf( cmd_par, ",%1d%1d%2d", &config_type, &config_section, &config_param_id );

  if ( field_count == 3 )
  {
    if ( config_section != 0 )
    {
      nmea_swconfig_dumpsingleitem( config_type, config_section, config_param_id );
    }
    else
    {
      nmea_swconfig_dumpallitems( config_type, SW_CONFIG_SEC_1, SW_CONFIG_SEC_1_ITEMS );
      nmea_swconfig_dumpallitems( config_type, SW_CONFIG_SEC_2, SW_CONFIG_SEC_2_ITEMS );
      nmea_swconfig_dumpallitems( config_type, SW_CONFIG_SEC_3, SW_CONFIG_SEC_3_ITEMS );
      nmea_swconfig_dumpallitems( config_type, SW_CONFIG_SEC_4, SW_CONFIG_SEC_4_ITEMS );
      nmea_swconfig_dumpallitems( config_type, SW_CONFIG_SEC_5, SW_CONFIG_SEC_5_ITEMS );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMGETPARERROR" );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}
/********************************************//**
 * \brief   Dump SW config block
 *
 * \param   sw_config_type  type
 * \param   out_buffer      output buffer
 * \return  None
 *
 ***********************************************/
void nmea_swconfig_senddatablock( tInt sw_config_type, tChar *out_buffer )
{
  sw_config_t temp_sw_config;
  tInt index;

  if ( sw_config_data_block_copy( sw_config_type, &temp_sw_config ) != GNSS_ERROR )
  {
    tInt i, j, lines, extra_bytes, n_msg;
    tU8 *ptr;
    lines = ( sizeof( sw_config_t ) / 64 );
    extra_bytes = sizeof( sw_config_t ) % 64;
    n_msg = lines;

    if ( extra_bytes > 0 )
    {
      n_msg++;
    }

    ptr = ( tU8 * )&temp_sw_config;

    for ( i = 0; i < lines; i++ )
    {
      index = _clibs_sprintf( out_buffer, "$PSTMSWCONFIG,%d,%d,%d,", sw_config_type, i, n_msg );

      for ( j = 0; j < 64; j++ )
      {
        index += _clibs_sprintf( &out_buffer[index], "%02x", *ptr );
        ptr++;
      }

      index += _clibs_sprintf( &out_buffer[index], "*%02X\r\n", nmea_support_checksum( out_buffer ) );
      nmea_send_msg_to_uart( out_buffer, index );
    }

    index = _clibs_sprintf( out_buffer, "$PSTMSWCONFIG,%d,%d,%d,", sw_config_type, i, n_msg );

    for ( j = 0; j < extra_bytes; j++ )
    {
      index += _clibs_sprintf( &out_buffer[index], "%02x", *ptr );
      ptr++;
    }

    index += _clibs_sprintf( &out_buffer[index], "*%02X\r\n", nmea_support_checksum( out_buffer ) );
    nmea_send_msg_to_uart( out_buffer, index );
  }
  else
  {
    index = _clibs_sprintf( out_buffer, "$PSTMSWCONFIGERROR" );
    index += _clibs_sprintf( &out_buffer[index], "*%02X\r\n", nmea_support_checksum( out_buffer ) );
    nmea_send_msg_to_uart( out_buffer, index );
  }
}

/********************************************//**
 * \brief   Save SW config block to NVM
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_swcfg_savepar( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  gnss_error_t err = GNSS_NO_ERROR;
  err = sw_config_save_param();

  if ( err == GNSS_NO_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMSAVEPAROK" );
    gnssapp_swconfig_reload();
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSAVEPARERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Restore SW config block from NVM
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_swcfg_restorepar( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  gnss_error_t err = GNSS_NO_ERROR;
  err = sw_config_restore_param();

  if ( err == GNSS_NO_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMRESTOREPAROK" );
    gnssapp_swconfig_reload();
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMRESTOREPARERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Dump out SW config block
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_swcfg_getblock( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count;
  tInt config_type;
  field_count = _clibs_sscanf( cmd_par, ",%d", &config_type );

  if ( field_count == 1 )
  {
    nmea_swconfig_senddatablock( config_type, out_msg );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSWCONFIGERROR" );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}

/********************************************//**
 * \brief   Set constellation mask
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setconstmask( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tInt field_count;
  gnss_sat_type_mask_t constellation_mask;
  tInt slave_reset = 0;
  field_count = _clibs_sscanf( cmd_par, ",%d,%d", &constellation_mask, &slave_reset );

  if ( ( field_count == 2 ) && ( slave_reset == 2 ) )
  {
    nmea_dynamic_set_application_on_off( constellation_mask );
    gnss_dynamic_set_constellation_mask( constellation_mask );
    index = _clibs_sprintf( out_msg, "$PSTMSETCONSTMASKOK,%d,%d", constellation_mask, slave_reset );
  }
  else if ( field_count == 1 )
  {
    nmea_dynamic_set_application_on_off( constellation_mask );
    gnss_dynamic_set_constellation_mask( constellation_mask );
    index = _clibs_sprintf( out_msg, "$PSTMSETCONSTMASKOK,%d", constellation_mask );
  }
  else if ( field_count >= 1 )
  {
    boolean_t lowpower_status = FALSE;
    svc_pwr_get_lowpower_allowed( &lowpower_status);
    svc_pwr_set_lowpower_allowed( FALSE);
    gnssapp_suspend();
    gnss_set_constellation_mask( constellation_mask );
    gnss_set_constellation_usage_mask( constellation_mask );

    if ( slave_reset == 1 )
    {
      platform_gnss_suspend();
      platform_gnss_restart();
    }

    gnssapp_restart();
    svc_pwr_set_lowpower_allowed( lowpower_status);
    index = _clibs_sprintf( out_msg, "$PSTMSETCONSTMASKOK,%d", constellation_mask );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSETCONSTMASKERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Write a time delta to RTC
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_rtcwrite( tChar *cmd_par )
{
  tInt field_count = 0;
  tInt rtc_delta;
  tUInt current_RTC;
  field_count = _clibs_sscanf( cmd_par, ",%d", &rtc_delta/*s*/ );

  if ( field_count == 1 )
  {
    rtc_drv_rtc_read( &current_RTC );
    current_RTC += rtc_delta;

    if ( rtc_drv_rtc_write( current_RTC ) )
    {
      GPS_DEBUG_MSG( ( "\r\n[rtc][test]: RTC write error \r\n" ) );
    }
    else
    {
      GPS_DEBUG_MSG( ( "\r\n[rtc][test]: RTC write %d \r\n", current_RTC ) );
    }
  }
}
/*}}}  */

#if defined( NMEA_NOTCH_SUPPORT )
/********************************************//**
 * \brief   Parse parameters for notch command
 *
 * \param[in]   cmd_par         Parameter string
 * \param[out]  notch_sat_type  tInt*
 * \param[out]  notch_status    tInt*
 * \param[out]  notch_frequency tInt*
 * \return tInt
 *
 ***********************************************/
static tInt nmea_cmdif_notch_parse_params( tChar *cmd_par, tInt *notch_sat_type, tInt *notch_status, tInt * notch_frequency, tShort* notch_kbw_gross, tShort* notch_kbw_fine, tInt* notch_threshold )
{
  /*{{{  decs*/
  tInt field_count = 0;
  tInt status;
  tInt sat_type_status;
  tInt frequency;
  tInt kbw_gross;
  tInt kbw_fine;
  tInt threshold;
  /*}}}  */
  *notch_sat_type     = -1;
  *notch_status       = -1;
  *notch_frequency    = 0;
  *notch_kbw_gross = 0;
  *notch_kbw_fine = 0;
  *notch_threshold = 0;

  field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d", &sat_type_status, &status, &frequency, &kbw_gross, &kbw_fine, &threshold );

  if ( field_count == 2 )
  {
    if ( ( ( status >= 0 ) && ( status < 3 ) ) && ( ( sat_type_status >= 0 ) && ( sat_type_status < 2 ) ) )
    {
      *notch_status = status;
      *notch_sat_type = sat_type_status;
      return ( GNSS_NO_ERROR );
    }
  }
  else if (( field_count >= 3 ) &&  ( field_count <= 6 ))
  {
    if ( ( ( status >= 0 ) && ( status < 3 ) ) && ( ( sat_type_status >= 0 ) && ( sat_type_status < 2 ) )  && ( frequency >= 0 ) ) /* check on params  */
    {
      tDouble kfreq_init ;
      *notch_status = status;
      *notch_sat_type = sat_type_status;


      if(field_count == 6)   /*complete parameters setting*/
      {
        *notch_kbw_gross = (tShort)kbw_gross;
        *notch_kbw_fine  = (tShort)kbw_fine;
        *notch_threshold = threshold;
      }


      if ( sat_type_status == 0 )
      {
        kfreq_init =  -1 * cos( 2 * ( PI ) * ( tDouble )frequency / ( 16 * 1023000.0 ) ) ;
        *notch_frequency = ( tInt )( kfreq_init * ( 8 * 1024 * 1024 ) ); /*(cos(2*pi*wnotch)/16*1023000)*2^23) , GPS*/
      }
      else
      {
        kfreq_init =  -1 * cos( 2 * ( PI ) * ( tDouble )frequency / ( 32 * 1023000.0 ) ) ;
        *notch_frequency = ( tInt )( kfreq_init * ( 8 * 1024 * 1024 ) ); /*(cos(2*pi*wnotch)/32*1023000)*2^23) , GLonass*/
      }

      GPS_DEBUG_MSG( ( "Notch Freq is %d, Freq Init set to %d for Branch %d, Ext[KBW_GROSS,KBW_FINE, Thr [0 -> Default]: %d,%d,%d\r\n", frequency, *notch_frequency, sat_type_status,kbw_gross,kbw_fine,threshold  ) );
      return ( NMEA_OK );
    }
  }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

  return ( NMEA_ERROR );
}
#endif

#if defined( NMEA_SQI_DATASTORAGE )
/********************************************//**
 * \brief   Erase user data region in SQI
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_sqids_erase( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tU32 dest_addr;
  tU8 spm_conf;
  gpOS_error_t error;

  dest_addr = SQI_START_ADDR + NMEA_SQIDS_USERREGIONOFFSET; // + sector_start_addr;

  // Check if Software Protection Mode is enabled
  error = svc_sqi_get_sw_protection( &spm_conf);

  if(( error == gpOS_SUCCESS) && ( spm_conf != 0))
  {
    svc_sqi_set_sw_protection( 0);
  }

  if ( svc_sqi_erase( ( void * )dest_addr, NMEA_SQIDS_USERREGIONSIZE, TRUE ) == gpOS_SUCCESS )
  {
    index = _clibs_sprintf( out_msg, "$PSTMSQIERASEOK" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSQIERASEERROR" );
  }

  // Check if Software Protection Mode is enabled
  if( spm_conf)
  {
    svc_sqi_set_sw_protection( spm_conf);
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Erase user data region in SQI
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_sqids_sector_erase( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tU32 dest_addr;
  tU8 spm_conf;
  tInt field_count = 0;
  tUInt offset;
  tUInt mem_size;
  gpOS_error_t error;

  field_count = _clibs_sscanf( cmd_par, ",0x%x,0x%x", &offset, &mem_size);

  if ( field_count == 2 )
  {
    if((offset + mem_size) <= NMEA_SQIDS_USERREGIONSIZE)
    {
      dest_addr = SQI_START_ADDR + NMEA_SQIDS_USERREGIONOFFSET + offset;

      // Check if Software Protection Mode is enabled
      error = svc_sqi_get_sw_protection( &spm_conf);

      if(( error == gpOS_SUCCESS) && ( spm_conf != 0))
      {
        svc_sqi_set_sw_protection( 0);
      }

      if ( svc_sqi_erase( ( void * )dest_addr, mem_size, TRUE ) == gpOS_SUCCESS )
      {
        index = _clibs_sprintf( out_msg, "$PSTMSQISECTORERASEOK" );
      }
      else
      {
        index = _clibs_sprintf( out_msg, "$PSTMSQISECTORERASEERROR" );
      }

      // Check if Software Protection Mode is enabled
      if( spm_conf)
      {
        svc_sqi_set_sw_protection( spm_conf);
      }
    }
    else
    {
      index = _clibs_sprintf( out_msg, "$PSTMSQISECTORERASEERROR" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSQISECTORERASEERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Read from user data region in SQI
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_sqids_get( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  tU32 dest_addr;
  tInt field_count = 0;
  tUInt offset;
  field_count = _clibs_sscanf( cmd_par, ",0x%x", &offset );

  if ( ( field_count == 1 ) && ( offset <= (NMEA_SQIDS_USERREGIONSIZE -  NMEA_SQIDS_BUFFER_SIZE)) )
  {
    dest_addr = SQI_START_ADDR + NMEA_SQIDS_USERREGIONOFFSET + offset;
    index = _clibs_sprintf( out_msg, "$PSTMSQIGETOK,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x", dest_addr,
                            ( ( tU32 * )dest_addr )[0], ( ( tU32 * )dest_addr )[1], ( ( tU32 * )dest_addr )[2], ( ( tU32 * )dest_addr )[3],
                            ( ( tU32 * )dest_addr )[4], ( ( tU32 * )dest_addr )[5], ( ( tU32 * )dest_addr )[6], ( ( tU32 * )dest_addr )[7] );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSQIGETERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Write to user data region in SQI
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_sqids_set( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tU32 cnt;
  tInt index;
  tInt field_count = 0;
  tUInt offset;
  tU32 data[8];
  tU32 *nmea_sqids_databuf_ptr = ( tU32 * )nmea_sqids_databuf;
  tU32 dest_addr;
  tU8 spm_conf;
  gpOS_error_t error;

  field_count = _clibs_sscanf( cmd_par, ",0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x",
                               &offset, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7] );

  if ( ( field_count == 9 ) && ( offset <= (NMEA_SQIDS_USERREGIONSIZE -  NMEA_SQIDS_BUFFER_SIZE) ) )
  {
    dest_addr = SQI_START_ADDR + NMEA_SQIDS_USERREGIONOFFSET + offset; //sector_start_addr + offset;

    for ( cnt = 0; cnt < ( nmea_sqids_databuf_size / 4 ); cnt++ )
    {
      nmea_sqids_databuf_ptr[cnt] = data[cnt];
    }

    // Check if Software Protection Mode is enabled
    error = svc_sqi_get_sw_protection( &spm_conf);

    if(( error == gpOS_SUCCESS) && ( spm_conf != 0))
    {
      svc_sqi_set_sw_protection( 0);
    }

    if ( svc_sqi_write( ( void * )dest_addr, ( void * )nmea_sqids_databuf_ptr, nmea_sqids_databuf_size ) == gpOS_FAILURE )
    {
      //GPS_DEBUG_MSG(("\r\n[sqi][write]: error \r\n"));
      index = _clibs_sprintf( out_msg, "$PSTMSQISETERROR" );
    }
    else
    {
      index = _clibs_sprintf( out_msg, "$PSTMSQISETOK,0x%x", dest_addr );
    }

    // Check if Software Protection Mode is enabled
    if( spm_conf)
    {
      svc_sqi_set_sw_protection( spm_conf);
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMSQISETERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}
#endif

/********************************************//**
 * \brief   Dump all registers of frontend
 *
 * \param   cmd_par   Command parameters
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_fedump( tChar *cmd_par )
{
  tInt index;
  FE_reg_item_t reg_table[30];
  tInt fe_selector = 0;  // 0 = internal (default)  1 = external
  LLD_ErrorTy FE_error;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  fe_selector = 0;

  if ( ( fe_selector == 0 ) || ( fe_selector == 1 ) )
  {
    index = _clibs_sprintf( out_msg, "$PSTMFEDUMP" );

#if defined( NMEA_REMOTE_FRONTEND_SUPPORT )
    fe_regs_size = sizeof( reg_table );

    if ( remote_fe_dump( reg_table, &fe_regs_size ) == GNSS_NO_ERROR )
#else
    FE_error = FE_dump_regs( reg_table );

    if ( FE_error == LLD_NO_ERROR )
#endif
    {
      tU32 cnt = 0;

      while ( reg_table[cnt].addr != 0xff )
      {
        index += _clibs_sprintf( &out_msg[index], ",%02x,%02x", reg_table[cnt].addr, reg_table[cnt].data );
        cnt++;

        if ( ( cnt & 0x7 ) == 0 )
        {
          index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
          nmea_send_msg_to_uart( out_msg , index );
          index = _clibs_sprintf( out_msg, "$PSTMFEDUMP" );
        }
      }
    }
    else
    {
      index += _clibs_sprintf( &out_msg[index], ",KO" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMFEDUMPERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg , index );
}

/********************************************//**
 * \brief   Write a value in a frontend register
 *
 * \param   msg     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_fewrite( tChar *cmd_par )
{
#if defined( NMEA_FRONTEND_SUPPORT )
  tInt field_count = 0;
  tUInt addr, data;
  tInt fe_selector = 0;  // 0 = internal (default)  1 = external
  LLD_ErrorTy lld_error = LLD_ERROR;
  field_count = _clibs_sscanf( cmd_par, ",0x%x,0x%x,%d", &addr, &data, &fe_selector );

  if ( field_count == 2 )
  {
    lld_error = FE_write_data( addr, data );
  }
  else if ( field_count == 3 )
  {
    if ( fe_selector == 0 )
    {
      lld_error = FE_write_data( addr, data );
    }

  }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

  if ( lld_error == LLD_ERROR )
  {
    GPS_DEBUG_MSG( ( "\r\n[fe][write]: error \r\n" ) );
  }
  else
  {
    GPS_DEBUG_MSG( ( "\r\n[fe][write]: 0x%x = 0x%x (FE=%d)\r\n", addr, data, fe_selector ) );
  }

#endif
}

/********************************************//**
 * \brief   Switch NMEA output on off
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setnmea( tChar *cmd_par )
{
  tInt field_count, on_off, config;
  field_count = _clibs_sscanf( cmd_par, ",%d,%02x", &on_off, &config );

  if ( field_count >= 1 )
  {
    if ( on_off != 0 )
    {
      if ( nmea_outmsg_enabled == FALSE )
      {
        nmea_outmsg_enabled = TRUE;

        if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ) )
        {
          gnss_events_install( nmea_outmsg_syncevent_id, nmea_outmsg2_synchdlr_ptr );
        }

        gnss_events_install( nmea_outmsg_syncevent_id, nmea_outmsg_synchdlr_ptr );
      }
    }
    else
    {
      if ( nmea_outmsg_enabled == TRUE )
      {
        nmea_outmsg_enabled = FALSE;

        if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ) )
        {
          gnss_events_uninstall( nmea_outmsg_syncevent_id, nmea_outmsg2_synchdlr_ptr );
        }

        gnss_events_uninstall( nmea_outmsg_syncevent_id, nmea_outmsg_synchdlr_ptr );
      }
    }

    if ( field_count == 2 )
    {
      nmea_on_debug_setting = config;

      if ( ( nmea_on_debug_setting & 0x1 ) != 0 )
      {
        gnss_debug_set_status( GNSS_DEBUG_OFF );
      }
      else
      {
        gnss_debug_set_status( GNSS_DEBUG_ON );
      }
    }
  }
  else // this is for backward compatibility
  {
    if ( nmea_outmsg_enabled == TRUE )
    {
      nmea_outmsg_enabled = FALSE;

      if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ) )
      {
        gnss_events_uninstall( nmea_outmsg_syncevent_id, nmea_outmsg2_synchdlr_ptr );
      }

      gnss_events_uninstall( GNSS_EVENTID_FIXREADY, nmea_outmsg_synchdlr_ptr );
    }
    else
    {
      if ( ( NMEA_msg_list_2[0] != 0 ) || ( NMEA_msg_list_2[1] != 0 ) )
      {
        gnss_events_install( nmea_outmsg_syncevent_id, nmea_outmsg2_synchdlr_ptr );
      }

      gnss_events_install( GNSS_EVENTID_FIXREADY, nmea_outmsg_synchdlr_ptr );
      nmea_outmsg_enabled = TRUE;
    }
  }
}

/********************************************//**
 * \brief   Switch debug output on off
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_setdebug( tChar *cmd_par )
{
  tInt field_count, on_off;
  field_count = _clibs_sscanf( cmd_par, ",%d", &on_off );

  if ( field_count == 1 )
  {
    if ( on_off != 0 )
    {
      in_out_open_port( IN_OUT_DEBUG_PORT );
      gnss_debug_set_status( GNSS_DEBUG_ON );
    }
    else
    {
      gnss_debug_set_status( GNSS_DEBUG_OFF );
      in_out_close_port( IN_OUT_DEBUG_PORT );
    }
  }
}

/********************************************//**
 * \brief   Select datum
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_datum_select( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt field_count = 0;
  tInt datum;
  tInt index = 0;
  datum_code_t datum_local;
  field_count = _clibs_sscanf( cmd_par, ",%d", &datum );

  if ( field_count == 1 )
  {
    switch ( datum )
    {
      case 0:
        datum_local = DATUM_CODE_WGS84;
        break;

      case 1:
        datum_local = DATUM_CODE_TOY_M;
        break;

      case 2:
        datum_local = DATUM_CODE_OGB_M;
        break;

      default:
        datum_local = DATUM_CODE_WGS84;
        datum = 0;
        break;
    }

    if ( GNSS_NO_ERROR == datum_select( datum_local ) )
    {
      index = _clibs_sprintf( out_msg, "$PSTMDATUMSELECTOK,%1d", datum );
    }
    else
    {
      index = _clibs_sprintf( out_msg, "$PSTMDATUMSELECTERROR" );
    }
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMDATUMSELECTERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Set datum parameter
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_datum_set_param( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt field_count = 0;
  tInt index = 0;
  datum_local_to_WGS84_param_t user_datum;
  nmea_error_t datum_error = NMEA_OK;
  field_count = _clibs_sscanf( cmd_par, ",%lf,%lf,%lf,%lf,%lf", &( user_datum.delta_x ), &( user_datum.delta_y ),
                               &( user_datum.delta_z ), &( user_datum.delta_a ), &( user_datum.delta_f )
                             );

  if ( field_count == 5 )
  {
    if ( GNSS_ERROR == datum_set_by_user( user_datum ) )
    {
      datum_error = NMEA_ERROR;
    }
  }
  else
  {
    datum_error = NMEA_ERROR;
  }

  if ( datum_error == NMEA_OK )
  {
    index = _clibs_sprintf( out_msg, "$PSTMDATUMSETPARAMOK" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMDATUMSETPARAMERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Process for secondary output messages stream
 *
 * \param   p   Task parameter (NULL)
 * \return  None
 *
 ***********************************************/
static gpOS_task_exit_status_t nmea_outmsg2_process( void *p )
{
  boolean_t exit_flag = FALSE;

  while ( exit_flag == FALSE )
  {
    nmea_outmsg2_transmit_after_fix();
  }

  // should never reach this
  return -1;
}

/********************************************//**
 * \brief   Transmit
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg2_transmit_after_fix( void )
{
  tInt week_n;
  gpOS_clock_t fix_clock;
  tDouble tow;
  gnss_time_t gnss_time = {0};
  gnss_time_t utc_time;
  tUInt msg_list[2];

  while ( TRUE )
  {
    gnss_events_wait( GNSS_EVENTID_FIXREADY, nmea_outmsg2_synchdlr_ptr );
    gnss_fix_store_local( nmea_outmsg_fixdata_ptr );
    gnss_fix_get_time_local( &week_n, &tow, &fix_clock, nmea_outmsg_fixdata_ptr );
    gnss_time.week_n = week_n;
    gnss_time.tow = tow;
    utc_time = gnss_time_to_utc_time( gnss_time, gnss_fix_get_time_sat_type_local( nmea_outmsg_fixdata_ptr ) );
    msg_list[0] = NMEA_msg_list_2[0] | NMEA_on_debug_msg_list_2[0];
    msg_list[1] = NMEA_msg_list_2[1] | NMEA_on_debug_msg_list_2[1];
    gpOS_task_delay_until( gpOS_time_plus( fix_clock, nmea_outmsg_delaytonextfix ) );
    nmea_outmsg_transmit( msg_list, nmea_outmsg_fixdata_ptr, 0, utc_time );
  }
}

/********************************************//**
 * \brief   Send TM message
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TM( void * data_p )
{
  /*{{{  decs*/
  tInt index     = 0;
  tInt week;
  gpOS_clock_t cpu_time;
  gpOS_clock_t requested_cpu_time;
  gpOS_clock_t fix_available_cpu_time;
  gpOS_clock_t get_measure_data_cpu_time;
  gpOS_clock_t sending_cpu_time;
  tDouble tow;
  time_validity_t time_validity;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tDouble clock_drift;
  tInt noise_floor;
  tDouble clock_offset;
  /*}}}  */
  gnss_fix_get_time_local( &week, &tow, &cpu_time, data_p );
  requested_cpu_time = gnss_fix_get_measure_requested_time_local( data_p );
  get_measure_data_cpu_time = gnss_fix_get_measure_get_data_time_local( data_p );
  fix_available_cpu_time = gnss_fix_get_fix_available_time_local( data_p );
  time_validity = gnss_time_get_validity( gnss_time_get_master() );
  sending_cpu_time = nmea_outmsg_gga_timestamp;
  clock_drift = gnss_fix_get_clock_drift_local( data_p );
  noise_floor = gnss_get_noise_floor_raw( GNSS_SAT_TYPE_GPS );
  clock_offset = gnss_fix_get_clock_offset_local( data_p );
  index = 0;
  index += _clibs_sprintf( out_msg, "$PSTMTM" );
  index += _clibs_sprintf( &out_msg[index], ",%04d", week );
  index += _clibs_sprintf( &out_msg[index], ",%011.4f", tow );
  index += _clibs_sprintf( &out_msg[index], ",%10d", cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%2d", time_validity );
  index += _clibs_sprintf( &out_msg[index], ",%10d", requested_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", sending_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", get_measure_data_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", fix_available_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", ( tInt )gpOS_time_minus( fix_available_cpu_time, get_measure_data_cpu_time ) );
  index += _clibs_sprintf( &out_msg[index], ",%10d", ( tInt )gpOS_time_minus( sending_cpu_time, fix_available_cpu_time ) );
  index += _clibs_sprintf( &out_msg[index], ",%10d", ( tInt )gpOS_time_minus( cpu_time, requested_cpu_time ) );
  index += _clibs_sprintf( &out_msg[index], ",%10d", ( tInt )gpOS_time_minus( sending_cpu_time, cpu_time ) );
  index += _clibs_sprintf( &out_msg[index], ",%06.2f", clock_drift );
  index += _clibs_sprintf( &out_msg[index], ",%06d", noise_floor );
  index += _clibs_sprintf( &out_msg[index], ",%06.2f", clock_offset );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, TM_NMEA_MSG );
}

/********************************************//**
 * \brief   Send NOISE message
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_NOISE( void * data_p )
{
  /*{{{  decs*/
  tInt index     = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  index = _clibs_sprintf( out_msg, "$PSTMNOISE" );
  index += _clibs_sprintf( &out_msg[index], ",%d", gnss_get_noise_floor_raw( GNSS_SAT_TYPE_GPS ) );
  index += _clibs_sprintf( &out_msg[index], ",%d", gnss_get_noise_floor_raw( GNSS_SAT_TYPE_GLONASS ) );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, NOISE_NMEA_MSG );
}

/********************************************//**
 * \brief   Execute a PPS interface command
 *
 * \param   cmd_par     Parameters string
 * \return  None
 *
 ***********************************************/
void nmea_cmdif_exec_ppsif( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt field_count = 0;
  tInt index = 0;
  tInt cmd_mode;
  tInt cmd_type;
  gnss_error_t error = GNSS_NO_ERROR;
  field_count = _clibs_sscanf( cmd_par, ",%d,%d", &cmd_mode, &cmd_type );

  if ( field_count == 2 )
  {
    if ( cmd_mode == NMEA_PPSIF_CMDMODE_GET )
    {
      switch ( cmd_type )
      {
        case NMEA_PPSIF_CMDID_PULSE_DATA:
          {
            tDouble time_delay = gnss_pps_get_time_delay() * 1E9;
            time_delay = ( time_delay >= 0 ) ? ( time_delay + 0.5 ) : ( time_delay - 0.5 );
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d,%d,%d,%d,%lf,%d",
                                    cmd_mode,
                                    cmd_type,
                                    gnss_pps_get_output_mode(),
                                    gnss_pps_get_reference_time(),
                                    ( tShort )time_delay,
                                    gnss_pps_get_pulse_duration(),
                                    gnss_pps_get_polarity()
                                  );
          }
          break;

        case NMEA_PPSIF_CMDID_TIMING_DATA:
          {
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d,%d,%d,%d,%08x,%d,%d",
                                    cmd_mode,
                                    cmd_type,
                                    gnss_pps_get_fix_condition(),
                                    gnss_pps_get_sat_threshold(),
                                    gnss_pps_get_elevation_mask(),
                                    gnss_pps_get_constellation_mask(),
                                    ( tShort )( (gnss_pps_get_rf_compensation( GNSS_SAT_TYPE_GPS ) * 1E9) + 0.5 ),
                                    ( tShort )( (gnss_pps_get_rf_compensation( GNSS_SAT_TYPE_GLONASS ) * 1E9) + 0.5 )
                                  );
          }
          break;

        case NMEA_PPSIF_CMDID_POSITION_HOLD_DATA:
          {
            tDouble lat, lon, height;
            tInt    lat_deg;
            tInt    lat_min;
            tInt    lat_min_frac;
            tInt    lon_deg;
            tInt    lon_min;
            tInt    lon_min_frac;
            tChar   lon_sense_ch;
            tChar   lat_sense_ch;
            tChar   format[64];
            gnss_pps_get_position_hold_llh_pos( &lat, &lon, &height );
            nmea_support_degrees_to_int( lat, 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
            nmea_support_degrees_to_int( lon, 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
            _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d", cmd_mode, cmd_type );
            index += _clibs_sprintf( &out_msg[index], ",%d", gnss_pps_get_position_hold_status() );
            index += _clibs_sprintf( &out_msg[index], format, lat_deg, lat_min, lat_min_frac, lat_sense_ch, lon_deg, lon_min, lon_min_frac, lon_sense_ch );
            index += _clibs_sprintf( &out_msg[index], ",%06.2f", height );
          }
          break;

        case NMEA_PPSIF_CMDID_TRAIM:
          {
            timing_data_t timing_data;
            gnss_pps_get_timing_data( &timing_data );
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d,%d,%d,%d,%d,%d", cmd_mode, cmd_type, timing_data.traim_enabled, timing_data.traim_data.traim_solution, timing_data.traim_data.ave_error, timing_data.traim_data.used_sat_id, timing_data.traim_data.removed_sat_id );
          }
          break;

        case NMEA_PPSIF_CMDID_TRAIM_USED:
          {
            timing_data_t timing_data;
            gnss_pps_get_timing_data( &timing_data );
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d,%d,%d", cmd_mode, cmd_type, timing_data.traim_enabled, timing_data.traim_data.used_sat_id );
            {
              tInt i;

              for ( i = 0; i < timing_data.traim_data.used_sat_id; i++ )
              {
                index += _clibs_sprintf( &out_msg[index], ",%d", timing_data.traim_data.used_sat_id_table[i] );
              }
            }
          }
          break;

        case NMEA_PPSIF_CMDID_TRAIM_RES:
          {
            timing_data_t timing_data;
            gnss_pps_get_timing_data( &timing_data );
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d,%d,%d", cmd_mode, cmd_type, timing_data.traim_enabled, timing_data.traim_data.used_sat_id );
            {
              tInt i;

              for ( i = 0 ; i < timing_data.traim_data.used_sat_id; i++ )
              {
                index += _clibs_sprintf( &out_msg[index], ",%d", timing_data.traim_data.residual[i] );
              }
            }
          }
          break;

        case NMEA_PPSIF_CMDID_TRAIM_REMOVED:
          {
            timing_data_t timing_data;
            gnss_pps_get_timing_data( &timing_data );
            index = _clibs_sprintf( out_msg, "$PSTMPPS,%d,%d,%d,%d", cmd_mode, cmd_type, timing_data.traim_enabled, timing_data.traim_data.removed_sat_id );
            {
              tInt i;

              for ( i = 0; i < timing_data.traim_data.removed_sat_id; i++ )
              {
                index += _clibs_sprintf( &out_msg[index], ",%d", timing_data.traim_data.removed_sat_id_table[i] );
              }
            }
          }
          break;
      }
    }
    else if ( cmd_mode == NMEA_PPSIF_CMDMODE_SET )
    {
      switch ( cmd_type )
      {
        case NMEA_PPSIF_CMDID_ON_OFF:
          {
            tUInt on_off;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &on_off );

            if ( field_count == 3 )
            {
              gnss_pps_set_signal_on_off_status( ( boolean_t )on_off );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_OUT_MODE:
          {
            tUInt out_mode;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &out_mode );

            if ( field_count == 3 )
            {
              gnss_pps_set_output_mode( ( pps_output_mode_t )out_mode );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_REFERENCE_CONSTELLATION:
          {
            tUInt ref_const;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &ref_const );

            if ( field_count == 3 )
            {
              gnss_pps_set_reference_constellation( ( gnss_sat_type_t )ref_const );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_REFERENCE_TIME:
          {
            tUInt ref_time;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &ref_time );

            if ( field_count == 3 )
            {
              gnss_pps_set_reference_time( ( pps_reference_time_t )ref_time );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_PULSE_DELAY:
          {
            tInt time_delay;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &time_delay );

            if ( field_count == 3 )
            {
              gnss_pps_set_time_delay( ( tDouble )time_delay * 1E-9 );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_PULSE_DURATION:
          {
            tDouble pulse_duration;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%lf", &cmd_mode, &cmd_type, &pulse_duration );

            if ( field_count == 3 )
            {
              gnss_pps_set_pulse_duration( pulse_duration );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_PULSE_POLARITY:
          {
            tUInt pulse_polarity;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &pulse_polarity );

            if ( field_count == 3 )
            {
              gnss_pps_set_polarity( pulse_polarity );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_PULSE_DATA:
          {
            tDouble pulse_duration;
            tInt plartity_inversion, out_mode, ref_time, pulse_delay;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%lf,%d", &cmd_mode, &cmd_type, &out_mode, &ref_time, &pulse_delay, &pulse_duration, &plartity_inversion );

            if ( field_count == 7 )
            {
              gnss_pps_set_output_mode( ( pps_output_mode_t )out_mode );
              gnss_pps_set_reference_time( ( pps_reference_time_t )ref_time );
              gnss_pps_set_time_delay( ( tDouble )( pulse_delay * 1E-9 ) );
              gnss_pps_set_pulse_duration( pulse_duration );
              gnss_pps_set_polarity( ( plartity_inversion > 0 ) );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_FIX_CONDITION:
          {
            tUInt fix_condition;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &fix_condition );

            if ( field_count == 3 )
            {
              gnss_pps_set_fix_condition( ( fix_status_t )fix_condition );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_SAT_TRHESHOLD:
          {
            tUInt sat_th;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &sat_th );

            if ( field_count == 3 )
            {
              gnss_pps_set_sat_threshold( ( tU8 )sat_th );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_CONSTELLATION_RF_DELAY:
          {
            tUInt sat_type;
            tInt rf_compensation;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d", &cmd_mode, &cmd_type, &sat_type, &rf_compensation );

            if ( field_count == 4 )
            {
              gnss_pps_set_rf_compensation( ( gnss_sat_type_t )sat_type, ( tDouble )rf_compensation * 1E-9 );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_ELEVATION_MASK:
          {
            tInt elevation_mask;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &elevation_mask );

            if ( field_count == 3 )
            {
              gnss_pps_set_elevation_mask( elevation_mask );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_COSTELLATION_MASK:
          {
            tUInt const_mask;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &const_mask );

            if ( field_count == 3 )
            {
              gnss_pps_set_constellation_mask( const_mask );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_TIMING_DATA:
          {
            tUInt fix_status;
            tUInt sat_th, const_mask;
            tInt elevation_mask;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d", &cmd_mode, &cmd_type, &fix_status, &sat_th, &elevation_mask, &const_mask );

            if ( field_count == 6 )
            {
              gnss_pps_set_fix_condition( ( fix_status_t )fix_status );
              gnss_pps_set_sat_threshold( ( tU8 )sat_th );
              gnss_pps_set_elevation_mask( elevation_mask );
              gnss_pps_set_constellation_mask( const_mask );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_POSITION_HOLD_DATA:
          {
            tUInt on_off;
            tDouble lat, lat_deg, lat_min;
            tDouble lon, lon_deg, lon_min;
            tChar lat_sense_ch, lon_sense_ch;
            tDouble height_val;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%2lf%10lf,%1c,%3lf%10lf,%1c,%10lf", &cmd_mode, &cmd_type, &on_off, &lat_deg, &lat_min, &lat_sense_ch, &lon_deg, &lon_min, &lon_sense_ch, &height_val );

            if ( field_count == 10 )
            {
              if ( on_off != 0 )
              {
                if ( ( lat_deg > 90.0 )        || ( lat_deg < 0.0 )     ||
                     ( lat_min >= 60.0 )       || ( lat_min < 0.0 )     ||
                     ( lon_deg > 180.0 )       || ( lon_deg < 0.0 )     ||
                     ( lon_min >= 60.0 )       || ( lon_min < 0.0 )     ||
                     ( height_val > 100000.0 ) || ( height_val < -1500.0 )  ||
                     ( ( lat_sense_ch != 'N' )  && ( lat_sense_ch != 'S' ) ) ||
                     ( ( lon_sense_ch != 'W' )  && ( lon_sense_ch != 'E' ) )
                   )
                {
                  error = GNSS_ERROR;
                }

                lat = lat_deg + (lat_min / 60.0);
                lon = lon_deg + (lon_min / 60.0);

                if ( lat_sense_ch == 'S' )
                {
                  lat = -lat;
                }

                if ( lon_sense_ch == 'W' )
                {
                  lon = -lon;
                }

                if ( ( lat > 90.0 ) || ( lat < ( -90.0 ) ) )
                {
                  error = GNSS_ERROR;
                }

                if ( ( lon > 180.0 ) || ( lon < ( -180.0 ) ) )
                {
                  error = GNSS_ERROR;
                }

                if ( error == GNSS_NO_ERROR )
                {
                  gnss_pps_set_position_hold_llh_pos( lat, lon, height_val );
                  gnss_pps_set_position_hold_status( ( boolean_t )on_off );
                }
              }
              else
              {
                gnss_pps_set_position_hold_status( ( boolean_t )on_off );
              }
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_AUTO_HOLD_SAMPLES:
          {
            tUInt samples;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d", &cmd_mode, &cmd_type, &samples );

            if ( field_count == 3 )
            {
              gnss_pps_set_auto_hold_samples( samples );
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;

        case NMEA_PPSIF_CMDID_TRAIM:
          {
            tUInt on_off;
            tDouble alarm;
            field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%lf", &cmd_mode, &cmd_type, &on_off, &alarm );

            if ( field_count == 4 )
            {
              if ( on_off != 0 )
              {
                gnss_pps_enable_traim( alarm );
              }
              else
              {
                gnss_pps_disable_traim();
              }
            }
            else
            {
              error = GNSS_ERROR;
            }
          }
          break;
      }

      if ( error == GNSS_NO_ERROR )
      {
        index = _clibs_sprintf( out_msg, "$PSTMPPSOK,%d,%d", cmd_mode, cmd_type );
      }
    }
    else
    {
      error = GNSS_ERROR;
    }
  }
  else
  {
    error = GNSS_ERROR;
  }

  if ( ( error == GNSS_ERROR ) || ( index == 0 ) )
  {
    index = _clibs_sprintf( out_msg, "$PSTMPPSERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Execute notch interface commands
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void nmea_cmdif_exec_notch( tChar *cmd_par )
{
#if defined( NMEA_NOTCH_SUPPORT )
  tInt notch_sat_type, notch_status, notch_frequency;
  tShort kbw_gross;
  tShort kbw_fine;
  tInt  threshold;

  if ( ( nmea_cmdif_notch_parse_params( cmd_par, &notch_sat_type, &notch_status, &notch_frequency,&kbw_gross,&kbw_fine,&threshold ) == NMEA_OK ) )
  {
    tInt starting_frequency = notch_frequency;

    if ( notch_status == 1 )
    {
      if ( notch_sat_type == 0 )
      {
        gnss_notch_filter_ext_enable( GNSS_SAT_TYPE_GPS, starting_frequency, 1 , kbw_gross,kbw_fine,threshold );
        GPS_DEBUG_MSG( ( "\r\n\n\n NOTCH FILTER GPS status: ON \r\n\n\n" ) );
      }
      else if ( notch_sat_type == 1 )
      {
        gnss_notch_filter_ext_enable( GNSS_SAT_TYPE_GLONASS, starting_frequency, 1 , kbw_gross,kbw_fine,threshold );
        GPS_DEBUG_MSG( ( "\r\n\n\n NOTCH FILTER GLONASS status: ON \r\n\n\n" ) );
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    }

    if ( notch_status == 2 )
    {
      if ( notch_sat_type == 0 )
      {
        gnss_notch_filter_ext_enable( GNSS_SAT_TYPE_GPS, starting_frequency, 2 , kbw_gross,kbw_fine,threshold );
        GPS_DEBUG_MSG( ( "\r\n\n\n NOTCH FILTER GPS AUTO status: ON \r\n\n\n" ) );
      }
      else if ( notch_sat_type == 1 )
      {
        gnss_notch_filter_ext_enable( GNSS_SAT_TYPE_GLONASS, starting_frequency, 2 , kbw_gross,kbw_fine,threshold );
        GPS_DEBUG_MSG( ( "\r\n\n\n NOTCH FILTER GLONASS AUTO status: ON \r\n\n\n" ) );
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    }

    if ( notch_status == 0 ) /* disable a branch*/
    {
      if ( notch_sat_type == 0 )
      {
        gnss_notch_filter_disable( GNSS_SAT_TYPE_GPS );
        GPS_DEBUG_MSG( ( "\r\n\n\n NOTCH FILTER GPS status: OFF \r\n\n\n" ) );
      }
      else if ( notch_sat_type == 1 )
      {
        gnss_notch_filter_disable( GNSS_SAT_TYPE_GLONASS );
        GPS_DEBUG_MSG( ( "\r\n\n\n NOTCH FILTER GLONASS status: OFF \r\n\n\n" ) );
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    }
  }

#endif
}

/********************************************//**
 * \brief   Execute NMEA output messages under request
 *
 * \param   tChar *
 * \return  void
 *
 ***********************************************/
void nmea_cmdif_exec_nmea_request( tChar *cmd_par )
{
  tInt field_count = 0;
  tUInt new_msg_list[2], msglist_l, msglist_h;
  tInt week_n;
  gpOS_clock_t fix_clock;
  tDouble tow;
  gnss_time_t gnss_time = {0};
  gnss_time_t utc_time;
  field_count = _clibs_sscanf( cmd_par, ",%x,%x", &msglist_l, &msglist_h );

  if ( field_count == 1 )
  {
    new_msg_list[0] = msglist_l;
    new_msg_list[1] = 0;
  }
  else if ( field_count == 2 )
  {
    new_msg_list[0] = msglist_l;
    new_msg_list[1] = msglist_h;
  }
  else
  {
    new_msg_list[0] = NMEA_msg_list[0];
    new_msg_list[1] = NMEA_msg_list[1];
  }

  gnss_fix_store_local( NULL );
  nmea_fix_data_lock();
  gnss_fix_get_time_local( &week_n, &tow, &fix_clock, NULL );
  gnss_time.week_n = week_n;
  gnss_time.tow = tow;
  utc_time = gnss_time_to_utc_time( gnss_time, gnss_time_get_master() );
  nmea_outmsg_transmit( new_msg_list, NULL, 0, utc_time );
  nmea_fix_data_unlock();
}


/********************************************//**
 * \brief   Switches to STBIN protocol
 *
 * \param   tChar *
 * \return  void
 *
 ***********************************************/
void nmea_cmdif_exec_stbin( tChar *cmd_par )
{
  nmea_set_if_mode( NMEA_EXTERNAL_IF_MODE );
}



/********************************************//**
 * \brief   Transmits a message which contains information
 *          on the Low Power mode/status
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_LOWPOWERSTATUS( void )
{
  tInt    index     = 0;
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_low_power_data_t data;
  gnss_low_power_get_data( &data );
  index = _clibs_sprintf( out_msg, "$PSTMLOWPOWERDATA,%d,%d,%d,%d,%.1lf,%d,%.1lf,%d,%d,%d,%d,%d,%d,%d",
                          data.cyclic.state,
                          data.steady_state,
                          data.no_fix,
                          data.cyclic.sats_used,
                          data.cyclic.ehpe,
                          data.glonass_tow_referesh,
                          data.cyclic.average_ehpe_nmea,
                          data.cyclic.average_NO_FIX,
                          data.counter_glonass_eph_ON,
                          data.eph_const_mask,
                          data.cyclic.reduced_type,
                          data.cyclic.duty_cycle_on_off,
                          data.cyclic.ms_off,
                          data.cyclic.duty_cycle_state
                        );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, LOWPOWER_NMEA_MSG );
}

/********************************************//**
 * \brief   Set Low Power status
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_setlowpower( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index = 0;
  tInt field_count = 0;
  tInt on_off;
  tInt const_type;
  tInt ehpe_threshold;
  tInt N_sats_reduced;
  tInt reduced_type;
  tInt duty_cycle_on_off;
  tInt ms_off;
  tInt fix_period;
  tInt fix_on_time;
  tInt EPH_refresh;
  tInt RTC_refresh;
  tInt NoFixTimeout;
  tInt NoFixOffTime;
  tInt periodic_mode;
  gnss_app_lowpow_standby_type_t Standby;

  gnss_low_power_cyclic_mode_t cyclic;
  gnss_low_power_periodic_mode_t periodic;

  field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &on_off, &const_type, &ehpe_threshold, &N_sats_reduced, &reduced_type, &duty_cycle_on_off, &ms_off, &periodic_mode, &fix_period, &fix_on_time, &EPH_refresh, &RTC_refresh, &NoFixTimeout, &NoFixOffTime );

  if ( ( field_count == 14 ) && ( on_off == 1 ) )
  {
    cyclic.ehpe_threshold         = (tU8)ehpe_threshold;
    cyclic.N_sats_reduced         = (tU8)N_sats_reduced;

    periodic.periodic_mode        = (boolean_t)((periodic_mode >> 0) & 0x01);
    periodic.fix_period           = (tU32)fix_period;
    periodic.fix_on_time          = (tU8)fix_on_time;
    periodic.EPH_refresh          = (tU8)EPH_refresh;
    periodic.RTC_refresh          = (tU8)RTC_refresh;
    periodic.NoFixTimeout         = (tU8)NoFixTimeout;
    periodic.NoFixOffTime         = (tU16)NoFixOffTime;

    index = _clibs_sprintf( out_msg, "$PSTMLOWPOWERON,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", ehpe_threshold, N_sats_reduced, reduced_type, duty_cycle_on_off, ms_off, periodic_mode, fix_period, fix_on_time, EPH_refresh, RTC_refresh, NoFixTimeout, NoFixOffTime );

    if ( reduced_type == 0 )
    {
      if ( duty_cycle_on_off == 1 )
      {
        cyclic.reduced_type       = FALSE;
        cyclic.duty_cycle_on_off  = TRUE;
        cyclic.ms_off             = (tShort)ms_off;
      }
      else if ( duty_cycle_on_off == 0 )
      {
        cyclic.reduced_type       = FALSE;
        cyclic.duty_cycle_on_off  = FALSE;
        cyclic.ms_off             = (tShort)ms_off;
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    }
    else if ( reduced_type == 1 )
    {
      if ( duty_cycle_on_off == 1 )
      {
        cyclic.reduced_type       = TRUE;
        cyclic.duty_cycle_on_off  = TRUE;
        cyclic.ms_off             = (tShort)ms_off;
      }
      else if ( duty_cycle_on_off == 0 )
      {
        cyclic.reduced_type       = TRUE;
        cyclic.duty_cycle_on_off  = FALSE;
        cyclic.ms_off             = (tShort)ms_off;
      }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

    /* load constellation applicable */
    cyclic.const_mask_init = const_type;

    if(((periodic_mode >> 1) & 0x01) == 0x01)
    {
      Standby = GNSSAPP_LOW_POWER_STANDBY_ENABLE;
    }
    else
    {
      Standby = GNSSAPP_LOW_POWER_STANDBY_DISABLE;
    }
    gnssapp_low_power_setup_update( Standby, &cyclic, &periodic );

  }
  else if ( ( field_count == 2 ) && ( on_off == 0 ) )
  {
    gnssapp_low_power_setup_update( GNSSAPP_LOW_POWER_STANDBY_DISABLE , NULL, NULL );
    index = _clibs_sprintf( out_msg, "$PSTMLOWPOWEROFF" );
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMLOWPOWERERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/********************************************//**
 * \brief   Force Standby
 *
 * \param   cmd_par   Parameters string
 * \return  None
 *
 ***********************************************/
static void nmea_cmdif_exec_force_standby( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index = 0;
  tInt field_count = 0;
  tInt duration;

  field_count = _clibs_sscanf( cmd_par, ",%d", &duration );

  if (( field_count == 1 ) && (svc_pwr_force_standby((tU16)duration) == gpOS_SUCCESS))
  {
    if( GNSS_ENGINE_STATUS_RUNNING == gnss_get_engine_status())
    {
      gnssapp_suspend();
      platform_gnss_suspend();
    }

    index = _clibs_sprintf( out_msg, "$PSTMFORCESTANDBYOK");
  }
  else
  {
    index = _clibs_sprintf( out_msg, "$PSTMFORCESTANDBYERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

/*{{{  nmea_outmsg_send_PV()*/

/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_outmsg_send_PV()

Description      : Transmits a PSTMPV type message

Parameters       : void

Return           : None.

Globals Accessed : out_msg
******************************************************************************/

/*}}}  */

static void nmea_outmsg_send_PV( void *data_p, const nmea_outmsg_common_data_t *nmea_common_data_p )
{
  tInt    index = 0;
  tInt    lat_deg;
  tInt    lat_min;
  tInt    lat_min_frac;
  tInt    lon_deg;
  tInt    lon_min;
  tInt    lon_min_frac;
  tChar   lat_sense_ch;
  tChar   lon_sense_ch;
  tDouble n_cov, e_cov, v_cov, en_cov, ev_cov, nv_cov;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  velocity_t *vel;
  tChar    format[64];

  vel = gnss_fix_get_fil_vel_local( data_p );

  gnss_fix_get_position_all_covariance_local( &n_cov,
      &e_cov,
      &v_cov,
      &en_cov,
      &ev_cov,
      &nv_cov,
      data_p );
  index += _clibs_sprintf( out_msg, "$PSTMPV" );
  index += _clibs_sprintf( &out_msg[index], ",%02d%02d%02d.%03d", nmea_common_data_p->hours, nmea_common_data_p->mins, nmea_common_data_p->secs, nmea_common_data_p->msecs );
  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.latitude, 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense_ch );
  nmea_support_degrees_to_int( nmea_common_data_p->extrap_pos.longitude, 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense_ch );
  _clibs_sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
  index += _clibs_sprintf( &out_msg[index], format,
                    lat_deg, lat_min, lat_min_frac, lat_sense_ch,
                    lon_deg, lon_min, lon_min_frac, lon_sense_ch
                  );
  index += _clibs_sprintf( &out_msg[index], ",%06.2f,M", nmea_common_data_p->extrap_pos.height );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf,%.1lf,%.1lf", vel->vel_north, vel->vel_east, vel->vel_vert );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf,%.1lf,%.1lf,%.1lf,%.1lf,%.1lf",
                    sqrt( n_cov ), sqrt( _clibs_abs( en_cov ) ), sqrt( _clibs_abs( nv_cov ) ),
                    sqrt( e_cov ), sqrt( _clibs_abs( ev_cov ) ), sqrt( v_cov ) );
  gnss_fix_get_velocity_all_covariance_local( &n_cov,
      &e_cov,
      &v_cov,
      &en_cov,
      &ev_cov,
      &nv_cov,
      data_p );
  index += _clibs_sprintf( &out_msg[index], ",%.1lf,%.1lf,%.1lf,%.1lf,%.1lf,%.1lf",
                    sqrt( n_cov ), sqrt( _clibs_abs( en_cov ) ), sqrt( _clibs_abs( nv_cov ) ),
                    sqrt( e_cov ), sqrt( _clibs_abs( ev_cov ) ), sqrt( v_cov ) );
  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, PV_NMEA_MSG );
}

/*{{{  nmea_outmsg_send_PVRAW()*/

/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_outmsg_send_PVRAW()

Description      : Transmits a PSTMPVRAW type message

Parameters       : void

Return           : None.

Globals Accessed : out_msg
******************************************************************************/

/*}}}  */

static void nmea_outmsg_send_PVRAW( void *data_p, const tDouble delay, const gnss_time_t utc_time )
{
  tInt    index = 0;
  tInt    lat_deg;
  tInt    lat_min;
  tInt    lat_min_frac;
  tChar   lat_sense;
  tInt    lon_deg;
  tInt    lon_min;
  tInt    lon_min_frac;
  tChar   lon_sense;
  tInt    hours;
  tInt    mins;
  tInt    secs;
  tInt    msecs;
  tInt    fix_type;
  tInt    num_sats_used = 0;
  tDouble  geoid_msl=0;
  tDouble  pdop;
  tDouble  vdop;
  tDouble  hdop;
  tDouble  gdop;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  position_t *extrap_pos;
  velocity_t *vel;
  tChar    format[64];

  gnss_get_utc_time( utc_time.tow, &hours, &mins, &secs, &msecs );
  geoid_msl = gnss_fix_get_geoid_msl_local( data_p );
  extrap_pos = gnss_fix_get_raw_pos_local( data_p );
  vel = gnss_fix_get_raw_vel_local( data_p );
  fix_type = gnss_fix_get_raw_pos_status_local(data_p);
  num_sats_used = gnss_fix_get_raw_pos_sats_local(data_p);
  gnss_fix_get_raw_pos_dops_local( &pdop, &hdop, &vdop, &gdop, data_p );

  if ( ( fix_type == ( tInt )FIX_2D ) || ( fix_type == ( tInt )FIX_3D ) )
  {
    if ( gnss_fix_get_diff_status_local( data_p ) == DIFF_STATUS_ON )
    {
      fix_type = 2; /* available with diff corrections */
    }
    else
    {
      fix_type = 1; /* available*/
    }
  }
  else
  {
    fix_type = 0;
  }

  index += sprintf( out_msg, "$PSTMPVRAW" );
  index += sprintf( out_msg + index, ",%02d%02d%02d.%03d", hours, mins, secs, msecs );
  nmea_support_degrees_to_int( extrap_pos->latitude, 'N', 'S', nmea_outmsg_GGA_posdigit, &lat_deg, &lat_min, &lat_min_frac, &lat_sense );
  nmea_support_degrees_to_int( extrap_pos->longitude, 'E', 'W', nmea_outmsg_GGA_posdigit, &lon_deg, &lon_min, &lon_min_frac, &lon_sense );
  sprintf( format, ",%%02d%%02d.%%0%dd,%%c,%%03d%%02d.%%0%dd,%%c", nmea_outmsg_GGA_posdigit, nmea_outmsg_GGA_posdigit );
  index += sprintf( out_msg + index, format,
                    lat_deg, lat_min, lat_min_frac, lat_sense,
                    lon_deg, lon_min, lon_min_frac, lon_sense
                  );
  index += _clibs_sprintf( out_msg + index, ",%01d,%02d,%01.1f,%06.2f,M,%01.1f,M",
                           fix_type, num_sats_used, hdop, extrap_pos->height, geoid_msl);
  index += sprintf( out_msg + index, ",%.1lf,%.1lf,%.1lf", vel->vel_north, vel->vel_east, vel->vel_vert );

  index += sprintf( out_msg + index, "*%02X", nmea_support_checksum( out_msg ) );
  index += sprintf( out_msg + index, "\r\n" );
  nmea_send_outmsg( out_msg, index, PVRAW_NMEA_MSG );
}
/*{{{  nmea_outmsg_send_PVQ()*/

/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_outmsg_send_PVQ()

Description      : Transmits a PSTMPVQ type message

Parameters       : void

Return           : None.

Globals Accessed : out_msg
******************************************************************************/

/*}}}  */

static void nmea_outmsg_send_PVQ( void *data_p, const tDouble delay, const gnss_time_t utc_time )
{
  tInt  index = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tDouble *q_matrix_ptr;
  tInt i;
  q_matrix_ptr = gnss_fix_get_position_q_matrix_diag_local( data_p );
  index += _clibs_sprintf( out_msg, "$PSTMPVQ" );

  for ( i = 0; i < N_POS_STATE; i++ )
  {
    index += _clibs_sprintf( &out_msg[index], ",%.1lf", sqrt( _clibs_abs( q_matrix_ptr[i] ) ) );
  }

  q_matrix_ptr = gnss_fix_get_velocity_q_matrix_diag_local( data_p );

  for ( i = 0; i < N_VEL_STATE; i++ )
  {
    index += _clibs_sprintf( &out_msg[index], ",%.1lf", sqrt( _clibs_abs( q_matrix_ptr[i] ) ) );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X", nmea_support_checksum( out_msg ) );
  index += _clibs_sprintf( &out_msg[index], "\r\n" );
  nmea_send_outmsg( out_msg, index, PVQ_NMEA_MSG );
}

/********************************************//**
 * \brief   Transmits a message which contains information
 *          on the Xtal
 *
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_XTALSTATUS( void )
{
  tInt    index     = 0;
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_xtal_monitor_t data;
  gnss_xtal_get_data( &data );
  index = _clibs_sprintf( out_msg, "$PSTMXTAL,%2.1f,%2.1f,%2.1f,%2.1f,%2.1f,%d,%d,",
                          data.T,
                          data.filtered_ramp_rate,
                          data.raw_ramp_rate,
                          data.nav_xtal_adc_stored_value,
                          data.nav_xtal_ADC_rate,
                          data.kf_ramp_rate_status,
                          data.acq_reactivation_status
                        );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, XTAL_NMEA_MSG );
}

/********************************************//**
 * \brief   Send TTFF message
 *
 * \param   data_p  Fix data structure
 * \return  None
 *
 ***********************************************/
static void nmea_outmsg_send_TTFF( void )
{
  /*{{{  decs*/
  tInt  index     = 0;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnssapp_startup_time_t * gnssapp_startup_time = NULL;
  gpOS_clock_t first_fix_timestamp  = 0;
  tDouble startup_time_delay_ms     = 0.0;
  tDouble set_timer_clock_delay_ms  = 0.0;
  tDouble send_header_delay_ms      = 0.0;
  tDouble send_first_gga_delay_ms   = 0.0;
  tDouble send_fix_gga_delay_ms     = 0.0;
  tDouble ttff_ms                   = 0.0;
  /*}}}  */
  first_fix_timestamp      = gnss_position_get_first_fix_timestamp();
  gnssapp_startup_time     = gnssapp_get_startup_time();
  set_timer_clock_delay_ms = ( tDouble )( gpOS_time_minus( gnssapp_startup_time->gnss_lib_set_timer_clock, gnssapp_startup_time->suspend_restart_support_time ) ) / ( tDouble )gnssapp_startup_time->MTU_timer_clock; // suspend_restart_support_time =0 when the receiver is turn ON
  startup_time_delay_ms    = set_timer_clock_delay_ms + (( tDouble )( gpOS_time_minus( gnssapp_startup_time->gnssapp_init_end_cpu_time, gnssapp_startup_time->gnss_lib_set_timer_clock ) ) / TRACKER_CPU_TICKS_PER_MSEC);
  send_header_delay_ms     = set_timer_clock_delay_ms + (( tDouble )( gpOS_time_minus( nmea_outmsg_header_timestamp, gnssapp_startup_time->gnss_lib_set_timer_clock ) ) / TRACKER_CPU_TICKS_PER_MSEC);
  send_first_gga_delay_ms  = set_timer_clock_delay_ms + (( tDouble )( gpOS_time_minus( nmea_outmsg_first_gga_timestamp, gnssapp_startup_time->gnss_lib_set_timer_clock ) ) / TRACKER_CPU_TICKS_PER_MSEC);
  send_fix_gga_delay_ms    = set_timer_clock_delay_ms + (( tDouble )( gpOS_time_minus( nmea_outmsg_gga_timestamp, gnssapp_startup_time->gnss_lib_set_timer_clock ) ) / TRACKER_CPU_TICKS_PER_MSEC);
  ttff_ms                  = set_timer_clock_delay_ms + (( tDouble )( gpOS_time_minus( first_fix_timestamp, gnssapp_startup_time->gnss_lib_set_timer_clock ) ) / TRACKER_CPU_TICKS_PER_MSEC);
  index = 0;
  index += _clibs_sprintf( out_msg, "$PSTMTTFF" );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", ttff_ms );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", startup_time_delay_ms );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", send_header_delay_ms );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", send_first_gga_delay_ms );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", send_fix_gga_delay_ms );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->nvm_start_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->sw_config_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->gnss_debug_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->xtal_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->gnss_lib_start_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->gnss_lib_set_timer_clock );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->gnss_module_start_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->nmea_start_cpu_start );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->as_start_cpu_start );
  index += _clibs_sprintf( &out_msg[index], ",%10d", gnssapp_startup_time->gnssapp_init_end_cpu_time );
  index += _clibs_sprintf( &out_msg[index], ",%10d", nmea_outmsg_header_timestamp );
  index += _clibs_sprintf( &out_msg[index], ",%10d", nmea_outmsg_first_gga_timestamp );
  index += _clibs_sprintf( &out_msg[index], ",%10d", nmea_outmsg_gga_timestamp );
  //index += _clibs_sprintf(&out_msg[index], ",%10d", (tInt)gpOS_time_minus( sending_cpu_time,fix_available_cpu_time));
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg, index );
}

static void nmea_cmdif_send_iono_params( const gnss_sat_type_t sat_type )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_error_t error_flag;
  gnss_iono_raw_t iono_raw;
  tInt index;
  error_flag = gnss_get_iono_params( &iono_raw, sat_type );

  if ( error_flag == GNSS_NO_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMIONOPARAMS,%d", ( tInt )sat_type );

    switch ( sat_type )
    {
      case GNSS_SAT_TYPE_GPS:
        index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d,%d,%d,%d,%d", iono_raw.gps.available,
                                 iono_raw.gps.A0, iono_raw.gps.A1, iono_raw.gps.A2, iono_raw.gps.A3,
                                 iono_raw.gps.B0, iono_raw.gps.B1, iono_raw.gps.B2, iono_raw.gps.B3 );
        break;

      case GNSS_SAT_TYPE_COMPASS:
        index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d,%d,%d,%d,%d", iono_raw.compass.available,
                                 iono_raw.compass.A0, iono_raw.compass.A1, iono_raw.compass.A2, iono_raw.compass.A3,
                                 iono_raw.compass.B0, iono_raw.compass.B1, iono_raw.compass.B2, iono_raw.compass.B3 );
        break;

      case GNSS_SAT_TYPE_GALILEO:
        index += _clibs_sprintf( &out_msg[index], ",%d,%d,%d,%d,%d,%d,%d,%d,%d", iono_raw.galileo.available,
                                 iono_raw.galileo.ai0, iono_raw.galileo.ai1, iono_raw.galileo.ai2, iono_raw.galileo.Region1,
                                 iono_raw.galileo.Region2, iono_raw.galileo.Region3, iono_raw.galileo.Region4, iono_raw.galileo.Region5 );
        break;
    }

    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_outmsg( out_msg, index, IONO_NMEA_MSG);
  }
}
/*}}}  */

/*{{{  dump_iono_params()*/
static void nmea_cmdif_exec_dump_iono_params( tChar *cmd_par )
{
  nmea_cmdif_send_iono_params( GNSS_SAT_TYPE_GPS );

  nmea_cmdif_send_iono_params( GNSS_SAT_TYPE_COMPASS );

  nmea_cmdif_send_iono_params( GNSS_SAT_TYPE_GALILEO );
}
/*}}}  */

/*{{{  nmea_outmsg_send_IONO()*/
static void nmea_outmsg_send_IONO( void )
{
  tUInt flags_mask;

  flags_mask = gnss_flags_update_mask_get_clear( ( 1 << FLAGS_UPDATE_BIT_IONO_GPS ) | ( 1 << FLAGS_UPDATE_BIT_IONO_GALILEO ) | ( 1 << FLAGS_UPDATE_BIT_IONO_COMPASS ) );

  if ( MCR_ISBITSET( flags_mask, FLAGS_UPDATE_BIT_IONO_GPS ) )
  {
    nmea_cmdif_send_iono_params( GNSS_SAT_TYPE_GPS );
  }

  if ( MCR_ISBITSET( flags_mask, FLAGS_UPDATE_BIT_IONO_COMPASS ) )
  {
    nmea_cmdif_send_iono_params( GNSS_SAT_TYPE_COMPASS );
  }

  if ( MCR_ISBITSET( flags_mask, FLAGS_UPDATE_BIT_IONO_GALILEO ) )
  {
    nmea_cmdif_send_iono_params( GNSS_SAT_TYPE_GALILEO );
  }
}
/*}}}  */

/*{{{  save_iono_params()*/
static void nmea_cmdif_exec_save_iono_params( tChar *cmd_par )
{
  gnss_error_t error_flag = GNSS_ERROR;
  tInt field_count;
  tInt sat_type;
  gnss_iono_raw_t iono_raw;
  /* 15 is $PSTMIONOPARAMS */
  field_count = _clibs_sscanf( cmd_par, ",%d", &sat_type );

  switch ( ( gnss_sat_type_t )sat_type )
  {
    case GNSS_SAT_TYPE_GPS:
      {
        tInt available, A0, A1, A2, A3, B0, B1, B2, B3;
        field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &sat_type, &available, &A0, &A1, &A2, &A3, &B0, &B1, &B2, &B3 );

        if ( field_count == 10 )
        {
          iono_raw.gps.available = available;
          iono_raw.gps.A0 = A0;
          iono_raw.gps.A1 = A1;
          iono_raw.gps.A2 = A2;
          iono_raw.gps.A3 = A3;
          iono_raw.gps.B0 = B0;
          iono_raw.gps.B1 = B1;
          iono_raw.gps.B2 = B2;
          iono_raw.gps.B3 = B3;
          error_flag = GNSS_NO_ERROR;
        }
      }
      break;

    case GNSS_SAT_TYPE_COMPASS:
      {
        tInt available, A0, A1, A2, A3, B0, B1, B2, B3;
        field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &sat_type, &available, &A0, &A1, &A2, &A3, &B0, &B1, &B2, &B3 );

        if ( field_count == 10 )
        {
          iono_raw.compass.available = available;
          iono_raw.compass.spare0    = 0;
          iono_raw.compass.A0 = A0;
          iono_raw.compass.A1 = A1;
          iono_raw.compass.A2 = A2;
          iono_raw.compass.A3 = A3;
          iono_raw.compass.B0 = B0;
          iono_raw.compass.B1 = B1;
          iono_raw.compass.B2 = B2;
          iono_raw.compass.B3 = B3;
          error_flag = GNSS_NO_ERROR;
        }
      }
      break;

    case GNSS_SAT_TYPE_GALILEO:
      {
        tInt available, ai0, ai1, ai2, Region1, Region2, Region3, Region4, Region5;
        field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &sat_type, &available,
                                     &ai0, &ai1, &ai2, &Region1, &Region2, &Region3, &Region4, &Region5 );

        if ( field_count == 10 )
        {
          iono_raw.galileo.available = available;
          iono_raw.galileo.spare0    = 0;
          iono_raw.galileo.spare1    = 0;
          iono_raw.galileo.ai0       = ai0;
          iono_raw.galileo.ai1       = ai1;
          iono_raw.galileo.ai2       = ai2;
          iono_raw.galileo.Region1   = Region1 & 1;
          iono_raw.galileo.Region2   = Region2 & 1;
          iono_raw.galileo.Region3   = Region3 & 1;
          iono_raw.galileo.Region4   = Region4 & 1;
          iono_raw.galileo.Region5   = Region5 & 1;
          error_flag = GNSS_NO_ERROR;
        }
      }
      break;
  }

  if ( error_flag == GNSS_NO_ERROR )
  {
    gnss_set_iono_params( &iono_raw, ( gnss_sat_type_t )sat_type );
  }
}
/*}}}  */

/*{{{  nmea_outmsg_send_GGTO()*/
static void nmea_outmsg_send_GGTO( void )
{
  tUInt flags_mask;
  flags_mask = gnss_flags_update_mask_get_clear( ( 1 << FLAGS_UPDATE_BIT_GALILEO_GGTO ) );

  if ( MCR_ISBITSET( flags_mask, FLAGS_UPDATE_BIT_GALILEO_GGTO ) )
  {
    nmea_send_galilo_ggto();
  }
}
/*}}}  */

static void nmea_send_galilo_ggto( void )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt index;
  galileo_ggto_raw_t ggto_raw;

  gnss_galileo_get_ggto_brdc_raw( &ggto_raw );
  index = _clibs_sprintf( out_msg, "$PSTMGALILEOGGTO,%d,%d,%d,%d,%d,%d", 1, ggto_raw.WN0G, ggto_raw.t0G, ggto_raw.A0G, ggto_raw.A1G, ggto_raw.valid );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, GGTO_NMEA_MSG);
}

/*{{{  nmea_cmdif_exec_galileo_dump_ggto()*/
static void nmea_cmdif_exec_galileo_dump_ggto( tChar *cmd_par )
{
  nmea_send_galilo_ggto();
}
/*}}}  */

/*{{{  nmea_cmdif_exec_galileo_ggto()*/
static void nmea_cmdif_exec_galileo_ggto( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_error_t error_flag = GNSS_ERROR;
  tInt field_count;
  tInt cmd_type, arg1, arg2, arg3, arg4, arg5;
  tInt index;
  galileo_ggto_raw_t ggto_raw;
  field_count = _clibs_sscanf( cmd_par, ",%d", &cmd_type );

  if ( ( field_count == 1 ) && ( cmd_type == 1 ) )
  {
    // GGTO in raw format
    field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d,%d,%d", &cmd_type, &arg1, &arg2, &arg3, &arg4, &arg5 );

    if ( field_count == 6 )
    {
      ggto_raw.WN0G   = arg1;
      ggto_raw.t0G    = arg2;
      ggto_raw.A0G    = arg3;
      ggto_raw.A1G    = arg4;
      ggto_raw.valid  = arg5;
      ggto_raw.spare0 = 0;
      error_flag = gnss_galileo_set_ggto_brdc_raw( &ggto_raw );
    }
  }

  if ( error_flag == GNSS_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMGALILEOGGTOERROR" );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}
/*}}}  */

/*{{{  nmea_cmdif_exec_galileo()*/
static void nmea_cmdif_exec_galileo( tChar *cmd_par )
{
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  gnss_error_t error_flag = GNSS_ERROR;
  tInt field_count;
  tInt cmd_type = -1;
  tInt arg1, arg2, arg3;
  tInt index;
  //task_delay(TRACKER_CPU_TICKS_PER_SECOND);
  field_count = _clibs_sscanf( cmd_par, ",%d,%d,%d,%d", &cmd_type, &arg1, &arg2, &arg3 );

  if ( ( field_count == 2 ) && ( cmd_type == 1 ) )
  {
    gnssapp_suspend();
    error_flag = gnss_galileo_set_pilot_mode( arg1 );
    gnssapp_restart();
  }

  if ( ( field_count == 2 ) && ( cmd_type == 2 ) )
  {
    gnssapp_suspend();
    error_flag = gnss_galileo_set_iono_model( arg1 );
    gnssapp_restart();
  }

  if ( ( field_count == 2 ) && ( cmd_type == 4 ) )
  {
    gnssapp_suspend();
    error_flag = gnss_galileo_set_ggto_mode( ( galileo_ggto_mode_t )arg1 );
    gnssapp_restart();
  }

  if ( error_flag == GNSS_ERROR )
  {
    index = _clibs_sprintf( out_msg, "$PSTMGALILEOERROR" );
    index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
    nmea_send_msg_to_uart( out_msg, index );
  }
}
/*}}}  */

/*{{{  nmea_outmsg_send_ALM()*/
static void nmea_outmsg_send_ALM( void )
{
  tUInt alm_update_mask[LINKED_SAT_IDS_MASK_SIZE];
  tInt j0, j1;
  gnss_alm_update_mask_copy_clear( alm_update_mask );

  for ( j0 = 0; j0 < LINKED_SAT_IDS_MASK_SIZE; j0++ )
  {
    if ( alm_update_mask[j0] )
    {
      for ( j1 = 0; j1 < 32; j1++ )
      {
        if ( MCR_ISBITSET( alm_update_mask[j0], j1 ) )
        {
          satid_t sat_id = gnss_gnsslib_id_to_sat_id( (j0 * 32) + j1 );
          nmea_cmdif_send_almanac( sat_id );
        }
      }
    }
  }
}
/*}}}  */

/*{{{  nmea_outmsg_send_EPHEM()*/
static void nmea_outmsg_send_EPHEM( void )
{
  tUInt eph_update_mask[LINKED_SAT_IDS_MASK_SIZE];
  tInt j0, j1;
  gnss_eph_update_mask_copy_clear( eph_update_mask );

  for ( j0 = 0; j0 < LINKED_SAT_IDS_MASK_SIZE; j0++ )
  {
    if ( eph_update_mask[j0] )
    {
      for ( j1 = 0; j1 < 32; j1++ )
      {
        if ( MCR_ISBITSET( eph_update_mask[j0], j1 ) )
        {
          satid_t sat_id = gnss_gnsslib_id_to_sat_id( (j0 * 32) + j1 );
          nmea_cmdif_send_ephemeris( sat_id );
        }
      }
    }
  }
}
/*}}}  */


/*{{{  nmea_outmsg_send_BIASDATA()*/
static void nmea_outmsg_send_BIASDATA( void *data_p )
{
  /*{{{  decs*/
  tInt index = 0;
  tChar out_msg[256];
  gnss_time_t time_best;
  time_validity_t time_best_validity;
  gpOS_clock_t cpu_time;
  tDouble ggto_brdc_bias, ggto_est_bias, ggto_fix_bias;
  boolean_t ggto_brdc_validity, ggto_est_validity, ggto_fix_validity;
  tInt ggto_validity_flags;
  tDouble glonass_path_delay;
  /*}}}  */

  gnss_fix_get_time_best_local( &time_best, &cpu_time, &time_best_validity, data_p );

  glonass_path_delay = gnss_fix_get_glonass_path_delay();

  if( MCR_ISBITSET( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO))
  {
    ggto_brdc_validity  = gnss_galileo_get_ggto( &time_best, &ggto_brdc_bias, GALILEO_GGTO_MODE_BRDC );
    ggto_est_validity   = gnss_galileo_get_ggto( &time_best, &ggto_est_bias, GALILEO_GGTO_MODE_EST );
    ggto_fix_validity   = gnss_fix_get_galileo_ggto_local( data_p, &ggto_fix_bias );
    ggto_validity_flags = ( ( ggto_brdc_validity == TRUE ) ? 1 : 0 ) | ( ( ggto_est_validity == TRUE ) ? 2 : 0 ) | ( ( ggto_fix_validity == TRUE ) ? 4 : 0 );
  }
  else
  {
    ggto_brdc_bias      = 0.0;
    ggto_est_bias       = 0.0;
    ggto_fix_bias       = 0.0;
    ggto_validity_flags = 0;
  }

  index = 0;
  index += _clibs_sprintf( out_msg, "$PSTMBIASDATA" );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", glonass_path_delay );
  index += _clibs_sprintf( &out_msg[index], ",%d",   ggto_validity_flags );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", ggto_brdc_bias );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", ggto_est_bias );
  index += _clibs_sprintf( &out_msg[index], ",%.2f", ggto_fix_bias );
  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_outmsg( out_msg, index, BIAS_NMEA_MSG);
}
/*}}}  */

/*{{{  nmea_outmsg_send_FEDATA()*/
static void nmea_outmsg_send_FEDATA( void )
{
#if defined( NMEA_FRONTEND_SUPPORT )
  tInt index=0;
  FE_reg_item_t reg_table[30];
  LLD_ErrorTy FE_error;
  tChar out_msg[NMEA_OUT_MSG_BUFFER_SIZE];

  #if defined( NMEA_REMOTE_FRONTEND_SUPPORT )
  tUInt fe_regs_size;
  #endif

  index = _clibs_sprintf( &out_msg[index], "$PSTMFEDATA" );

#if defined( NMEA_REMOTE_FRONTEND_SUPPORT )
  fe_regs_size = sizeof( reg_table );

  if ( remote_fe_dump( reg_table, &fe_regs_size ) == GNSS_NO_ERROR )
#else
  FE_error = FE_dump_regs( reg_table );

  if ( FE_error == LLD_NO_ERROR )
#endif
  {
    tU32 cnt = 0;

    while ( reg_table[cnt].addr != 0xFFU )
    {
      index += _clibs_sprintf( &out_msg[index], ",%02x",reg_table[cnt].data);
      cnt++;
    }
  }
  else
  {
    index += _clibs_sprintf( &out_msg[index], ",ERROR" );
  }

  index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
  nmea_send_msg_to_uart( out_msg , index );
#endif
}

/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_cmdif_exec_write_swconfig_data_block()

Description      : Save the  transmitted data block

Parameters       : cmd_par   Parameters string

Return           : None.

Globals Accessed : out_msg
******************************************************************************/

/*}}}  */
static void nmea_cmdif_exec_write_swconfig_data_block( tChar *cmd_par )
{
  tChar   out_msg[NMEA_OUT_MSG_BUFFER_SIZE];
  tInt    index = 0;
  tInt    length;
  tInt    msg_n;
  tInt    msg_tot;
  tChar   hexbuffer[128];
  tUChar  cfg_data[64];
  tChar*  cfg_pointer = ( tChar * )( cfg_data );
  tUInt   field_count;
  tUInt   cnt;

  GPS_DEBUG_MSG( ( "[nmea]entering write swconfig\r\n" ) );
  field_count =  _clibs_sscanf( cmd_par, ",%d,%d,%d,%s", &msg_n, &msg_tot, &length, hexbuffer );

  if ( field_count == 4 )
  {
    /* translate hex buffer to binary */
    for ( cnt = 0 ; cnt < length ; cnt++ )
    {
      cfg_pointer[cnt] = nmea_support_hex2toint( hexbuffer + ( cnt * 2 ) );
    }

    if ( sw_config_data_block_write(length, length * msg_n, cfg_data ) == GNSS_ERROR )
    {
      /* report error */
      GPS_DEBUG_MSG(("[nmea]sw_config_data_block_write failed! %d,%d \r\n",msg_n,msg_tot ));
    }
    else
    {
      GPS_DEBUG_MSG(("[nmea]Successfully write block ,%d,%d \r\n",msg_n,msg_tot));
    }

    /* return acknowledgement when all messages are received */
    if(msg_n == msg_tot -1)
    {
      index = _clibs_sprintf( out_msg, "$PSTMSETSWCFGOK" );
      index += _clibs_sprintf( &out_msg[index], "*%02X\r\n", nmea_support_checksum( out_msg ) );
      nmea_send_msg_to_uart( out_msg, index );
    }
  }
}


/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_outmsg_set_contellation()

Description      : Computes the constellation data needed for GNS, GSA, GSV messages
                   To be called each time the constellation mask or software switch status change

Parameters       : sat_type_mask   Constellation mask

Return           : None.

******************************************************************************/

/*}}}  */
void nmea_outmsg_set_contellation( const gnss_sat_type_mask_t sat_type_mask )
{
  tInt constellations_enabled = 0;
  tInt constallation_used     = 0;

  // Determine how many constellations are enabled (for GSA message)
  if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_ID, GPS_ON_OFF_SWITCH ) )
  {
    constellations_enabled++;
  }

  if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_ID, GLONASS_ON_OFF_SWITCH ) )
  {
    constellations_enabled++;
  }

  if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_ID, QZSS_ON_OFF_SWITCH ) )
  {
    constellations_enabled++;
  }

  if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, GALILEO_ON_OFF_SWITCH ) )
  {
    constellations_enabled++;
  }

  if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, COMPASS_ON_OFF_SWITCH ) )
  {
    constellations_enabled++;
  }

  nmea_outmsg_GSA_constellations_enabled = constellations_enabled;
  nmea_outmsg_GNS_constellations_enabled = constellations_enabled;


  // Determine the constellation mask for GSA and GSV messages
  // Constellation shall be activated in both gnss_manager.sat_type_mask and app_on_off / app_on_off_2
  // (except for GPS: only check in app_on_off)
  nmea_outmsg_const_mask = 0;

  if ( sw_config_get_software_switch_status( GPS_ON_OFF_SWITCH ) )
  {
    MCR_SETBIT( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GPS );
    constallation_used++;
  }

  if ( MCR_ISBITSET( sat_type_mask, GNSS_SAT_TYPE_GLONASS ) )
  {
    if ( sw_config_get_software_switch_status( GLONASS_ON_OFF_SWITCH ) )
    {
      MCR_SETBIT( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GLONASS );
      constallation_used++;
    }
  }

  if ( MCR_ISBITSET( sat_type_mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
  {
    if ( sw_config_get_software_switch_status( QZSS_ON_OFF_SWITCH ) )
    {
      MCR_SETBIT( nmea_outmsg_const_mask, GNSS_SAT_TYPE_QZSS_L1_CA );
      constallation_used++;
    }
  }

  if( MCR_ISBITSET( sat_type_mask, GNSS_SAT_TYPE_GALILEO))
  {
    if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, GALILEO_ON_OFF_SWITCH ) )
    {
      MCR_SETBIT( nmea_outmsg_const_mask, GNSS_SAT_TYPE_GALILEO );
      constallation_used++;
    }
  }

  if( MCR_ISBITSET( sat_type_mask, GNSS_SAT_TYPE_COMPASS))
  {
    if ( sw_config_get_software_switch_status_by_id( APP_ON_OFF_2_ID, COMPASS_ON_OFF_SWITCH ) )
    {
      MCR_SETBIT( nmea_outmsg_const_mask, GNSS_SAT_TYPE_COMPASS );
      constallation_used++;
    }
  }
  nmea_outmsg_GNS_constellations_used = constallation_used;

}

/*{{{  Function Header*/

/*****************************************************************************
Function         : nmea_dynamic_set_application_on_off()

Description      : Dynamically sets application on/off constellations

Parameters       : sat_type_mask   Constellation mask

Return           : None.

******************************************************************************/

/*}}}  */
void nmea_dynamic_set_application_on_off( const gnss_sat_type_mask_t mask)
{

  if ( MCR_ISBITSET( mask, GNSS_SAT_TYPE_GPS ) )
  {
    sw_config_set_software_switch_status( GPS_ON_OFF_SWITCH | GPS_USE_ON_OFF_SWITCH, 1 );
  }
  else
  {
    sw_config_set_software_switch_status( GPS_ON_OFF_SWITCH | GPS_USE_ON_OFF_SWITCH, 0 );
  }

  if ( MCR_ISBITSET( mask, GNSS_SAT_TYPE_GLONASS ) )
  {
    sw_config_set_software_switch_status( GLONASS_ON_OFF_SWITCH | GLONASS_USE_ON_OFF_SWITCH, 1 );
  }
  else
  {
    sw_config_set_software_switch_status( GLONASS_ON_OFF_SWITCH | GLONASS_USE_ON_OFF_SWITCH, 0 );
  }

  if ( MCR_ISBITSET( mask, GNSS_SAT_TYPE_QZSS_L1_CA ) )
  {
    sw_config_set_software_switch_status( QZSS_ON_OFF_SWITCH | QZSS_USE_ON_OFF_SWITCH, 1 );
  }
  else
  {
    sw_config_set_software_switch_status( QZSS_ON_OFF_SWITCH | QZSS_USE_ON_OFF_SWITCH, 0 );
  }

  if ( MCR_ISBITSET( mask, GNSS_SAT_TYPE_GALILEO ) )
  {
    sw_config_set_software_switch_status_by_id( APP_ON_OFF_2_ID, GALILEO_ON_OFF_SWITCH | GALILEO_USE_ON_OFF_SWITCH, 1 );
  }
  else
  {
    sw_config_set_software_switch_status_by_id( APP_ON_OFF_2_ID, GALILEO_ON_OFF_SWITCH | GALILEO_USE_ON_OFF_SWITCH, 0 );
  }

  if ( MCR_ISBITSET( mask, GNSS_SAT_TYPE_COMPASS ) )
  {
    sw_config_set_software_switch_status_by_id( APP_ON_OFF_2_ID, COMPASS_ON_OFF_SWITCH | COMPASS_USE_ON_OFF_SWITCH, 1 );
  }
  else
  {
    sw_config_set_software_switch_status_by_id( APP_ON_OFF_2_ID, COMPASS_ON_OFF_SWITCH | COMPASS_USE_ON_OFF_SWITCH, 0 );
  }
}
