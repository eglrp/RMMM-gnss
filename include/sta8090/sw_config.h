#ifndef SW_CONFIG
#define SW_CONFIG

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

#include "gnss_defs.h"

#ifdef SW_CONFIG_PRIVATE_BLOCK
  #include "sw_config_private.h"
#endif

/*****************************************************************************
   defines and macros
*****************************************************************************/

/* First APP ON/OFF parameter */
#define NMEA_PORT_TMODE_SWITCH                0x1
#define GPS_2D_FIX_SWITCH                     0x2
#define WAAS_ON_OFF_SWITCH                    0x4
#define WAAS_SAT_ON_GSV                       0x8
#define STAGPS_ON_OFF_SWITCH                  0x10
#define ACQ_2_5_PPM_TCXO_SWITCH               0x20
#define NMEA_301_FORMAT_SWITCH                0x40
#define QZSS_SLOW_ACQUISITION_MODE_SWITCH     0x80
#define POSITION_STATIC_FILTER_SWITCH         0x100    /*not used on sta2062/sta2064*/
#define CONFIG_TEXT_AS_HEADER_SWITCH          0x200
#define ST_HEADERS_SWITCH                     0x400
#define RTCM_ON_OFF_SWITCH                    0x800
#define GPS_FDE_SWITCH                        0x1000
#define GPS_ADS_SWITCH                        0x2000
#define GPS_WALKING_MODE_SWITCH               0x4000
#define GPS_STOP_DETECTION_SWITCH             0x8000
#define GPS_ON_OFF_SWITCH                     0x10000
#define GLONASS_ON_OFF_SWITCH                 0x20000
#define QZSS_ON_OFF_SWITCH                    0x40000
#define NMEA_GNGSV_ENABLE                     0x80000
#define NMEA_GNGSA_ENABLE                     0x100000
#define GLONASS_USE_ON_OFF_SWITCH             0x200000
#define GPS_USE_ON_OFF_SWITCH                 0x400000
#define QZSS_USE_ON_OFF_SWITCH                0x800000
#define PPS_ON_OFF_SWITCH                     0x1000000
#define PPS_INVERTED_POLARITY_ON_OFF_SWITCH   0x2000000
#define POSITION_HOLD_ON_OFF_SWITCH           0x4000000U
#define TIMING_TRAIM_ON_OFF_SWITCH            0x8000000
#define WAAS_AUTOSEARCH_ON_OFF_SWITCH         0x10000000
#define HIGH_DYNAMICS_ON_OFF_SWITCH           0x20000000
#define NMEA_RAW_ON_OFF_SWITCH                0x40000000
#define LOW_POWER_ON_OFF_SWITCH               0x80000000U

/* Second APP ON/OFF parameter */
#define NMEA_CMDIF_ECO_ON_OFF_SWITCH          0x1
#define NMEA_TTFF_MSG_ON_OFF_SWITCH           0x2
#define FEW_SATS_POS_EST_ON_OFF_SWITCH        0x4
#define STBIN_ON_OFF_SWITCH                   0x8
#define SMART_XTAL_SUPPORT_ON_OFF_SWITCH      0x10
#define NMEA_IN_OUT_INTERFACE_SWITCH          0x20
#define GALILEO_ON_OFF_SWITCH                 0x40
#define GALILEO_USE_ON_OFF_SWITCH             0x80
#define COMPASS_ON_OFF_SWITCH                 0x100
#define COMPASS_USE_ON_OFF_SWITCH             0x200
#define LVD_MONITOR_ON_OFF_SWITCH             0x400
#define RTC_USAGE_DISABLING_ON_OFF_SWITCH     0x800U
#define GNSS_FAST_CN0_MODE_ON_OFF_SWITCH      0x1000
#define NMEA_MESSAGE_TIMESTAMP_SWITCH         0x2000
#define SAT_EXCL_PRESENT_GSA_GGA              0x4000
#define WLS_ALGO_RUNTIME_ON_OFF_SWITCH        0x8000
#define RTCM3_ENC_ON_OFF_SWITCH               0x20000
#define RTCM_VERSION_SWITCH                   0x40000
#define L2C_ON_OFF_SWITCH                     0x80000
#define L2C_USE_ON_OFF_SWITCH                 0x100000
#define EXT_RTC_OSCI_ON_OFF_SWITCH            0x200000U
#define MFREQ_GPS_ON_OFF_SWITCH               0x400000
#define MFREQ_GLONASS_ON_OFF_SWITCH           0x800000
#define MFREQ_GALILEO_ON_OFF_SWITCH           0x1000000
#define MFREQ_COMPASS_ON_OFF_SWITCH           0x2000000
#define RTC_CALIBRATION_ON_OFF_SWITCH         0x4000000U

#define SW_CONFIG_TEXT_LENGTH           72

#define SW_CONFIG_KEY                   0xaef5321aU,0xb0134ac0U,0x3f855aa0U,0xdef0a534U
#define SW_CONFIG_CHECKSUM              0

#if defined( __STA8090__)
#define SW_CONFIG_HEADER                0x81000000U
#else
#define SW_CONFIG_HEADER                0x80000000U
#endif

#define SW_CONFIG_SEC_1_VERSION         1U    /* Must be incremented at each modification in section 1 */
#define SW_CONFIG_SEC_2_VERSION         4U    /* Must be incremented at each modification in section 2 */
#define SW_CONFIG_SEC_3_VERSION         1U    /* Must be incremented at each modification in section 3 */
#define SW_CONFIG_SEC_4_VERSION         1U    /* Must be incremented at each modification in section 4 */
#define SW_CONFIG_SEC_5_VERSION         1U    /* Must be incremented at each modification in section 5 */
#define SW_CONFIG_SEC_6_VERSION         3U    /* Must be incremented at each modification in section 6 */

#define SW_CONFIG_VERSION_ID \
      SW_CONFIG_HEADER | \
      ( \
        ((SW_CONFIG_SEC_1_VERSION & 0x03U) << 0U)   |\
        ((SW_CONFIG_SEC_2_VERSION & 0x3FU) << 2U)   |\
        ((SW_CONFIG_SEC_3_VERSION & 0x0FU) << 8U)   |\
        ((SW_CONFIG_SEC_4_VERSION & 0x03U) << 12U)  |\
        ((SW_CONFIG_SEC_5_VERSION & 0x03U) << 14U)  |\
        ((SW_CONFIG_SEC_6_VERSION & 0x3FU) << 16U)  \
      )

#define SW_CONFIG_SEC_1                 1
#define SW_CONFIG_SEC_2                 2
#define SW_CONFIG_SEC_3                 3
#define SW_CONFIG_SEC_4                 4
#define SW_CONFIG_SEC_5                 5

#define SW_CONFIG_SEC_1_ITEMS           104
#define SW_CONFIG_SEC_2_ITEMS           64
#define SW_CONFIG_SEC_3_ITEMS           14
#define SW_CONFIG_SEC_4_ITEMS           4
#define SW_CONFIG_SEC_5_ITEMS           1

#define SW_CONFIG_SEC_1_OFFSET          (0x0)
#define SW_CONFIG_SEC_2_OFFSET          (SW_CONFIG_SEC_1_OFFSET + (SW_CONFIG_SEC_1_ITEMS * sizeof(tU8)))
#define SW_CONFIG_SEC_3_OFFSET          (SW_CONFIG_SEC_2_OFFSET + (SW_CONFIG_SEC_2_ITEMS * sizeof(tInt)))
#define SW_CONFIG_SEC_4_OFFSET          (SW_CONFIG_SEC_3_OFFSET + (SW_CONFIG_SEC_3_ITEMS * sizeof(tDouble)))
#define SW_CONFIG_SEC_5_OFFSET          (SW_CONFIG_SEC_4_OFFSET + (SW_CONFIG_SEC_4_ITEMS * sizeof(sw_config_dops_t)))
#define SW_CONFIG_SEC_6_OFFSET          (SW_CONFIG_SEC_5_OFFSET + SW_CONFIG_TEXT_LENGTH)

#define SW_CONFIG_BAUDRATE_300_BPS      0x0
#define SW_CONFIG_BAUDRATE_600_BPS      0x1
#define SW_CONFIG_BAUDRATE_1200_BPS     0x2
#define SW_CONFIG_BAUDRATE_2400_BPS     0x3
#define SW_CONFIG_BAUDRATE_4800_BPS     0x4
#define SW_CONFIG_BAUDRATE_9600_BPS     0x5
#define SW_CONFIG_BAUDRATE_14400_BPS    0x6
#define SW_CONFIG_BAUDRATE_19200_BPS    0x7
#define SW_CONFIG_BAUDRATE_38400_BPS    0x8
#define SW_CONFIG_BAUDRATE_57600_BPS    0x9
#define SW_CONFIG_BAUDRATE_115200_BPS   0xA
#define SW_CONFIG_BAUDRATE_230400_BPS   0xB
#define SW_CONFIG_BAUDRATE_460800_BPS   0xC
#define SW_CONFIG_BAUDRATE_921600_BPS   0xD
#define SW_CONFIG_BAUDRATE_INVALID      0xE

#define SWCFG_OUTPUT_CFG_DEBUG_UART     (1 << 0)
#define SWCFG_OUTPUT_CFG_DEBUG_USB      (1 << 1)
#define SWCFG_OUTPUT_CFG_DEBUG_FILE     (1 << 2)
#define SWCFG_OUTPUT_CFG_DEBUG_SOCK     (1 << 3)
#define SWCFG_OUTPUT_CFG_NMEA_UART      (1 << 4)
#define SWCFG_OUTPUT_CFG_NMEA_USB       (1 << 5)
#define SWCFG_OUTPUT_CFG_NMEA_FILE      (1 << 6)
#define SWCFG_OUTPUT_CFG_NMEA_SOCK      (1 << 7)

#define NOTCH_FILTER_DISABLED               0x0
#define NOTCH_FILTER_ENABLED_GPS            0x1
#define NOTCH_FILTER_ENABLED_GLONASS        0x2
#define NOTCH_FILTER_ENABLED_AUTO_GPS       0x4
#define NOTCH_FILTER_ENABLED_AUTO_GLONASS   0x8

#define HW_SOC_CFG                          0x0
#define HW_SAL_CFG                          0x1

#define GLONASS_OUTPUT_FORMAT_FREQ          0x0
#define GLONASS_OUTPUT_FORMAT_SLOT          0x1

#define DELAY_TO_NEXT_FIX_50                (gpOS_clock_t)(50 * (tDouble)(NAV_CPU_TICKS_PER_SECOND/1000))
#define DELAY_TO_NEXT_FIX_500               (gpOS_clock_t)(500 * (tDouble)(NAV_CPU_TICKS_PER_SECOND/1000))

#define SWCFG_NMEA_INPUT_ON_DEBUG           (1 << 4)
#define SWCFG_NMEA_OUTPUT_ON_DEBUG_ENABLE   (1 << 5)
#define SWCFG_NMEA_OUTPUT_ENABLE            (1 << 6)
#define SWCFG_DUAL_NMEA_PORT_ENABLE         (1 << 7)

/*SW config section 1*/
#define DEBUG_PORT_NUMBER_ID                  100
#define NMEA_PORT_NUMBER_ID                   101
#define NMEA_PORT_BAUDRATE_ID                 102
#define GPS_DEBUG_MODE_ID                     103
#define GPS_MASK_ANGLE_ID                     104
#define GPS_TRACKING_TH_ID                    105
#define DEBUG_PORT_BAUDRATE_ID                106
#define GPS_NAVIGATE_PPS_TASK_PRIORITY_ID     107
#define GPS_DSP_TRK_TASK_PRIORITY_ID          108
#define GPS_TRACKER_TASK_PRIORITY_ID          109
#define GPS_DSP_ACQ_TASK_PRIORITY_ID          110
#define GPS_GPS_FLASH_ERASE_PRIORITY_ID       111
#define WAAS_TASK_PRIORITY_ID                 112
#define WAAS_BACK_END_TASK_PRIORITY_ID        113
#define ST_AGPS_EPHMGR_TASK_PRIORITY_ID       114
#define NMEA_MSG_TASK_PRIORITY_ID             115
#define GPS_NAVIGATE_TASK_PRIORITY_ID         116
#define ST_AGPS_TASK_PRIORITY_ID              117
#define BACKUP_TASK_PRIORITY_ID               118
#define C2C_IST_TASK_PRIORITY_ID              119
#define COLD_START_TYPE_ID                    120
#define NMEA_SPEED_DIGIT_ID                   121
#define GPS_RTC_WRITE_TASK_PRIORITY_ID        122
#define STREAM_TASK_PRIORITY_ID               123
#define OUTPUT_CFG_ID                         124
#define NOTCH_FILTER_CFG_ID                   125
#define HARDWARE_CONFIGURATION_ID             126
#define NMEA_POSITION_DIGIT                   127
#define DIFFERENTIAL_SOURCE                   128
#define GLONASS_OUTPUT_FORMAT_ID              129
#define CPU_CLOCK_SPEED                       130
#define NMEA_TALKER_ID                        131
#define GNSS_POSITIONING_THRESHOLD_ID         132
#define CONFIG_STATUS_ID                      133
#define CONFIG_VER_ID                         134
#define SBAS_PRN_ID                           135
#define T_SHORT_ID                            136
#define T_LONG_ID                             137
#define RTCM_PORT_NUMBER_ID                   138
#define RTCM_PORT_BAUDRATE_ID                 139
#define FE_A0_ID                              140
#define FE_D0_ID                              141
#define FE_A1_ID                              142
#define FE_D1_ID                              143
#define FE_A2_ID                              144
#define FE_D2_ID                              145
#define FE_A3_ID                              146
#define FE_D3_ID                              147
#define FE_A4_ID                              148
#define FE_D4_ID                              149
#define FE_A5_ID                              150
#define FE_D5_ID                              151
#define FE_A6_ID                              152
#define FE_D6_ID                              153
#define FE_A7_ID                              154
#define FE_D7_ID                              155
#define FE_A8_ID                              156
#define FE_D8_ID                              157
#define FE_A9_ID                              158
#define FE_D9_ID                              159
#define FE_A10_ID                             160
#define FE_D10_ID                             161
#define FE_A11_ID                             162
#define FE_D11_ID                             163
#define FE_A12_ID                             164
#define FE_D12_ID                             165
#define FE_A13_ID                             166
#define FE_D13_ID                             167
#define FE_A14_ID                             168
#define FE_D14_ID                             169
#define FE_A15_ID                             170
#define FE_D15_ID                             171
#define FE_A16_ID                             172
#define FE_D16_ID                             173
#define FE_A17_ID                             174
#define FE_D17_ID                             175
#define FE_A18_ID                             176
#define FE_D18_ID                             177
#define FE_A19_ID                             178
#define FE_D19_ID                             179
#define FE_A20_ID                             180
#define FE_D20_ID                             181
#define FE_A21_ID                             182
#define FE_D21_ID                             183
#define FE_A22_ID                             184
#define FE_D22_ID                             185
#define FE_A23_ID                             186
#define FE_D23_ID                             187
#define FE_A24_ID                             188
#define FE_D24_ID                             189
#define DEFAULT_MSGLIST_SCALING_ID            190
#define DEFAULT_MSGLIST_SCALING1_ID           191
#define DEFAULT_MSGLIST_SCALING2_ID           192
#define USB_DET_CONFIGURATION_ID              193
#define USB_DET_PIN_ID                        194
#define USB_DTE_CONFIGURATION_ID              195
#define NMEA_MSG_TASK_PRIORITY_2_ID           196
#define PPS_CLOCK_SETTING_ID                  197
#define GPS_MASK_ANGLE_POSITIONING_ID         198
#define DATUM_ID                              199


/*SW config section 2*/
#define APP_ON_OFF_ID                         200
#define NMEA_PORT_MSGLIST_L_ID                201
#define NCO_RANGE_MAX_ID                      202
#define NCO_RANGE_MIN_ID                      203
#define NCO_CENTER_ID                         204
#define DELAY_TO_NEXT_FIX_ID                  205
#define GPIO_PORT0_CFG0_ID                    206
#define GPIO_PORT0_CFG1_ID                    207
#define GPIO_PORT1_CFG0_ID                    208
#define GPIO_PORT1_CFG1_ID                    209
#define NMEA_PORT_MSGLIST1_L_ID               210
#define NMEA_PORT_MSGLIST2_L_ID               211
#define SBAS_SATELLITES_ENABLE_MASK_ID        212
#define PPS_OPERATING_MODE_SETTING_1_ID       213
#define PPS_OPERATING_MODE_SETTING_2_ID       214
#define PPS_AUTO_SURVEY_SAMPLES_ID            215
#define SBAS_AUTO_SEARCH_TIMEOUT_1_ID         216
#define SBAS_AUTO_SEARCH_TIMEOUT_2_ID         217
#define SBAS_SATELLITE_PARAM_1_ID             218
#define SBAS_SATELLITE_PARAM_2_ID             219
#define LOW_POWER_CFG_PARAMS_1_ID             220
#define LOW_POWER_CFG_PARAMS_2_ID             221
#define LMS_ALGO_CFG_PARAMS_1_ID              222
#define LMS_ALGO_CFG_PARAMS_2_ID              223
#define LOW_POWER_CFG_PARAMS_3_ID             224
#define ADC_CHAN_READ_CFG_PARAMS_ID           225
#define ADC_ANTENNA_SENSING_CFG_PARAMS_ID     226
#define APP_ON_OFF_2_ID                       227
#define NMEA_PORT_MSGLIST_H_ID                228
#define NMEA_PORT_MSGLIST_1_H_ID              229
#define NMEA_PORT_MSGLIST_2_H_ID              230
#define NMEA_ON_DEBUG_PORT_MSGLIST_L_ID       231
#define NMEA_ON_DEBUG_PORT_MSGLIST_H_ID       232
#define NMEA_ON_DEBUG_PORT_MSGLIST_1_L_ID     233
#define NMEA_ON_DEBUG_PORT_MSGLIST_1_H_ID     234
#define NMEA_ON_DEBUG_PORT_MSGLIST_2_L_ID     235
#define NMEA_ON_DEBUG_PORT_MSGLIST_2_H_ID     236
#define GPS_MIN_MAX_WEEK_NUMBER_ID            237
#define GPS_UTC_DEFAULT_SETTING_ID            238
#define SMART_XTAL_SETTING_1_ID               239
#define STBIN_MSGLIST_L_ID                    240
#define STBIN_MSGLIST_H_ID                    241
#define ANTENNA_SENSING_CFG_GPIO_PIN_ID       242
#define ANTENNA_SENSING_CFG_GPIO_MODE_ID      243
#define ANTENNA_SENSING_CFG_ACTIVE_LEVELS_ID  244
#define TCXO_CONFIG_SELECTOR_ID               245
#define NMEA_EXT_OPTIONS_ID                   246
#define DSP_CONFIG_OPTIONS_ID                 248
#define SPM_ID                                249
#define SPM_CONFIGURATION_ID                  250
#define SMART_XTAL_SETTING_2_ID               251
#define ANTENNA_SENSING_CFG_ADC_IN_ID         252
#define GPIO_PORT0_MODE_AFSLA_ID              253
#define GPIO_PORT0_MODE_AFSLB_ID              254
#define GPIO_PORT1_MODE_AFSLA_ID              255
#define GPIO_PORT1_MODE_AFSLB_ID              256
#define LOW_POWER_CFG_PARAMS_4_ID             257
#define LOW_POWER_CFG_PARAMS_5_ID             258
#define LOW_POWER_CFG_PARAMS_6_ID             259
#define WLS_CFG_PARAMS_1_ID                   260
#define DYNAMIC_MODE_CFG_PARAMS_ID            261
#define SHUTDN_CTRL_CFG_ID                    262
#define SPARE_263                             263

/*SW config section 3*/
#define NMEA_PORT_SLEEP_TIME_ID               300
#define PPS_PULSE_DURATION_ID                 301
#define RF_TIME_CORRECTION_ID                 302
#define GPS_FIX_RATE_ID                       303
#define POSITION_HOLD_LAT_ID                  304
#define POSITION_HOLD_LON_ID                  305
#define POSITION_HOLD_HEIGHT_ID               306
#define GPS_RF_TIME_CORRECTION_ID             307
#define GLONASS_RF_TIME_CORRECTION_ID         308
#define TIMING_TRAIM_ALARM_ID                 309
#define COMPASS_RF_TIME_CORRECTION_ID         310
#define RESERVED_311                          311
#define RESERVED_312                          312
#define RESERVED_313                          313

/*SW config section 4*/
#define DEFAULT_2D_DOPS_ID                    400
#define DEFAULT_3D_DOPS_ID                    401
#define STARTUP_2D_DOPS_ID                    402
#define STARTUP_3D_DOPS_ID                    403

#define TEXT_MESSAGE_ID                       500

#define DEFAULT_CONFIG                  0
#define SAVED_CONFIG                    1
#define MODIFIED_CONFIG                 2

#define CURRENT_CONFIG_DATA             1
#define DEFAULT_CONFIG_DATA             2
#define SAVED_CONFIG_DATA               3

#define DEFAULT_NOT_USED_FE_REGISTER    0xFF
#define DEFAULT_RF_REG_ADD              (0x80 | 0x06)
#define DEFAULT_RF_REG_VAL              0x3F

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct sw_config_dops_tag
{
  tU8   pdop;
  tU8   vdop;
  tU8   hdop;
  tU8   gdop;
}sw_config_dops_t;

typedef struct sw_config_tag
{
  tU8   debug_port_number;                      /*100*/
  tU8   nmea_port_number;                       /*101*/
  tU8   nmea_port_baudrate;                     /*102*/
  tU8   gnss_debug_mode;                        /*103*/
  tU8   gnss_mask_angle;                        /*104*/
  tU8   gnss_tracking_th;                       /*105*/
  tU8   debug_port_baudrate;                    /*106*/
  tU8   gnss_navigate_pps_task_priority;        /*107*/
  tU8   gnss_dsp_trk_task_priority;             /*108*/
  tU8   gnss_tracker_task_priority;             /*109*/
  tU8   gnss_dsp_acq_task_priority;             /*110*/
  tU8   gnss_flash_erase_priority;              /*111*/
  tU8   waas_task_priority;                     /*112*/
  tU8   waas_back_end_task_priority;            /*113*/
  tU8   st_agps_ephmgr_task_priority;           /*114*/
  tU8   nmea_cmdif_task_priority;               /*115*/
  tU8   gnss_navigate_task_priority;            /*116*/
  tU8   st_agps_task_priority;                  /*117*/
  tU8   backup_task_priority;                   /*118*/
  tU8   c2c_ist_task_priority;                  /*119*/
  tU8   cold_start_type;                        /*120*/
  tU8   nmea_speed_digit;                       /*121*/
  tU8   gnss_rtc_write_task_priority;           /*122*/
  tU8   stream_task_priority;                   /*123*/
  tU8   output_cfg;                             /*124*/
  tU8   notch_filter_cfg;                       /*125*/
  tU8   hardware_configuration;                 /*126*/
  tU8   nmea_position_digit;                    /*127*/
  tU8   differential_source;                    /*128*/
  tU8   glonass_output_format;                  /*129*/
  tU8   cpu_clock_speed;                        /*130*/
  tU8   nmea_talker_id;                         /*131*/
  tU8   gnss_positioning_th;                    /*132*/
  tU8   config_status;                          /*133*/
  tU8   config_ver;                             /*134*/
  tU8   sbas_prn;                               /*135*/
  tU8   T_SHORT_val;                            /*136*/
  tU8   T_LONG_val;                             /*137*/
  tU8   rtcm_port_number;                       /*138*/
  tU8   rtcm_port_baudrate;                     /*139*/
  tU8   fe_A0;                                  /*140*/
  tU8   fe_D0;                                  /*141*/
  tU8   fe_A1;                                  /*142*/
  tU8   fe_D1;                                  /*143*/
  tU8   fe_A2;                                  /*144*/
  tU8   fe_D2;                                  /*145*/
  tU8   fe_A3;                                  /*146*/
  tU8   fe_D3;                                  /*147*/
  tU8   fe_A4;                                  /*148*/
  tU8   fe_D4;                                  /*149*/
  tU8   fe_A5;                                  /*150*/
  tU8   fe_D5;                                  /*151*/
  tU8   fe_A6;                                  /*152*/
  tU8   fe_D6;                                  /*153*/
  tU8   fe_A7;                                  /*154*/
  tU8   fe_D7;                                  /*155*/
  tU8   fe_A8;                                  /*156*/
  tU8   fe_D8;                                  /*157*/
  tU8   fe_A9;                                  /*158*/
  tU8   fe_D9;                                  /*159*/
  tU8   fe_A10;                                 /*160*/
  tU8   fe_D10;                                 /*161*/
  tU8   fe_A11;                                 /*162*/
  tU8   fe_D11;                                 /*163*/
  tU8   fe_A12;                                 /*164*/
  tU8   fe_D12;                                 /*165*/
  tU8   fe_A13;                                 /*166*/
  tU8   fe_D13;                                 /*167*/
  tU8   fe_A14;                                 /*168*/
  tU8   fe_D14;                                 /*169*/
  tU8   fe_A15;                                 /*170*/
  tU8   fe_D15;                                 /*171*/
  tU8   fe_A16;                                 /*172*/
  tU8   fe_D16;                                 /*173*/
  tU8   fe_A17;                                 /*174*/
  tU8   fe_D17;                                 /*175*/
  tU8   fe_A18;                                 /*176*/
  tU8   fe_D18;                                 /*177*/
  tU8   fe_A19;                                 /*178*/
  tU8   fe_D19;                                 /*179*/
  tU8   fe_A20;                                 /*180*/
  tU8   fe_D20;                                 /*181*/
  tU8   fe_A21;                                 /*182*/
  tU8   fe_D21;                                 /*183*/
  tU8   fe_A22;                                 /*184*/
  tU8   fe_D22;                                 /*185*/
  tU8   fe_A23;                                 /*186*/
  tU8   fe_D23;                                 /*187*/
  tU8   fe_A24;                                 /*188*/
  tU8   fe_D24;                                 /*189*/
  tU8   msglist_scaling;                        /*190*/
  tU8   msglist_scaling_1;                      /*191*/
  tU8   msglist_scaling_2;                      /*192*/
  tU8   usb_det_configuration;                  /*193*/
  tU8   usb_det_pin;                            /*194*/
  tU8   usb_dte_configuration;                  /*195*/
  tU8   nmea_outmsg2_task_priority;             /*196*/
  tU8   pps_clock_setting;                      /*197*/
  tU8   gnss_mask_angle_positioning;            /*198*/
  tU8   datum_select;                           /*199*/
  tU8   spare_byte4;                            /*1100*/
  tU8   spare_byte5;                            /*1101*/
  tU8   spare_byte6;                            /*1102*/
  tU8   spare_byte7;                            /*1103*/

  tUInt app_on_off;                             /*200*/
  tUInt nmea_port_msglist_l;                    /*201*/
  tInt  nco_range_max;                          /*202*/
  tInt  nco_range_min;                          /*203*/
  tInt  nco_center;                             /*204*/
  tInt  delay_to_next_fix;                      /*205*/
  tUInt gpio_port0_cfg0;                        /*206*/
  tUInt gpio_port0_cfg1;                        /*207*/
  tUInt gpio_port1_cfg0;                        /*208*/
  tUInt gpio_port1_cfg1;                        /*209*/
  tUInt nmea_port_msglist_1_l;                  /*210*/
  tUInt nmea_port_msglist_2_l;                  /*211*/
  tUInt sbas_satellites_enable_mask;            /*212*/
  tUInt pps_operating_mode_setting_1;           /*213*/
  tUInt pps_operating_mode_setting_2;           /*214*/
  tUInt pps_autosurvey_samples;                 /*215*/
  tUInt sbas_autosearch_timeout_1;              /*216*/
  tUInt sbas_autosearch_timeout_2;              /*217*/
  tUInt sbas_satellite_parameter_1;             /*218*/
  tUInt sbas_satellite_parameter_2;             /*219*/
  tUInt low_power_cfg_params_1;                 /*220*/
  tUInt low_power_cfg_params_2;                 /*221*/
  tUInt lms_algo_cfg_params_1;                  /*222*/
  tUInt lms_algo_cfg_params_2;                  /*223*/
  tUInt low_power_cfg_params_3;                 /*224*/
  tUInt adc_chan_read_cfg_params;               /*225*/
  tUInt adc_antenna_sensing_cfg_params;         /*226*/
  tUInt app_on_off_2;                           /*227*/
  tUInt nmea_port_msglist_h;                    /*228*/
  tUInt nmea_port_msglist_1_h;                  /*229*/
  tUInt nmea_port_msglist_2_h;                  /*230*/
  tUInt nmea_on_debug_port_msglist_l;           /*231*/
  tUInt nmea_on_debug_port_msglist_h;           /*232*/
  tUInt nmea_on_debug_port_msglist_1_l;         /*233*/
  tUInt nmea_on_debug_port_msglist_1_h;         /*234*/
  tUInt nmea_on_debug_port_msglist_2_l;         /*235*/
  tUInt nmea_on_debug_port_msglist_2_h;         /*236*/
  tUInt gps_min_max_week_number;                /*237*/
  tUInt gps_utc_default_setting;                /*238*/
  tUInt smart_xtal_setting_1;                   /*239*/
  tUInt stbin_msglist_l;                        /*240*/
  tUInt stbin_msglist_h;                        /*241*/
  tUInt antenna_sensing_cfg_gpio_pin;           /*242*/
  tUInt antenna_sensing_cfg_gpio_mode;          /*243*/
  tUInt antenna_sensing_cfg_active_levels;      /*244*/
  tUInt tcxo_config_selector;                   /*245*/
  tUInt nmea_ext_options;                       /*246*/
  tUInt reserved_247;                           /*247*/
  tUInt dsp_config_options;                     /*248*/
  tUInt spm;                                    /*249*/
  tUInt spm_configuration;                      /*250*/
  tUInt smart_xtal_setting_2;                   /*251*/
  tUInt antenna_sensing_cfg_adc_in;             /*252*/
  tUInt gpio_port0_mode_AFSLA;                  /*253*/
  tUInt gpio_port0_mode_AFSLB;                  /*254*/
  tUInt gpio_port1_mode_AFSLA;                  /*255*/
  tUInt gpio_port1_mode_AFSLB;                  /*256*/
  tUInt low_power_cfg_params_4;                 /*257*/
  tUInt low_power_cfg_params_5;                 /*258*/
  tUInt low_power_cfg_params_6;                 /*259*/
  tUInt wls_cfg_params_1;                       /*260*/
  tUInt dynamic_mode_cfg_params;                /*261*/
  tUInt shutdn_ctrl_cfg;                        /*262*/
  tUInt spare_word_263;                         /*263*/

  tDouble nmea_port_sleep_time;                 /*300*/
  tDouble pps_pulse_duration;                   /*301*/
  tDouble rf_time_correction;                   /*302*/
  tDouble gps_fix_rate;                         /*303*/
  tDouble position_hold_lat;                    /*304*/
  tDouble position_hold_lon;                    /*305*/
  tDouble position_hold_height;                 /*306*/
  tDouble gps_rf_time_correction;               /*307*/
  tDouble glonass_rf_time_correction;           /*308*/
  tDouble timing_traim_alarm;                   /*309*/
  tDouble compass_rf_time_correction;           /*310*/
  tDouble reserved_311;                         /*311*/
  tDouble reserved_312;                         /*312*/
  tDouble reserved_313;                         /*313*/

  sw_config_dops_t  dops_default_2D;            /*400*/
  sw_config_dops_t  dops_default_3D;            /*401*/
  sw_config_dops_t  dops_startup_2D;            /*402*/
  sw_config_dops_t  dops_startup_3D;            /*403*/

  tChar   text_message[SW_CONFIG_TEXT_LENGTH];  /*500*/
#ifdef SW_CONFIG_PRIVATE_BLOCK
  sw_config_private_block_t private_config_block;
#endif
}sw_config_t;


typedef struct sw_config_key_tag
{
  tUInt   code[4];
} sw_config_key_t;

typedef struct sw_config_package_tag
{
  tU8               items[8];
  sw_config_key_t   key;
  sw_config_t       config;
  tUInt sw_config_checksum;
} sw_config_package_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

extern void           sw_config_init                              ( void);
extern void           sw_config_reload                            ( void);
extern gnss_error_t   sw_config_get_param                         ( tU8 config_type,tInt param_id,void *value);
extern gnss_error_t   sw_config_set_param                         ( tInt param_id,void *value,tInt mode);
extern tUInt          sw_config_get_software_switch_status        ( tUInt mask);
extern void           sw_config_set_software_switch_status        ( tUInt mask, tInt status);
extern tUInt          sw_config_get_software_switch_status_by_id  ( tInt param_id, tUInt mask);
extern void           sw_config_set_software_switch_status_by_id  ( tInt param_id, tUInt mask, tInt status);
extern gnss_error_t   sw_config_save_param                        ( void);
extern gnss_error_t   sw_config_restore_param                     ( void);
extern void           sw_config_print                             ( void);
extern gnss_error_t   sw_config_get_default                       ( sw_config_t **);
extern gnss_error_t   sw_config_get_current                       ( sw_config_t **);
extern gnss_error_t   sw_config_data_block_copy                   ( tU8 config_type, sw_config_t *dest_sw_config_ptr);
extern gnss_error_t   sw_config_data_block_write                  ( tU16 length,tU16 offset, void *value);
extern tUInt          sw_config_convert_baudrate                  ( const tU8 baud_rate_id);
#ifdef __cplusplus
}
#endif
#endif
