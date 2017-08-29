#ifndef GNSS_BSP_DEFS_H
#define GNSS_BSP_DEFS_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "fixpoint.h"
#include "gnss_const.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define TRK_CHANNELS_SUPPORTED  24U

#define RAM_CODES_ENABLED

#define GNSS_BSP_CONST(X)                 (gnss_tcxo_const.X)

#define F_XTAL                            GNSS_BSP_CONST(tcxo_config.f_xtal)
#define GPS_RDIV                          GNSS_BSP_CONST(tcxo_config.rdiv)
#define GPS_NDIV                          GNSS_BSP_CONST(tcxo_config.ndiv)
#define GPS_16F0_FREQ                     GNSS_BSP_CONST(gps_16f0_freq_f)
#define GPS_CLOCK_OFFSET                  GNSS_BSP_CONST(gps_clock_offset_f)
#define GLONASS_IF_FREQ                   GNSS_BSP_CONST(glonass_if_freq_f)
#define COMPASS_IF_FREQ                   GNSS_BSP_CONST(compass_if_freq_f)
#define COMPASS_SEL_SPEC_INV              GNSS_BSP_CONST(compass_sel_spec_inv)
#define COMPASS_CLOCK_OFFSET              GNSS_BSP_CONST(compass_clock_offset_f)
#define L2C_IF_FREQ                       GNSS_BSP_CONST(l2c_if_freq_f)
#define GLO_L2_IF_FREQ                    GNSS_BSP_CONST(glo_l2_if_freq_f)

#define DEFAULT_CODE_ONE                  GNSS_BSP_CONST(default_code_one_i)
#define DEFAULT_CODE_ONE_HALF             GNSS_BSP_CONST(default_code_one_half_i)
#define CARRIER_NCO_RATIO                 GNSS_BSP_CONST(carrier_nco_ratio_f)
#define DSP_NCO_TO_HZ_FP                  GNSS_BSP_CONST(dsp_nco_to_hz_fp_i)
#define DSP_NCO_TO_HZ_1000_FP             GNSS_BSP_CONST(dsp_nco_to_hz_fp_1000_u)
#define SLV_TICKS_PER_MSEC                GNSS_BSP_CONST(slv_ticks_per_msec_f)
#define PRECISE_SLV_TICKS_PER_SECOND      GNSS_BSP_CONST(precise_slv_ticks_per_second_f)

#define SLV_TICKS_PER_SECOND              slv_ticks_per_second
#define MST_TICKS_PER_SECOND              mst_ticks_per_second
#define GNSS_BSP_TICKS_PER_SECOND         (gnss_bsp_ticks_per_second)
#define GNSS_BSP_TICKS_PER_MSEC           (gnss_bsp_ticks_per_msec)
#define TRACKER_CPU_TICKS_TO_MSEC_FP(x)   gnss_bsp_tracker_cpu_ticks_to_msec(x)

#define L2C_CLOCK_OFFSET                  (GPS_CLOCK_OFFSET * L2_TRANSMITTED_FREQ / GPS_TRANSMITTED_FREQ)
#define GLONASS_CLOCK_OFFSET(channel)     (GPS_CLOCK_OFFSET * (GLONASS_TRANSMITTED_FREQ + GLONASS_CHANNEL_SPACING*(channel)) / GPS_TRANSMITTED_FREQ)
#define GLO_L2_CLOCK_OFFSET(channel)      (GPS_CLOCK_OFFSET * (GLO_L2_TRANSMITTED_FREQ + GLO_L2_CHANNEL_SPACING*(channel)) / GPS_TRANSMITTED_FREQ)

#define CHAN_ID_VALID(chan_id)            ((((chan_id) >= 0) && ((chan_id) < (chanid_t)TRK_CHANNELS_SUPPORTED)) ? TRUE:FALSE)

#define TRACKER_CPU_TICKS_PER_SECOND      SLV_TICKS_PER_SECOND
#define TRACKER_CPU_TICKS_PER_MSEC        SLV_TICKS_PER_MSEC
#define NAV_CPU_TICKS_PER_SECOND          MST_TICKS_PER_SECOND

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef enum
{
  GNSS_BSP_RF_POWER_MODE_ON,
  GNSS_BSP_RF_POWER_MODE_IF_OFF,
  GNSS_BSP_RF_POWER_MODE_RF_OFF
} gnss_bsp_rf_power_mode_t;

typedef struct dsp_duty_cycle_synch_data_s
{
  tU32  MTU_value;
  tU16  MTB_L_glitch_pos;
  tU16  MTB_H;
  tU16  MTB_L;
} dsp_duty_cycle_synch_data_t;

typedef enum
{
  GNSS_BSP_T2_CUT_AA    = 0x0201,   /* Use second Byte as Teseo number */
  GNSS_BSP_T2_CUT_BB,
  GNSS_BSP_T2_CUT_BC,
  GNSS_BSP_T3_CUT_AA    = 0x0301,
  GNSS_BSP_T3_CUT_AB,
  GNSS_BSP_T3_CUT_BA,
  GNSS_BSP_T3_CUT_BB,
  GNSS_BSP_T3_CUT_BC,
  GNSS_BSP_T3_CUT_BD,
  GNSS_BSP_CUT_UNKNOWN  = 0xFFFF
} gnss_bsp_cut_t;

typedef struct
{
  tUInt   f_xtal;
  tInt    ndiv;
  tInt    rdiv;
  tInt    r_lo2;
} gnss_tcxo_config_t;

typedef struct
{
  gnss_tcxo_config_t  tcxo_config;

  tDouble             gps_16f0_freq_f;
  tDouble             carrier_nco_ratio_f;
  tDouble             gps_clock_offset_f;

  tDouble             glonass_if_freq_f;

  tDouble             compass_if_freq_f;
  tInt                compass_sel_spec_inv;
  tDouble             compass_clock_offset_f;

  tInt                default_code_one_i;
  tInt                default_code_one_half_i;

  tInt                dsp_nco_to_hz_fp_i;
  tInt                dsp_nco_to_hz_fp_1000_u;

  tDouble             slv_ticks_per_second_f;
  tDouble             slv_ticks_per_msec_f;
  tDouble             precise_slv_ticks_per_second_f;
  tDouble             mst_ticks_per_second_f;
} gnss_tcxo_const_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

extern tDouble            mst_ticks_per_second;
extern tDouble            slv_ticks_per_second;
extern tUInt              gnss_bsp_ticks_per_second;
extern tUInt              gnss_bsp_ticks_per_msec;
extern gnss_tcxo_const_t  gnss_tcxo_const;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* GNSS_BSP_DEFS_H */

