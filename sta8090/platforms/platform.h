/*****************************************************************************
   FILE:          platform.h
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   This is the header that describes APIs for actions that
                  depends on boards and package
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      FB:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   07.07.23  |  FB  | Original version
*****************************************************************************/

/*!
 * @file    platform.h
 * @brief   Platform module header file
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"
#include "lld_prcc_sta8090.h"
#include "gpOS.h"
#include "gpOS_bsp_timer.h"
#include "gnss_api.h"
#include "svc_mcu.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

// ID used to determine the bus speed for each peripheral
#define PLATFORM_BUSCLK_ID_MCLK           0U
#define PLATFORM_BUSCLK_ID_USB_CLK        1U
#define PLATFORM_BUSCLK_ID_CAN_CLK        2U

// number of nop to wait in order to have 32KHz oscillator running and stable
#if defined (NVM_RAM)
#define PLATFORM_WAIT_FOR_OSCI_OK         (140000U*4U)
#else
#define PLATFORM_WAIT_FOR_OSCI_OK         (140000U*1U)
#endif

// Ratio used for frequency computation
#define PLATFORM_FREQ_RATIO_MULTIPLIER    ((tU32)1U << 25)

/* Definition of the organization of pwr_high_low_configs bits */
#define PLATFORM_PWR_BKUPLDO_BITNB  (0U)
#define PLATFORM_PWR_LDO1_BITNB     (4U)
#define PLATFORM_PWR_LDO2_BITNB     (8U)
#define PLATFORM_PWR_SMPS_BITNB     (12U)

#define PLATFORM_PWR_MASK_2BITS     (3U)

/* Available bus speed */
#define PLATFORM_BUS_FREQ_64F0      (64U * 1023000U)
#define PLATFORM_BUS_FREQ_55MHZ     (55U * 1000000U)
#define PLATFORM_BUS_FREQ_48F0      (48U * 1023000U)
#define PLATFORM_BUS_FREQ_48MHZ     (48U * 1000000U)
#define PLATFORM_BUS_FREQ_26MHZ     (26U * 1000000U)
#define PLATFORM_BUS_FREQ_4F0       (4092000U)
#define PLATFORM_BUS_FREQ_2F0       (2046000U)
#define PLATFORM_BUS_FREQ_F0        (1023000U)
#define PLATFORM_BUS_FREQ_RESET     (127875U)

/* Indicates the factor applied to F0 (multiplied by PLATFORM_FREQ_RATIO_MULTIPLIER) */
#define PLATFORM_FACTOR_F0          PLATFORM_FREQ_RATIO_MULTIPLIER
#define PLATFORM_FACTOR_1_625F0     ((tU32)(1.625 * PLATFORM_FREQ_RATIO_MULTIPLIER))
#define PLATFORM_FACTOR_2F0         ((tU32)(2 * PLATFORM_FREQ_RATIO_MULTIPLIER))
#define PLATFORM_FACTOR_4F0         ((tU32)(4 * PLATFORM_FREQ_RATIO_MULTIPLIER))

/* Available ARM frequency configurations */
#define PLATFORM_CLKCFG_192F0       0U
#define PLATFORM_CLKCFG_96F0        1U
#define PLATFORM_CLKCFG_64F0        2U
#define PLATFORM_CLKCFG_55MHZ       3U
#define PLATFORM_CLKCFG_48F0        4U
#define PLATFORM_CLKCFG_48MHZ       5U
#define PLATFORM_CLKCFG_26MHZ       6U
#define PLATFORM_CLKCFG_4F0         7U
#define PLATFORM_CLKCFG_2F0         8U
#define PLATFORM_CLKCFG_F0          9U
#define PLATFORM_CLKCFG_RESET       10U

/* This is a marker at the limit of TCXO and noTCXO configs */
#define PLATFORM_CLKCFG_MINTCXOCFG  PLATFORM_CLKCFG_26MHZ

/* Frequency used in Low Power case */
#define PLATFORM_CLKCFG_LPCFG       PLATFORM_CLKCFG_4F0

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/* Available TCXO values */
typedef enum
{
  PLATFORM_TCXO_SPEED_26MHZ = 26000000,
  PLATFORM_TCXO_SPEED_48MHZ = 48000000,
  PLATFORM_TCXO_SPEED_55MHZ = 55000000
} platform_tcxo_speed_t;

// Available MCU Speed values
typedef enum
{
  PLATFORM_MCU_SPEED_26MHZ  = 26000000,
  PLATFORM_MCU_SPEED_48MHZ  = 48000000,
  PLATFORM_MCU_SPEED_55MHZ  = 55000000
} platform_mcu_speed_t;

/*****************************************************************************
   exported variables
*****************************************************************************/
/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern void           platform_reset                          ( void);
extern void           platform_get_fast_partition             ( void **base_ptr, tU32 *size_ptr);
extern gpOS_error_t   platform_config                         ( void);
extern gpOS_error_t   platform_start_services                 ( gpOS_partition_t *, tU8);

extern void           platform_get_clock_configuration_status ( tInt *clk_source_ptr, tInt *clk_arm_speed_ptr);
extern tU32           platform_get_boot_pins_config           ( tU32 *);
extern void           platform_set_cpu_clock_speed            ( tInt source, tInt divisor);
extern void           platform_usb_sense_init                 ( void);

extern void           platform_fwupgrade                      ( void);
extern gpOS_error_t   platform_sw_protection_setup            ( tU32 spm, tU32 spm_configuration);

extern void           platform_set_fsmc_cache                 ( gpOS_syscall_param_t);

extern tBool          platform_get_sal_ext_sqi_status         ( void);

extern gpOS_error_t   platform_mcu_init                       ( gpOS_partition_t *part);
extern void           platform_mcu_setspeed                   ( platform_mcu_speed_t tcxo_clock);
extern platform_tcxo_speed_t  platform_get_tcxo_speed         ( tVoid);

extern gpOS_error_t   platform_gnss_init                      ( void);
extern void           platform_gnss_get_nvm_config            ( tUInt *nvm_primary_addr_ptr, tUInt *nvm_secondary_addr_ptr, tUInt nvm_size);
extern void           platform_gnss_suspend                   ( void);
extern void           platform_gnss_restart                   ( void);
extern void           platform_gnss_set_pps_clock             ( tU8 pps_cfg);
extern void           platform_gnss_notch_filter_config       ( tInt );
extern void           platform_gnss_notch_filter_debug        ( void);
extern void           platform_set_stop_mode_status           ( tU8 );

extern void           platform_set_pwr_high_low_configs       ( tUInt high_low_configs );
#endif /* PLATFORM_H */

