/*****************************************************************************
   FILE:          platform_common.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The common code for all STA8090 platforms
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 2007.07.25

*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"

#include "platform.h"

// OS related
#include "gpOS.h"

// LLD for STA8090
#include "lld.h"
#include "lld_vic.h"
#include "lld_prcc_sta8090.h"
#include "lld_gpio.h"
#include "lld_mtu.h"

// Modules needed for STA8090
#include "svc_ssp.h"
#include "svc_i2c.h"
#include "svc_fsmc.h"
#include "svc_sqi.h"
#include "frontend.h"
#include "gnss_defs.h"
#include "gnss_api.h"
#include "gnss_debug.h"

// For runtime switch
#include "svc_mcu.h"
#include "svc_fsw.h"
#include "svc_pwr.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define PLATFORM_PLL_MX              0x18            /**< PLL multiplier */

#define PLATFORM_WORKING_HW_SETTINGS 0xA3F5          /**< Verified HW settings in case of missing initialization */

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static platform_tcxo_speed_t tcxo_speed = PLATFORM_TCXO_SPEED_26MHZ;
static tUInt pwr_high_low_configs = 0U;

// Clocks and speed configuration => this must always be sorted in decreasing order: first one is the highest ARM frequency
static svc_mcu_clkcfg_t platform_clkcfg_table[] =
{
  { PLATFORM_CLKCFG_192F0, LLD_PRCC_ARMCLKSRC_192f0, 0, FALSE, 0, PLATFORM_BUS_FREQ_48F0,  {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_96F0,  LLD_PRCC_ARMCLKSRC_192f0, 2, FALSE, 1, PLATFORM_BUS_FREQ_48F0,  {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_64F0,  LLD_PRCC_ARMCLKSRC_192f0, 3, FALSE, 2, PLATFORM_BUS_FREQ_48F0,  {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_55MHZ, LLD_PRCC_ARMCLKSRC_TCXO,  0, FALSE, 0, PLATFORM_BUS_FREQ_55MHZ, {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_48F0,  LLD_PRCC_ARMCLKSRC_192f0, 4, FALSE, 3, PLATFORM_BUS_FREQ_48F0,  {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_48MHZ, LLD_PRCC_ARMCLKSRC_TCXO,  0, FALSE, 0, PLATFORM_BUS_FREQ_48MHZ, {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_26MHZ, LLD_PRCC_ARMCLKSRC_TCXO,  0, FALSE, 0, PLATFORM_BUS_FREQ_26MHZ, {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 1 /* Will set TRACKER_CPU_TICKS_PER_SECOND later */}},
  { PLATFORM_CLKCFG_4F0,   LLD_PRCC_ARMCLKSRC_RING,  0, FALSE, 0, PLATFORM_BUS_FREQ_2F0,   {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 0}},
  { PLATFORM_CLKCFG_2F0,   LLD_PRCC_ARMCLKSRC_RING,  1, FALSE, 1, PLATFORM_BUS_FREQ_2F0,   {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 0}},
  { PLATFORM_CLKCFG_F0,    LLD_PRCC_ARMCLKSRC_RING,  2, FALSE, 2, PLATFORM_BUS_FREQ_F0,    {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 0}},
  { PLATFORM_CLKCFG_RESET, LLD_PRCC_ARMCLKSRC_RING, 30, FALSE, 0, PLATFORM_BUS_FREQ_RESET, {(tS32)PLATFORM_FACTOR_F0, LLD_MTU_PRESCALER_DIV_1, 0}},    // Dummy configuration for init
};

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief   Set clock speed
 *
 * \param   source  Clock source
 * \param   speed   Clock divider
 * \return  void
 *
 ***********************************************/
static void platform_set_clkcfg( svc_mcu_clkcfg_t *cfg_ptr, tBool increase_freq)
{
  // Configure proper clock
  if( cfg_ptr->clk_source == LLD_PRCC_ARMCLKSRC_192f0)
  {
    LLD_PRCC_SetARMClkSrcStatus( LLD_PRCC_ARMCLKSRC_192f0, TRUE);

    if( cfg_ptr->corefreqid == PLATFORM_CLKCFG_48F0)
    {
      #if defined(NVM_SQI)
      LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
      #endif
      if (increase_freq == TRUE)
      {
        LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
      }
      LLD_PRCC_SetARMClkDiv( LLD_PRCC_ARMCLKSRC_192f0, 4);
      LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_192f0, TRUE);
      if (increase_freq == FALSE)
      {
        LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
      }
    }
    else if( cfg_ptr->corefreqid == PLATFORM_CLKCFG_64F0)
    {
      #if defined(NVM_SQI)
      LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
      #endif
      if (increase_freq == TRUE)
      {
        LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
      }
      LLD_PRCC_SetARMClkDiv( LLD_PRCC_ARMCLKSRC_192f0, 3);
      LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_192f0, TRUE);
      if (increase_freq == FALSE)
      {
        LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
      }
    }
    else if( cfg_ptr->corefreqid == PLATFORM_CLKCFG_96F0)
    {
      #if defined(NVM_SQI)
      LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
      #endif
      if (increase_freq == TRUE)
      {
        LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV2);
      }
      LLD_PRCC_SetARMClkDiv( LLD_PRCC_ARMCLKSRC_192f0, 2);
      LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_192f0, TRUE);
      if (increase_freq == FALSE)
      {
        LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV2);
      }
    }
    else if( cfg_ptr->corefreqid == PLATFORM_CLKCFG_192F0)
    {
      #if defined(NVM_SQI)
      LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV2);
      #endif
      LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV4);
      LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_192f0, FALSE);
    }
    else
    {
      /* Should not end here */
    }
  }
  else if( cfg_ptr->clk_source == LLD_PRCC_ARMCLKSRC_TCXO)
  {
    LLD_PRCC_SetARMClkSrcStatus( LLD_PRCC_ARMCLKSRC_TCXO, TRUE);

    LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_TCXO, FALSE);

    #if defined(NVM_SQI)
    LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
    #endif
    LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
  }
  else if( cfg_ptr->clk_source == LLD_PRCC_ARMCLKSRC_RING)
  {
    tUInt i;

    LLD_PRCC_SetOscillator( TRUE);
    #if defined( WAIT_FOR_OSCI_OK)
    while(LLD_PRCC_GetOscillatorStatus() == FALSE);
    #else
    for( i = 0U; i < PLATFORM_WAIT_FOR_OSCI_OK; i++)
    {
      MCR_NOP();
    }
    #endif

    LLD_PRCC_SetARMClkSrcStatus( LLD_PRCC_ARMCLKSRC_RING, TRUE);

    if( cfg_ptr->clk_divider > 0U)
    {
      LLD_PRCC_SetARMClkDiv( LLD_PRCC_ARMCLKSRC_RING, cfg_ptr->clk_divider);
      LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_RING, TRUE);
      LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
    }
    else
    {
      LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_RING, FALSE);
      LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV2);
    }

    #if defined(NVM_SQI)
    LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
    #endif
  }
  else
  {
    /* Should not end here */
  }
}

/********************************************//**
 * \brief   Set LDO and SMPS config
 *
 * \param   config  ARM frequency config
 * \param   speed   Clock divider
 * \return  void
 *
 ***********************************************/
static void platform_set_pwrcfg( svc_mcu_corefreqcfg_t config )
{
  tUInt temp_bits;
  tUInt offset;
  tUInt temp_pwr_high_low_configs;
  boolean_t BigMos;
/* The config is made of a 32 bits word.
4 bits for each power supply - SMPS or LDO
2 bits to describe the state of the related LDO during high frequency period
2 bits to describe the state of the related LDO during low frequency period */

  if((pwr_high_low_configs == 0U) || (pwr_high_low_configs == 1U))
  {
    /* Impossible value - nothing will work if we apply this value */
    /* Set a working value */
    temp_pwr_high_low_configs = PLATFORM_WORKING_HW_SETTINGS;
  }
  else
  {
    temp_pwr_high_low_configs = pwr_high_low_configs;
  }

  if(config < PLATFORM_CLKCFG_MINTCXOCFG)
  {
    /* Apply High frequency power settings */
    offset = 0U;
    BigMos = TRUE;
  }
  else
  {
    /* Apply Low frequency power settings */
    offset = 2U;
    BigMos = FALSE;
  }

  /* SWITCH ON POWER SUPPLIES FIRST */

  /* Backup LDO */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_BKUPLDO_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits != 0U)
  {
    platform_set_stop_mode_status(1U);
  }

  /* LDO1 */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_LDO1_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits != 0U)
  {
    LLD_PRCC_CoreVoltageTy CoreVoltage = LLD_PRCC_COREVOLTAGE_1_2V;

    LLD_PRCC_LDOEnable(LLD_PRCC_LDO1);
    /* 01 = 1.0V 02 = 1.1V 03 = 1.2V */
    if(temp_bits == 1U)
    {
      CoreVoltage = LLD_PRCC_COREVOLTAGE_1_0V;
    }
    else if(temp_bits == 2U)
    {
      CoreVoltage = LLD_PRCC_COREVOLTAGE_1_1V;
    }
    else if(temp_bits == 3U)
    {
      CoreVoltage = LLD_PRCC_COREVOLTAGE_1_2V;
    }
    else
    {
      /* Should not end here */
    }
    LLD_PRCC_SetCoreVoltage(CoreVoltage);
  }

  /* LDO2 */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_LDO2_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits != 0U)
  {
    LLD_PRCC_LDOEnable(LLD_PRCC_LDO2);
  }

  /* SMPS */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_SMPS_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits != 0U)
  {
    LLD_PRCC_CoreVoltageTy CoreVoltage = LLD_PRCC_COREVOLTAGE_1_2V;

    /* 01 = 1.0V 02 = 1.1V 03 = 1.2V */
    if(temp_bits == 1U)
    {
      CoreVoltage = LLD_PRCC_COREVOLTAGE_1_0V;
    }
    else if(temp_bits == 2U)
    {
      CoreVoltage = LLD_PRCC_COREVOLTAGE_1_1V;
    }
    else if(temp_bits == 3U)
    {
      CoreVoltage = LLD_PRCC_COREVOLTAGE_1_2V;
    }
    else
    {
      /* Should not end here */
    }
    LLD_PRCC_EnableSMPS(CoreVoltage, BigMos);
  }

  /* SWITCH OFF POWER SUPPLIES */

  /* Backup LDO */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_BKUPLDO_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits == 0U)
  {
    platform_set_stop_mode_status(0U);
  }

  /* LDO1 */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_LDO1_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits == 0U)
  {
    LLD_PRCC_LDODisable(LLD_PRCC_LDO1);
  }

  /* LDO2 */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_LDO2_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits == 0U)
  {
    LLD_PRCC_LDODisable(LLD_PRCC_LDO2);
  }

  /* SMPS */
  temp_bits = (temp_pwr_high_low_configs >> (PLATFORM_PWR_SMPS_BITNB + offset)) & PLATFORM_PWR_MASK_2BITS;
  if(temp_bits == 0U)
  {
    LLD_PRCC_DisableSMPS();
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Setup mcu
 *
 * \param   source  tcxo clock
 * \return  void
 *
 ***********************************************/
gpOS_error_t platform_mcu_init( gpOS_partition_t *part)
{
  const svc_fsw_cfg_t fsw_cfg =
  {
    platform_clkcfg_table,
    PLATFORM_CLKCFG_RESET,
    platform_set_clkcfg,
    platform_set_pwrcfg,
    PLATFORM_BUSCLK_ID_MCLK,
    PLATFORM_CLKCFG_LPCFG,
    PLATFORM_FREQ_RATIO_MULTIPLIER,
    PLATFORM_BUS_FREQ_4F0,
    0U
  };

  /* Init MCU service */
  svc_mcu_init( part, PLATFORM_BUSCLK_ID_MCLK);

  /* Init Frequency switch service */
  svc_fsw_init( part, &fsw_cfg);
  svc_fsw_setextfreq( TRACKER_CPU_TICKS_PER_SECOND);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Setup mcu
 *
 * \param   source  tcxo clock
 * \return  void
 *
 ***********************************************/
void platform_mcu_setspeed( platform_mcu_speed_t tcxo_clock)
{
  switch( tcxo_clock )
  {
    case PLATFORM_MCU_SPEED_26MHZ:
      tcxo_speed = PLATFORM_TCXO_SPEED_26MHZ;
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_MCLK, 26000000U);
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_USB_CLK, 0U);
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_CAN_CLK, 26000000U);
    break;

    case PLATFORM_MCU_SPEED_48MHZ:
      tcxo_speed = PLATFORM_TCXO_SPEED_48MHZ;
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_MCLK, 48000000U);
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_USB_CLK, 48000000U);
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_CAN_CLK, 48000000U);
    break;

    case PLATFORM_MCU_SPEED_55MHZ:
      tcxo_speed = PLATFORM_TCXO_SPEED_55MHZ;
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_MCLK, 55000000U);
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_USB_CLK, 55000000U);
      svc_mcu_busclk_set( PLATFORM_BUSCLK_ID_CAN_CLK, 55000000U);
    break;

    default:
    break;
  }
}

/********************************************//**
 * \brief   Get tcxo speed
 *
 * \param   void
 * \return  tUInt tcxo clock
 *
 ***********************************************/
platform_tcxo_speed_t platform_get_tcxo_speed( tVoid)
{
  return tcxo_speed;
}

/********************************************//**
 * \brief   Set clock speed
 *
 * \param   source  Clock source
 * \param   speed   Clock divider
 * \return  void
 *
 ***********************************************/
void platform_set_cpu_clock_speed( tInt source, tInt divisor)
{

  if( (LLD_PRCC_ARMClkSrcTy)source == LLD_PRCC_ARMCLKSRC_192f0)
  {
    if( (LLD_PRCC_ClkSelTy)divisor == LLD_PRCC_CLKSEL_48)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_48F0 , TRUE);
    }
    else if( (LLD_PRCC_ClkSelTy)divisor == LLD_PRCC_CLKSEL_64)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_64F0 , TRUE);
    }
    else if( (LLD_PRCC_ClkSelTy)divisor == LLD_PRCC_CLKSEL_96)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_96F0 , TRUE);
    }
    else if( (LLD_PRCC_ClkSelTy)divisor == LLD_PRCC_CLKSEL_192)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_192F0 , TRUE);
    }
    else
    {
      /* Should not end there */
    }
  }
  else if( (LLD_PRCC_ARMClkSrcTy)source == LLD_PRCC_ARMCLKSRC_TCXO)
  {
    if(tcxo_speed == PLATFORM_TCXO_SPEED_26MHZ)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_26MHZ , TRUE);
    }
    else if(tcxo_speed == PLATFORM_TCXO_SPEED_48MHZ)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_48MHZ , TRUE);
    }
    else if(tcxo_speed == PLATFORM_TCXO_SPEED_55MHZ)
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_55MHZ , TRUE);
    }
    else
    {
      /* Should not end there */
    }
  }
  else if( (LLD_PRCC_ARMClkSrcTy)source == LLD_PRCC_ARMCLKSRC_RING)
  {
    if( divisor > 0)
    {
      // Only one configuration supported with divider > 0
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_2F0 , TRUE);
    }
    else
    {
      svc_fsw_set_corefreq(PLATFORM_CLKCFG_4F0 , TRUE);
    }
  }
  else
  {
    gpOS_bsp_timer_cfg_t timer_config;

    // This code is not managed by "platform_set_clkcfg()" because it does not correspond to any requested configuration for now.
    LLD_PRCC_SetARMClkSrcStatus( (LLD_PRCC_ARMClkSrcTy)source, TRUE);
    #if defined(NVM_SQI)
    LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
    #endif
    LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);

    if( divisor > 0)
    {
      LLD_PRCC_SetARMClkDiv( (LLD_PRCC_ARMClkSrcTy)source, divisor+1);
      LLD_PRCC_SelectARMClkSrc( (LLD_PRCC_ARMClkSrcTy)source, TRUE);
    }
    else
    {
      LLD_PRCC_SelectARMClkSrc( (LLD_PRCC_ARMClkSrcTy)source, FALSE);
    }

    timer_config.prescaler = LLD_MTU_PRESCALER_DIV_1; // This value could be adjusted depending on divider value.
    timer_config.ext_freq = TRACKER_CPU_TICKS_PER_SECOND; // This value could be modified if no GNSS clock.

    gpOS_timer_set_clock( &timer_config, true);
  }
}

/********************************************//**
 * \brief   Get clock configuration
 *
 * \param   source_ptr  Pointer where to save clock source
 * \param   speed_ptr   Pointer where to save clock divider
 * \return  void
 *
 ***********************************************/
void platform_get_clock_configuration_status( tInt *clk_source_ptr, tInt *clk_arm_speed_ptr)
{
  svc_mcu_clkcfg_t* curr_clkcfg;

  curr_clkcfg = svc_fsw_get_clkcfg();

  *clk_source_ptr   = curr_clkcfg->clk_source;
  *clk_arm_speed_ptr  = curr_clkcfg->clk_arm_speed;
}

/********************************************//**
 * \brief   Initialize USB signals
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void platform_usb_sense_init( void)
{
#ifdef USB_LINKED
  SET32_BIT( PRCC_REG_START_ADDR + 0x40, BIT_21);
  SET32_BIT(( PRCC_REG_START_ADDR + 0x44), BIT_21);
  CLEAR32_BIT(( PRCC_REG_START_ADDR + 0x14), BIT_0);

  WRITE32(( UART1_REG_START_ADDR + 0x88), 0);
  WRITE32(( UART1_REG_START_ADDR + 0x80), 1);
#endif
}

/********************************************//**
 * \brief   Gets BOOT pins configuration
 *
 * \param   tU32 * boot pin configuration
 * \return  tU32 index
 *
 ***********************************************/
tU32 platform_get_boot_pins_config( tU32 *boot_pins)
{
  tU32 index = 0;
  tU32 boot_p = 0;

  boot_p = LLD_PRCC_GetBOOTPins();

/* BOOT pin 0 is inverted in cuts AA and AB */

  boot_p ^= 0x1U; // this instruction must be removed when the inversion is no more present on the HW

  *boot_pins = boot_p;

  if(boot_p < 4U)
  {
    index = 0;
  }
  else
  {
    index = boot_p - 3U;
  }

  return index;
}

/********************************************//**
 * \brief   Execute FW upgrade procedure
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void platform_fwupgrade( void)
{
  LLD_PRCC_ARMBootTy arm_boot;
  tInt invalid_word = 0;

  LLD_MTU_Reset( (LLD_MTU_IPTy)MTU0_REG_START_ADDR );
  arm_boot = LLD_PRCC_GetHWBootStatus();

  switch ( arm_boot )
  {
    case LLD_PRCC_ARMBOOT_FSMC:
      gpOS_kernel_user_system_call( platform_set_fsmc_cache, (void *)0 );
      svc_fsmc_nor_unlock( ( void * )0x20010000, 64 * 1024 );
      svc_fsmc_nor_write( ( void * )0x20010000, &invalid_word, sizeof( invalid_word ) );
      break;

    case LLD_PRCC_ARMBOOT_SQI:
      // Check if Software Protection Mode is enabled
      {
        gpOS_error_t error;
        tU8 spm_conf;

        error = svc_sqi_get_sw_protection( &spm_conf);

        if(( error == gpOS_SUCCESS) && ( spm_conf != 0U))
        {
          svc_sqi_set_sw_protection( 0);
        }
      }
      svc_sqi_write( ( void * )0x10010000, &invalid_word, sizeof( invalid_word ) );
      break;

    default:
      /* Nothing to do */
      break;
  }

  LLD_PRCC_GenerateSoftReset();
}

/********************************************//**
 * \brief   Initialize platform
 *
 * \return  gpOS_SUCCESS
 *
 ***********************************************/
gpOS_error_t platform_gnss_init( void)
{
  LLD_PRCC_RFChipEnable();
  LLD_PRCC_RFEnable();
  LLD_PRCC_SetExternalRF( FALSE);

  LLD_PRCC_LDOEnable( LLD_PRCC_LDO2);
  LLD_PRCC_RFRegRstEn();

  // Initialize SPIoSSP module
  svc_ssp_open_port( 0, gpOS_INTERRUPT_NOPRIORITY);

  FE_init( FE_PER_IF_SSP);

  LLD_VIC_EnableChannel( (LLD_VIC_ChannelTy)VIC_SSP_LINE);
  FE_reset();

  // Enable RTC peripheral
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_RTC);

  platform_gnss_restart();

  return( gpOS_SUCCESS);
}

/********************************************//**
 * \brief   Configure PPS for specific speed
 *
 * \param   pps_cfg   PPS clock speed (in MHz)
 * \return  void
 *
 ***********************************************/
void platform_gnss_set_pps_clock( tU8 pps_cfg)
{
  #if 0
  // Apply configuration from library
  FE_def_write_bits( FE_PPS_CLK_SETTING_REG_INDEX, 0xC0, ((~((pps_cfg >> 4) - 1)) << 6) & 0xff);
  #endif
}

/********************************************//**
 * \brief   Actions to do after GNSS library suspension
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void platform_gnss_suspend( void)
{

}

/********************************************//**
 * \brief   Actions to do before GNSS library restart
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void platform_gnss_restart( void)
{
  // Enable all G3E+ clocks
  LLD_PRCC_RFChipEnable();
  LLD_PRCC_RFEnable();
  LLD_PRCC_G3EPClkEnable();
  LLD_PRCC_G3TBClkEnable();

  LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_G3EP_AHB);
  LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_G3EP);

  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_G3EP);
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_G3EP_AHB);
}

/********************************************//**
 * \brief   Actions to do after GNSS library suspension
 *
 * \param   tU8 on_off status
 * \return  void
 *
 ***********************************************/
void platform_set_stop_mode_status( tU8 on_off)
{
  if(on_off != 0U)
  {
    LLD_PRCC_StopModeEnable();
  }
  else
  {
    LLD_PRCC_StopModeDisable();
  }
}

/********************************************//**
 * \brief   Set LDO and SMPS config
 *
 * \param   config  ARM frequency config
 * \param   speed   Clock divider
 * \return  void
 *
 ***********************************************/
void platform_set_pwr_high_low_configs( tUInt high_low_configs )
{
  pwr_high_low_configs = high_low_configs;
}
