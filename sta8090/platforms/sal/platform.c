/*****************************************************************************
   FILE:          platform.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The code for STA8090 SAL
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
*****************************************************************************/

/*!
 * @file    platform.c
 * @brief   Platform module for SAL (Standalone)
 */

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"

// LLD for STA8088
#include "lld.h"
#include "lld_prcc_sta8090.h"
#include "lld_arm946.h"
#include "lld_gpio.h"
#include "lld_fsmc_ctrl.h"
#include "lld_fsmc_nor.h"
#include "lld_sqi_ctrl.h"

#include "gps_flash.h"
#include "gpOS.h"

#include "svc_fsmc.h"
#include "svc_sqi.h"
//#include "svc_ssp.h"
//#include "svc_i2c.h"
//#include "svc_gpio.h"
#include "svc_pwr.h"
#include "svc_fsw.h"

#include "platform.h"
#include "gnssapp.h"
#include "gnssapp_plugins.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#if defined( __ARMCC_VERSION)
extern tUInt Image$$DTCM_AREA_LIMIT$$Base;

#define DTCM_AREA_LIMIT       (tUInt)(&Image$$DTCM_AREA_LIMIT$$Base)
#endif

#if defined( __GNUC__)
extern tUInt __dtcm_area_limit__;

#define DTCM_AREA_LIMIT       (tUInt)(&__dtcm_area_limit__)
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static const LLD_ARM946_MemoryDescTy memory_table[LLD_ARM946_MEMORYREGIONS] =
/*lint -e{960} Rule 10.1, Implicit conversion changes signedness */
{
  { 0, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_2GB,    (tU16)LLD_ARM946_GETBASEADDR( 0x0        ), 0},   // Peripherals
  { 1, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_1MB,    (tU16)LLD_ARM946_GETBASEADDR( 0x10000000 ), 0},   // SQI memory for code
#if defined( NVM_SQI )
  { 2, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_64KB,   (tU16)LLD_ARM946_GETBASEADDR( 0x100F0000 ), 0},   // SQI writeable data
#endif
#if defined( NVM_RAM )
  { 2, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_32KB,   (tU16)LLD_ARM946_GETBASEADDR( 0x40000000 ), 0},   // Backup RAM for NVM
#endif
  { 3, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_NULL,   (tU16)LLD_ARM946_GETBASEADDR( 0x0        ), 0},
  { 4, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_NULL,   (tU16)LLD_ARM946_GETBASEADDR( 0x0        ), 0},
  { 5, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_NULL,   (tU16)LLD_ARM946_GETBASEADDR( 0x0        ), 0},
  { 6, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_NULL,   (tU16)LLD_ARM946_GETBASEADDR( 0x0        ), 0},
  { 7, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_NULL,   (tU16)LLD_ARM946_GETBASEADDR( 0x0        ), 0},
};

static tUInt platform_nvm_memory __attribute__((section("NVM_DATA_REGION")));

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Set up protection unit
 *
 * \param dummy void*
 * \return void
 *
 ***********************************************/
static void platform_setup_mpu( void *dummy)
{
  tU32 cnt;
  tUInt *mt_u32p = (tUInt *)&memory_table[0];

  LLD_ARM946_DisableMPU();
  LLD_ARM946_FlushICache();
  LLD_ARM946_FlushDCache();

  for( cnt = 0; cnt < LLD_ARM946_MEMORYREGIONS; cnt++)
  {
    LLD_ARM946_SetRegion( *((LLD_ARM946_MemoryDescTy *)mt_u32p++));
  }

  LLD_ARM946_EnableMPU();
}

/********************************************//**
 * \brief   Setup TCM based on scatter file
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
static void platform_setup_tcm( void)
{
  register tUInt itcm_cfg, dtcm_cfg;
  tUInt cnt;
  tUInt dtcm_size;

  itcm_cfg = (tU32)ITCM_START_ADDR | 0x12U;
  dtcm_cfg = (tU32)DTCM_START_ADDR | 0x12U;

  #if defined( __ARMCC_VERSION)
  #pragma arm
  __asm
  {
    mcr   p15,0,dtcm_cfg,c9,c1,0
    mcr   p15,0,itcm_cfg,c9,c1,1
    mrc   p15,0,r0,c1,c0,0
    mov   r2,#0x00050000
    orr   r0,r0,r2
    mcr   p15,0,r0,c1,c0,0
  }
  #endif
  #if defined( __GNUC__)
  __asm volatile(
    INLINE_ASM_ENTER
    "\
    mcr   p15,0,%0,c9,c1,0        ;\
    mcr   p15,0,%1,c9,c1,1        ;\
    mrc   p15,0,r0,c1,c0,0        ;\
    ldr   r2,=0x00050000          ;\
    orr   r0,r0,r2                ;\
    mcr   p15,0,r0,c1,c0,0        ;\
    "
    INLINE_ASM_EXIT
    :
    : "r"(dtcm_cfg), "r"(itcm_cfg)
    : "r0", "r2", "r3"
  );
  #endif

  cnt = 7;

  dtcm_size = DTCM_AREA_LIMIT;  // Splitted in two instructions to avoid warnings
  dtcm_size = ((dtcm_size - (tUInt)DTCM_START_ADDR) - (tUInt)DTCM_SIZE_MIN);

  while( dtcm_size > 0U)
  {
    dtcm_size -= (tUInt)TCM_BLOCK_SIZE;
    cnt--;
  }

  LLD_PRCC_SetTCMSize( (LLD_PRCC_TCMCfgTy)cnt);
}

/********************************************//**
 * \brief   Enable only needed peripherals at reset
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
static void platform_setup_min_per( void)
{
  LLD_PRCC_PeriphMaskTy per_mask;

  per_mask =
    (1U << LLD_PRCC_PERIPHID_ARM)             |
    (1U << LLD_PRCC_PERIPHID_FSMC)            |
    (1U << LLD_PRCC_PERIPHID_SQIO)            |
    (1U << LLD_PRCC_PERIPHID_SRAM)            |
    (1U << LLD_PRCC_PERIPHID_GPIO0)           |
    (1U << LLD_PRCC_PERIPHID_GPIO1)           |
    (1U << LLD_PRCC_PERIPHID_AHB)             |
    (1U << LLD_PRCC_PERIPHID_APB1)            |
    (1U << LLD_PRCC_PERIPHID_APB2)            |
    (1U << LLD_PRCC_PERIPHID_RTC)             |
    (1U << LLD_PRCC_PERIPHID_UART1)           |
    (1U << LLD_PRCC_PERIPHID_CLK_CTRL_FSM);

  LLD_PRCC_DisablePerMask( ~per_mask);
  LLD_PRCC_EnablePerMask( per_mask);
}

/********************************************//**
 * \brief   Setup memory after reset
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
static void platform_setup_memory( void)
{
  LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_FSMC);
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Reset platform
 *
 * \param   void
 * \return  void
 * \note    This code is and must be executed in supervisor mode
 *
 ***********************************************/
void platform_setup( void)
{
  // Enable LDO2
  LLD_PRCC_LDOEnable( LLD_PRCC_LDO2);

  LLD_PRCC_SetCoreVoltage(LLD_PRCC_COREVOLTAGE_1_2V);

  // Configure clock sources
  LLD_PRCC_RFChipEnable();
  LLD_PRCC_RFEnable();
  LLD_PRCC_SetSQICLkDiv( LLD_PRCC_SQICLKDIV_DIV1);
  LLD_PRCC_SetHCLKClkDiv( LLD_PRCC_HCLKDIV_DIV1);
  LLD_PRCC_SetARMClkSrcStatus( LLD_PRCC_ARMCLKSRC_TCXO, TRUE);
  LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_TCXO, FALSE);

  // Enable only peripherals that are needed at startup
  platform_setup_min_per();

  // Setup MPU
  platform_setup_mpu( NULL);

  // Setup TCMs
  platform_setup_tcm();

  // Setup memories
  platform_setup_memory();
}

/********************************************//**
 * \brief   Configure platform
 *
 * \param   void
 * \return  LLD_NO_ERROR if successful, LLD_ERROR otherwise
 *
 ***********************************************/
gpOS_error_t platform_config( void)
{
  tUInt i;
  LLD_PRCC_DisableReset();

  LLD_PRCC_SetOscillator( TRUE);
  #if defined( WAIT_FOR_OSCI_OK)
  while(LLD_PRCC_GetOscillatorStatus() == FALSE);
  #else
  for( i = 0U; i < PLATFORM_WAIT_FOR_OSCI_OK; i++)
  {
    MCR_NOP();
  }
  #endif

  LLD_PRCC_SetRTCStatus( TRUE);
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_RTC);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Get parameters for fast partition
 *
 * \param   base_ptr  pointer to fast partition base address value
 * \param   size_ptr  pointer to size of partition
 * \return  void
 *
 ***********************************************/
void platform_get_fast_partition( void **base_ptr, tU32 *size_ptr)
{
    *base_ptr = (void *)NULL;
    *size_ptr = 0;
  }

/********************************************//**
 * \brief   Start specific platform services
 *
 * \param   part  pointer to custom partition, NULL for std heap
 * \param   spm_conf Software Protection Mode configuration value
 * \return  gpOS_error_t
 *
 ***********************************************/
gpOS_error_t platform_start_services( gpOS_partition_t *part, tU8 spm_conf)
{
  // Setup SQI service
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_SQIO);

  svc_sqi_init( part);
  svc_sqi_create_region( SQI_START_ADDR, 0x1000000, spm_conf);
  svc_sqi_reset_region( (void *)SQI_START_ADDR);

  /* Init PWR service */
  svc_pwr_init( part);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Returns NVM region pointers based on nvm size
 *
 * \param[out] nvm_primary_addr_ptr   Pointer for primary region
 * \param[out] nvm_secondary_addr_ptr Pointer for secondary region
 * \param[out] nvm_size               size of each region
 * \return void
 *
 * \note If region is in RAM, secondary pointer is NULL
 *
 ***********************************************/
void platform_gnss_get_nvm_config( tUInt *nvm_primary_addr_ptr, tUInt *nvm_secondary_addr_ptr, tUInt nvm_size)
{
  #if defined( NVM_SQI )
  flash_init( NULL, (tU32 *)&platform_nvm_memory, nvm_size * 2);

  *nvm_primary_addr_ptr = (tUInt)&platform_nvm_memory;
  *nvm_secondary_addr_ptr = ((tUInt)&platform_nvm_memory) + nvm_size;
  #endif

  #if defined( NVM_RAM )
  *nvm_primary_addr_ptr = (tUInt)&platform_nvm_memory;
  *nvm_secondary_addr_ptr = (tUInt)NULL;
  #endif
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return tBool
 *
 ***********************************************/
tBool platform_get_sal_ext_sqi_status( void)
{
  return (tBool)(LLD_PRCC_GetExtSQIStatus());
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t platform_sw_protection_setup( tU32 spm, tU32 spm_configuration)
{
  tU8 spm_conf;
  gpOS_error_t error;

  error = svc_sqi_get_sw_protection( &spm_conf);

  if( error == gpOS_FAILURE)
  {
    return error;
  }

  if(( spm > 0U) && ( spm_conf != spm_configuration))
  {
    svc_sqi_set_sw_protection( (tU8)spm_configuration);
  }
  else if(( spm == 0U) && ( spm_conf != 0U))
  {
    svc_sqi_set_sw_protection( 0U);
  }
  else
  {
    /* Nothing to do */
  }

  return gpOS_SUCCESS;
}

void platform_set_fsmc_cache( gpOS_syscall_param_t param)
{
  // Do nothing for SAL
}
