/*****************************************************************************
   FILE:          platform.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The code for STA8090 SoC
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
*****************************************************************************/

/*!
 * @file    platform.c
 * @brief   Platform module for SoC (System On Chip)
 */

/*****************************************************************************
   includes
*****************************************************************************/

// general
#include "macros.h"
#include "fixpoint.h"

// LLD for STA8090
#include "lld.h"
#include "lld_arm946.h"
#include "lld_prcc_sta8090.h"
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

/*
 *  Select CS used for NOR and RAM
 */
#define PLATFORM_NOR_CS       LLD_FSMC_SRAMNOR_BANK0
#define PLATFORM_SRAM_CS      LLD_FSMC_SRAMNOR_BANK1

/*
 *  Select which NOR and which RAM will be used
 */
#define PLATFORM_NOR_TYPE     PLATFORM_NOR_SPANSION_S29WS064R
#define PLATFORM_RAM_TYPE     PLATFORM_RAM_SPANSION_S71VS064R

/*
 *  Select size of NOR used by FSMC service. Typically it is 2MB
 */
#define PLATFORM_NOR_SIZE     0x200000

/*
 *  Setup some constants from linker symbols
 */

#if defined( __ARMCC_VERSION)
extern tUInt Image$$CODE$$ZI$$Base;
extern tUInt Image$$DTCM_AREA$$ZI$$Limit;
extern tUInt Image$$DTCM_AREA_LIMIT$$Base;
extern tUInt Image$$OS_STACK_AREA$$Base;

#define FAST_MEMORY_BASE      ((void *)((tUInt)&Image$$DTCM_AREA$$ZI$$Limit))
#define FAST_MEMORY_SIZE      (((tUInt)&Image$$DTCM_AREA_LIMIT$$Base) - ((tUInt)&Image$$DTCM_AREA$$ZI$$Limit))
#define FAST_MEMORY_AVAILABLE (boolean_t)( ((tUInt)&Image$$OS_STACK_AREA$$Base == (tUInt)&Image$$DTCM_AREA_LIMIT$$Base) ? FALSE : TRUE)

#define LOAD_REGION_BASE      (tUInt)(&Image$$CODE$$ZI$$Base)
#define DTCM_AREA_LIMIT       (tUInt)(&Image$$DTCM_AREA_LIMIT$$Base)
#endif

#if defined( __GNUC__)
extern tUInt __code_area_start__;
extern tUInt __dtcm_area_end__;
extern tUInt __dtcm_area_limit__;
extern tUInt __os_heap_area_start__;

#define FAST_MEMORY_BASE      ((void *)((tUInt)&__dtcm_area_end__))
#define FAST_MEMORY_SIZE      (((tUInt)&__dtcm_area_limit__) - ((tUInt)&__dtcm_area_end__))
#define FAST_MEMORY_AVAILABLE (boolean_t)( ((tUInt)&__os_heap_area_start__ == (tUInt)&__dtcm_area_end__) ? FALSE : TRUE)

#define LOAD_REGION_BASE      (tUInt)(&__code_area_start__)
#define DTCM_AREA_LIMIT       (tUInt)(&__dtcm_area_limit__)
#endif

#define PLATFORM_CODE_EXEC_NOR          (((LOAD_REGION_BASE >> 24) == 0x20) ? TRUE : FALSE)
#define PLATFORM_CODE_EXEC_RAM          (((LOAD_REGION_BASE >> 24) == 0x24) ? TRUE : FALSE)
#define PLATFORM_CODE_EXEC_SQI          (((LOAD_REGION_BASE >> 24) == 0x10) ? TRUE : FALSE)
#define PLATFORM_NVM_NOR                ((((tUInt)&platform_nvm_memory >> 24) == 0x20) ? TRUE : FALSE)
#define PLATFORM_NVM_RAM                ((((tUInt)&platform_nvm_memory >> 24) == 0x24) ? TRUE : FALSE)
#define PLATFORM_NVM_BCKPRAM            ((((tUInt)&platform_nvm_memory >> 24) == 0x40) ? TRUE : FALSE)
#define PLATFORM_NVM_SQI                ((((tUInt)&platform_nvm_memory >> 24) == 0x10) ? TRUE : FALSE)
#define PLATFORM_HEAP_EXTERNAL          ((FAST_MEMORY_AVAILABLE == TRUE) ? TRUE : FALSE)

#define PLATFORM_NOR_NEEDED             (((PLATFORM_CODE_EXEC_NOR == TRUE) || (PLATFORM_NVM_NOR == TRUE)) ? TRUE : FALSE)
#define PLATFORM_RAM_NEEDED             (((PLATFORM_CODE_EXEC_RAM == TRUE) || (PLATFORM_NVM_RAM == TRUE) || (PLATFORM_HEAP_EXTERNAL == TRUE)) ? TRUE : FALSE)
#define PLATFORM_BCKPRAM_NEEDED         ((PLATFORM_NVM_BCKPRAM == TRUE) ? TRUE : FALSE)
#define PLATFORM_SQI_NEEDED             (((PLATFORM_CODE_EXEC_SQI == TRUE) || (PLATFORM_NVM_SQI == TRUE)) ? TRUE : FALSE)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*
 *  List of tested NOR parts
 */
typedef enum platform_nortype_e
{
  PLATFORM_NOR_MUXED16,
  PLATFORM_NOR_NOTMUXED16,
  PLATFORM_NOR_MUXED8,
  PLATFORM_NOR_NOTMUXED8,

  PLATFORM_NOR_MICRON_M29DW323DB,
  PLATFORM_NOR_MICRON_M36W0R,
  PLATFORM_NOR_MICRON_M36W0T,
  PLATFORM_NOR_SPANSION_S29AL016J,
  PLATFORM_NOR_SPANSION_S29GL064S,
  PLATFORM_NOR_SPANSION_S29JL032J,
  PLATFORM_NOR_SPANSION_S29WS064R,
  PLATFORM_NOR_SPANSION_S71VS064R,
  PLATFORM_NOR_SPANSION_S98WS064R,
  PLATFORM_NOR_WINBOND_W29GL064C,

  PLATFORM_NOR_CUSTOM,

  PLATFORM_NOR_NUMBER
} platform_nortype_t;

/*
 *  List of tested RAM parts
 */
typedef enum platform_ramtype_e
{
  PLATFORM_RAM_MUXED16,
  PLATFORM_RAM_NOTMUXED16,
  PLATFORM_RAM_MUXED8,
  PLATFORM_RAM_NOTMUXED8,

  PLATFORM_RAM_ISSI_IS67WVExx16EBLL,
  PLATFORM_RAM_MICRON_M36W0R,
  PLATFORM_RAM_MICRON_M36W0T,
  PLATFORM_RAM_SPANSION_S71VS064R,
  PLATFORM_RAM_SPANSION_S98WS064R,

  PLATFORM_RAM_CUSTOM,

  PLATFORM_RAM_NUMBER
} platform_ramtype_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*
 *  Table of settings for tested NOR parts
 */
static const GENERIC_NORELOC tU32 platform_nor_fsmc_cfg_table[PLATFORM_NOR_NUMBER][2] =
{
  { 0x000010db, 0x00000404 },   /* MUXED16              */
  { 0x000010d9, 0x00000404 },   /* NOTMUXED16           */
  { 0x000010cb, 0x00000404 },   /* MUXED8               */
  { 0x000010c9, 0x00000404 },   /* NOTMUXED8            */

  { 0x000010d9, 0x01000511 },   /* MICRON_M29DW323DB    */
  { 0x000010d9, 0x01000511 },   /* MICRON_M36W0R        */
  { 0x000010d9, 0x01000511 },   /* MICRON_M36W0T        */
  { 0x000010d9, 0x00000301 },   /* SPANSION_S29AL016J   */
  { 0x000010d9, 0x00000301 },   /* SPANSION_S29GL064S   */
  { 0x000010d9, 0x00000301 },   /* SPANSION_S29JL032J   */
  { 0x000010d9, 0x00000301 },   /* SPANSION_S29WS064R   */
  { 0x000030db, 0x01000511 },   /* SPANSION_S71VS064R   */
  { 0x000010d9, 0x00000201 },   /* SPANSION_S98WS064R   */
  { 0x000010d9, 0x00000300 },   /* WINBOND_W29GL064C    */

  { 0x00000000, 0x00000000 }    /* Custom field */
};

/*
 *  Table of settings for tested RAM parts
 */
static const GENERIC_NORELOC tU32 platform_ram_fsmc_cfg_table[PLATFORM_RAM_NUMBER][2] =
{
  { 0x000010d3, 0x00000404 },   /* MUXED16              */
  { 0x000010d1, 0x00000404 },   /* NOTMUXED16           */
  { 0x000010c3, 0x00000404 },   /* MUXED8               */
  { 0x000010c1, 0x00000404 },   /* NOTMUXED8            */

  { 0x000030d1, 0x00000201 },   /* ISSI_IS67WVExx16EBLL */
  { 0x000010d1, 0x00000404 },   /* MICRON_M36W0R        */
  { 0x000010d1, 0x00000404 },   /* MICRON_M36W0T        */
  { 0x000030d7, 0x00000211 },   /* SPANSION_S71VS064R   */
  { 0x000010d1, 0x00000311 },   /* SPANSION_S98WS064R   */

  { 0x00000000, 0x00000000 }    /* Custom field */
};

static const LLD_ARM946_MemoryDescTy platform_mputable_per      = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_2GB,   (tU16)LLD_ARM946_GETBASEADDR( 0x0),         0};   // Peripherals
static const LLD_ARM946_MemoryDescTy platform_mputable_sqi      = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_1MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x10000000),  0};   // SQI memory for code
static const LLD_ARM946_MemoryDescTy platform_mputable_nor      = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_1MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x20000000),  0};   // NOR CS0 code
//static const LLD_ARM946_MemoryDescTy platform_mputable_nor      = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_2MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x20000000),  0};   // NOR CS0 code and data
static const LLD_ARM946_MemoryDescTy platform_mputable_ram      = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_2MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x24000000),  0};   // RAM CS1 code and data
static const LLD_ARM946_MemoryDescTy platform_mputable_bckpram  = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_32KB,  (tU16)LLD_ARM946_GETBASEADDR( 0x40000000),  0};   // Backup RAM for NVM
#if defined( NVM_SQI_CACHED )
static const LLD_ARM946_MemoryDescTy platform_mputable_sqi_nvm  = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, TRUE,  TRUE,  (tU8)LLD_ARM946_MEMORYSIZE_1MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x10100000),  0};   // SQI memory for NVM
#else
static const LLD_ARM946_MemoryDescTy platform_mputable_sqi_nvm  = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_1MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x10100000),  0};   // SQI memory for NVM
#endif
static const LLD_ARM946_MemoryDescTy platform_mputable_nor_nvm  = /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */{ 0, FALSE, FALSE, (tU8)LLD_ARM946_MEMORYSIZE_2MB,   (tU16)LLD_ARM946_GETBASEADDR( 0x20000000),  0};   // NOR CS0 code

static tUInt             platform_nvm_memory __attribute__((section("NVM_DATA_REGION")));

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
  LLD_ARM946_MemoryDescTy memory_table[LLD_ARM946_MEMORYREGIONS];
  /*lint -e{960} Rule 10.1, Implicit conversion changes signedness */
  LLD_ARM946_MemoryDescTy clean_region = {0, FALSE, FALSE, LLD_ARM946_MEMORYSIZE_NULL,  0, 0};
  tU32 pos = 0;
  tUInt *mt_u32p = (tUInt *)&memory_table[0];

  LLD_ARM946_DisableMPU();
  LLD_ARM946_FlushICache();
  LLD_ARM946_FlushDCache();

  // First region is always dedicated to first 2 GB for peripherals
  memory_table[pos] = platform_mputable_per;
  memory_table[pos].BIT.pos = pos;
  pos++;

  if( PLATFORM_NVM_NOR == TRUE)
  {
    memory_table[pos] = platform_mputable_nor_nvm;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  if( PLATFORM_NVM_SQI == TRUE)
  {
    memory_table[pos] = platform_mputable_sqi_nvm;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  if( PLATFORM_BCKPRAM_NEEDED == TRUE)
  {
    memory_table[pos] = platform_mputable_bckpram;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  if( PLATFORM_RAM_NEEDED == TRUE)
  {
    memory_table[pos] = platform_mputable_ram;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  if( PLATFORM_CODE_EXEC_SQI == TRUE)
  {
    memory_table[pos] = platform_mputable_sqi;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  if( PLATFORM_CODE_EXEC_NOR == TRUE)
  {
    memory_table[pos] = platform_mputable_nor;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  for( ; pos < LLD_ARM946_MEMORYREGIONS; )
  {
    memory_table[pos] = clean_region;
    memory_table[pos].BIT.pos = pos;
    pos++;
  }

  for( pos = 0; pos < LLD_ARM946_MEMORYREGIONS; pos++)
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
  WRITE32( FSMC_REG_START_ADDR + 0x00U, platform_nor_fsmc_cfg_table[PLATFORM_NOR_TYPE][0]);
  WRITE32( FSMC_REG_START_ADDR + 0x04U, platform_nor_fsmc_cfg_table[PLATFORM_NOR_TYPE][1]);

  WRITE32( FSMC_REG_START_ADDR + 0x08U, platform_ram_fsmc_cfg_table[PLATFORM_RAM_TYPE][0]);
  WRITE32( FSMC_REG_START_ADDR + 0x0cU, platform_ram_fsmc_cfg_table[PLATFORM_RAM_TYPE][1]);

  WRITE32( FSMC_REG_START_ADDR + 0x10U, 0x00000000);
  WRITE32( FSMC_REG_START_ADDR + 0x14U, 0x00000000);
}

/********************************************//**
 * \brief   Change protection unit parameter
 *
 * \param   param_ptr   mask:
 *                        4-2 > region to change
 *                        1   > new i cache status
 *                        0   > new d cache status
 * \return  void
 *
 ***********************************************/
void platform_set_fsmc_cache( gpOS_syscall_param_t param_ptr)
{
  LLD_ARM946_MemoryDescTy curr_region;
  tU32 pos;

  // Disable caching of FSMC to allow CFI configuration
  for( pos = 0; pos < LLD_ARM946_MEMORYREGIONS; pos++)
  {
    LLD_ARM946_GetRegion( pos, &curr_region);
    if(
        (curr_region.BIT.base_addr  == platform_mputable_nor.BIT.base_addr) &&
        (curr_region.BIT.size       == platform_mputable_nor.BIT.size)
      )
    {
      break;
    }
  }

  if( param_ptr == NULL)
  {
    LLD_ARM946_SetRegionCache( pos, FALSE, FALSE);
  }
  else
  {
    LLD_ARM946_SetRegionCache( pos, TRUE, TRUE);
  }
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
  boolean_t nor_needed = PLATFORM_NOR_NEEDED;
  boolean_t ram_needed = PLATFORM_RAM_NEEDED;
  boolean_t sqi_needed = PLATFORM_SQI_NEEDED;

  if( nor_needed || ram_needed)
  {
    if( nor_needed)
    {
      LLD_FSMC_CfgTy local_cfg;

      local_cfg.config.REG  = platform_nor_fsmc_cfg_table[PLATFORM_NOR_TYPE][0];
      local_cfg.timings.REG = platform_nor_fsmc_cfg_table[PLATFORM_NOR_TYPE][1];

      LLD_FSMC_ConfigureBank( PLATFORM_NOR_CS, &local_cfg);
    }
    else
    {
      WRITE32( FSMC_REG_START_ADDR + 0x00, 0x00000000);
      WRITE32( FSMC_REG_START_ADDR + 0x04, 0x00000000);
    }

    if( ram_needed)
    {
      LLD_FSMC_CfgTy local_cfg;

      local_cfg.config.REG  = platform_ram_fsmc_cfg_table[PLATFORM_RAM_TYPE][0];
      local_cfg.timings.REG = platform_ram_fsmc_cfg_table[PLATFORM_RAM_TYPE][1];

      LLD_FSMC_ConfigureBank( PLATFORM_SRAM_CS, &local_cfg);
    }
    else
    {
      WRITE32( FSMC_REG_START_ADDR + 0x08, 0x00000000);
      WRITE32( FSMC_REG_START_ADDR + 0x0c, 0x00000000);
    }
  }
  else
  {
    LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_FSMC);
  }

  if( sqi_needed == FALSE)
  {
    LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_SQIO);
  }

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
  if( FAST_MEMORY_AVAILABLE == TRUE)
  {
    *base_ptr = (void *)FAST_MEMORY_BASE;
    *size_ptr = (tU32)FAST_MEMORY_SIZE;
  }
  else
  {
    *base_ptr = (void *)NULL;
    *size_ptr = 0;
  }
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
  // Setup FSMC service if needed
  if( PLATFORM_NVM_NOR)
  {
    gpOS_kernel_user_system_call( platform_set_fsmc_cache, (void *)0);

    svc_fsmc_nor_init( part);
    svc_fsmc_nor_create_region( LLD_FSMC_SRAMNOR_BANK0, FSMC_BANK0_START_ADDR, PLATFORM_NOR_SIZE);
    svc_fsmc_nor_reset_region( (void *)FSMC_BANK0_START_ADDR, PLATFORM_NOR_SIZE);

    // Enable caching of FSMC back
    gpOS_kernel_user_system_call( platform_set_fsmc_cache, (void *)1);
  }

  // Setup SQI service if needed
  if( PLATFORM_NVM_SQI)
  {
    LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_SQIO);

    svc_sqi_init( part);
    svc_sqi_create_region( SQI_START_ADDR, 0x1000000, spm_conf);
    svc_sqi_reset_region( (void *)SQI_START_ADDR);
  }

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
  #if defined( NVM_NOR ) || defined( NVM_SQI )
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
  return FALSE;
}

/********************************************//**
 * \brief
 *
 * \param spm tU32
 * \param spm_configuration tU32
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t platform_sw_protection_setup( tU32 spm, tU32 spm_configuration)
{
  return gpOS_SUCCESS;
}
