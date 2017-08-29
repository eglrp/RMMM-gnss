//!
//!   \file       lld_arm946.h
//!   \brief      <i><b>ARM946 Low Level Driver header file</b></i>
//!   \author     Fulvio Boggia
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup ARM946
//!   \{
//!

#ifndef LLD_ARM946_H
#define LLD_ARM946_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

//! \brief Returns base address to use in region table
#define LLD_ARM946_GETBASEADDR( address)    (((tU32)(address) >> 16U) & 0xffffU)

//! \brief Number of available memory regions in MPU
#define LLD_ARM946_MEMORYREGIONS            8

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief Available sizes in MPU configuration
typedef enum
{
  LLD_ARM946_MEMORYSIZE_NULL  = 0,      /**<  */
  LLD_ARM946_MEMORYSIZE_4KB   = 11,     /**<  */
  LLD_ARM946_MEMORYSIZE_8KB,            /**<  */
  LLD_ARM946_MEMORYSIZE_16KB,           /**<  */
  LLD_ARM946_MEMORYSIZE_32KB,           /**<  */
  LLD_ARM946_MEMORYSIZE_64KB,           /**<  */
  LLD_ARM946_MEMORYSIZE_128KB,          /**<  */
  LLD_ARM946_MEMORYSIZE_256KB,          /**<  */
  LLD_ARM946_MEMORYSIZE_512KB,          /**<  */
  LLD_ARM946_MEMORYSIZE_1MB,            /**<  */
  LLD_ARM946_MEMORYSIZE_2MB,            /**<  */
  LLD_ARM946_MEMORYSIZE_4MB,            /**<  */
  LLD_ARM946_MEMORYSIZE_8MB,            /**<  */
  LLD_ARM946_MEMORYSIZE_16MB,           /**<  */
  LLD_ARM946_MEMORYSIZE_32MB,           /**<  */
  LLD_ARM946_MEMORYSIZE_64MB,           /**<  */
  LLD_ARM946_MEMORYSIZE_128MB,          /**<  */
  LLD_ARM946_MEMORYSIZE_256MB,          /**<  */
  LLD_ARM946_MEMORYSIZE_512MB,          /**<  */
  LLD_ARM946_MEMORYSIZE_1GB,            /**<  */
  LLD_ARM946_MEMORYSIZE_2GB,            /**<  */
  LLD_ARM946_MEMORYSIZE_4GB             /**<  */
} LLD_ARM946_MemorySizeTy;

typedef enum
{
  LLD_ARM946_REGIONACCESS_SNO_UNO = 0x0,
  LLD_ARM946_REGIONACCESS_SRW_UNO = 0x1,
  LLD_ARM946_REGIONACCESS_SRW_URO = 0x2,
  LLD_ARM946_REGIONACCESS_SRW_URW = 0x3,
  LLD_ARM946_REGIONACCESS_SRO_UNO = 0x5,
  LLD_ARM946_REGIONACCESS_SRO_URO = 0x6
} LLD_ARM946_RegionAccessTy;

//! \brief MPU item descriptor
/*lint -estring(9018,*LLD_ARM946_MemoryDescTy*) suppress declaration of symbol with union based type */
typedef union
{
  struct
  {
    tUBitField  pos             : 3;        /**<  Position in MPU */
    tUBitField  inst_cacheable  : 1;        /**<  Defines if memory region must be cached for instructions or not */
    tUBitField  data_cacheable  : 1;        /**<  Defines if memory region must be cached for data or not */
    tUBitField  size            : 5;        /**<  Size of memory region
                                                \see LLD_ARM946_MemorySizeTy */
    tUBitField  base_addr       : 16;       /**<  Memory region base address
                                                \warning must be aligned to size specified */
    tUBitField  res             : 6;
  } BIT;
  tU32 REG;
} LLD_ARM946_MemoryDescTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid    LLD_ARM946_Reset                  ( tVoid);
extern tVoid    LLD_ARM946_InitMPU                ( tVoid);

extern tVoid    LLD_ARM946_EnableMPU              ( tVoid);
extern tVoid    LLD_ARM946_DisableMPU             ( tVoid);

extern tVoid    LLD_ARM946_SetRegion              ( const LLD_ARM946_MemoryDescTy);
extern tVoid    LLD_ARM946_GetRegion              ( const tUInt, LLD_ARM946_MemoryDescTy *);
extern tVoid    LLD_ARM946_RemoveRegion           ( const LLD_ARM946_MemoryDescTy);
extern tVoid    LLD_ARM946_SetRegionCache         ( const tUInt, const tBool, const tBool);
extern tVoid    LLD_ARM946_SetRegionAccess        ( const tUInt, const LLD_ARM946_RegionAccessTy);
extern tVoid    LLD_ARM946_EnableRegion           ( const tUInt);
extern tVoid    LLD_ARM946_DisableRegion          ( const tUInt);

extern tVoid    LLD_ARM946_EnterWFI               ( tVoid);

extern tVoid    LLD_ARM946_FlushICache            ( tVoid);
extern tVoid    LLD_ARM946_FlushDCache            ( tVoid);

extern tVoid    LLD_ARM946_FlushDCacheAddr        ( const tUInt addr);

extern tU32     LLD_ARM946_CountZeros             ( tU32 word);

extern tVoid    LLD_ARM946_EnterCritical          ( tVoid);
extern tVoid    LLD_ARM946_ExitCritical           ( tVoid);

#endif  // LLD_ARM946_H

//!   \}

//!   \}

