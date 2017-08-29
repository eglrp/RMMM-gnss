/*****************************************************************************
   FILE:          frontend.h
   PROJECT:       STA2062 GPS application
   SW PACKAGE:    STA2062 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   API to set and load values from 5620
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      FB:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
 ------------+------+------------------------------------------------------
 2007.12.19  |  FB  | Original version
*****************************************************************************/

#ifndef FRONTEND_H
#define FRONTEND_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "defines.h"
#include "typedefs.h"
#include "lld.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define FE_NUMOFREGS      25

#define FE_PPS_CLK_SETTING_REG_INDEX    6
#define FE_PPS_CLK_SETTING_64_MHz       0x37
#define FE_PPS_CLK_SETTING_48_MHz       0x77
#define FE_PPS_CLK_SETTING_32_MHz       0xB7
#define FE_PPS_CLK_SETTING_16_MHz       0xF7

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum FE_peripheral_if_e
{
  FE_PER_IF_NONE,
  FE_PER_IF_SSP
} FE_peripheral_if_t;

typedef struct
{
  tU8 addr;
  tU8 data;
} FE_reg_item_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern LLD_ErrorTy  FE_init                   ( FE_peripheral_if_t);
extern void         FE_reset                  ( void);
extern LLD_ErrorTy  FE_write_data             ( tUInt, tU8);
extern LLD_ErrorTy  FE_write_burst_data       ( tU8 *);
extern LLD_ErrorTy  FE_write_bits             ( tUInt, tU8, tU8);
extern LLD_ErrorTy  FE_read_data              ( tUInt, tUInt *);
extern void         FE_def_write_data         ( tU8 reg_index, tU8 value);
extern void         FE_def_write_bits         ( tUInt addr, tU8 bitmask, tU8 value);
extern void         FE_def_read_data          ( tU8 addr, tUInt *data_ptr);
extern LLD_ErrorTy  FE_dump_regs              ( FE_reg_item_t *);

#endif /* FRONTEND_H */
