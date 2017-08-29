//!
//!   \file     lld_can.h
//!   \brief    <i><b>CAN Low Level Driver header file</b></i>
//!   \author   Aldo Occhipinti
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_CAN_H
#define LLD_CAN_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef void * LLD_CAN_IdTy;

typedef enum {
  LLD_CAN_50KBPS  = 0x0,
  LLD_CAN_100KBPS = 0x1,
  LLD_CAN_125KBPS = 0x2,
  LLD_CAN_250KBPS = 0x3,
  LLD_CAN_500KBPS = 0x4,
  LLD_CAN_1MBPS   = 0x5
} LLD_CAN_BaudRateTy;

typedef enum
{
  LLD_CAN_STD_ID = 0x0,
  LLD_CAN_EXT_ID = 0x1
} LLD_CAN_IdType;

typedef enum
{
  LLD_CAN_IF1 = 0x1,
  LLD_CAN_IF2 = 0x2
} LLD_CAN_IFnTy;


typedef struct
{
  tU64 read_data_0:8 ;
  tU64 read_data_1:8 ;
  tU64 read_data_2:8 ;
  tU64 read_data_3:8 ;
  tU64 read_data_4:8 ;
  tU64 read_data_5:8 ;
  tU64 read_data_6:8 ;
  tU64 read_data_7:8 ;

} LLD_CAN_Read_Msg_t;

typedef enum
{
  LLD_CAN_IFn_DATAB         =  0x0001U,
  LLD_CAN_IFn_DATAA         =  0x0002U,
  LLD_CAN_IFn_TRXREQ_NEWDAT =  0x0004U,
  LLD_CAN_IFn_CLRINTPND     =  0x0008U,
  LLD_CAN_IFn_CONTROL       =  0x0010U,
  LLD_CAN_IFn_ARB           =  0x0020U,
  LLD_CAN_IFn_MASK          =  0x0040U,
  LLD_CAN_IFn_WR_RD         =  0x0080U
} LLD_CAN_CommandTy;

typedef enum
{
  LLD_CAN_IFn_EOB           =  0x0080U,
  LLD_CAN_IFn_TXRQST        =  0x0100U,
  LLD_CAN_IFn_RMTEN         =  0x0200U,
  LLD_CAN_IFn_RXIE          =  0x0400U,
  LLD_CAN_IFn_TXIE          =  0x0800U,
  LLD_CAN_IFn_UMASK         =  0x1000U,
  LLD_CAN_IFn_INTPND        =  0x2000U,
  LLD_CAN_IFn_MSGLST        =  0x4000U,
  LLD_CAN_IFn_NEWDAT        =  0x8000U
} LLD_CAN_MessageControlTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
/* Initialization Function Calls */
extern tVoid  LLD_CAN_Initialize                ( const LLD_CAN_IdTy, tU16, tU16, tU16, tU16, tBool);
extern tVoid  LLD_CAN_Enable_LoopBack           ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_InterruptEnable           ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_InterruptDisable          ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_ErrorInterruptEnable      ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_StatusInterruptEnable     ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_DisableAutoRetransmission ( const LLD_CAN_IdTy);
extern tU16   LLD_CAN_GetInterruptID            ( const LLD_CAN_IdTy);
extern tBool  LLD_CAN_CheckBusOff               ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_SetBitTiming              ( const LLD_CAN_IdTy, tU16, tU16, tU16, tU16);
extern tVoid  LLD_CAN_StartConfig               ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_StopConfig                ( const LLD_CAN_IdTy);

/* TX/RX Function Calls */
extern tVoid  LLD_CAN_Setup_RX_Object           ( const LLD_CAN_IdTy, tU32, tU32, tU32, LLD_CAN_IdType, tBool);
extern tVoid  LLD_CAN_Read_Object_Data          ( const LLD_CAN_IdTy, tU32, LLD_CAN_Read_Msg_t *, tBool);
extern tU16   LLD_CAN_GetTxOK_Bit               ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_ClearTxOK_Bit             ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_Setup_TX_Object           ( const LLD_CAN_IdTy, tU32, tU32, tU8 *, tU32, LLD_CAN_IdType);
extern tVoid  LLD_CAN_Write_Object_Data         ( const LLD_CAN_IdTy, tU32, tU8 *, tU32);
extern tVoid  LLD_CAN_Update_Message_Valid      ( const LLD_CAN_IdTy, tU32, tBool);
extern tU16   LLD_CAN_Check_Transmit_Request    ( const LLD_CAN_IdTy, tU32);
extern tBool  LLD_CAN_Transmit_Request          ( const LLD_CAN_IdTy, tU32);
extern tU32   LLD_CAN_GetStatusRegister         ( const LLD_CAN_IdTy);
extern tVoid  LLD_CAN_ResetReg                  ( const LLD_CAN_IdTy);

/* Single Object management Function Calls */
extern tVoid  LLD_CAN_single_obj_manager_init   ( tVoid);
extern tU32   LLD_CAN_single_obj_sw_filtering   ( const LLD_CAN_IdTy, tU32, LLD_CAN_IdType, tU32 *, tU32 *);

/* Interfaces (IF1/IF2) Function Calls */
extern tVoid  LLD_CAN_Set_Object                ( const LLD_CAN_IdTy, tU32, LLD_CAN_IFnTy);
extern tBool  LLD_CAN_IFnBusy                   ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_Mask1               ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_Mask2               ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_Arb1                ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_Arb2                ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_CMR                   ( const LLD_CAN_IdTy, LLD_CAN_CommandTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_CMR                 ( const LLD_CAN_IdTy, LLD_CAN_CommandTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_MCR                   ( const LLD_CAN_IdTy, LLD_CAN_MessageControlTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_MCR                 ( const LLD_CAN_IdTy, LLD_CAN_MessageControlTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_DA1R                ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_DA2R                ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_DB1R                ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_DB2R                ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_A2R_MsgVal            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_A2R_MsgVal          ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DataLengthCode        ( const LLD_CAN_IdTy, tU8, LLD_CAN_IFnTy);
extern tU16   LLD_CAN_Get_DataLengthCode        ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DA1R_DATA0            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DA1R_DATA1            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DA2R_DATA2            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DA2R_DATA3            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DB1R_DATA4            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DB1R_DATA5            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DB2R_DATA6            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_DB2R_DATA7            ( const LLD_CAN_IdTy, tU8 *, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DA1R_DATA0            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DA1R_DATA1            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DA2R_DATA2            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DA2R_DATA3            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DB1R_DATA4            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DB1R_DATA5            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DB2R_DATA6            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tU8    LLD_CAN_Get_DB2R_DATA7            ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_Dir                   ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Clear_Dir                 ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_StdId                 ( const LLD_CAN_IdTy, tU16, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_Xtd                   ( const LLD_CAN_IdTy, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_Mask                  ( const LLD_CAN_IdTy, tU16, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_ExtId                 ( const LLD_CAN_IdTy, tU32, LLD_CAN_IFnTy);
extern tVoid  LLD_CAN_Set_ExtMask               ( const LLD_CAN_IdTy, tU32, LLD_CAN_IFnTy);
extern tU32   LLD_CAN_Get_ID                    ( const LLD_CAN_IdTy, tU32, LLD_CAN_IFnTy);
extern tU32   LLD_CAN_Get_ExtID                 ( const LLD_CAN_IdTy, tU32, LLD_CAN_IFnTy);

#endif // _LLD_CAN_H_
// End of file
