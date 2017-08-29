//!
//!   \file       lld_vic.h
//!   \brief      <i><b> VIC Low Level Driver header file </b></i>
//!   \author     Fulvio Boggia
//!   \authors    (original version) Alberto Saviotti, Luigi Cotignano
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup VIC
//!   \{
//!

#ifndef LLD_VIC_H
#define LLD_VIC_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

//! \brief It's the VIC's vectored interrupts number
#define LLD_VIC_INTERRUPT_PRIORITIES        (16)

//! \brief It's the VIC's channels number
#define LLD_VIC_INTERRUPT_CHANNELS          (64)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief It's the type definition for VIC channels (0..63) */
typedef tU8 LLD_VIC_ChannelTy;

//! \brief It's the type definition for VIC priorities */
typedef enum
{
  LLD_VIC_PRIORITY0,            //!< VIC's priority number 0 (highest priority) */
  LLD_VIC_PRIORITY1,            //!< VIC's priority number 1 */
  LLD_VIC_PRIORITY2,            //!< VIC's priority number 2 */
  LLD_VIC_PRIORITY3,            //!< VIC's priority number 3 */
  LLD_VIC_PRIORITY4,            //!< VIC's priority number 4 */
  LLD_VIC_PRIORITY5,            //!< VIC's priority number 5 */
  LLD_VIC_PRIORITY6,            //!< VIC's priority number 6 */
  LLD_VIC_PRIORITY7,            //!< VIC's priority number 7 */
  LLD_VIC_PRIORITY8,            //!< VIC's priority number 8 */
  LLD_VIC_PRIORITY9,            //!< VIC's priority number 9 */
  LLD_VIC_PRIORITY10,           //!< VIC's priority number 10 */
  LLD_VIC_PRIORITY11,           //!< VIC's priority number 11 */
  LLD_VIC_PRIORITY12,           //!< VIC's priority number 12 */
  LLD_VIC_PRIORITY13,           //!< VIC's priority number 13 */
  LLD_VIC_PRIORITY14,           //!< VIC's priority number 14 */
  LLD_VIC_PRIORITY15,           //!< VIC's priority number 15 (lowest priority) */
  LLD_VIC_PRIORITYNONE          //!< No priority VIC */
} LLD_VIC_PriorityLevelTy;

//! \brief It's the type definition for VIC interrupt assignment status */
typedef enum
{
  LLD_VIC_VECTORED_INTERRUPT,             //!< The channel is assigned to a vectored interrupt */
  LLD_VIC_NON_VECTORED_INTERRUPT,         //!< The channel is assigned to a non vectored interrupt */
  LLD_VIC_NON_VECTORED_INITIALIZED,       //!< The function pointers array for non vectored interrupts has been initialized */
  LLD_VIC_NON_VECTORED_NOT_INITIALIZED,   //!< The function pointers array for non vectored interrupts has not been initialized */
  LLD_VIC_NOT_ASSIGNED                    //!< The channel is not assigned to any interrupt source */
}LLD_VIC_ErrorStatusTy;

//! \brief It's the type definition for VIC's interrupts types */
typedef enum
{
  LLD_VIC_IRQ_INTERRUPT,          //!< The channel is dedicated to IRQ interrupts only */
  LLD_VIC_FIQ_INTERRUPT           //!< The channel is dedicated to FIQ interrupts only */
}LLD_VIC_TypeTy;

//! \brief It's the type definition for VIC Control Register */
typedef struct
{
  tUBitField IS         :6;         /**< (rw) Interrupt source */
  tUBitField E          :1;         /**< (rw) Interrupt enable */
  tUBitField reserved   :25;        /**< (r-) Reserved */
} LLD_VIC_VcrTy;

//! \brief It's the type definition for the generic ISR function pointer */
typedef tVoid (*LLD_VIC_IsrHandlerTy)(tVoid *);

//! \brief It's the type definition for the function pointers array */
typedef struct
{
  LLD_VIC_IsrHandlerTy  func_ptr;
  tVoid *               arg_ptr;
} LLD_VIC_IsrNonVectoredTableItemTy;

//! \brief It's the type definition for the mask for vectored IRQs */
typedef tU64 LLD_VIC_VectoredMaskTy;

//! \brief It's the type definition for the function pointers array */
typedef struct
{
  LLD_VIC_IsrNonVectoredTableItemTy   line[LLD_VIC_INTERRUPT_CHANNELS];
  LLD_VIC_VectoredMaskTy              vectored;
} LLD_VIC_NonVectoredTableTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid                  LLD_VIC_Reset                               ( tVoid);
extern tVoid                  LLD_VIC_DefaultISR                          ( tVoid *arg_ptr);
extern tVoid                  LLD_VIC_ClearAllPriorities                  ( tVoid);
extern LLD_VIC_ErrorStatusTy  LLD_VIC_VectoredChannelConfig               ( LLD_VIC_ChannelTy channel, LLD_VIC_PriorityLevelTy priority, LLD_VIC_IsrHandlerTy isr);
extern tVoid                  LLD_VIC_NonVectoredInterruptManager         ( LLD_VIC_NonVectoredTableTy *table, LLD_VIC_IsrHandlerTy vect_addr);
extern tBool                  LLD_VIC_NonVectoredInterruptDispatcher      ( LLD_VIC_NonVectoredTableTy *table, LLD_VIC_IsrHandlerTy vect_addr, tU64 irq_status, LLD_VIC_IsrNonVectoredTableItemTy **item_ptr, LLD_VIC_ChannelTy *line_ptr);
extern LLD_VIC_ErrorStatusTy  LLD_VIC_NonVectoredTableInit                ( LLD_VIC_IsrHandlerTy isr, LLD_VIC_NonVectoredTableTy *table);
extern LLD_VIC_ErrorStatusTy  LLD_VIC_NonVectoredChannelConfig            ( LLD_VIC_ChannelTy channel, LLD_VIC_PriorityLevelTy priority, LLD_VIC_NonVectoredTableTy *table, LLD_VIC_IsrHandlerTy isr, tVoid *isr_arg);
extern tVoid                  LLD_VIC_SetProtectionStatus                 ( tBool flag);
extern tBool                  LLD_VIC_GetProtectionStatus                 ( tVoid);
extern tVoid                  LLD_VIC_SetCurrentInterruptAddress          ( LLD_VIC_IsrHandlerTy isr);
extern LLD_VIC_IsrHandlerTy   LLD_VIC_GetCurrentInterruptAddress          ( tVoid);
extern tVoid                  LLD_VIC_SetDefaultInterruptAddress          ( LLD_VIC_IsrHandlerTy isr);
extern LLD_VIC_IsrHandlerTy   LLD_VIC_GetDefaultInterruptAddress          ( tVoid);
extern tVoid                  LLD_VIC_SetVectoredInterruptAddress         ( LLD_VIC_PriorityLevelTy priority, LLD_VIC_IsrHandlerTy isr);
extern LLD_VIC_IsrHandlerTy   LLD_VIC_GetVectoredInterruptAddress         ( LLD_VIC_PriorityLevelTy priority);
extern tVoid                  LLD_VIC_SetVectoredInterruptControlRegister ( LLD_VIC_ChannelTy channel, LLD_VIC_PriorityLevelTy priority, tBool enable_bit);
extern LLD_VIC_VcrTy *        LLD_VIC_GetVectoredInterruptControlRegister ( LLD_VIC_PriorityLevelTy priority);
extern tVoid                  LLD_VIC_SetInterruptType                    ( LLD_VIC_ChannelTy channel, LLD_VIC_TypeTy type);
extern LLD_VIC_TypeTy         LLD_VIC_GetInterruptType                    ( LLD_VIC_ChannelTy channel);
extern tVoid                  LLD_VIC_EnableChannel                       ( LLD_VIC_ChannelTy channel);
extern tVoid                  LLD_VIC_DisableChannel                      ( LLD_VIC_ChannelTy channel);
extern tVoid                  LLD_VIC_DisableAllChannels                  ( tVoid);
extern tVoid                  LLD_VIC_EnableAllChannels                   ( tVoid);
extern tVoid                  LLD_VIC_BackupAndDisableAllChannels         ( tU64 *interrupt_status);
extern tVoid                  LLD_VIC_RestoreAllChannelsStatus            ( tU64 *interrupt_status);
extern tVoid                  LLD_VIC_GenerateSoftwareInterrupt           ( LLD_VIC_ChannelTy channel);
extern tVoid                  LLD_VIC_ClearSoftwareInterrupt              ( LLD_VIC_ChannelTy channel);
extern tU64                   LLD_VIC_GetRawInterruptStatus               ( tVoid);
extern tU64                   LLD_VIC_GetIRQInterruptStatus               ( tVoid);
extern tU64                   LLD_VIC_GetFIQInterruptStatus               ( tVoid);
extern tVoid                  LLD_VIC_SetDefaultISR                       ( LLD_VIC_IsrHandlerTy isr);

#endif  // LLD_VIC_H

//!   \}

//!   \}

