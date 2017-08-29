//!
//!   \file     lld_sdi.h
//!   \brief    <i><b>SDI low level driver header file</b></i>
//!   \author   Giovanni De Angelis
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_SDI_H
#define LLD_SDI_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define LLD_SDI__VERSION_ID  3
#define LLD_SDI__MAJOR_ID    2
#define LLD_SDI__MINOR_ID    0

#define CONTEXT_COUNT       8

/*MAX NO OF CARDS THAT CAN BE SUPPORTED.*/
#define MAXCARDS    1

#define LLD_SDI_FilterMode   tU32
#define NO_FILTER_MODE      0

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef void * LLD_SDI_IdTy;

typedef enum
{
  /* standard error defines */
  LLD_SDI_REQUEST_NOT_APPLICABLE          = ( -5 ),
  LLD_SDI_INVALID_PARAMETER               = ( -4 ),
  LLD_SDI_UNSUPPORTED_FEATURE             = ( -3 ),
  LLD_SDI_UNSUPPORTED_HW                  = ( -2 ),
  LLD_SDI_ERROR                           = ( -1 ),
  LLD_SDI_OK                              = ( LLD_NO_ERROR ),
  LLD_SDI_INTERNAL_EVENT                  = ( 1 ),
  LLD_SDI_REMAINING_PENDING_EVENTS        = ( 2 ),
  LLD_SDI_REMAINING_FILTER_PENDING_EVENTS = ( 3 ),
  LLD_SDI_NO_MORE_PENDING_EVENT           = ( 4 ),
  LLD_SDI_NO_MORE_FILTER_PENDING_EVENT    = ( 5 ),
  LLD_SDI_NO_PENDING_EVENT_ERROR          = ( 6 ),

  /* MMC specific error defines */
  LLD_SDI_CMD_CRC_FAIL                    = ( 7 ),          /* Command response received (but CRC check failed) */
  LLD_SDI_DATA_CRC_FAIL                   = ( 8 ),          /* Data bock sent/received (CRC check Failed) */
  LLD_SDI_CMD_RSP_TIMEOUT                 = ( 9 ),          /* Command response timeout */
  LLD_SDI_DATA_TIMEOUT                    = ( 10 ),          /* Data time out*/
  LLD_SDI_TX_UNDERRUN                     = ( 11 ),          /* Transmit FIFO under-run */
  LLD_SDI_RX_OVERRUN                      = ( 12 ),          /* Receive FIFO over-run */
  LLD_SDI_START_BIT_ERR                   = ( 13 ),          /* Start bit not detected on all data signals in widE bus mode */

  LLD_SDI_ADDR_OUT_OF_RANGE               = ( 14 ),          /* CMD's argument was out of range.*/
  LLD_SDI_ADDR_MISALIGNED                 = ( 15 ),          /* Misaligned address */
  LLD_SDI_BLOCK_LEN_ERR                   = ( 16 ),         /* Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
  LLD_SDI_ERASE_SEQ_ERR                   = ( 17 ),         /* An error in the sequence of erase command occurs.*/
  LLD_SDI_ERASE_PARAM_ERR                 = ( 18 ),         /* An Invalid selection for erase groups */
  LLD_SDI_WP_VIOLATION                    = ( 19 ),         /* Attempt to program a write protect block */
  LLD_SDI_LOCK_UNLOCK_FAILED              = ( 20 ),         /* Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
  LLD_SDI_COM_CRC_ERR                     = ( 21 ),         /* CRC check of the previous command failed */
  LLD_SDI_ILLEGAL_CMD                     = ( 22 ),         /* Command is not legal for the card state */
  LLD_SDI_CARD_ECC_FAILED                 = ( 23 ),         /* Card internal ECC was applied but failed to correct the data */
  LLD_SDI_CC_ERR                          = ( 24 ),         /* Internal card controller error */
  LLD_SDI_UNKNOWN_ERR                     = ( 25 ),         /* General or Unknown error */
  LLD_SDI_UNDERRUN_ERR                    = ( 26 ),         /* The card could not sustain data transfer in stream read operation. */
  LLD_SDI_OVERRUN_ERR                     = ( 27 ),         /* The card could not sustain data programming in stream mode */
  LLD_SDI_CID_CSD_OVERWRITE               = ( 28 ),         /* CID/CSD overwrite error */
  LLD_SDI_WP_ERASE_SKIP                   = ( 29 ),         /* only partial address space was erased */
  LLD_SDI_ERASE_RESET                     = ( 30 ),         /* Erase sequence was cleared before executing because an out of erase sequence command was received */
  LLD_SDI_SWITCH_ERR                      = ( 31 ),
  LLD_SDI_AKE_SEQ_ERR                     = ( 32 ),         /* Error in sequence of authentication. */

  LLD_SDI_INVALID_VOLTRANGE               = ( 33 ),
  LLD_SDI_SDIO_DISABLED                   = ( 34 ),
  LLD_SDI_SDIO_FUNCTION_BUSY              = ( 35 ),
  LLD_SDI_SDIO_FUNCTION_FAILED            = ( 36 ),
  LLD_SDI_SDIO_UNKNOWN_FUNCTION           = ( 37 )
} LLD_SDI_ErrorTy;

typedef enum
{
  LLD_SDI_DISABLE                         = 0,
  LLD_SDI_ENABLE
} LLD_SDI_StateTy;

typedef enum
{
  LLD_SDI_1_BIT_WIDE                      = 0,
  LLD_SDI_4_BIT_WIDE,
  LLD_SDI_8_BIT_WIDE
} LLD_SDI_WideModeTy;

typedef enum
{
  LLD_SDI_POWER_OFF                       = 0x0,
  LLD_SDI_POWER_UP                        = 0x2,
  LLD_SDI_POWER_ON                        = 0x3
} LLD_SDI_PowerStateTy;

typedef enum
{
  LLD_SDI_PUSH_PULL                       = 0,
  LLD_SDI_OPEN_DRAIN
} LLD_SDI_BusModeTy;

typedef enum
{
  LLD_SDI_EMBEDDED_CARD                   = 0,
  LLD_SDI_EXTERNAL_CARD                   = 1
} LLD_SDI_SelectedCardTy;

typedef enum
{
  LLD_SDI_GO_IDLE_STATE                       = 0,
  LLD_SDI_SEND_OP_COND                        = 1,
  LLD_SDI_ALL_SEND_CID                        = 2,
  LLD_SDI_SET_REL_ADDR                        = 3,
  LLD_SDI_SET_DSR                             = 4,
  LLD_SDI_SLEEP_AWAKE                         = 5,
  LLD_SDI_SWITCH                              = 6,
  LLD_SDI_SEL_DESEL_CARD                      = 7,
  LLD_SDI_SEND_EXT_CSD                        = 8,
  LLD_SDI_SEND_IF_COND                        = 8,
  LLD_SDI_SEND_CSD                            = 9,
  LLD_SDI_SEND_CID                            = 10,
  LLD_SDI_READ_DAT_UNTIL_STOP                 = 11,
  LLD_SDI_STOP_TRANSMISSION                   = 12,
  LLD_SDI_SEND_STATUS                         = 13,
  LLD_SDI_BUSTEST_READ                        = 14,
  LLD_SDI_GO_INACTIVE_STATE                   = 15,
  LLD_SDI_SET_BLOCKLEN                        = 16,
  LLD_SDI_READ_SINGLE_BLOCK                   = 17,
  LLD_SDI_READ_MULT_BLOCK                     = 18,
  LLD_SDI_BUSTEST_WRITE                       = 19,
  LLD_SDI_WRITE_DAT_UNTIL_STOP                = 20,
  LLD_SDI_SET_BLOCK_COUNT                     = 23,
  LLD_SDI_WRITE_SINGLE_BLOCK                  = 24,
  LLD_SDI_WRITE_MULT_BLOCK                    = 25,
  LLD_SDI_PROG_CID                            = 26,
  LLD_SDI_PROG_CSD                            = 27,
  LLD_SDI_SET_WRITE_PROT                      = 28,
  LLD_SDI_CLR_WRITE_PROT                      = 29,
  LLD_SDI_SEND_WRITE_PROT                     = 30,
  LLD_SDI_SEND_WRITE_PROT_TYPE                = 31,
  LLD_SDI_ERASE_WR_BLK_START                  = 32,
  LLD_SDI_ERASE_WR_BLK_END                    = 33,
  LLD_SDI_ERASE_GROUP_START                   = 35,
  LLD_SDI_ERASE_GROUP_END                     = 36,
  LLD_SDI_ERASE                               = 38,
  LLD_SDI_FAST_IO                             = 39,
  LLD_SDI_GO_IRQ_STATE                        = 40,
  LLD_SDI_LOCK_UNLOCK                         = 42,
  LLD_SDI_APP_CMD                             = 55,
  LLD_SDI_GEN_CMD                             = 56,
  LLD_SDI_NO_CMD                              = 64,

  /*Following commands are SD Card Specific commands. LLD_SDI_APP_CMD should be sent before sending these commands.*/
  LLD_SDI_SD_APP_SET_BUSWIDTH                 = 6,                                /* For SD Card only.*/
  LLD_SDI_SD_APP_SD_STATUS                    = 13,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SEND_NUM_WRITE_BLOCKS        = 22,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SET_WR_BLK_ERASE_COUNT       = 23,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_OP_COND                      = 41,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SET_CLR_CARD_DETECT          = 42,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SEND_SCR                     = 51,                               /* For SD Card only.*/

  /*Following commands are SD security applications commands. LLD_SDI_APP_CMD should be sent before sending these commands.*/
  LLD_SDI_SD_APP_SECURE_READ_MULTIPLE_BLOCK   = 18,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SECURE_WRITE_MULTIPLE_BLOCK  = 25,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SECURE_WRITE_MKB             = 26,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SECURE_ERASE                 = 38,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_GET_MKB                      = 43,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_GET_MID                      = 44,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SET_CER_RN1                  = 45,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SET_CER_RN2                  = 46,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SET_CER_RES2                 = 47,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_SET_CER_RES1                 = 48,                               /* For SD Card only.*/
  LLD_SDI_SD_APP_CHANGE_SECURE_AREA           = 49,                               /* For SD Card only.*/

  /*Following commands are SD Card Specific commands. LLD_SDI_APP_CMD should be sent before sending these commands.*/
  LLD_SDI_SDIO_SEN_OP_COND                    = 5,
  LLD_SDI_SDIO_RW_DIRECT                      = 52,                               /* For SDIO Card only. */
  LLD_SDI_SDIO_RW_EXTENDED                    = 53                                /* For SDIO Card only. */
} LLD_SDI_CommandIndexTy;

typedef enum
{
  LLD_SDI_SHORT_RESP                          = 0,
  LLD_SDI_LONG_RESP
} LLD_SDI_ResponseTy;

typedef enum
{
  LLD_SDI_WRITE                               = 0,
  LLD_SDI_READ
} LLD_SDI_TransferDirectionTy;

typedef enum
{
  LLD_SDI_BLOCK                               = 0,
  LLD_SDI_STREAM
} LLD_SDI_TransferTy;

/* For SDIO card support */
typedef enum
{
  LLD_SDI_READ_WAIT_CONTROLLING_CLK           = 0,
  LLD_SDI_READ_WAIT_CONTROLLING_DAT2
} LLD_SDI_ReadWaitModeTy;

typedef enum
{
  LLD_SDI_POLLING_MODE                        = 0,
  LLD_SDI_INTERRUPT_MODE,
  LLD_SDI_DMA_MODE
} LLD_SDI_DeviceModeTy;

typedef enum
{
  LLD_SDI_IRQ_CMD_CRC_FAIL        = BIT_0,
  LLD_SDI_IRQ_DATA_CRC_FAIL       = BIT_1,
  LLD_SDI_IRQ_CMD_TIMEOUT         = BIT_2,
  LLD_SDI_IRQ_DATA_TIMEOUT        = BIT_3,
  LLD_SDI_IRQ_TX_UNDERRUN         = BIT_4,
  LLD_SDI_IRQ_RX_OVERRUN          = BIT_5,
  LLD_SDI_IRQ_CMD_RESP_OK         = BIT_6,
  LLD_SDI_IRQ_CMD_SENT            = BIT_7,
  LLD_SDI_IRQ_DATA_END            = BIT_8,
  LLD_SDI_IRQ_START_BIT_ERR       = BIT_9,
  LLD_SDI_IRQ_DATA_BLOCK_END      = BIT_10,
  LLD_SDI_IRQ_CMD_ACTIVE          = BIT_11,
  LLD_SDI_IRQ_TX_ACTIVE           = BIT_12,
  LLD_SDI_IRQ_RX_ACTIVE           = BIT_13,
  LLD_SDI_IRQ_TX_FIFO_HALF_EMPTY  = BIT_14,
  LLD_SDI_IRQ_RX_FIFO_HALF_FULL   = BIT_15,
  LLD_SDI_IRQ_TX_FIFO_FULL        = BIT_16,
  LLD_SDI_IRQ_RX_FIFO_FULL        = BIT_17,
  LLD_SDI_IRQ_TX_FIFO_EMPTY       = BIT_18,
  LLD_SDI_IRQ_RX_FIFO_EMPTY       = BIT_19,
  LLD_SDI_IRQ_TX_DATA_AVLBL       = BIT_20,
  LLD_SDI_IRQ_RX_DATA_AVLBL       = BIT_21,
  LLD_SDI_IRQ_SDIO                = BIT_22,
  LLD_SDI_IRQ_CE_ATA_END          = BIT_23,
  LLD_SDI_IRQ_CARD_BUSY           = BIT_24,
  LLD_SDI_IRQ_BOOT_MODE           = BIT_25,
  LLD_SDI_IRQ_BOOT_ACK_ERR        = BIT_26,
  LLD_SDI_IRQ_BOOT_ACK_TIMEOUT    = BIT_27,
  LLD_SDI_IRQ_RSTN_END            = BIT_28,
  LLD_SDI_IRQ_ALL_STATIC          = 0x1DC007FF,
  LLD_SDI_IRQ_ALL                 = 0x1FFFFFFF
} LLD_SDI_IrqSrcTy;

typedef struct
{
  tU32  minor: 8;
  tU32  major: 8;
  tU32  version: 16;
} LLD_SDI_VersionTy;

typedef struct
{
  tU32    card_ready_after_init;
  tU32    no_of_io_funcs;
  tU32    memory_present;
  tU32    op_cond_register;
} LLD_SDI_SDIOResp4Ty;

typedef struct
{
  LLD_SDI_BusModeTy  mode;
  LLD_SDI_StateTy     rodctrl;        /*For MuPoC LITE only.*/
} LLD_SDI_BusConfigurationTy;

typedef struct
{
  LLD_SDI_StateTy pwr_save;
  LLD_SDI_StateTy bypass;
} LLD_SDI_ClockControlTy;

typedef struct
{
  tBool           is_resp_expected;
  tBool           is_long_resp;
  tBool           is_interrupt_mode;
  tBool           is_pending;
  LLD_SDI_StateTy cmd_path;
} LLD_SDI_CommandControlTy;

typedef tU32    LLD_SDI_EventTy;

typedef enum
{
  LLD_SDI_SDIO_READ                           = 0,
  LLD_SDI_SDIO_WRITE
} LLD_SDI_SDIODataDirTy;

typedef struct
{
  tU32                    card_addr;
  tU16                    no_of_bytes;
  tU16                    no_of_blocks;
  tU16                    block_size;
  tU8                     function_number;
  tU8                     padding;
  LLD_SDI_StateTy         read_after_write;
  LLD_SDI_StateTy         block_mode;
  LLD_SDI_StateTy         incremental_transfer;
  LLD_SDI_SDIODataDirTy   read_write;
} LLD_SDI_SDIOTransferInfoTy;

typedef enum
{
  LLD_SDI_IRQ_STATE_NEW                       = 0,
  LLD_SDI_IRQ_STATE_OLD
} LLD_SDI_IrqStateTy;

typedef struct
{
  LLD_SDI_IrqStateTy irq_state;
  tU8         padding[3];
  tU32        initial_irq;
  tU32        pending_irq;
} LLD_SDI_IrqStatusTy;

typedef enum
{
  LLD_SDI_MMC_CARD,
  LLD_SDI_MMCPLUS_CARD,
  LLD_SDI_MMCHC_CARD,
  LLD_SDI_SD_CARD,
  LLD_SDI_SDHC_CARD,
  LLD_SDI_SDIO_CARD,
  LLD_SDI_SDIO_COMBO_CARD
} LLD_SDI_CardTypeTy;

typedef struct
{
  tU32                cid[4];
  tU32                csd[4];
  tU16                rca;
  LLD_SDI_CardTypeTy  card_type;
  tU8                 padding;
  tU8                 sdio_cccr[4];   /* I/O ready, CCCR/SDIO revision, SD Specification revision, and Card Capability registers */
} LLD_SDI_CardInfoTy;

typedef struct
{
  tU32    transferred_bytes;
  LLD_SDI_ErrorTy transfer_error;
} LLD_SDI_LastTransferInfoTy;

typedef enum
{
  LLD_SDI_NO_TRANSFER                         = 0,
  LLD_SDI_TRANSFER_IN_PROGRESS
} LLD_SDI_TransferState;

typedef enum
{
  LLD_SDI_IDLE            = 0,
  LLD_SDI_READY           = 1,
  LLD_SDI_IDENT           = 2,
  LLD_SDI_STANDBY         = 3,
  LLD_SDI_TRANSFERRING    = 4,
  LLD_SDI_DATA            = 5,
  LLD_SDI_RECEIVING       = 6,
  LLD_SDI_PROGRAMMING     = 7,
  LLD_SDI_DISABLED        = 8
} LLD_SDI_CurrentState;

typedef enum
{
  LLD_SDI_WRITE_PROT_WHOLE_CARD_TEMP          = 0,
  LLD_SDI_WRITE_PROT_WHOLE_CARD_PERM,
  LLD_SDI_WRITE_PROT_SINGLE_GROUP
} LLD_SDI_WriteProtectTy;

/*  STRUCTURE FOR USE */
typedef struct
{
  LLD_SDI_DeviceModeTy    device_mode;
  LLD_SDI_ErrorTy         transfer_error;
  tU32                    total_no_of_bytes;  /*TOTAL NO OF BYTES TO TRANSFER*/
  tU32                    *source_buffer;
  tU32                    *dest_buffer;
  LLD_SDI_EventTy         mmc_event;          /*FOR EVENT MANAGEMENT */
} LLD_SDI_ContextTy;

typedef struct
{
  LLD_SDI_ContextTy   mmc_context;            /*CONTAINING OTHER GLOBAL VARIABLES */
  tU32                blk_length;             /*BLOCK LENGTH OF SELECTED CARD */
  LLD_SDI_StateTy     wide_mode;
  LLD_SDI_CardTypeTy  cardtype;               /*CURRENT CARD TYPE */
  tBool               stopcmd_needed;         /*WHETHER THERE IS NEED TO SEND STOP CMD*/
  tU8                 selected_card;          /*SELECTED CARD NO*/
  tU8                 sdio_function;          /*CURRENT ACTIVE SDIO FUNCTION*/
  tU8                 padding[3];
} LLD_SDI_CurrentCardTy;

/* MMC system context */
typedef struct
{
  tU32                    mmc_dvcontext[CONTEXT_COUNT];
  LLD_SDI_CardInfoTy      card_array[MAXCARDS];
  LLD_SDI_CurrentCardTy   current_card;
  tU8                     no_of_cards;    /* NO OF CARDS CURRENTLY ACTIVE*/
  tU8                     padding[3];
} LLD_SDI_SystemContextTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern LLD_SDI_ErrorTy              LLD_SDI_Init                  ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_GetVersion            ( LLD_SDI_VersionTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_SetClockFrequency     ( const LLD_SDI_IdTy, tU8 );
extern LLD_SDI_ErrorTy              LLD_SDI_SetFeedBackClock      ( const LLD_SDI_IdTy, LLD_SDI_StateTy );
extern LLD_SDI_ErrorTy              LLD_SDI_EnableDirSignals      ( const LLD_SDI_IdTy, LLD_SDI_StateTy );
extern tVoid                        LLD_SDI_SelectCard            ( const LLD_SDI_IdTy, LLD_SDI_SelectedCardTy );

extern tVoid                        LLD_SDI_SaveDeviceContext     ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern tVoid                        LLD_SDI_RestoreDeviceContext  ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );

extern LLD_SDI_ErrorTy              LLD_SDI_SetWideBus            ( const LLD_SDI_IdTy, LLD_SDI_WideModeTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetPowerState         ( const LLD_SDI_IdTy, LLD_SDI_PowerStateTy );
extern LLD_SDI_PowerStateTy         LLD_SDI_GetPowerState         ( const LLD_SDI_IdTy );
extern LLD_SDI_ErrorTy              LLD_SDI_ConfigBus             ( const LLD_SDI_IdTy, LLD_SDI_BusModeTy );

extern LLD_SDI_ErrorTy              LLD_SDI_ConfigClockControl    ( const LLD_SDI_IdTy, LLD_SDI_ClockControlTy );

extern LLD_SDI_ErrorTy              LLD_SDI_SendCommand           ( const LLD_SDI_IdTy, LLD_SDI_CommandIndexTy, tU32, LLD_SDI_CommandControlTy );
extern LLD_SDI_CommandIndexTy       LLD_SDI_GetCommandResponse    ( const LLD_SDI_IdTy );
extern LLD_SDI_ErrorTy              LLD_SDI_GetResponse           ( const LLD_SDI_IdTy, LLD_SDI_ResponseTy, tU32 * );

extern LLD_SDI_ErrorTy              LLD_SDI_SetDataPath           ( const LLD_SDI_IdTy, LLD_SDI_StateTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetDataTimeOut        ( const LLD_SDI_IdTy, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_SetDataLength         ( const LLD_SDI_IdTy, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_SetDataBlockLength    ( const LLD_SDI_IdTy, tU8 );
extern LLD_SDI_ErrorTy              LLD_SDI_SetTransferDirection  ( const LLD_SDI_IdTy, LLD_SDI_TransferDirectionTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetTransferType       ( const LLD_SDI_IdTy, LLD_SDI_TransferTy );
extern LLD_SDI_ErrorTy              LLD_SDI_HandleDMA             ( const LLD_SDI_IdTy, LLD_SDI_StateTy );
extern tU32                         LLD_SDI_GetDataCounter        ( const LLD_SDI_IdTy );

extern LLD_SDI_ErrorTy              LLD_SDI_ReadFIFO              ( const LLD_SDI_IdTy, tU32 * );
extern LLD_SDI_ErrorTy              LLD_SDI_WriteFIFO             ( const LLD_SDI_IdTy, tU32 );
extern tU32                         LLD_SDI_GetFIFOCount          ( const LLD_SDI_IdTy );

extern LLD_SDI_ErrorTy              LLD_SDI_StartSdioReadWait     ( const LLD_SDI_IdTy );
extern LLD_SDI_ErrorTy              LLD_SDI_StopSdioReadWait      ( const LLD_SDI_IdTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetSdioReadWaitMode   ( const LLD_SDI_IdTy, LLD_SDI_ReadWaitModeTy );
extern LLD_SDI_ErrorTy              LLD_SDI_ResetSdioReadWait     ( const LLD_SDI_IdTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetSdioOperation      ( const LLD_SDI_IdTy, LLD_SDI_StateTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SendSdioSuspendCmd    ( const LLD_SDI_IdTy, LLD_SDI_StateTy );

extern LLD_SDI_ErrorTy              LLD_SDI_PowerON               ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_PowerOFF              ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_InitializeCards       ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_GetCardInfo           ( LLD_SDI_SystemContextTy *, tU8, LLD_SDI_CardInfoTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_EnableWideBusOperation( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, LLD_SDI_WideModeTy );

extern LLD_SDI_ErrorTy              LLD_SDI_SetIRQMode            ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, LLD_SDI_StateTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetDeviceMode         ( LLD_SDI_SystemContextTy *, LLD_SDI_DeviceModeTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SendStatus            ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32 * );

extern LLD_SDI_ErrorTy              LLD_SDI_ReadBytes             ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32, tU32 *, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_ReadBlocks            ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32, tU32 *, tU16, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_WriteBytes            ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32, tU32 *, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_WriteBlocks           ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32, tU32 *, tU16, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_StopTransfer          ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern LLD_SDI_TransferState        LLD_SDI_GetTransferState      ( const LLD_SDI_IdTy );
extern LLD_SDI_LastTransferInfoTy   LLD_SDI_LastTransferInfo      ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_IsCardProgramming     ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU32, tU8 * );

extern LLD_SDI_ErrorTy              LLD_SDI_Erase                 ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_SetWriteProtect       ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, LLD_SDI_WriteProtectTy, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_ClearFullWriteProtect ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8 );
extern LLD_SDI_ErrorTy              LLD_SDI_ClearWriteProtect     ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32 );
extern LLD_SDI_ErrorTy              LLD_SDI_SendWriteProtectStatus( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32, tU32 * );
extern LLD_SDI_ErrorTy              LLD_SDI_SetPassword           ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8, tU8 * );
extern LLD_SDI_ErrorTy              LLD_SDI_ClearPassword         ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8, tU8 * );
extern LLD_SDI_ErrorTy              LLD_SDI_ChangePassword        ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8, tU8 *, tU8, tU8 * );
extern LLD_SDI_ErrorTy              LLD_SDI_LockCard              ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8, tU8 * );
extern LLD_SDI_ErrorTy              LLD_SDI_UnlockCard            ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8, tU8 * );
extern LLD_SDI_ErrorTy              LLD_SDI_ForceErase            ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8 );
extern LLD_SDI_ErrorTy              LLD_SDI_GetSCR                ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32 * );
extern LLD_SDI_ErrorTy              LLD_SDI_SendSDStatus          ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32 * );
extern LLD_SDI_ErrorTy              LLD_SDI_GetExtCSD             ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU32 * );
extern LLD_SDI_ErrorTy              LLD_SDI_SetHighSpeedModeTiming( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, LLD_SDI_StateTy );
extern LLD_SDI_ErrorTy              LLD_SDI_SetPowerClass         ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8 );

extern LLD_SDI_ErrorTy              LLD_SDI_SDIOReadWriteData     ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, LLD_SDI_SDIOTransferInfoTy, tU32 * );
extern LLD_SDI_ErrorTy              LLD_SDI_SDIOResumeFunction    ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8, tU8, tU8 * );
extern LLD_SDI_ErrorTy              LLD_SDI_SDIOSuspendFunction   ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, tU8 );
extern LLD_SDI_ErrorTy              LLD_SDI_SDIOSendOpCond        ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, LLD_SDI_SDIOResp4Ty * );

/*New Interrupt strategy(M1 functions) */
extern tVoid                        LLD_SDI_GetIRQSrcStatus       ( const LLD_SDI_IdTy, LLD_SDI_IrqSrcTy, LLD_SDI_IrqStatusTy * );
extern tBool                        LLD_SDI_IsIRQSrcActive        ( const LLD_SDI_IdTy, LLD_SDI_IrqSrcTy, LLD_SDI_IrqStatusTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_ProcessIRQSrc         ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, LLD_SDI_IrqStatusTy * );
extern LLD_SDI_ErrorTy              LLD_SDI_FilterProcessIRQSrc   ( const LLD_SDI_IdTy, LLD_SDI_SystemContextTy *, LLD_SDI_IrqStatusTy *, LLD_SDI_EventTy *, LLD_SDI_FilterMode );
extern tBool                        LLD_SDI_IsEventActive         ( LLD_SDI_SystemContextTy *, LLD_SDI_EventTy * );
extern tVoid                        LLD_SDI_AcknowledgeEvent      ( LLD_SDI_SystemContextTy *, LLD_SDI_EventTy * );

extern tVoid                        LLD_SDI_EnableIRQSrc          ( const LLD_SDI_IdTy, LLD_SDI_IrqSrcTy );
extern tVoid                        LLD_SDI_DisableIRQSrc         ( const LLD_SDI_IdTy, LLD_SDI_IrqSrcTy );
extern LLD_SDI_IrqSrcTy             LLD_SDI_GetIRQSrc             ( const LLD_SDI_IdTy );
extern tVoid                        LLD_SDI_ClearIRQSrc           ( const LLD_SDI_IdTy, LLD_SDI_IrqSrcTy );
extern tBool                        LLD_SDI_IsPendingIRQSrc       ( const LLD_SDI_IdTy, LLD_SDI_IrqSrcTy );

#ifdef __cplusplus
}   /* allow C++ to use these headers */
#endif /* __cplusplus */

#endif /* LLD_SDI_H */

