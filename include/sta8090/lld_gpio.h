//!
//!   \file     lld_gpio.h
//!   \brief    <i><b>GPIO low level driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \authors  (original version) Alberto Saviotti, Luigi Cotignano
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_GPIO_H
#define LLD_GPIO_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "macros.h"
#include "lld.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

//! \brief Number of Pins for group.
#define  PINS_FOR_GROUP       32

//! \brief Use this define to indicate all pins of a GPIO port.
#define LLD_GPIO_ALL_PINS  (LLD_GPIO_PinTy)0xFFFFFFFFU

//! \brief These defines can be used as pin selector.
#define LLD_GPIO_PIN0     BIT_0
#define LLD_GPIO_PIN1     BIT_1
#define LLD_GPIO_PIN2     BIT_2
#define LLD_GPIO_PIN3     BIT_3
#define LLD_GPIO_PIN4     BIT_4
#define LLD_GPIO_PIN5     BIT_5
#define LLD_GPIO_PIN6     BIT_6
#define LLD_GPIO_PIN7     BIT_7
#define LLD_GPIO_PIN8     BIT_8
#define LLD_GPIO_PIN9     BIT_9
#define LLD_GPIO_PIN10    BIT_10
#define LLD_GPIO_PIN11    BIT_11
#define LLD_GPIO_PIN12    BIT_12
#define LLD_GPIO_PIN13    BIT_13
#define LLD_GPIO_PIN14    BIT_14
#define LLD_GPIO_PIN15    BIT_15
#define LLD_GPIO_PIN16    BIT_16
#define LLD_GPIO_PIN17    BIT_17
#define LLD_GPIO_PIN18    BIT_18
#define LLD_GPIO_PIN19    BIT_19
#define LLD_GPIO_PIN20    BIT_20
#define LLD_GPIO_PIN21    BIT_21
#define LLD_GPIO_PIN22    BIT_22
#define LLD_GPIO_PIN23    BIT_23
#define LLD_GPIO_PIN24    BIT_24
#define LLD_GPIO_PIN25    BIT_25
#define LLD_GPIO_PIN26    BIT_26
#define LLD_GPIO_PIN27    BIT_27
#define LLD_GPIO_PIN28    BIT_28
#define LLD_GPIO_PIN29    BIT_29
#define LLD_GPIO_PIN30    BIT_30
#define LLD_GPIO_PIN31    BIT_31

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief Define as tU32 the LLD_GPIO_idTy type.
typedef void * LLD_GPIO_idTy; // GPIO Id

//! \brief Type definition for GPIO mode.
typedef enum
{
  LLD_GPIO_MODE_SOFTWARE  = 0,
  LLD_GPIO_MODE_ALTA      = 1,
  LLD_GPIO_MODE_ALTB      = 2,
  LLD_GPIO_MODE_ALTC      = 3
} RCIF_DATA_ALIGN LLD_GPIO_PinModeTy;

//! \brief Type definition for alternate function table item
typedef struct
{
  tU32 mask   : 24;
  tU32 shift  : 4;
  tU32 mode   : 2;
  tU32 ip     : 2;
} LLD_GPIO_AltFuncItemTy;

//! \brief This enumerator indicates GPIO state:
typedef enum                // GPIO pin states
{
  LLD_GPIO_LOW   = 0,     /**< the GPIO state is ground */
  LLD_GPIO_HIGH  = 1      /**< the GPIO state is Vcc */
} RCIF_DATA_ALIGN LLD_GPIO_StateTy;

//! \brief    LLD_GPIO_PinTy
//! \details  This enumerator is used to address particular GPIO that belong to a port.
//!           Each port has 32 GPIOs. If a function accept more than one GPIO a OR-ed
//!           of more than one GPIO can be passed. To pass all 32 GPIO use
//!           LLD_GPIO_ALL_PINS.
//!
typedef tU32 LLD_GPIO_PinTy;

//!
//! \typedef LLD_GPIO_ChanTy
//! This type is used to address channels in GPIO battery (max val 32 * banks)
//!
typedef tU32 LLD_GPIO_ChanTy;

//!
//! \enum LLD_GPIO_PullNetTy
//! This enumerator indicates GPIO direction:
//! - LLD_GPIO_INPUT
//! - LLD_GPIO_OUTPUT.
//!
typedef enum
{
  LLD_GPIO_PULL_ENABLE,
  LLD_GPIO_PULL_DISABLE
} LLD_GPIO_PullNetTy;

//!
//! \enum LLD_GPIO_DirectionTy
//! This enumerator indicates GPIO direction:
//! - LLD_GPIO_INPUT
//! - LLD_GPIO_OUTPUT.
//!
typedef enum            // GPIO direction types
{
  LLD_GPIO_INPUT,
  LLD_GPIO_OUTPUT
} LLD_GPIO_DirectionTy;

//!
//! \enum LLD_GPIO_IntTy
//! This enumerator indicates GPIO interrupt type:
//! - LLD_GPIO_DISABLED_INT, interrupt disabled
//! - LLD_GPIO_BOTH_EDGES_INT, interrupt on both edges
//! - LLD_GPIO_RISE_EDGE_INT, interrupt on the rising edge
//! - LLD_GPIO_FALL_EDGE_INT, interrupt on the falling edge.
//!
typedef enum            // GPIO pin interrupt event types
{
  LLD_GPIO_DISABLED_INT,
  LLD_GPIO_BOTH_EDGES_INT,
  LLD_GPIO_RISE_EDGE_INT,
  LLD_GPIO_FALL_EDGE_INT
} LLD_GPIO_IntTy;

//!
//! \enum LLD_GPIO_ModeTy
//! This enumerator indicates GPIO mode:
//! - LLD_GPIO_ALTERNATE_NONE, GPIO mode
//! - LLD_GPIO_ALTERNATE_MODE_A, alternate mode A
//! - LLD_GPIO_ALTERNATE_MODE_B, alternate mode B
//! - LLD_GPIO_ALTERNATE_MODE_C, alternate mode C.
//!
typedef enum            // GPIO pin control modes
{
  LLD_GPIO_ALTERNATE_NONE    = 0x00,
  LLD_GPIO_ALTERNATE_MODE_A  = 0x01,
  LLD_GPIO_ALTERNATE_MODE_B  = 0x02,
  LLD_GPIO_ALTERNATE_MODE_C  = 0x03
} LLD_GPIO_ModeTy;

//!
//! \enum LLD_GPIO_ModeTy
//! This enumerator indicates GPIO mode on IC set into SLEEP mode:
//! - LLD_GPIO_DEFAULT_SLEEP, GPIO mode set to input
//! - LLD_GPIO_DRIVEN_SLEEP, GPIO mode unchanged.
//!
typedef enum        // GPIO set sleep mode behavior
{
  LLD_GPIO_DEFAULT_SLEEP,
  LLD_GPIO_DRIVEN_SLEEP
} LLD_GPIO_SleepModeTy;

//!
//! \struct LLD_GPIO_IdentifierTy
//! This structure reports the GPIO port identifier.
//!
typedef struct
{
  //! The cell part number lower part (8 bits)
  tU8 partNumber0;
  //! The cell part number higher part (8 bits)
  tU8 partNumber1;
  //! The cell designer number lower part (8 bits)
  tU8 designer0;
  //! The cell designer number higher part (8 bits)
  tU8 designer1;
  //! Cell revision
  tU8 revision;
  //! Cell configuration
  tU8 configuration;
  //! Cell ID field 0
  tU8 GPIOPCellID0;
  //! Cell ID field 1
  tU8 GPIOPCellID1;
  //! Cell ID field 2
  tU8 GPIOPCellID2;
  //! Cell ID field 3
  tU8 GPIOPCellID3;
} LLD_GPIO_IdentifierTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid            LLD_GPIO_SetControlMode     ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins, LLD_GPIO_ModeTy mode);
extern LLD_GPIO_ModeTy  LLD_GPIO_GetControlMode     ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pin);

extern tVoid            LLD_GPIO_SetState           ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins, LLD_GPIO_StateTy state);
extern tVoid            LLD_GPIO_SetStateHigh       ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern tVoid            LLD_GPIO_SetStateLow        ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern tVoid            LLD_GPIO_TogglePinState     ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern LLD_GPIO_StateTy LLD_GPIO_GetPinState        ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pin);
extern tU32             LLD_GPIO_GetPortState       ( LLD_GPIO_idTy id);

extern tVoid            LLD_GPIO_SetDirection       ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins, LLD_GPIO_DirectionTy direction);
extern tVoid            LLD_GPIO_SetDirectionInput  ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern tVoid            LLD_GPIO_SetDirectionOutput ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern tU32             LLD_GPIO_GetDirection       ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);

extern tVoid            LLD_GPIO_SetSleepModeReg    ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins, LLD_GPIO_SleepModeTy mode);
extern tU32             LLD_GPIO_GetSleepModeReg    ( LLD_GPIO_idTy id);

extern tVoid            LLD_GPIO_SetInterruptType   ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins, LLD_GPIO_IntTy type);
extern LLD_GPIO_IntTy   LLD_GPIO_GetInterruptType   ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pin);
extern tVoid            LLD_GPIO_InterruptDisable   ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);

extern tVoid            LLD_GPIO_DisablePull        ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern tVoid            LLD_GPIO_EnablePull         ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);
extern tU32             LLD_GPIO_GetPullState       ( LLD_GPIO_idTy id);

extern tU32             LLD_GPIO_GetInterruptStatus ( LLD_GPIO_idTy id);
extern tVoid            LLD_GPIO_ClearInterrupt     ( LLD_GPIO_idTy id, LLD_GPIO_PinTy pins);

// Following function addresses GPIO linearly:
// - PIN: 0 -> 127
extern tVoid            LLD_GPIO_ReadPin            ( LLD_GPIO_ChanTy pin, LLD_GPIO_StateTy *p_value);
extern tVoid            LLD_GPIO_SetPinHigh         ( LLD_GPIO_ChanTy pin);
extern tVoid            LLD_GPIO_SetPinLow          ( LLD_GPIO_ChanTy pin);
extern tVoid            LLD_GPIO_SetPinControlMode  ( LLD_GPIO_ChanTy pin, LLD_GPIO_ModeTy mode);

extern tVoid            LLD_GPIO_SetPinMode         ( LLD_GPIO_ChanTy pin, LLD_GPIO_PinModeTy mode);

extern tVoid            LLD_GPIO_SetPinPullNet      ( LLD_GPIO_ChanTy pin, LLD_GPIO_PullNetTy pullNet);
extern tVoid            LLD_GPIO_GetPinPullNet      ( LLD_GPIO_ChanTy pin, LLD_GPIO_PullNetTy *pPullNet);

extern tVoid            LLD_GPIO_SetPinDirection    ( LLD_GPIO_ChanTy pin, LLD_GPIO_DirectionTy direction);
extern tVoid            LLD_GPIO_GetPinDirection    ( LLD_GPIO_ChanTy pin, LLD_GPIO_DirectionTy *pDirection);

extern tVoid            LLD_GPIO_SetSleepMode       ( LLD_GPIO_ChanTy pin, LLD_GPIO_SleepModeTy mode);
extern tVoid            LLD_GPIO_GetSleepMode       ( LLD_GPIO_ChanTy pin, LLD_GPIO_SleepModeTy *pmode);

extern tVoid            LLD_GPIO_PinIRQEnable       ( LLD_GPIO_ChanTy pin, LLD_GPIO_IntTy irq);
extern tVoid            LLD_GPIO_PinIRQDisable      ( LLD_GPIO_ChanTy pin);
extern tVoid            LLD_GPIO_PinIRQGetConfig    ( LLD_GPIO_ChanTy pin, LLD_GPIO_IntTy *irq);
extern tVoid            LLD_GPIO_PinIRQClear        ( LLD_GPIO_ChanTy pin);

extern tBool            LLD_GPIO_EnAltFunction      ( LLD_GPIO_AltFuncTy per);
extern tVoid            LLD_GPIO_DisAltFunction     ( LLD_GPIO_AltFuncTy per);

// Cell ID (test only)
extern tBool            LLD_GPIO_CellID             ( LLD_GPIO_idTy id, LLD_GPIO_IdentifierTy *idPtr);

#ifdef __cplusplus
}
#endif

#endif  // _LLD_GPIO_H_

// End of file
