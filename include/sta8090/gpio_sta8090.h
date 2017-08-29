//!
//!   \file       gpio_sta8090.h
//!   \brief      <i><b>GPIO alternate functions table header file</b></i>
//!   \author     Fulvio Boggia
//!   \version    1.0
//!   \date       2012.01.03
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup GPIO
//!   \{
//!

#ifndef GPIO_STA8090_H
#define GPIO_STA8090_H

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

//! \brief Number of available banks
#define LLD_GPIO_BANKS    2

//! \brief Number of available GPIO pins
#define LLD_GPIOCHUNDEF   (LLD_GPIO_BANKS * 32)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/**< \brief Supported alternate functions */
typedef enum
{
  LLD_GPIO_AF_UART0,                      /**< UART0 configuration */
  LLD_GPIO_AF_UART0_HW_FLOW_CTRL,         /**< UART0 HW Flow Ctrl configuration */
  LLD_GPIO_AF_UART2,                      /**< UART2 configuration */
  LLD_GPIO_AF_I2C,                        /**< I2C configuration */
  LLD_GPIO_AF_EFT,                        /**< External EFT configuration */
  LLD_GPIO_AF_INVALID,
  LLD_GPIO_AF_NUMBER
} LLD_GPIO_AltFuncTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  // GPIO_STA8090_H

// End of file
