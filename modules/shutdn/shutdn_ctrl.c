//!
//!   \file     shutdn_ctrl.c
//!   \brief    <i><b> Low Voltage Detector Management source file</b></i>
//!   \author   Aldo Occhipinti
//!   \version  1.0
//!   \date     2014.03.14
//!   \bug      Unknown
//!   \warning  None
//!

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"
#include "shutdn_ctrl.h"

// GPS library
#include "gnss_debug.h"

#include "lld_sqi_ctrl.h"
#include "svc_gpio.h"
#include "svc_sqi.h"
#include "sw_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \cond internal_docs */

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SHUTDOWN_PIN_DISABLE                    0U
#define SHUTDOWN_PIN_ENABLE                     1U

#define SHUTDOWN_PIN_RISING_EDGE                0U
#define SHUTDOWN_PIN_FALLING_EDGE               1U
#define SHUTDOWN_PIN_RISING_AND_FALLING_EDGE    2U

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static void shutdn_ctrl_callback( const void * param );

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param
 * \return void
 *
 ***********************************************/
static GENERIC_CODE_ISR void shutdn_ctrl_callback( const void * param)
{
  gpOS_interrupt_lock();

  // Suspend any program/erase
  if( gpOS_SUCCESS == svc_sqi_power_down())
  {
    for( ; ; )
    { }
  }
  else
  {
    // To satisfy Misra check!
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param
 * \param
 * \return
 *
 ***********************************************/
gpOS_error_t shutdn_ctrl_start( void)
{
  gpOS_error_t error = gpOS_SUCCESS;
  tU32 shutdn_cfg;
  tU8 shutdn_pin_en, shutdn_pin_edge, shutdn_pin_id, shutdn_pin_alt;
  tChar edge[19] = "";

  // Read current Shutdown management configuration (enable, interrupt edge, GPIO pin ID, GPIO alternate function)
  if( GNSS_ERROR == sw_config_get_param( CURRENT_CONFIG_DATA, SHUTDN_CTRL_CFG_ID, &shutdn_cfg))
  {
    error = gpOS_FAILURE;
  }
  else
  {
    shutdn_pin_en   = ( tU8)( shutdn_cfg & 0x0001U);
    shutdn_pin_edge = ( tU8)(( shutdn_cfg & 0x0006U) >> 1U);
    shutdn_pin_id   = ( tU8)(( shutdn_cfg & 0x3F00U) >> 8U);
    shutdn_pin_alt  = ( tU8)(( shutdn_cfg & 0xC000U) >> 14U);

    // Shutdown by external PIN is disabled
    if( shutdn_pin_en == SHUTDOWN_PIN_DISABLE)
    {
      error = gpOS_SUCCESS;
    }
    else
    {
      // GPIO pin id is not valid
      if(( shutdn_pin_id >= GPIO_TOTAL_PINS_NUMBER) || ( shutdn_pin_edge > SHUTDOWN_PIN_RISING_AND_FALLING_EDGE))
      {
        error = gpOS_FAILURE;
      }
      else
      {
        // Case of GPIO pin up to GPIO_31
        if( shutdn_pin_id < 32U)
        {
          if( gpOS_FAILURE == svc_gpio_open_port( SVC_GPIO_PORT_0, gpOS_INTERRUPT_NOPRIORITY))
          {
            error = gpOS_FAILURE;
          }
          else
          {
            if( gpOS_FAILURE == svc_gpio_interrupt_install( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)shutdn_pin_id, shutdn_ctrl_callback))      /*lint !e9030 !e9034 */
            {
              error = gpOS_FAILURE;
            }
            else
            {
              // Case of GPIO interrupt Rising Edge
              if( shutdn_pin_edge == SHUTDOWN_PIN_RISING_EDGE)
              {
                if( gpOS_FAILURE == svc_gpio_interrupt_enable( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)shutdn_pin_id, SVC_GPIO_RISING_EDGE))   /*lint !e9030 !e9034 */
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  _clibs_sprintf( edge, "Rising");
                }
              }
              // Case of GPIO interrupt Falling Edge
              else if( shutdn_pin_edge == SHUTDOWN_PIN_FALLING_EDGE)
              {
                if( gpOS_FAILURE == svc_gpio_interrupt_enable( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)shutdn_pin_id, SVC_GPIO_FALLING_EDGE))  /*lint !e9030 !e9034 */
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  _clibs_sprintf( edge, "Falling");
                }
              }
              // Case of GPIO interrupt Rising and Falling Edge
              else if( shutdn_pin_edge == SHUTDOWN_PIN_RISING_AND_FALLING_EDGE)
              {
                if( gpOS_FAILURE == svc_gpio_interrupt_enable( SVC_GPIO_PORT_0, ( svc_gpio_pin_id_t)shutdn_pin_id, SVC_GPIO_RISING_AND_FALLING_EDGE)) /*lint !e9030 !e9034 */
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  _clibs_sprintf( edge, "Rising and Falling");
                }
              }
              else
              {
                // To satisfy Misra check!!!
              }

              if( gpOS_FAILURE != error)
              {
                LLD_GPIO_SetControlMode(( LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( BIT_0 << shutdn_pin_id), ( LLD_GPIO_ModeTy)shutdn_pin_alt); /*lint !e835 !e923 !e9030 !e9034 */
                LLD_GPIO_SetDirectionInput(( LLD_GPIO_idTy)GPIO0_REG_START_ADDR, ( BIT_0 << shutdn_pin_id));                                /*lint !e835 !e923 !e9030 !e9034 */

                GPS_DEBUG_MSG(("[shutdn] Shutdown from external GPIO pin is enabled\r\n"));
                GPS_DEBUG_MSG(("[shutdn] External GPIO pin is: GPIO_%d\r\n", shutdn_pin_id));
                GPS_DEBUG_MSG(("[shutdn] Interrupt GPIO edge is: %s\r\n", edge));
              }
            }
          }
        }
        // Case of GPIO pin from GPIO_32 to GPIO_63
        else
        {
          shutdn_pin_id = shutdn_pin_id - 32U;

          if( gpOS_FAILURE == svc_gpio_open_port( SVC_GPIO_PORT_1, gpOS_INTERRUPT_NOPRIORITY))
          {
            error = gpOS_FAILURE;
          }
          else
          {
            if( gpOS_FAILURE == svc_gpio_interrupt_install( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)shutdn_pin_id, shutdn_ctrl_callback))      /*lint !e9030 !e9034 */
            {
              error = gpOS_FAILURE;
            }
            else
            {
              // Case of GPIO interrupt Rising Edge
              if( shutdn_pin_edge == SHUTDOWN_PIN_RISING_EDGE)
              {
                if( gpOS_FAILURE == svc_gpio_interrupt_enable( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)shutdn_pin_id, SVC_GPIO_RISING_EDGE))   /*lint !e9030 !e9034 */
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  _clibs_sprintf( edge, "Rising");
                }
              }
              // Case of GPIO interrupt Falling Edge
              else if( shutdn_pin_edge == SHUTDOWN_PIN_FALLING_EDGE)
              {
                if( gpOS_FAILURE == svc_gpio_interrupt_enable( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)shutdn_pin_id, SVC_GPIO_FALLING_EDGE))  /*lint !e9030 !e9034 */
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  _clibs_sprintf( edge, "Falling");
                }
              }
              // Case of GPIO interrupt Rising and Falling Edge
              else if( shutdn_pin_edge == SHUTDOWN_PIN_RISING_AND_FALLING_EDGE)
              {
                if( gpOS_FAILURE == svc_gpio_interrupt_enable( SVC_GPIO_PORT_1, ( svc_gpio_pin_id_t)shutdn_pin_id, SVC_GPIO_RISING_AND_FALLING_EDGE)) /*lint !e9030 !e9034 */
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  _clibs_sprintf( edge, "Rising and Falling");
                }
              }
              else
              {
                // To satisfy Misra check!!!
              }

              if( gpOS_FAILURE != error)
              {
                LLD_GPIO_SetControlMode(( LLD_GPIO_idTy)GPIO1_REG_START_ADDR, ( BIT_0 << shutdn_pin_id), ( LLD_GPIO_ModeTy)shutdn_pin_alt); /*lint !e835 !e923 !e9030 !e9034 */
                LLD_GPIO_SetDirectionInput(( LLD_GPIO_idTy)GPIO1_REG_START_ADDR, ( BIT_0 << shutdn_pin_id));                                /*lint !e835 !e923 !e9030 !e9034 */

                GPS_DEBUG_MSG(("[shutdn] Shutdown from external GPIO pin is enabled\r\n"));
                GPS_DEBUG_MSG(("[shutdn] External GPIO pin is: GPIO_%d\r\n", shutdn_pin_id+32U));
                GPS_DEBUG_MSG(("[shutdn] Interrupt GPIO edge is: %s\r\n", edge));
              }
            }
          }
        }
      }
    }
  }

  return error;
}

#ifdef __cplusplus
}
#endif

// End of file
