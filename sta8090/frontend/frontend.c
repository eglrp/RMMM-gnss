/*****************************************************************************
   FILE:          frontend.c
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

/*****************************************************************************
   includes
*****************************************************************************/

#include "clibs.h"
#include "lld.h"
#include "lld_ssp.h"
#include "lld_gpio.h"
#include "lld_prcc_sta8090.h"
#include "svc_ssp.h"
#include "svc_mcu.h"
#include "frontend.h"
#include "platform.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SSP_USED          SSP_REG_START_ADDR

#define FE_ADDR(addr)     ((addr & 0x7f) << 1)
#define FE_DATA(data)     (data & 0xff)
#define FE_READBIT        0x1

#define FE_TXRX_BUF_SIZE  2

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

static svc_ssp_com_handler_t *FE_handler = NULL;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/**< Default values for FE registers */
static tU8 FE_default_val[FE_NUMOFREGS] =
{
  0xFF, 0xFF, 0x3C, 0x6F, 0x9D, 0x78, 0xB7, 0x90,
  0x00, 0x00, 0x00, 0x9A, 0xA8, 0xF0, 0x3F, 0x30,
  0x80, 0x1A, 0x28, 0xE0, 0x7F, 0x30, 0x40, 0x0D,
  0x0D
};

/**< Backup value for SSP GPIOs */
static LLD_GPIO_ModeTy FE_prev_gpio_status[4];

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief   Pre transfer callback for SSP service
 *
 * \param   dummy   Not used
 * \return  void
 *
 ***********************************************/
static void FE_pre_cb( void *dummy)
{
  tUInt cnt;

  // For SSP pins (P0.24-27) check if any is configured as SSP and (if so) change it to GPIO (AF A).
  for( cnt = 0; cnt < sizeof( FE_prev_gpio_status); cnt++)
  {
    LLD_GPIO_PinTy curr_pin = (BIT_x(24) << cnt);

    FE_prev_gpio_status[cnt] = LLD_GPIO_GetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, curr_pin);

    if( FE_prev_gpio_status[cnt] == LLD_GPIO_ALTERNATE_NONE)
    {
      LLD_GPIO_SetStateHigh( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, curr_pin);
      if( cnt == 2)
      {
        LLD_GPIO_SetDirectionInput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, curr_pin);
      }
      else
      {
        LLD_GPIO_SetDirectionOutput( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, curr_pin);
      }
      LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, curr_pin, LLD_GPIO_ALTERNATE_MODE_A);
    }
  }

  LLD_PRCC_RFSPIIfEnable();
}

/********************************************//**
 * \brief   Post transfer callback for SSP service
 *
 * \param   dummy   Not used
 * \return  void
 *
 ***********************************************/
static void FE_post_cb( void *dummy)
{
  tUInt cnt;

  LLD_PRCC_RFSPIIfDisable();

  // Restore SSP pins (P1.0-3) to previous status if needed
  for( cnt = 0; cnt < sizeof( FE_prev_gpio_status); cnt++)
  {
    LLD_GPIO_PinTy curr_pin = (LLD_GPIO_PinTy)(((tUInt)1U<<24) << cnt);

    if( FE_prev_gpio_status[cnt] == LLD_GPIO_ALTERNATE_NONE)
    {
      LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, curr_pin, LLD_GPIO_ALTERNATE_NONE);
    }
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Initialize front end
 *
 * \param   periph        not used
 * \return  LLD_NO_ERROR if all ok
 *
 ***********************************************/
LLD_ErrorTy FE_init( FE_peripheral_if_t periph)
{
  if( FE_handler != NULL)
  {
    return LLD_NO_ERROR;
  }

  if( platform_get_sal_ext_sqi_status() == TRUE)
  {
    FE_handler = svc_ssp_create_com( 0, LLD_SSP_INTERFACE_MOTOROLA_SPI, 1000000, LLD_SSP_DATA_BITS_8,
                                      NULL, NULL, NULL, NULL);
  }
  else
  {
    FE_handler = svc_ssp_create_com( 0, LLD_SSP_INTERFACE_MOTOROLA_SPI, 1000000, LLD_SSP_DATA_BITS_8,
                                      (svc_ssp_hook_t)FE_pre_cb, NULL, (svc_ssp_hook_t)FE_post_cb, NULL);
  }

  if( FE_handler == NULL)
  {
    return LLD_ERROR;
  }

  LLD_PRCC_RFRegRstDis();
  svc_mcu_enable( SVC_MCU_PER_ID_GPIO, 0);

  return LLD_NO_ERROR;
}

/********************************************//**
 * \brief   Reset front end to default values
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void FE_reset( void)
{
  FE_write_burst_data( FE_default_val);
}

/********************************************//**
 * \brief   Write a data to frontend
 *
 * \param   addr  address to write
 * \param   data  data to write
 * \return  LLD_NO_ERROR if all ok
 *
 ***********************************************/
LLD_ErrorTy FE_write_data( tUInt addr, tU8 data)
{
  LLD_ErrorTy error = LLD_ERROR;

  if( (FE_handler != NULL) && ( addr < FE_NUMOFREGS))
  {
    tU8 tx_buf[FE_TXRX_BUF_SIZE], rx_buf[FE_TXRX_BUF_SIZE];

    tx_buf[0] = FE_ADDR( addr);
    tx_buf[1] = FE_DATA( data);

    svc_ssp_write( FE_handler, tx_buf, FE_TXRX_BUF_SIZE, rx_buf, gpOS_TIMEOUT_INFINITY);

    error = LLD_NO_ERROR;
  }

  return error;
}

/********************************************//**
 * \brief   Write a data to frontend
 *
 * \param   addr  address to write
 * \param   data  data to write
 * \return  LLD_NO_ERROR if all ok
 *
 ***********************************************/
LLD_ErrorTy FE_write_burst_data( tU8 *data)
{
  LLD_ErrorTy error = LLD_ERROR;

  if(FE_handler != NULL)
  {
    tU8 tx_buf[FE_NUMOFREGS+1], rx_buf[FE_NUMOFREGS+1];

    tx_buf[0] = 0x0;
    _clibs_memcpy(&(tx_buf[1]), data, FE_NUMOFREGS);

    svc_ssp_write( FE_handler, tx_buf, FE_NUMOFREGS+1, rx_buf, gpOS_TIMEOUT_INFINITY);

    error = LLD_NO_ERROR;
  }

  return error;
}

/********************************************//**
 * \brief   Read a data from frontend
 *
 * \param   addr      address to write
 * \param   data_ptr  pointer where to save read data
 * \return  LLD_NO_ERROR if all ok
 *
 ***********************************************/
LLD_ErrorTy FE_read_data( tUInt addr, tUInt *data_ptr)
{
  LLD_ErrorTy error = LLD_ERROR;

  if( (FE_handler != NULL) && ( addr < FE_NUMOFREGS))
  {
    tU8 tx_buf[FE_TXRX_BUF_SIZE], rx_buf[FE_TXRX_BUF_SIZE];

    tx_buf[0] = FE_ADDR( addr) | FE_READBIT;
    tx_buf[1] = 0;

    svc_ssp_write( FE_handler, tx_buf, FE_TXRX_BUF_SIZE, rx_buf, gpOS_TIMEOUT_INFINITY);

    *data_ptr = rx_buf[1];

    error = LLD_NO_ERROR;
  }

  return error;
}

/********************************************//**
 * \brief   Writes register bits according to the bitmask
 *
 * \param   addr      address to write
 * \param   bitmask   bitmask to modify
 * \param   value     value to write
 * \return  LLD_NO_ERROR if all ok
 *
 ***********************************************/
LLD_ErrorTy FE_write_bits( tUInt addr, tU8 bitmask, tU8 value)
{
  tUInt ssp_data;
  LLD_ErrorTy error = LLD_ERROR;

  if( FE_read_data( addr, &ssp_data) == LLD_NO_ERROR)
  {
    ssp_data &= ~bitmask;
    ssp_data |= (value & bitmask);
    error = FE_write_data( addr, ssp_data);
  }

  return error;
}

/********************************************//**
 * \brief   Writes register bits according to the bitmask in default registers array
 *
 * \param   addr      address to write
 * \param   bitmask   bitmask to modify
 * \param   value     value to write
 * \return  void
 *
 ***********************************************/
void FE_def_write_bits( tUInt addr, tU8 bitmask, tU8 value)
{
  tU32 data = FE_default_val[addr];

  data &= ~bitmask;
  data |= (value & bitmask);
  FE_default_val[addr] = data;
}

/********************************************//**
 * \brief   Set default value for given register into the FE_default_val array
 *
 * \param   addr    address to write
 * \param   value   value to write
 * \return  void
 *
 ***********************************************/
void FE_def_write_data( tU8 addr, tU8 value)
{
  FE_default_val[addr] = value;
}

/********************************************//**
 * \brief   Return default value for given register into the FE_default_val array
 *
 * \param   addr    address to write
 * \param   value   value to write
 * \return  void
 *
 ***********************************************/
void FE_def_read_data( tU8 addr, tUInt *data_ptr)
{
  *data_ptr = FE_default_val[addr];
}

/********************************************//**
 * \brief   Dumps registers in a buffer
 *
 * \param   reg_table_ptr   pointer to buffer
 * \return  LLD_NO_ERROR if all ok
 *
 ***********************************************/
LLD_ErrorTy FE_dump_regs( FE_reg_item_t *reg_table_ptr)
{
  LLD_ErrorTy error = LLD_ERROR;

  if(( FE_handler != NULL) && ( reg_table_ptr != NULL))
  {
    tU32 addr = 0;

    while( addr < FE_NUMOFREGS)
    {
      tUInt data;

      FE_read_data( addr, &data);

      reg_table_ptr[addr].addr = addr;
      reg_table_ptr[addr].data = data;

      addr++;
    }

    reg_table_ptr[addr].addr = 0xff;
    reg_table_ptr[addr].data = 0xff;

    error = LLD_NO_ERROR;
  }

  return error;
}
