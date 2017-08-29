/*****************************************************************************
   FILE:          svc_can
   PROJECT:
   SW PACKAGE:    CAN svc_mcu
------------------------------------------------------------------------------
   DESCRIPTION:   CAN svc_mcu
------------------------------------------------------------------------------
   COPYRIGHT:     (c) <year> STMicroelectronics, (<group>) <site>
------------------------------------------------------------------------------
   Developers:
      AI:   Author Initials
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   yy.mm.dd  |  AI  | Original version
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include <string.h>
#include "typedefs.h"
#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_can.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define SVC_CAN_HANDLER_SIZE         sizeof( svc_can_handler_t)
#define SVC_CAN_PORT_HANDLER_SIZE    sizeof( svc_can_port_handler_t *)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief CAN port handler definition
 ***********************************************/

typedef struct svc_can_msg_data_s
{
  tU32 obj_id;
  tU32 msg_id;
  tU8  data[8];
}svc_can_msg_data_t;

typedef struct svc_can_port_handler_s
{
  //config infos
  tUInt                    id;
  LLD_CAN_BaudRateTy       baud_rate;

  // access mutex to the critical resource
  gpOS_mutex_t *                access_mutex;
  gpOS_semaphore_t *            irq_sem;

  //Data
  LLD_CAN_Read_Msg_t       *CanRxDataPtr;
  tU16                     irq_status;
  tU16                     TSeg2;
  tU16                     TSeg1;
  tU16                     SJW;
  tU16                     BRP;
  LLD_CAN_IdType           IdType;
  tBool                    isSOMactive;

  svc_can_msg_data_t      *msg_buffer;
  tU32                     msg_buffer_widx;
  tU32                     msg_buffer_ridx;
  tU32                     max_msg_buffered;

} svc_can_port_handler_t;

/********************************************//**
 * \brief CAN handler definition
 ***********************************************/
typedef struct svc_can_handler_s
{
  svc_mcu_item_t             svc_mcu_item_handler;

  svc_can_port_handler_t *   port_head;

  svc_can_port_handler_t ** port;
} svc_can_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
static svc_can_handler_t *svc_can_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static void   svc_can_HandleRXMsgObj   ( svc_can_port_handler_t *hdlr_ptr);
static void   svc_can_callback         ( svc_can_port_handler_t *hdlr_ptr);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Load data from RX
 *
 * \param hdlr_ptr Pointer to CAN port handler
 * \return void
 *
 ***********************************************/
static LLD_ISR_CAN void svc_can_HandleRXMsgObj( svc_can_port_handler_t *hdlr_ptr)
{
  LLD_CAN_IdTy can_phy_id = (LLD_CAN_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_CAN, hdlr_ptr->id);

  LLD_CAN_Read_Object_Data( can_phy_id, hdlr_ptr->irq_status, (LLD_CAN_Read_Msg_t *)&hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_widx].data, hdlr_ptr->isSOMactive);

  hdlr_ptr->CanRxDataPtr = (LLD_CAN_Read_Msg_t *)&hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_widx].data;

}

/********************************************//**
 * \brief Callback for CAN svc_mcu to signal the receiver task
 *
 * \param hdlr_ptr  pointer to can handler associated with
                    svc_mcud can
 * \return void
 *
 ***********************************************/
static LLD_ISR_CAN void svc_can_callback( svc_can_port_handler_t *hdlr_ptr)
{
  LLD_CAN_IdTy can_phy_id = (LLD_CAN_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_CAN, hdlr_ptr->id);

  /**< Handle RX interrupts */
  hdlr_ptr->irq_status = LLD_CAN_GetInterruptID( can_phy_id);
  while(hdlr_ptr->irq_status != 0)
  {

    if (hdlr_ptr->irq_status == 0x8000 ) //CAN_STATUS_INTERRUPT)
    {
      hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_widx].obj_id = 0;
      *((tU64 *)hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_widx].data) = (tU64)LLD_CAN_GetStatusRegister(can_phy_id);
      hdlr_ptr->msg_buffer_widx = (hdlr_ptr->msg_buffer_widx + 1) % hdlr_ptr->max_msg_buffered;
      gpOS_semaphore_signal( hdlr_ptr->irq_sem);
    }
    else
    {
       if ( hdlr_ptr->isSOMactive == TRUE )
          {
            tU32 obj_id, id;

            if(LLD_CAN_single_obj_sw_filtering( can_phy_id, hdlr_ptr->irq_status, hdlr_ptr->IdType, &obj_id, &id) == 0)
              {
                hdlr_ptr->msg_buffer[ hdlr_ptr->msg_buffer_widx].obj_id = obj_id;
                hdlr_ptr->msg_buffer[ hdlr_ptr->msg_buffer_widx].msg_id = id;
                svc_can_HandleRXMsgObj( hdlr_ptr);
                hdlr_ptr->msg_buffer_widx = ( hdlr_ptr->msg_buffer_widx + 1) % hdlr_ptr->max_msg_buffered;
                gpOS_semaphore_signal( hdlr_ptr->irq_sem);
              }
          } else
             {
                hdlr_ptr->msg_buffer[ hdlr_ptr->msg_buffer_widx].obj_id = hdlr_ptr->irq_status;
                svc_can_HandleRXMsgObj( hdlr_ptr);

                if ( hdlr_ptr->IdType == LLD_CAN_STD_ID)
                     hdlr_ptr->msg_buffer[ hdlr_ptr->msg_buffer_widx].msg_id = ( LLD_CAN_Get_ID( can_phy_id, hdlr_ptr->irq_status, LLD_CAN_IF2) >> 2);
                     else if ( hdlr_ptr->IdType == LLD_CAN_EXT_ID)
                       hdlr_ptr->msg_buffer[ hdlr_ptr->msg_buffer_widx].msg_id = LLD_CAN_Get_ExtID( can_phy_id, hdlr_ptr->irq_status, LLD_CAN_IF2);

                hdlr_ptr->msg_buffer_widx = ( hdlr_ptr->msg_buffer_widx + 1) % hdlr_ptr->max_msg_buffered;
                gpOS_semaphore_signal( hdlr_ptr->irq_sem);
             }
    }

    hdlr_ptr->irq_status = LLD_CAN_GetInterruptID( can_phy_id);

  }
}

/********************************************//**
 * \brief Delete all OS allocated items
 *
 * \param hdlr_ptr  Pointer to handler to delete
 *
 ***********************************************/
static void svc_can_com_create_fail( svc_can_port_handler_t *hdlr_ptr)
{
  gpOS_interrupt_uninstall( svc_mcu_get_irq_line( SVC_MCU_PER_ID_CAN, hdlr_ptr->id));

  gpOS_mutex_delete( hdlr_ptr->access_mutex);

  gpOS_semaphore_delete( hdlr_ptr->irq_sem);

  gpOS_memory_deallocate_p( svc_can_handler->svc_mcu_item_handler.part, hdlr_ptr);
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initalize CAN svc_mcu
 *
 * \param partition     Partition used for svc_mcu data
 * \param bus_speed     Bus speed
 * \return gpOS_error_t gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_can_init( gpOS_partition_t *partition, tU32 bus_id)
{
  tUInt cnt;
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_can_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_can_handler = gpOS_memory_allocate_p( partition, SVC_CAN_HANDLER_SIZE);

  if( svc_can_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_CAN, bus_id, &svc_can_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_can_handler);
    return gpOS_FAILURE;
  }

  svc_can_handler->svc_mcu_item_handler.part = partition;
  svc_can_handler->svc_mcu_item_handler.bus_id  = bus_id;
  svc_can_handler->svc_mcu_item_handler.mem_used   = mem_at_start - gpOS_memory_getheapfree_p( partition);
  svc_can_handler->svc_mcu_item_handler.cmdif     = NULL;
  svc_can_handler->port_head  = NULL;

  svc_can_handler->port = gpOS_memory_allocate_p( partition, SVC_CAN_PORT_HANDLER_SIZE * svc_can_handler->svc_mcu_item_handler.phy_item->number);
  if( svc_can_handler->port == NULL)
  {
    gpOS_memory_deallocate_p( partition, svc_can_handler);
    return gpOS_FAILURE;
  }

  for( cnt = 0; cnt < svc_can_handler->svc_mcu_item_handler.phy_item->number; cnt++)
  {
    svc_can_handler->port[cnt] = (svc_can_port_handler_t *)NULL;
  }

  return gpOS_SUCCESS;
}

/********************************************************************//**
 * \brief Open a CAN port
 *
 * \param  can_id           can port to open
 * \param  irq_pri          can ID
 * \param  baud_rate        can baud rate
 * \param  id_type          Standard or Extended frames
 * \param  SOM              true to enable SOM; false to disable SOM
 * \param  max_msg_buffered max number of frame buffered
 * \return gpOS_error_t gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_can_open_port( tUInt can_id, gpOS_interrupt_priority_t irq_pri, LLD_CAN_BaudRateTy baud_rate,
                                 LLD_CAN_IdType id_type, tBool SOM, tU32 max_msg_buffered)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_can_handler->svc_mcu_item_handler.part);
  svc_can_port_handler_t *hdlr_ptr;
  tU32 bus_clk;

  LLD_CAN_IdTy can_phy_id       = (LLD_CAN_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_CAN, can_id);
  VicLineTy    can_phy_irq_line = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_CAN, can_id);

  if( svc_can_handler == NULL)
  {
    return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
  }

  if( svc_can_handler->port[can_id] != NULL)
  {
    return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
  }

  /**< Allocate needed memory */
  hdlr_ptr = gpOS_memory_allocate_p( svc_can_handler->svc_mcu_item_handler.part, sizeof( svc_can_port_handler_t));

  if( hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
  }

  hdlr_ptr->access_mutex = gpOS_mutex_create_p( MUTEX_FIFO, svc_can_handler->svc_mcu_item_handler.part);
  hdlr_ptr->irq_sem      = gpOS_semaphore_create_p( SEM_FIFO, svc_can_handler->svc_mcu_item_handler.part, 0);

  if(
      ( hdlr_ptr->access_mutex == NULL) ||
      ( hdlr_ptr->irq_sem == NULL)
    )
  {
    svc_can_com_create_fail( hdlr_ptr);
    return( gpOS_FAILURE);  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
  }

  hdlr_ptr->msg_buffer = (svc_can_msg_data_t *)gpOS_memory_allocate_p( svc_can_handler->svc_mcu_item_handler.part, max_msg_buffered * sizeof(svc_can_msg_data_t));

  if( hdlr_ptr->msg_buffer == NULL)
  {
    svc_can_com_create_fail( hdlr_ptr);
    return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
  }

  hdlr_ptr->msg_buffer_widx = 0;
  hdlr_ptr->msg_buffer_ridx = 0;
  hdlr_ptr->max_msg_buffered = max_msg_buffered;

  /**< Initialize CAN device and handler */
  hdlr_ptr->id            = can_id;
  hdlr_ptr->baud_rate     = baud_rate;
  hdlr_ptr->IdType        = id_type;

  svc_mcu_enable( SVC_MCU_PER_ID_CAN, can_id);

  svc_mcu_busclk_get( svc_can_handler->svc_mcu_item_handler.bus_id, &bus_clk);

  if( bus_clk == 52000000)
  {
    switch (baud_rate)
    {
      case LLD_CAN_50KBPS:    //50kbps @ 52MHz (S2 in usbcan)
        hdlr_ptr->TSeg2 =  8;
        hdlr_ptr->TSeg1 = 12;
        hdlr_ptr->SJW   =  4;
        hdlr_ptr->BRP   = 50;
      break;

      case LLD_CAN_100KBPS:     //100kbps @ 52MHz (S3 in usbcan)
        hdlr_ptr->TSeg2 =  4;
        hdlr_ptr->TSeg1 =  8;
        hdlr_ptr->SJW   =  4;
        hdlr_ptr->BRP   = 40;
      break;

      case LLD_CAN_125KBPS:     //125kbps @ 52MHz (S4 in usbcan)
        hdlr_ptr->TSeg2 =  6;
        hdlr_ptr->TSeg1 = 16;
        hdlr_ptr->SJW   =  4;
        hdlr_ptr->BRP   = 18;
      break;

      case LLD_CAN_250KBPS:     //250kbps @ 52MHz (S5 in usbcan)
        hdlr_ptr->TSeg2 =  4;
        hdlr_ptr->TSeg1 =  8;
        hdlr_ptr->SJW   =  4;
        hdlr_ptr->BRP   = 16;
      break;

      case LLD_CAN_500KBPS:     //500kbps @ 52MHz (S6 in usbcan)
        hdlr_ptr->TSeg2 =  4;
        hdlr_ptr->TSeg1 =  8;
        hdlr_ptr->SJW   =  4;
        hdlr_ptr->BRP   =  8;
      break;

      case LLD_CAN_1MBPS:      //1Mbps @ 52MHz (S8 in usbcan)
        hdlr_ptr->TSeg2 =  4;
        hdlr_ptr->TSeg1 =  8;
        hdlr_ptr->SJW   =  4;
        hdlr_ptr->BRP   =  4;
      break;

      default:
        return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
    }  /*lint !e9077: missing unconditional break from final switch case [MISRA 2012 Rule 16.3, required]*/
  }
  else if( bus_clk == 26000000)
  {
    switch (baud_rate)
    {
      case LLD_CAN_50KBPS:    //50kbps @ 26MHz (S2 in usbcan)
        hdlr_ptr->TSeg2 =  2;
        hdlr_ptr->TSeg1 = 10;
        hdlr_ptr->SJW   =  2;
        hdlr_ptr->BRP   = 40;
      break;

      case LLD_CAN_100KBPS:     //100kbps @ 26MHz (S3 in usbcan)
        hdlr_ptr->TSeg2 =  3;
        hdlr_ptr->TSeg1 = 16;
        hdlr_ptr->SJW   =  2;
        hdlr_ptr->BRP   = 13;
      break;

      case LLD_CAN_125KBPS:     //125kbps @ 26MHz (S4 in usbcan)
        hdlr_ptr->TSeg2 =  2;
        hdlr_ptr->TSeg1 = 13;
        hdlr_ptr->SJW   =  2;
        hdlr_ptr->BRP   = 13;
      break;

      case LLD_CAN_250KBPS:     //250kbps @ 26MHz (S5 in usbcan)
        hdlr_ptr->TSeg2 =  1;
        hdlr_ptr->TSeg1 =  6;
        hdlr_ptr->SJW   =  2;
        hdlr_ptr->BRP   = 13;
      break;

      case LLD_CAN_500KBPS:     //500kbps @ 26MHz (S6 in usbcan)
        hdlr_ptr->TSeg2 =  1;
        hdlr_ptr->TSeg1 =  2;
        hdlr_ptr->SJW   =  2;
        hdlr_ptr->BRP   =  13;
      break;

      case LLD_CAN_1MBPS:      //1Mbps @ 26MHz (S8 in usbcan)
        hdlr_ptr->TSeg2 =  2;
        hdlr_ptr->TSeg1 = 10;
        hdlr_ptr->SJW   =  2;
        hdlr_ptr->BRP   =  2;
      break;

      default:
        return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
    }  /*lint !e9077: missing unconditional break from final switch case [MISRA 2012 Rule 16.3, required]*/
  }
  else if( bus_clk == 48000000)
    {
      switch (baud_rate)
      {
        case LLD_CAN_50KBPS:     //50kbps @ 48MHz (S2 in usbcan)
          hdlr_ptr->TSeg2 =  3;
          hdlr_ptr->TSeg1 = 16;
          hdlr_ptr->SJW   =  2;
          hdlr_ptr->BRP   = 48;
        break;

        case LLD_CAN_100KBPS:     //100kbps @ 48MHz (S3 in usbcan)
          hdlr_ptr->TSeg2 =  3;
          hdlr_ptr->TSeg1 = 16;
          hdlr_ptr->SJW   =  2;
          hdlr_ptr->BRP   = 24;
        break;

        case LLD_CAN_125KBPS:     //125kbps @ 48MHz (S4 in usbcan)
          hdlr_ptr->TSeg2 =  3;
          hdlr_ptr->TSeg1 = 12;
          hdlr_ptr->SJW   =  3;
          hdlr_ptr->BRP   = 24;
        break;

        case LLD_CAN_250KBPS:     //250kbps @ 48MHz (S5 in usbcan)
          hdlr_ptr->TSeg2 =  2;
          hdlr_ptr->TSeg1 = 13;
          hdlr_ptr->SJW   =  1;
          hdlr_ptr->BRP   = 12;
        break;

        case LLD_CAN_500KBPS:     //500kbps @ 48MHz (S6 in usbcan)
          hdlr_ptr->TSeg2 =  1;
          hdlr_ptr->TSeg1 = 6;
          hdlr_ptr->SJW   =  1;
          hdlr_ptr->BRP   = 12;
        break;

        case LLD_CAN_1MBPS:     //1000kbps @ 48MHz (S7 in usbcan)
          hdlr_ptr->TSeg2 =  2;
          hdlr_ptr->TSeg1 = 13;
          hdlr_ptr->SJW   =  1;
          hdlr_ptr->BRP   = 3;
        break;

        default:
          return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
      }  /*lint !e9077: missing unconditional break from final switch case [MISRA 2012 Rule 16.3, required]*/
    }
  else
  {
    return gpOS_FAILURE;  /*lint !e904: Return statement before end of function [MISRA 2012 Rule 15.5, advisory] */
  }

  svc_mcu_enable( SVC_MCU_PER_ID_CAN, can_id);

  LLD_CAN_ResetReg( can_phy_id);

  if ( SOM == TRUE)
     LLD_CAN_single_obj_manager_init();

  // Initialize the CAN LLD
  LLD_CAN_Initialize( can_phy_id,
                      hdlr_ptr->TSeg2,
                      hdlr_ptr->TSeg1,
                      hdlr_ptr->SJW,
                      hdlr_ptr->BRP,
                      TRUE);

  gpOS_interrupt_install( can_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_can_callback, hdlr_ptr);
  gpOS_interrupt_enable( can_phy_irq_line);

  gpOS_mutex_release( hdlr_ptr->access_mutex);

  svc_can_handler->port[can_id] = hdlr_ptr;

  svc_can_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_can_handler->svc_mcu_item_handler.part);

  //SET32_BIT  ( 0x1200D004, BIT_3);
  //CLEAR32_BIT( 0x1200D004, BIT_3);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Setup the Object to be received on the CAN
 *
 * \param
 * \param
 * \param
 * \return gpOS_SUCCESS
 *
 ***********************************************/
tVoid svc_can_setup_rx_objects( tUInt can_id, tU32 object, tU32 message_id, tU32 mask, tBool SOM)
{
  LLD_CAN_IdTy can_phy_id       = (LLD_CAN_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_CAN, can_id);
  svc_can_port_handler_t       *hdlr_ptr = svc_can_handler->port[can_id];

  hdlr_ptr->isSOMactive = SOM;

  LLD_CAN_Setup_RX_Object( can_phy_id, object,  message_id,  mask,  hdlr_ptr->IdType, hdlr_ptr->isSOMactive);

  return;
}

/********************************************//**
 * \brief Receive a CAN Frame
 *
 * \param can_id    identifier of the can to be used
 * \param can_obj   CAN Object
 * \param msg_id    id of the CAN message
 * \param out_buf   buffer where to write chars
 * \param timeout   timeout
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_can_receive( tUInt can_id, tU32 *can_obj, tU32 *msg_id, tU64 *out_buf, gpOS_clock_t *timeout)
{
  gpOS_error_t error = gpOS_SUCCESS;
  svc_can_port_handler_t *hdlr_ptr = svc_can_handler->port[can_id];

  gpOS_mutex_lock( hdlr_ptr->access_mutex);

  if( gpOS_semaphore_wait_timeout( hdlr_ptr->irq_sem, timeout) == gpOS_FAILURE)
    error = gpOS_FAILURE;

  *can_obj = hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_ridx].obj_id;
  *msg_id  = hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_ridx].msg_id;

  memcpy(out_buf,hdlr_ptr->msg_buffer[hdlr_ptr->msg_buffer_ridx].data,8);

  hdlr_ptr->msg_buffer_ridx = (hdlr_ptr->msg_buffer_ridx + 1) % hdlr_ptr->max_msg_buffered;

  gpOS_mutex_release( hdlr_ptr->access_mutex);

  return error;

}

/********************************************//**
 * \brief Sends a CAN Frame
 *
 * \param can_id    identifier of the can to be used
 * \param can_obj   CAN Object
 * \param msg_id    id of the CAN message
 * \param out_buf   buffer where to write chars
 * \param dlc       data lenght code
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_can_send( tUInt can_id, tU32 *can_obj, tU32 *msg_id, tU64 *out_buf, tU32 dlc, LLD_CAN_IdType id_type)
{
  svc_can_port_handler_t *hdlr_ptr = svc_can_handler->port[can_id];
  LLD_CAN_IdTy can_phy_id = (LLD_CAN_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_CAN, hdlr_ptr->id);
  tU32 Object;
  tU32 ID;
  tInt timeout1 = 1000;
  tInt timeout2 = 3000;

  Object = (tU32) *can_obj;
  ID     = (tU32) *msg_id;

  LLD_CAN_Setup_TX_Object( can_phy_id, Object, ID, (tU8 *) out_buf, dlc, id_type);

  // Waiting the Message Object to be ready to transmit
  while( LLD_CAN_Check_Transmit_Request ( can_phy_id, Object) && ( timeout1 >0 ))
    {
       timeout1--;
    }

  if( !timeout1)
    {
      return( gpOS_FAILURE );
    }

  LLD_CAN_Transmit_Request( can_phy_id, Object);

  while( !LLD_CAN_GetTxOK_Bit( can_phy_id)  && ( timeout2 > 0))
    {
       timeout2--;
    }

  if ( !timeout2)
   {

     return( gpOS_FAILURE );

   } else {

            LLD_CAN_ClearTxOK_Bit(can_phy_id);
            return gpOS_SUCCESS;

          }
}

/********************************************//**
 * \brief Lock CAN for a specific task
 *
 * \param can_id CAN port
 * \return gpOS_error_t
 *
 ***********************************************/
void svc_can_lock( tUInt can_id)
{
  gpOS_mutex_lock( svc_can_handler->port[can_id]->access_mutex);
}

/********************************************//**
 * \brief Release CAN from a specific task
 *
 * \param can_id CAN port
 * \return gpOS_error_t
 *
 ***********************************************/
void svc_can_release( tUInt can_id)
{
  gpOS_mutex_release( svc_can_handler->port[can_id]->access_mutex);
}
/* End of file */
