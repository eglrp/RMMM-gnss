/*****************************************************************************
   FILE:          svc_usb
   PROJECT:
   SW PACKAGE:    USB svc_mcu
------------------------------------------------------------------------------
   DESCRIPTION:   USB svc_mcu
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

#include "typedefs.h"
#include "clibs.h"
#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_usb.h"
#include "gnss_api.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_USB_RX_PACKET_SIZE           64

#define SVC_USB_RX_PACKET_BUFPTR( x)     ((x)->fifo_rx.buf_ptr)

/*===========================================================================*/
/* CDC related stuff.                                                        */
/*===========================================================================*/

#define CDC_SEND_ENCAPSULATED_COMMAND   0x00
#define CDC_GET_ENCAPSULATED_RESPONSE   0x01
#define CDC_SET_COMM_FEATURE            0x02
#define CDC_GET_COMM_FEATURE            0x03
#define CDC_CLEAR_COMM_FEATURE          0x04
#define CDC_SET_AUX_LINE_STATE          0x10
#define CDC_SET_HOOK_STATE              0x11
#define CDC_PULSE_SETUP                 0x12
#define CDC_SEND_PULSE                  0x13
#define CDC_SET_PULSE_TIME              0x14
#define CDC_RING_AUX_JACK               0x15
#define CDC_SET_LINE_CODING             0x20
#define CDC_GET_LINE_CODING             0x21
#define CDC_SET_CONTROL_LINE_STATE      0x22
#define CDC_SEND_BREAK                  0x23
#define CDC_SET_RINGER_PARMS            0x30
#define CDC_GET_RINGER_PARMS            0x31
#define CDC_SET_OPERATION_PARMS         0x32
#define CDC_GET_OPERATION_PARMS         0x33

#define LC_STOP_1                       0
#define LC_STOP_1P5                     1
#define LC_STOP_2                       2

#define LC_PARITY_NONE                  0
#define LC_PARITY_ODD                   1
#define LC_PARITY_EVEN                  2
#define LC_PARITY_MARK                  3
#define LC_PARITY_SPACE                 4

/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

#define SVC_USB_VCOM_EP_DATA     1
#define SVC_USB_VCOM_EP_IRQ      2

#define SVC_USB_TASK_STACK_SIZE        512   /* size of the USB task stack */
#define SVC_USB_TASK_PRIORITY          8

#define SVC_USB_HANDLER_SIZE           sizeof( svc_usb_handler_t)
#define SVC_USB_PORT_HANDLER_SIZE      sizeof( svc_usb_port_handler_t)
#define SVC_USB_COM_HANDLER_SIZE       sizeof( svc_usb_com_handler_t)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief I2C COM handler
 ***********************************************/
struct svc_usb_com_handler_s
{
  svc_usb_com_handler_t *   next;           /**< Pointer to next COM in the queue */

  tUInt                     port_id;        /**< I2C port associated to the COM */
  gpOS_semaphore_t *          access_sem;     /**< access semaphore to com */

  //svc_usb_com_hooks_t      hooks;          /**< Pre/post callbacks for specified COM */

  //LLD_USB_configTy          config;         /**< I2C configuration for COM */
  //tUInt                     addr;           /**< Address of connected peripheral */
};

/********************************************//**
 * \brief OS20 UART FIFO handler type
 ***********************************************/
typedef struct svc_usb_fifo_handler_s {
  // access semaphore to the critical resource
  gpOS_semaphore_t *  access_sem;

  // FIFO conditions tools
  gpOS_semaphore_t *  init_sem;
  gpOS_semaphore_t *  fifo_sem;

  tU8               ep;
} svc_usb_fifo_handler_t;

/********************************************//**
 * \brief USB port handler definition
 ***********************************************/
typedef struct svc_usb_port_handler_s svc_usb_port_handler_t;

struct svc_usb_port_handler_s
{
  svc_usb_port_handler_t *  next;

  boolean_t                 active;
  boolean_t                 open;
  boolean_t                 init;

  svc_usb_com_handler_t *   curr_com_ptr;           /**< COM currently using the port */

  // access mutex to the critical resource
  gpOS_mutex_t *              access_mutex;

  gpOS_task_t *               usb_task;
  gpOS_semaphore_t *          irq_sem;

  // Stream config infos
  tUInt                     id;

  // FIFOs handlers
  svc_usb_fifo_handler_t    fifo_rx;
  svc_usb_fifo_handler_t fifo_tx;

  // USB port data
  LLD_USB_config_t          usbcfg;
  LLD_USB_driver_t          usb;
};

/********************************************//**
 * \brief USB handler definition
 ***********************************************/
typedef struct svc_usb_handler_s
{
  svc_mcu_item_t             svc_mcu_item_handler;

  svc_usb_com_handler_t *   com_head;     /**< Linked list of COMs */
  svc_usb_port_handler_t *  port_head;
} svc_usb_handler_t;

/********************************************//**
 * \brief   Type of Line Coding structure.
 ***********************************************/
typedef struct svc_usb_cdc_linecoding_s {
  tU8                       dwDTERate[4];
  tU8                       bCharFormat;
  tU8                       bParityType;
  tU8                       bDataBits;
} svc_usb_cdc_linecoding_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
//This enable/disable the data terminal equimp
tBool usb_dte_configuration = FALSE;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static gpOS_error_t                   svc_usb_reset                ( svc_usb_port_handler_t *hdlr_ptr);
static svc_usb_port_handler_t *     svc_usb_id_to_port           ( tUInt usb_id);
static svc_usb_port_handler_t *     svc_usb_usbp_to_port         ( LLD_USB_driver_t *usbp);
static LLD_ISR_USB void             svc_usb_callback             ( svc_usb_port_handler_t *hdlr_ptr);
static void                         svc_usb_com_create_fail      ( svc_usb_port_handler_t *hdlr_ptr);
static void                         svc_usb_reset_port           ( svc_usb_port_handler_t *hdlr_ptr);

static void                         svc_usb_vcom_event           ( LLD_USB_driver_t *usbp, LLD_USB_event_t event);
static const LLD_USB_descriptor_t * svc_usb_vcom_get_descriptor  ( LLD_USB_driver_t *usbp, tU8 dtype, tU8 dindex, tU16 lang);
static void                         svc_usb_vcom_data_txed       ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep);
static void                         svc_usb_vcom_data_rxed       ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep);
static void                         svc_usb_vcom_irq_txed        ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep);
static tBool                        svc_usb_vcom_requests_hook   ( LLD_USB_driver_t *usbp);

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_usb_handler_t *svc_usb_handler = NULL;
static tU8 ep1setup_buffer[8];
static tU8 ep2setup_buffer[8];
#if defined( __STA8090__)
static tU8 ep3setup_buffer[8];
static tU8 ep4setup_buffer[8];
#endif

static tU8 rx_data_ptr[SVC_USB_RX_PACKET_SIZE];
static tU16 rx_data_index = 0;
static tU16 rx_data_len = 0;

/********************************************//**
 * \brief   Type of Line Coding structure.
 ***********************************************/
static const svc_usb_cdc_linecoding_t svc_usb_cdc_linecoding = {
  {0x00, 0x96, 0x00, 0x00},             /* 38400.                           */
  LC_STOP_1, LC_PARITY_NONE, 8
};

/********************************************//**
 * \brief   USB Device Descriptor for CDC.
 ***********************************************/
static const tU8 svc_usb_vcom_ddd[] = {
  18,                               /* bLength.                             */
  LLD_USB_DESCRIPTOR_DEVICE,        /* bDescriptorType.                     */
  0x10, 0x01,                       /* bcdUSB (1.1).                        */
  0x02,                             /* bDeviceClass (CDC).                  */
  0x00,                             /* bDeviceSubClass.                     */
  0x00,                             /* bDeviceProtocol.                     */
  0x40,                             /* bMaxPacketSize.                      */
  0x83, 0x04,                       /* idVendor (0x0483, STMicroelectonics).*/
#if defined( __STA8088__)
  0x01, 0xCC,                       /* idProduct (0xCC01, STA8088xx).       */
#elif defined( __STA8090__)
  0x03, 0xCC,                       /* idProduct (0xCC03, STA8090xx).       */
#endif
  0x00, 0x02,                       /* bcdDevice (2.00).                    */
  1,                                /* iManufacturer.                       */
  2,                                /* iProduct.                            */
  3,                                /* IiSerialNumber.                      */
  1                                 /* bNumConfigurations.                  */
};

/********************************************//**
 * \brief   Device descriptor wrapper.
 ***********************************************/
static const LLD_USB_descriptor_t svc_usb_vcom_dd = {
  sizeof (svc_usb_vcom_ddd),
  svc_usb_vcom_ddd
};

/********************************************//**
 * \brief   Configuration Descriptor tree for a VCOM.
 ***********************************************/
static const tU8 svc_usb_vcom_cdd[] = {
  /* Configuration descriptor.*/
  9,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_CONFIGURATION, /* bDescriptorType.                     */
  67,  0,                           /* wTotalLength.                        */
  0x02,                             /* bNumInterfaces.                      */
  0x01,                             /* bConfigurationValue.                 */
  0,                                /* iConfiguration.                      */
  0xC0,                             /* bmAttributes (self powered).         */
  50,                               /* MaxPower (100mA).                    */
  /* Interface Descriptor.*/
  9,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_INTERFACE,     /* bDescriptorType.                     */
  0x00,                             /* bInterfaceNumber.                    */
  0x00,                             /* bAlternateSetting.                   */
  0x01,                             /* bNumEndpoints.                       */
  0x02,                             /* bInterfaceClass (Communications
                                       Interface Class, CDC section 4.2).   */
  0x02,                             /* bInterfaceSubClass (Abstract Control
                                       Model, CDC section 4.3).             */
  0x01,                             /* bInterfaceProtocol (AT commands, CDC
                                       section 4.4).                        */
  0,                                /* iInterface.                          */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  5,                                /* bLength.                             */
  0x24,                             /* bDescriptorType (CS_INTERFACE).      */
  0x00,                             /* bDescriptorSubtype (Header Functional
                                       Descriptor.                          */
  0x10, 0x01,                       /* bcdCDC (1.10).                       */
  /* Call Managment Functional Descriptor. */
  5,                                /* bFunctionLength.                     */
  0x24,                             /* bDescriptorType (CS_INTERFACE).      */
  0x01,                             /* bDescriptorSubtype (Call Management
                                       Functional Descriptor).              */
  0x00,                             /* bmCapabilities (D0+D1).              */
  0x01,                             /* bDataInterface.                      */
  /* ACM Functional Descriptor.*/
  4,                                /* bFunctionLength.                     */
  0x24,                             /* bDescriptorType (CS_INTERFACE).      */
  0x02,                             /* bDescriptorSubtype (Abstract Control
                                       Management Descriptor).              */
  0x02,                             /* bmCapabilities.                      */
  /* Union Functional Descriptor.*/
  5,                                /* bFunctionLength.                     */
  0x24,                             /* bDescriptorType (CS_INTERFACE).      */
  0x06,                             /* bDescriptorSubtype (Union Functional
                                       Descriptor).                         */
  0x00,                             /* bMasterInterface (Communication Class
                                       Interface).                          */
  0x01,                             /* bSlaveInterface0 (Data Class
                                       Interface).                          */
  /* Endpoint 2 IN Descriptor.*/
  7,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_ENDPOINT,      /* bDescriptorType.                     */
  SVC_USB_VCOM_EP_IRQ | 0x80,      /* bEndpointAddress (IN).               */
  0x03,                             /* bmAttributes (Interrupt).            */
  0x10, 0x00,                       /* wMaxPacketSize.                      */
  0xFF,                             /* bInterval.                           */
  /* Interface Descriptor.*/
  9,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_INTERFACE,     /* bDescriptorType.                     */
  0x01,                             /* bInterfaceNumber.                    */
  0x00,                             /* bAlternateSetting.                   */
  0x02,                             /* bNumEndpoints.                       */
  0x0A,                             /* bInterfaceClass (Data Class
                                       Interface, CDC section 4.5).         */
  0x00,                             /* bInterfaceSubClass (CDC section 4.6).*/
  0x00,                             /* bInterfaceProtocol (CDC section 4.7).*/
  0x00,                             /* iInterface.                          */
  /* Endpoint 1 OUT Descriptor.*/
  7,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_ENDPOINT,      /* bDescriptorType.                     */
  SVC_USB_VCOM_EP_DATA,            /* bEndpointAddress (OUT).              */
  0x02,                             /* bmAttributes (Bulk).                 */
  0x40, 0x00,                       /* wMaxPacketSize.                      */
  0x00,                             /* bInterval (ignored for bulk.         */
  /* Endpoint 1 IN Descriptor.*/
  7,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_ENDPOINT,      /* bDescriptorType.                     */
  SVC_USB_VCOM_EP_DATA | 0x80,     /* bEndpointAddress (IN).               */
  0x02,                             /* bmAttributes (Bulk).                 */
  0x40, 0x00,                       /* wMaxPacketSize.                      */
  0x00                              /* bInterval (ignored for bulk.         */
};

/********************************************//**
 * \brief   Configuration descriptor wrapper.
 ***********************************************/
static const LLD_USB_descriptor_t svc_usb_vcom_cd = {
  sizeof (svc_usb_vcom_cdd),
  svc_usb_vcom_cdd
};

/********************************************//**
 * \brief   U.S. English language identifier.
 ***********************************************/
static const tU8 svc_usb_vcom_str_langid[] = {
  4,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_STRING,        /* bDescriptorType.                     */
  0x09, 0x04                        /* wLANGID (0x0409, U.S. English).      */
};

/********************************************//**
 * \brief   Vendor string.
 ***********************************************/
static const tU8 svc_usb_vcom_str_vendorid[] = {
  36,                               /* bLength.                             */
  LLD_USB_DESCRIPTOR_STRING,        /* bDescriptorType.                     */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/********************************************//**
 * \brief   Device description string.
 ***********************************************/
#if defined( __STA8088__)
static const tU8 svc_usb_vcom_str_devdesc[] = {
  48,                               /* bLength.                             */
  LLD_USB_DESCRIPTOR_STRING,        /* bDescriptorType.                     */
  'T', 0, 'e', 0, 's', 0, 'e', 0, 'o', 0, '2', 0, ' ', 0, 'G', 0,
  'N', 0, 'S', 0, 'S', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0, ' ', 0,
  'R', 0, 'e', 0, 'c', 0, 'e', 0, 'i', 0, 'v', 0, 'e', 0, 'r', 0
};
#elif defined( __STA8090__)
static const tU8 svc_usb_vcom_str_devdesc[] = {
  48,                               /* bLength.                             */
  LLD_USB_DESCRIPTOR_STRING,        /* bDescriptorType.                     */
  'T', 0, 'e', 0, 's', 0, 'e', 0, 'o', 0, '3', 0, ' ', 0, 'G', 0,
  'N', 0, 'S', 0, 'S', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0, ' ', 0,
  'R', 0, 'e', 0, 'c', 0, 'e', 0, 'i', 0, 'v', 0, 'e', 0, 'r', 0
};
#endif

/********************************************//**
 * \brief   Serial number string.
 ***********************************************/
static const tU8 svc_usb_vcom_str_sernum[] = {
  8,                                /* bLength.                             */
  LLD_USB_DESCRIPTOR_STRING,        /* bDescriptorType.                     */
  '1', 0,
  '2', 0,
  '3', 0
};

/********************************************//**
 * \brief   Strings wrappers array.
 ***********************************************/
static const LLD_USB_descriptor_t svc_usb_vcom_str_ids[] = {
  {sizeof(svc_usb_vcom_str_langid),    svc_usb_vcom_str_langid    },
  {sizeof(svc_usb_vcom_str_vendorid),  svc_usb_vcom_str_vendorid  },
  {sizeof(svc_usb_vcom_str_devdesc),   svc_usb_vcom_str_devdesc   },
  {sizeof(svc_usb_vcom_str_sernum),    svc_usb_vcom_str_sernum    }
};

/********************************************//**
 * \brief   EP1 initialization structure (IN and OUT).
 ***********************************************/
static const LLD_USB_endpoint_config_t ep1cfg = {
  LLD_EP_TYPE_BULK,
  NULL,
  svc_usb_vcom_data_txed,
  svc_usb_vcom_data_rxed,
  NULL,
  NULL,
  64,
  64,
  16,
  ep1setup_buffer
};

/********************************************//**
 * \brief   EP2 initialization structure (IN only).
 ***********************************************/
static const LLD_USB_endpoint_config_t ep2cfg = {
  LLD_EP_TYPE_INTR,
  NULL,
  svc_usb_vcom_irq_txed,
  NULL,
  NULL,
  NULL,
  64,
  0,
  1,
  ep2setup_buffer
};

#if defined( __STA8090__)
/********************************************//**
 * \brief   EP3 initialization structure (IN and OUT).
 ***********************************************/
static const LLD_USB_endpoint_config_t ep3cfg = {
  LLD_EP_TYPE_BULK,
  NULL,
  svc_usb_vcom_data_txed,
  svc_usb_vcom_data_rxed,
  NULL,
  NULL,
  64,
  64,
  16,
  ep3setup_buffer
};

/********************************************//**
 * \brief   EP4 initialization structure (IN only).
 ***********************************************/
static const LLD_USB_endpoint_config_t ep4cfg = {
  LLD_EP_TYPE_INTR,
  NULL,
  svc_usb_vcom_irq_txed,
  NULL,
  NULL,
  NULL,
  64,
  0,
  1,
  ep4setup_buffer
};
#endif

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

static gpOS_task_exit_status_t svc_usb_process( gpOS_task_param_t dummy)
{
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_id_to_port( 0);

  gpOS_semaphore_wait( hdlr_ptr->irq_sem);

  LLD_USB_ISR_callback( &hdlr_ptr->usb);

  LLD_USB_EnableGlobalInt();

  while( 1)
  {
    gpOS_semaphore_wait( hdlr_ptr->irq_sem);

    gpOS_mutex_lock( hdlr_ptr->access_mutex);

    LLD_USB_ISR_callback( &hdlr_ptr->usb);

    LLD_USB_EnableGlobalInt();

    gpOS_mutex_release( hdlr_ptr->access_mutex);
  }
}

/********************************************//**
 * \brief   Get handler pointer from USB driver
 *
 * \param   usbp    pointer to USB driver
 * \return  svc_usb_port_handler_t *  pointer to USB handler, or NULL if not open
 *
 ***********************************************/
static svc_usb_port_handler_t *svc_usb_usbp_to_port( LLD_USB_driver_t *usbp)
{
  svc_usb_port_handler_t *hdlr_ptr = NULL;

  if( svc_usb_handler != NULL)
  {
    hdlr_ptr = svc_usb_handler->port_head;

    while( hdlr_ptr != NULL)
    {
      if( &hdlr_ptr->usb == usbp)
      {
        break;
      }
      hdlr_ptr = hdlr_ptr->next;
    }
  }

  return hdlr_ptr;
}

/********************************************//**
 * \brief   Get USB handler for specific ID
 *
 * \param   usb_id   ID of wanted USB handler
 * \return  svc_usb_port_handler_t *  pointer to USB handler, or NULL if not open
 *
 ***********************************************/
static svc_usb_port_handler_t *svc_usb_id_to_port( tUInt usb_id)
{
  svc_usb_port_handler_t *hdlr_ptr;

  hdlr_ptr = svc_usb_handler->port_head;

  while( hdlr_ptr != NULL)
  {
    if( hdlr_ptr->id == usb_id)
    {
      return( hdlr_ptr);
    }
    else
    {
      hdlr_ptr = hdlr_ptr->next;
    }
  }

  return( NULL);
}

/********************************************//**
 * \brief   Reset USB svc_mcu and driver
 *
 * \param   void
 * \return  gpOS_error_t gpOS_SUCCESS if all is ok
 *
 ***********************************************/
static gpOS_error_t svc_usb_reset( svc_usb_port_handler_t *hdlr_ptr)
{
  gpOS_interrupt_lock();

  svc_usb_reset_port( hdlr_ptr);

  gpOS_interrupt_unlock();

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   USB event handler for VCOM callback
 *
 * \param   usb_id  ID of wanted USB handler
 * \param   event   Incoming event
 * \return  void
 *
 ***********************************************/
static void svc_usb_vcom_event( LLD_USB_driver_t *usbp, LLD_USB_event_t event)
{
  svc_usb_port_handler_t *port_hdlr = svc_usb_usbp_to_port( usbp);

  if (event == LLD_USB_EVENT_RESET)
  {
    svc_usb_reset( port_hdlr);
  }
  else if (event == LLD_USB_EVENT_ADDRESS)
  { }
  else if (event == LLD_USB_EVENT_CONFIGURED)
  {
    /* Opens the endpoints specified into the configuration.*/
    LLD_USB_InitEndpoint( usbp, 1 /*SVC_USB_VCOM_EP_DATA*/, &ep1cfg);
    LLD_USB_InitEndpoint( usbp, 2 /*SVC_USB_VCOM_EP_IRQ*/, &ep2cfg);
#if defined( __STA8090__)
    LLD_USB_InitEndpoint( usbp, 3 /*SVC_USB_VCOM_EP_IRQ*/, &ep3cfg);
    LLD_USB_InitEndpoint( usbp, 4 /*SVC_USB_VCOM_EP_IRQ*/, &ep4cfg);
#endif

    port_hdlr->init = TRUE;
    gpOS_semaphore_signal( port_hdlr->fifo_rx.init_sem);
  }
  else if (event == LLD_USB_EVENT_SUSPEND)
  { }
  else if (event == LLD_USB_EVENT_RESUME)
  { }
  else if (event == LLD_USB_EVENT_STALLED)
  { }

  return;
}

/********************************************//**
 * \brief   Handles the GET_DESCRIPTOR callback.
 *
 * \param   usbp    pointer to USB driver
 * \param   dtype   Type of descriptor
 * \param   dindex  Index of descriptor
 * \param   lang    Language of descriptor
 * \return  const LLD_USB_descriptor_t*
 *
 ***********************************************/
static const LLD_USB_descriptor_t *svc_usb_vcom_get_descriptor(LLD_USB_driver_t *usbp, tU8 dtype, tU8 dindex, tU16 lang)
{
  switch( dtype)
  {
    case LLD_USB_DESCRIPTOR_DEVICE:
      return &svc_usb_vcom_dd;

    case LLD_USB_DESCRIPTOR_CONFIGURATION:
      return &svc_usb_vcom_cd;

    case LLD_USB_DESCRIPTOR_STRING:
      if( dindex < 4)
      {
        return &svc_usb_vcom_str_ids[dindex];
      }
      break;
  }
  return NULL;
}

/********************************************//**
 * \brief   Data transmitted callback.
 *
 * \param usbp LLD_USB_driver_t*
 * \param ep LLD_USB_endpoint_t
 * \return void
 *
 ***********************************************/
static void svc_usb_vcom_data_txed( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep)
{
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_usbp_to_port( usbp);
  svc_usb_fifo_handler_t *fifo_ptr = &hdlr_ptr->fifo_tx;

  fifo_ptr->ep = ep;
  gpOS_semaphore_signal( fifo_ptr->fifo_sem);
}

/********************************************//**
 * \brief   Data available callback.
 *
 * \param usbp LLD_USB_driver_t*
 * \param ep LLD_USB_endpoint_t
 * \return void
 *
 ***********************************************/
static void svc_usb_vcom_data_rxed(LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep)
{
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_usbp_to_port( usbp);
  svc_usb_fifo_handler_t *fifo_ptr = &hdlr_ptr->fifo_rx;

  fifo_ptr->ep = ep;
  gpOS_semaphore_signal( fifo_ptr->fifo_sem);
}

/********************************************//**
 * \brief   Interrupt endpoint finished sending.
 *
 * \param usbp LLD_USB_driver_t*
 * \param ep LLD_USB_endpoint_t
 * \return void
 *
 * \note    Never triggered in this demo because nothing is sent over the
 *          interrupt endpoint.
 ***********************************************/
static void svc_usb_vcom_irq_txed(LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep)
{
}

/********************************************//**
 * \brief
 *
 * \param usbp LLD_USB_driver_t*
 * \return void
 *
 ***********************************************/
void svc_usb_vcom_ep0setup( LLD_USB_driver_t *usbp)
{
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_usbp_to_port( usbp);

  gpOS_semaphore_signal( hdlr_ptr->irq_sem);
}

/********************************************//**
 * \brief   CDC requests hook.
 *
 * \details CDC specific requests can be implemented into this driver hook.<br>
 *          The following requests are currently emulated:
 *          - CDC_GET_LINE_CODING.
 *          - CDC_SET_LINE_CODING.
 *          - CDC_SET_CONTROL_LINE_STATE.
 *
 * \param usbp LLD_USB_driver_t*
 * \return tBool
 *
 ***********************************************/
static tBool svc_usb_vcom_requests_hook( LLD_USB_driver_t *usbp)
{
  tBool response = FALSE;
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_usbp_to_port( usbp);

  if( ( usbp->setup[0] & LLD_USB_RTYPE_TYPE_MASK ) == LLD_USB_RTYPE_TYPE_CLASS )
  {
    switch( usbp->setup[1])
    {
      case CDC_GET_LINE_CODING:
        LLD_USB_SetupTransfer( usbp, ( tU8 * )&svc_usb_cdc_linecoding,
        sizeof( svc_usb_cdc_linecoding ) );
        response = TRUE;
        break;

      case CDC_SET_LINE_CODING:
        LLD_USB_SetupTransfer( usbp, ( tU8 * )&svc_usb_cdc_linecoding,
        sizeof( svc_usb_cdc_linecoding ) );
        response = TRUE;
        break;

      case CDC_SET_CONTROL_LINE_STATE:
        /* Nothing to do, there are no control lines.*/
        LLD_USB_SetupTransfer( usbp, NULL, 0 );

        if( usb_dte_configuration)
        {
          if( usbp->setup[2] & BIT_0)
            hdlr_ptr->active = TRUE;
          else
            hdlr_ptr->active = FALSE;
        }
        else
        {
          hdlr_ptr->active = TRUE;
        }
        response = TRUE;
        break;

      default:
        break;
    }
  }

  return response;
}

/********************************************//**
 * \brief Callback for USB svc_mcu to signal the receiver task
 *
 * \param hdlr_ptr  pointer to uart handler associated with
                    svc_mcud USB
 * \return void
 *
 ***********************************************/
static LLD_ISR_USB void svc_usb_callback( svc_usb_port_handler_t *hdlr_ptr)
{
  LLD_USB_DisableGlobalInt();
  gpOS_semaphore_signal( hdlr_ptr->irq_sem);
}

/********************************************//**
 * \brief Delete all OS allocated items
 *
 * \param hdlr_ptr  Pointer to handler to delete
 *
 ***********************************************/
static void svc_usb_com_create_fail( svc_usb_port_handler_t *hdlr_ptr)
{
  VicLineTy usb_phy_irq_line = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_USB, hdlr_ptr->id);

  gpOS_interrupt_uninstall( usb_phy_irq_line);
  gpOS_interrupt_disable( usb_phy_irq_line);

  gpOS_semaphore_delete( hdlr_ptr->fifo_rx.init_sem);
  gpOS_semaphore_delete( hdlr_ptr->fifo_rx.fifo_sem);
  gpOS_semaphore_delete( hdlr_ptr->fifo_rx.access_sem);

  gpOS_semaphore_delete( hdlr_ptr->fifo_tx.fifo_sem);
  gpOS_semaphore_delete( hdlr_ptr->fifo_tx.access_sem);

  gpOS_mutex_delete( hdlr_ptr->access_mutex);

  gpOS_task_delete( hdlr_ptr->usb_task);
  gpOS_semaphore_delete( hdlr_ptr->irq_sem);

  gpOS_memory_deallocate_p( svc_usb_handler->svc_mcu_item_handler.part, hdlr_ptr);
}

/********************************************//**
 * \brief   Reset USB port
 *
 * \param   hdlr_ptr svc_usb_port_handler_t*
 * \return  void
 *
 ***********************************************/
static void svc_usb_reset_port( svc_usb_port_handler_t *hdlr_ptr)
{
  hdlr_ptr->active = FALSE;

  rx_data_len = 0;
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initalize USB svc_mcu
 *
 * \param partition     Partition used for svc_mcu data
 * \param bus_speed     Bus speed
 * \return gpOS_error_t gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_usb_init( gpOS_partition_t *partition, tU32 bus_id)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_usb_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_usb_handler = gpOS_memory_allocate_p( partition, sizeof( svc_usb_handler_t));

  if( svc_usb_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_USB, bus_id, &svc_usb_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_usb_handler);
    return gpOS_FAILURE;
  }

  // Fill specific fields
  svc_usb_handler->svc_mcu_item_handler.part       = partition;
  svc_usb_handler->svc_mcu_item_handler.mem_used   = mem_at_start - gpOS_memory_getheapfree_p( partition);

  svc_usb_handler->com_head   = NULL;
  svc_usb_handler->port_head  = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Open a VCOM
 *
 * \param usb_id        uart port to open
 * \param irq_pri       uart ID
 * \param baud_rate     uart baud rate
 * \param fifo_tx_size  size of TX FIFO (0 if not needed)
 * \param fifo_rx_size  size of RX FIFO (0 if not needed)
 * \return gpOS_error_t gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_usb_open_vcom( tUInt usb_id, gpOS_interrupt_priority_t irq_pri, tU16 fifo_tx_size, tU16 fifo_rx_size, tU32 priority)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_usb_handler->svc_mcu_item_handler.part);
  svc_usb_port_handler_t *last_hdlr_ptr, *hdlr_ptr;

  VicLineTy     usb_phy_irq_line = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_USB, usb_id);

  if( svc_usb_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Check if port was already open */
  last_hdlr_ptr = hdlr_ptr = svc_usb_handler->port_head;

  while( hdlr_ptr != NULL)
  {
    if( hdlr_ptr->id == usb_id)
    {
      if( hdlr_ptr->open == TRUE)
      {
        return gpOS_FAILURE;
      }
      else
      {
        break;
      }
    }
    else
    {
      last_hdlr_ptr = hdlr_ptr;
      hdlr_ptr = hdlr_ptr->next;
    }
  }

  if( hdlr_ptr == NULL)
  {
    /**< Allocate needed memory */
    hdlr_ptr = gpOS_memory_allocate_p( svc_usb_handler->svc_mcu_item_handler.part, sizeof( svc_usb_port_handler_t));

    if( hdlr_ptr == NULL)
    {
      return gpOS_FAILURE;
    }

    hdlr_ptr->access_mutex = gpOS_mutex_create_p( MUTEX_FIFO, svc_usb_handler->svc_mcu_item_handler.part);

    if( hdlr_ptr->access_mutex == NULL)
    {
      svc_usb_com_create_fail( hdlr_ptr);
      return gpOS_FAILURE;
    }

    hdlr_ptr->usb_task = gpOS_task_create_p( svc_usb_handler->svc_mcu_item_handler.part, svc_usb_process, NULL, SVC_USB_TASK_STACK_SIZE,
                                        SVC_USB_TASK_PRIORITY, "SVC_USB_SRVE", gpOS_TASK_FLAGS_ACTIVE);

    if( hdlr_ptr->usb_task == NULL)
    {
      svc_usb_com_create_fail( hdlr_ptr);
      return gpOS_FAILURE;
    }

    hdlr_ptr->irq_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_usb_handler->svc_mcu_item_handler.part, 0);

    if( hdlr_ptr->irq_sem == NULL)
    {
      svc_usb_com_create_fail( hdlr_ptr);
      return gpOS_FAILURE;
    }

    /**< Initialize UART device and handler */
    hdlr_ptr->id      = usb_id;
    hdlr_ptr->next    = NULL;
    hdlr_ptr->active  = FALSE;
    hdlr_ptr->init    = FALSE;

    hdlr_ptr->fifo_tx.ep = hdlr_ptr->fifo_rx.ep = 0;

    /**< Initialize tx FIFO access and irq semaphores */
    hdlr_ptr->fifo_tx.fifo_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_usb_handler->svc_mcu_item_handler.part, 0);
    hdlr_ptr->fifo_tx.access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_usb_handler->svc_mcu_item_handler.part, 0);

    if( (hdlr_ptr->fifo_tx.access_sem == NULL) || (hdlr_ptr->fifo_tx.fifo_sem == NULL))
    {
      svc_usb_com_create_fail( hdlr_ptr);
      return gpOS_FAILURE;
    }

    /**< Initialize rx FIFO access and irq semaphores */
    hdlr_ptr->fifo_rx.init_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_usb_handler->svc_mcu_item_handler.part, 0);
    hdlr_ptr->fifo_rx.fifo_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_usb_handler->svc_mcu_item_handler.part, 0);
    hdlr_ptr->fifo_rx.access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_usb_handler->svc_mcu_item_handler.part, 0);

    if( (hdlr_ptr->fifo_rx.access_sem == NULL) || (hdlr_ptr->fifo_rx.fifo_sem == NULL))
    {
      svc_usb_com_create_fail( hdlr_ptr);
      return gpOS_FAILURE;
    }
    hdlr_ptr->usbcfg.event_cb           = svc_usb_vcom_event;
    hdlr_ptr->usbcfg.get_descriptor_cb  = svc_usb_vcom_get_descriptor;
    hdlr_ptr->usbcfg.requests_hook_cb   = svc_usb_vcom_requests_hook;
    hdlr_ptr->usbcfg.sof_cb             = NULL;
    hdlr_ptr->usbcfg.rxfifosize         = 1024;
    hdlr_ptr->usbcfg.txfifosize         = 1024;

    /**< Program UART TX FIFO registers */
    gpOS_semaphore_signal( hdlr_ptr->fifo_tx.access_sem);

    /**< Program UART RX FIFO registers */
    gpOS_semaphore_signal( hdlr_ptr->fifo_rx.access_sem);

    if( last_hdlr_ptr == NULL)
    {
      svc_usb_handler->port_head = hdlr_ptr;
    }
    else
    {
      last_hdlr_ptr->next = hdlr_ptr;
    }

    svc_usb_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_usb_handler->svc_mcu_item_handler.part);
  }

  svc_mcu_enable( SVC_MCU_PER_ID_USB, usb_id);

  LLD_USB_Init( &hdlr_ptr->usb, &hdlr_ptr->usbcfg);

  svc_usb_reset_port( hdlr_ptr);

  gpOS_interrupt_install( usb_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_usb_callback, hdlr_ptr);
  gpOS_interrupt_enable( usb_phy_irq_line);

  hdlr_ptr->open = TRUE;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Close a VCOM
 *
 * \param usb_id        uart port to open
 * \return gpOS_error_t gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_usb_close_vcom( tUInt usb_id)
{
  VicLineTy usb_phy_irq_line;
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_id_to_port( usb_id);

  if( hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  if( hdlr_ptr->open == FALSE)
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( hdlr_ptr->fifo_rx.access_sem);
  gpOS_semaphore_wait( hdlr_ptr->fifo_tx.access_sem);
  gpOS_mutex_lock( hdlr_ptr->access_mutex);

  svc_mcu_disable( SVC_MCU_PER_ID_USB, usb_id);

  usb_phy_irq_line = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_USB, hdlr_ptr->id);

  gpOS_interrupt_uninstall( usb_phy_irq_line);
  gpOS_interrupt_disable( usb_phy_irq_line);

  LLD_USB_Reset( &hdlr_ptr->usb);

  hdlr_ptr->open = FALSE;

  gpOS_mutex_release( hdlr_ptr->access_mutex);
  gpOS_semaphore_signal( hdlr_ptr->fifo_tx.access_sem);
  gpOS_semaphore_signal( hdlr_ptr->fifo_rx.access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Reads characters from a stream
 *
 * \param hdlr_ptr  handler of the stream
 * \param out_buf   buffer where to write chars
 * \param max_chars bytes to read
 * \param timeout   timeout
 * \return read chars
 *
 ***********************************************/
tU32 svc_usb_read( tUInt usb_id, tU8 *out_buf, tU32 max_chars, gpOS_clock_t *timeout)
{
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_id_to_port( usb_id);
  svc_usb_fifo_handler_t *fifo_ptr;
  tU32 read_chars = 0;

  if( (hdlr_ptr == NULL) || (max_chars == 0))
  {
    return 0;
  }

  if( hdlr_ptr->open == FALSE)
  {
    return 0;
  }

  fifo_ptr = &hdlr_ptr->fifo_rx;

  if( hdlr_ptr->init == FALSE)
  {
    gpOS_semaphore_wait( fifo_ptr->init_sem);
  }

  gpOS_semaphore_wait( fifo_ptr->access_sem);

  // Loop until all characters are read
  while( read_chars < max_chars)
  {
    // Case of RX FIFO empty
    if( rx_data_len == 0)
    {
      gpOS_mutex_lock( hdlr_ptr->access_mutex);

      // Start USB transfer
      LLD_USB_StartReceive( &hdlr_ptr->usb, SVC_USB_VCOM_EP_DATA, rx_data_ptr, SVC_USB_RX_PACKET_SIZE);

      gpOS_mutex_release( hdlr_ptr->access_mutex);

      if( gpOS_semaphore_wait_timeout( fifo_ptr->fifo_sem, timeout) == gpOS_FAILURE)
      {
        gpOS_semaphore_signal( fifo_ptr->access_sem);
        return read_chars;
      }

      gpOS_mutex_lock( hdlr_ptr->access_mutex);

      rx_data_index = 0;
      rx_data_len = hdlr_ptr->usb.ep[fifo_ptr->ep].rxcnt;

      gpOS_mutex_release( hdlr_ptr->access_mutex);
    }
    // Write data and update counters
    *out_buf++ = rx_data_ptr[rx_data_index];
    rx_data_index++;
    read_chars++;
    rx_data_len--;
  }
  gpOS_semaphore_signal( fifo_ptr->access_sem);

  return read_chars;
}

/********************************************//**
 * \brief Writes characters from a stream

 *
 * \param hdlr_ptr  handler of the stream
 * \param in_buf    buffer from where to read chars
 * \param max_chars bytes to write
 * \param timeout   timeout
 * \return remaining chars
 *
 ***********************************************/
tU32 svc_usb_write( tUInt usb_id, tU8 *in_buf, tU32 max_chars, gpOS_clock_t *timeout)
{
  svc_usb_port_handler_t *hdlr_ptr = svc_usb_id_to_port( usb_id);
  svc_usb_fifo_handler_t *fifo_ptr;
  tU32 written_chars = 0;

  if(( hdlr_ptr == NULL) || ( max_chars == 0))
  {
    return 0;
  }

  if( hdlr_ptr->open == FALSE)
  {
    return 0;
  }

  fifo_ptr = &hdlr_ptr->fifo_tx;

  if( hdlr_ptr->active == TRUE)
  {
    gpOS_semaphore_wait( fifo_ptr->access_sem);

    gpOS_mutex_lock( hdlr_ptr->access_mutex);

    // Start USB transfer
    if( LLD_USB_StartTransmit( &hdlr_ptr->usb, SVC_USB_VCOM_EP_DATA, in_buf, max_chars))
    {
      gpOS_mutex_release( hdlr_ptr->access_mutex);
      gpOS_semaphore_signal( fifo_ptr->access_sem);
      return 0;
    }

    gpOS_mutex_release( hdlr_ptr->access_mutex);

    gpOS_semaphore_wait( fifo_ptr->fifo_sem);

    gpOS_mutex_lock( hdlr_ptr->access_mutex);

    written_chars = hdlr_ptr->usb.ep[fifo_ptr->ep].txcnt;

    gpOS_mutex_release( hdlr_ptr->access_mutex);

    gpOS_semaphore_signal( fifo_ptr->access_sem);
  }

  return written_chars;
}

/* End of file */
