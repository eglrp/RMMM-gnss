/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : lld_usb.h
* Author             : STMicroelectronics, Napoli
* Version            : V0.0.0
* Date               : 11/24/2010
* Description        : USB OTG IP hardware registers.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file    lld_usb.h
 * @brief   Synopsis OTG Low Level Driver definitions and macros.
 *
 * @addtogroup LLD
 * @{
 * @addtogroup USB
 * @{
 */

#ifndef LLD_USB_H
#define LLD_USB_H

#include "lld.h"

/*--------------------------------------------------------------------------*
 * Definitions.                                                             *
 *--------------------------------------------------------------------------*/

#define LLD_USB_RTYPE_DIR_MASK                  0x80U
#define LLD_USB_RTYPE_DIR_HOST2DEV              0x00U
#define LLD_USB_RTYPE_DIR_DEV2HOST              0x80U
#define LLD_USB_RTYPE_TYPE_MASK                 0x60U
#define LLD_USB_RTYPE_TYPE_STD                  0x00U
#define LLD_USB_RTYPE_TYPE_CLASS                0x20U
#define LLD_USB_RTYPE_TYPE_VENDOR               0x40U
#define LLD_USB_RTYPE_TYPE_RESERVED             0x60U
#define LLD_USB_RTYPE_RECIPIENT_MASK            0x1FU
#define LLD_USB_RTYPE_RECIPIENT_DEVICE          0x00U
#define LLD_USB_RTYPE_RECIPIENT_INTERFACE       0x01U
#define LLD_USB_RTYPE_RECIPIENT_ENDPOINT        0x02U
#define LLD_USB_RTYPE_RECIPIENT_OTHER           0x03U

#define LLD_USB_REQ_GET_STATUS                  0
#define LLD_USB_REQ_CLEAR_FEATURE               1
#define LLD_USB_REQ_SET_FEATURE                 3
#define LLD_USB_REQ_SET_ADDRESS                 5
#define LLD_USB_REQ_GET_DESCRIPTOR              6
#define LLD_USB_REQ_SET_DESCRIPTOR              7
#define LLD_USB_REQ_GET_CONFIGURATION           8
#define LLD_USB_REQ_SET_CONFIGURATION           9
#define LLD_USB_REQ_GET_INTERFACE               10
#define LLD_USB_REQ_SET_INTERFACE               11
#define LLD_USB_REQ_SYNCH_FRAME                 12

#define LLD_USB_DESCRIPTOR_DEVICE               1
#define LLD_USB_DESCRIPTOR_CONFIGURATION        2
#define LLD_USB_DESCRIPTOR_STRING               3
#define LLD_USB_DESCRIPTOR_INTERFACE            4
#define LLD_USB_DESCRIPTOR_ENDPOINT             5
#define LLD_USB_DESCRIPTOR_DEVICE_QUALIFIER     6
#define LLD_USB_DESCRIPTOR_OTHER_SPEED_CFG      7
#define LLD_USB_DESCRIPTOR_INTERFACE_POWER      8

#define LLD_USB_FEATURE_ENDPOINT_HALT           0
#define LLD_USB_FEATURE_DEVICE_REMOTE_WAKEUP    1
#define LLD_USB_FEATURE_TEST_MODE               2

/**
 * @brief   Number of the implemented endpoints.
 * @details This value does not include the endpoint 0 that is always present.
 */
#if defined( __STA8090__)
#define LLD_USB_ENDOPOINTS_NUMBER 4
#else
#define LLD_USB_ENDOPOINTS_NUMBER 2
#endif

/*--------------------------------------------------------------------------*
 * Types and structures.                                                    *
 *--------------------------------------------------------------------------*/

/**
 * @brief   Type of a structure representing an USB driver.
 */
typedef struct LLD_USB_driver LLD_USB_driver_t;

/**
 * @brief   Type of an endpoint identifier.
 */
typedef tU8 LLD_USB_endpoint_t;

/**
 * @brief   Type of a driver state machine possible states.
 */
typedef enum {
  LLD_USB_UNINIT   = 0,                     /**< Not initialized.               */
  LLD_USB_READY    = 1,                     /**< Ready, after bus reset.        */
  LLD_USB_SELECTED = 2,                     /**< Address assigned.              */
  LLD_USB_ACTIVE   = 3,                     /**< Active, configuration selected.*/
} LLD_USB_state_t;

/**
 * @brief   Type of an endpoint type.
 */
typedef enum {
  LLD_EP_TYPE_CTRL = 0,                     /**< Control endpoint.              */
  LLD_EP_TYPE_ISOC = 1,                     /**< Isochronous endpoint.          */
  LLD_EP_TYPE_BULK = 2,                     /**< Bulk endpoint.                 */
  LLD_EP_TYPE_INTR = 3                      /**< Interrupt endpoint.            */
} LLD_USB_endpoint_type_t;

/**
 * @brief   Type of an endpoint status.
 */
typedef enum {
  LLD_EP_STATUS_DISABLED = 0,               /**< Endpoint not opened.           */
  LLD_EP_STATUS_STALLED = 1,                /**< Endpoint opened but stalled.   */
  LLD_EP_STATUS_ACTIVE = 2                  /**< Active endpoint.               */
} LLD_USB_endpoint_status_t;

/**
 * @brief   Type of an endpoint zero state machine states.
 */
typedef enum {
  LLD_USB_EP0_WAITING_SETUP,                /**< Waiting for SETUP data.        */
  LLD_USB_EP0_TX,                           /**< Trasmitting.                   */
  LLD_USB_EP0_WAITING_TX0,                  /**< Waiting transmit 0.            */
  LLD_USB_EP0_WAITING_STS,                  /**< Waiting status.                */
  LLD_USB_EP0_RX,                           /**< Receiving.                     */
  LLD_USB_EP0_SENDING_STS,                  /**< Sending status.                */
  LLD_USB_EP0_ERROR                         /**< Error, EP0 stalled.            */
} LLD_USB_ep0_state_t;

/**
 * @brief   Type of an enumeration of the possible USB events.
 */
typedef enum {
  LLD_USB_EVENT_RESET = 0,                  /**< Driver has been reset by host. */
  LLD_USB_EVENT_ADDRESS = 1,                /**< Address assigned.              */
  LLD_USB_EVENT_CONFIGURED = 2,             /**< Configuration selected.        */
  LLD_USB_EVENT_SUSPEND = 3,                /**< Entering suspend mode.         */
  LLD_USB_EVENT_RESUME = 4,                 /**< Leaving suspend mode.          */
  LLD_USB_EVENT_STALLED = 5,                /**< Endpoint 0 error, stalled.     */
} LLD_USB_event_t;

/**
 * @brief   Type of an USB descriptor.
 */
typedef struct {
  /**
   * @brief   Descriptor size in unicode characters.
   */
  tU32                          ud_size;
  /**
   * @brief   Pointer to the descriptor.
   */
  const tU8                     *ud_string;
} LLD_USB_descriptor_t;

/**
 * @brief   Type of an USB generic notification callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 */
typedef void (*LLD_USB_generic_cb_t)(LLD_USB_driver_t *usbp);

/**
 * @brief   Type of an USB endpoint callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @param[in] ep        endpoint number
 */
typedef void (*LLD_USB_endpoint_cb_t)(LLD_USB_driver_t *usbp,
                                      LLD_USB_endpoint_t ep);

/**
 * @brief   Type of an USB event notification callback.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @param[in] event     event type
 */
typedef void (*LLD_USB_event_cb_t)(LLD_USB_driver_t *usbp,
                                   LLD_USB_event_t event);

/**
 * @brief   Type of an USB descriptor-retrieving callback.
 */
typedef const LLD_USB_descriptor_t * (*LLD_USB_get_descriptor_cb_t)(
    LLD_USB_driver_t *usbp, tU8 dtype, tU8 dindex, tU16 lang);

/**
 * @brief   Type of a requests handler callback.
 * @details The request is encoded in the @p usb_setup buffer.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @return              The request handling exit code.
 * @retval false        Request not recognized by the handler.
 * @retval true         Request handled.
 */
typedef tBool (*LLD_USB_request_handler_cb_t)(LLD_USB_driver_t *usbp);

/**
 * @brief   Type of an USB Endpoint configuration structure.
 * @note    Platform specific restrictions may apply to endpoints.
 */
typedef struct {
  /**
   * @brief   Endpoint type.
   */
  LLD_USB_endpoint_type_t       eptype;
  /**
   * @brief   Setup packet notification callback.
   */
  LLD_USB_endpoint_cb_t         setup_cb;
  /**
   * @brief   IN endpoint notification callback.
   */
  LLD_USB_endpoint_cb_t         in_cb;
  /**
   * @brief   OUT endpoint notification callback.
   */
  LLD_USB_endpoint_cb_t         out_cb;
  /**
   * @brief   IN endpoint notification callback.
   */
  LLD_USB_endpoint_cb_t         txfifo_cb;
  /**
   * @brief   OUT endpoint notification callback.
   */
  LLD_USB_endpoint_cb_t         rxfifo_cb;
  /**
   * @brief   IN endpoint maximum packet size.
   */
  tU16                          in_maxsize;
  /**
   * @brief   OUT endpoint maximum packet size.
   */
  tU16                          out_maxsize;
  /**
   * @brief   Determines the space allocated for the TXFIFO as multiples of
   *          the packet size (@p in_maxsize). Note that zero is interpreted
   *          as one for simplicity and robustness.
   */
  tU16                          in_multiplier;
  /**
   * @brief   Pointer to a buffer for setup packets.
   */
  tU8                           *setup_buf;
  /* End of the mandatory fields.*/
} LLD_USB_endpoint_config_t;

/**
 * @brief   Type of an USB driver configuration structure.
 */
typedef struct {
  /**
   * @brief   USB events callback.
   * @details This callback is invoked when the USB driver changes state
   *          because an external event.
   */
  LLD_USB_event_cb_t            event_cb;
  /**
   * @brief   Device GET_DESCRIPTOR request callback.
   * @note    This callback is mandatory and cannot be set to @p NULL.
   */
  LLD_USB_get_descriptor_cb_t   get_descriptor_cb;
  /**
   * @brief   Requests hook callback.
   * @details This hook allows to be notified of standard requests or to
   *          handle non standard requests.
   */
  LLD_USB_request_handler_cb_t  requests_hook_cb;
  /**
   * @brief   Start Of Frame callback, @p NULL if not required.
   */
  LLD_USB_generic_cb_t          sof_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief   Size of the shared RX FIFO in double words.
   * @details This value is the size of the receive FIFO, please refer to
   *          the reference manual in order to define the optmimal value.
   */
  tU32                          rxfifosize;
  /**
   * @brief   Size of the shared TX FIFO in double words.
   * @details This value is the size of the transim FIFO, please refer to
   *          the reference manual in order to define the optmimal value.
   */
  tU32                          txfifosize;
} LLD_USB_config_t;

/**
 * @brief   Type of an endpoint state structure.
 */
typedef struct {
  /**
   * @brief   Configuration associated to the endpoint.
   */
  const LLD_USB_endpoint_config_t *config;
  /**
   * @brief   Pointer to the transmission buffer.
   */
  const tU8                     *txbuf;
  /**
   * @brief   Pointer to the receive buffer.
   */
  tU8                           *rxbuf;
  /**
   * @brief   Requested transmit transfer size.
   */
  tU32                          txsize;
  /**
   * @brief   Requested receive transfer size.
    */
  tU32                          rxsize;
  /**
   * @brief   Transmitted bytes so far.
   */
  tU32                          txcnt;
  /**
   * @brief   Received bytes so far.
    */
  tU32                          rxcnt;
  /**
   * @brief   @p TRUE if transmitting else @p FALSE.
   */
  tU8                           transmitting;
  /**
   * @brief   @p TRUE if receiving else @p FALSE.
   */
  tU8                           receiving;
  /**
   * @brief   @p TRUE if timeout else @p FALSE.
   */
  tU8                           timeout;
} LLD_USB_EP_state_t;

/**
 * @brief   Structure representing an USB driver.
 */
struct LLD_USB_driver {
  /**
   * @brief   Driver state.
   */
  LLD_USB_state_t               state;
  /**
   * @brief   Current configuration data.
   */
  const LLD_USB_config_t        *config;
  /**
   * @brief   Field available to user, it can be used to associate an
   *          application-defined handler to the USB driver.
   */
  void                          *param;
  /**
   * @brief   Active endpoints configurations.
   */
//  const LLD_USB_endpoint_config_t *epc[LLD_USB_ENDOPOINTS_NUMBER + 1];
  LLD_USB_EP_state_t            ep[LLD_USB_ENDOPOINTS_NUMBER + 1];
  /**
   * @brief   Endpoint 0 state.
   */
  LLD_USB_ep0_state_t           ep0state;
  /**
   * @brief   Next position in the buffer to be transferred through endpoint 0.
   */
  tU8                           *ep0next;
  /**
   * @brief   Number of bytes yet to be tranferred through endpoint 0.
   */
  tU16                          ep0n;
  /**
   * @brief   Setup packet buffer.
   */
  tU8                           setup[8];
  /**
   * @brief   Current USB device status.
   */
  tU8                           status;
  /**
   * @brief   Assigned USB address.
   */
  tU8                           address;
  /**
   * @brief   Current USB device configuration.
   */
  tU8                           configuration;
  /* End of the mandatory fields.*/
  /**
   * @brief   Currently transmitting endpoint or 0xFF.
   * @details This field is required in order to make the currently
   *          transmitting endpoint finish its transmission before other
   *          endpoint are allowed to transmit. This is required because
   *          Teseo II uses a single transmit FIFO.
   */
  LLD_USB_endpoint_t            tx_current;
  /**
   * @brief   Current USB device configuration.
   */
  tU32                          pmnext;
};

/*--------------------------------------------------------------------------*
 * Macros.                                                                  *
 *--------------------------------------------------------------------------*/

/**
 * @brief   Fetches a 16 bits word value from an USB message.
 *
 * @param[in] p             Pointer to the 16 bits word.
 *
 * @notapi
 */
#define LLD_USB_FetchWord(p) (*(tU16 *)p)

/**
 * @brief   Request transfer setup.
 * @details This macro is used by the request handling callbacks in order to
 *          prepare a transaction over the endpoint zero.
 *
 * @param[in] usbp          Pointer to an @p LLD_USB_driver_t object.
 * @param[in] buf           Pointer to a buffer for the transaction data.
 * @param[in] n             Number of bytes to be transferred.
 *
 * @api
 */
#define LLD_USB_SetupTransfer(usbp, buf, n) {                               \
  (usbp)->ep0next  = (buf);                                                 \
  (usbp)->ep0n     = (n);                                                   \
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp          Pointer to an @p LLD_USB_driver_t object.
 * @param[in] ep            IN endpoint number.
 * @return                  The operation status.
 * @retval FALSE            Endpoint not transmiting.
 * @retval TRUE             Endpoint already transmiting.
 *
 * @api
 */
#define LLD_USB_GetTransmitStatus(usbp, ep_idx) (usbp)->ep[ep_idx].transmitting

/**
 * @brief   Returns the transmitted data size.
 *
 * @param[in] usbp          Pointer to an @p LLD_USB_driver_t object.
 * @param[in] ep            IN endpoint number.
 * @return                  The number of bytes in the current buffer.
 * @retval 0                Zero sized packet received.
 *
 * @api
 */
#define LLD_USB_GetTransmittedSize(usbp, ep) (usbp)->ep[ep].txcnt

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp          Pointer to an @p LLD_USB_driver_t object.
 * @param[in] ep            OUT endpoint number.
 * @return                  The operation status.
 * @retval FALSE            Endpoint not receiving.
 * @retval TRUE             Endpoint already receiving.
 *
 * @api
 */
#define LLD_USB_GetReceiveStatus(usbp, ep_idx) (usbp)->ep[ep_idx].receiving

/**
 * @brief   Returns the received data size.
 *
 * @param[in] usbp          Pointer to an @p LLD_USB_driver_t object.
 * @param[in] ep            OUT endpoint number.
 * @return                  The number of bytes in the current buffer.
 * @retval 0                Zero sized packet received.
 *
 * @api
 */
#define LLD_USB_GetReceivedSize(usbp, ep) (usbp)->ep[ep].rxcnt

#ifdef __cplusplus
extern "C" {
#endif
extern void   LLD_USB_ISR_callback      ( void *p);
extern void   LLD_USB_Init              ( LLD_USB_driver_t *usbp, const LLD_USB_config_t *config);
extern void   LLD_USB_Deinit            ( LLD_USB_driver_t *usbp);
extern void   LLD_USB_Reset             ( LLD_USB_driver_t *usbp);
extern void   LLD_USB_InitEndpoint      ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep, const LLD_USB_endpoint_config_t *epconfig);
extern tBool  LLD_USB_StartReceive      ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep, tU8 *buf, tU32 n);
extern tBool  LLD_USB_StartTransmit     ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep, const tU8 *buf, tU32 n);
extern tBool  LLD_USB_StallReceive      ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep);
extern tBool  LLD_USB_StallTransmit     ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep);
extern void   LLD_USB_WriteTransmitFIFO ( LLD_USB_driver_t *usbp, LLD_USB_endpoint_t ep);
extern void   LLD_USB_EnableGlobalInt   ( void);
extern void   LLD_USB_DisableGlobalInt  ( void);
#ifdef __cplusplus
}
#endif

#endif /* LLD_USB_H */

/** @} */

/** @} */
