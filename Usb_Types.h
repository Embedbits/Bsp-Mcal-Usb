/**
 * \defgroup Usb Usb
 * \brief Usb module
 */

/**
 * \author Mr.Nobody
 * \file Usb_Types.h
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 */

#ifndef USB_USB_TYPES_H
#define USB_USB_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Stm32_usb.h"                      /* USB RAL functionality          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define USB_NULL_PTR                        ( ( void* ) 0u )

/** Endpoint address bit selecting the IN direction (device to host) */
#define USB_EP_ADDR_DIR_IN                  ( 0x80u )

/** Endpoint address bits holding the endpoint number */
#define USB_EP_ADDR_NUM_MASK                ( 0x0Fu )

/** Length of the SETUP packet in bytes */
#define USB_SETUP_PACKET_LEN                ( 8u )

/** Maximal device address assigned by the host */
#define USB_DEV_ADDR_MAX                    ( 127u )

/** Peripheral identification bit offset in encoded value */
#define USB_BIT_MASK_PERIPH_BIT_OFFSET      ( 15u )

/** Port identification bit offset in encoded value */
#define USB_BIT_MASK_PORT_BIT_OFFSET        ( 10u )

/** Pin identification bit offset in encoded value */
#define USB_BIT_MASK_PIN_BIT_OFFSET         ( 5u )

/** Alternative function identification bit offset in encoded value */
#define USB_BIT_MASK_AF_BIT_OFFSET          ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/** Endpoint address of the OUT (host to device) endpoint with given number */
#define USB_EP_ADDR_OUT(NUM)                ( (usb_EpAddr_t)( (NUM) & USB_EP_ADDR_NUM_MASK ) )

/** Endpoint address of the IN (device to host) endpoint with given number */
#define USB_EP_ADDR_IN(NUM)                 ( (usb_EpAddr_t)( USB_EP_ADDR_DIR_IN | ( (NUM) & USB_EP_ADDR_NUM_MASK ) ) )

/** Endpoint number extracted from the endpoint address */
#define USB_EP_ADDR_NUMBER(ADDR)            ( (ADDR) & USB_EP_ADDR_NUM_MASK )

/** Non-zero if the endpoint address selects the IN direction */
#define USB_EP_ADDR_IS_IN(ADDR)             ( 0u != ( (ADDR) & USB_EP_ADDR_DIR_IN ) )

/** Encode pin configuration into single bit-mask */
#define USB_PIN_BIT_MASK_ENCODE(PERIPH_ID,PORT_ID,PIN_ID,AF_ID)   ( ( PERIPH_ID  << USB_BIT_MASK_PERIPH_BIT_OFFSET  ) | \
                                                                    ( PORT_ID    << USB_BIT_MASK_PORT_BIT_OFFSET    ) | \
                                                                    ( PIN_ID     << USB_BIT_MASK_PIN_BIT_OFFSET     ) | \
                                                                    ( AF_ID      << USB_BIT_MASK_AF_BIT_OFFSET      )   )

/** Extract peripheral ID from encoded value */
#define USB_BIT_MASK_DECODE_PERIPH(CODED_VAL)                     ( ( CODED_VAL >> USB_BIT_MASK_PERIPH_BIT_OFFSET ) & 0x1F )

/** Extract port ID from encoded value */
#define USB_BIT_MASK_DECODE_PORT(CODED_VAL)                       ( ( CODED_VAL >> USB_BIT_MASK_PORT_BIT_OFFSET ) & 0x1F )

/** Extract pin ID from encoded value */
#define USB_BIT_MASK_DECODE_PIN(CODED_VAL)                        ( ( CODED_VAL >> USB_BIT_MASK_PIN_BIT_OFFSET ) & 0x1F )

/** Extract alternative function ID from encoded value */
#define USB_BIT_MASK_DECODE_AF(CODED_VAL)                         ( ( CODED_VAL >> USB_BIT_MASK_AF_BIT_OFFSET ) & 0x1F )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t usb_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t usb_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t usb_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    usb_MajorVersion_t Major; /**< Major version */
    usb_MinorVersion_t Minor; /**< Minor version */
    usb_PatchVersion_t Patch; /**< Patch version */
}   usb_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    USB_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    USB_FUNCTION_ACTIVE         /**< Function status is active   */
}   usb_FunctionState_t;


/** Flag states enumeration */
typedef enum
{
    USB_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    USB_FLAG_ACTIVE         /**< Active flag state   */
}   usb_FlagState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    USB_REQUEST_ERROR = 0u, /**< Processing request failed  */
    USB_REQUEST_OK          /**< Processing request succeed */
}   usb_RequestState_t;


/** Endpoint address as used in USB descriptors: bit 7 = direction (1 = IN), bits 3:0 = endpoint number */
typedef uint8_t usb_EpAddr_t;

/** Maximal packet size of an endpoint in bytes */
typedef uint16_t usb_EpMaxPacket_t;

/** Device address assigned by the host (0 = default address) */
typedef uint8_t usb_DevAddr_t;

/** Frame number (11 bits, incremented every 1 ms) */
typedef uint16_t usb_FrameNumber_t;

/** Type representing a data byte of a transfer */
typedef uint8_t usb_Data_t;

/** Type representing count of transferred data bytes */
typedef uint32_t usb_DataCnt_t;

/** Interrupt priority type definition */
typedef uint32_t usb_IrqPrio_t;

/** SETUP packet field bmRequestType */
typedef uint8_t usb_ReqType_t;

/** SETUP packet field bRequest */
typedef uint8_t usb_ReqId_t;

/** SETUP packet field wValue */
typedef uint16_t usb_ReqValue_t;

/** SETUP packet field wIndex */
typedef uint16_t usb_ReqIndex_t;

/** SETUP packet field wLength */
typedef uint16_t usb_ReqLength_t;

/** Host channel identification (index of the channel register of the controller) */
typedef uint8_t usb_ChannelId_t;


/** \brief USB controller identification */
typedef enum
{
    USB_PERIPH_FS = 0u, /**< Full-speed controller (USB_DRD_FS) */
    USB_PERIPH_CNT      /**< Count of USB controllers           */
}   usb_PeriphId_t;


/** \brief USB operation mode */
typedef enum
{
    USB_MODE_DEVICE = 0u, /**< Device (function) mode                       */
    USB_MODE_HOST,        /**< Host mode (control, bulk and interrupt channels, one downstream port) */
    USB_MODE_CNT          /**< Count of operation modes                     */
}   usb_Mode_t;


/** \brief USB bus speed */
typedef enum
{
    USB_SPEED_LOW = 0u, /**< Low speed (1.5 Mbit/s)   */
    USB_SPEED_FULL,     /**< Full speed (12 Mbit/s)   */
    USB_SPEED_HIGH,     /**< High speed (480 Mbit/s)  */
    USB_SPEED_CNT       /**< Count of bus speeds      */
}   usb_Speed_t;


/**
 * \brief Source of the 48 MHz USB kernel clock
 *
 * The selected clock source has to provide 48 MHz (HSI48 or a PLL output configured by the RCC
 * module) - the module checks the frequency during initialization.
 */
typedef enum
{
    USB_CLK_SOURCE_HSI48 = 0u, /**< 48 MHz internal oscillator (HSI48) */
    USB_CLK_SOURCE_PLL1Q,      /**< PLL1 output Q                      */
#if defined(RCC_CR_PLL3ON)
    USB_CLK_SOURCE_PLL3Q,      /**< PLL3 output Q                      */
#else
    USB_CLK_SOURCE_PLL2Q,      /**< PLL2 output Q                      */
#endif
    USB_CLK_SOURCE_CNT         /**< Count of clock sources             */
}   usb_ClkSource_t;


/**
 * \brief Method of the bus event and endpoint transfer servicing
 *
 * Callbacks are called from the context servicing the controller.
 */
typedef enum
{
    USB_SERVICE_MODE_ISR = 0u, /**< USB interrupt service routine                           */
    USB_SERVICE_MODE_POLL,     /**< Usb_Task() polling the controller flags (no interrupt)  */
    USB_SERVICE_MODE_CNT       /**< Count of servicing methods                              */
}   usb_ServiceMode_t;


/** \brief USB device state (states of USB 2.0 specification chapter 9.1) */
typedef enum
{
    USB_DEVICE_STATE_DISCONNECTED = 0u, /**< Pull-up disconnected - the host does not see the device */
    USB_DEVICE_STATE_CONNECTED,         /**< Pull-up connected, waiting for the bus reset (powered)  */
    USB_DEVICE_STATE_DEFAULT,           /**< Bus reset received, default address 0                   */
    USB_DEVICE_STATE_ADDRESSED,         /**< Address assigned by the host                            */
    USB_DEVICE_STATE_SUSPENDED,         /**< Bus suspended (no activity for 3 ms)                    */
    USB_DEVICE_STATE_CNT                /**< Count of device states                                  */
}   usb_DeviceState_t;


/** \brief Endpoint transfer type */
typedef enum
{
    USB_EP_TYPE_CONTROL = 0u, /**< Control endpoint                           */
    USB_EP_TYPE_ISOCHRONOUS,  /**< Isochronous endpoint (not implemented yet) */
    USB_EP_TYPE_BULK,         /**< Bulk endpoint                              */
    USB_EP_TYPE_INTERRUPT,    /**< Interrupt endpoint                         */
    USB_EP_TYPE_CNT           /**< Count of endpoint types                    */
}   usb_EpType_t;


/** \brief Endpoint packet buffering */
typedef enum
{
    USB_EP_BUFFER_SINGLE = 0u, /**< One packet buffer                                               */
    USB_EP_BUFFER_DOUBLE,      /**< Two packet buffers - bulk / isochronous (not implemented yet)   */
    USB_EP_BUFFER_CNT          /**< Count of buffering options                                      */
}   usb_EpBufferMode_t;


/** \brief Endpoint state */
typedef enum
{
    USB_EP_STATE_CLOSED = 0u, /**< Endpoint is not initialized                           */
    USB_EP_STATE_IDLE,        /**< Endpoint is initialized, no transfer is running (NAK) */
    USB_EP_STATE_BUSY,        /**< Transfer is running                                   */
    USB_EP_STATE_STALLED,     /**< Endpoint answers STALL                                */
    USB_EP_STATE_CNT          /**< Count of endpoint states                              */
}   usb_EpState_t;


/** \brief Result of a data transfer reported by \ref usb_XferCallback_t */
typedef enum
{
    USB_XFER_RESULT_OK = 0u,   /**< Transfer finished (requested size transferred or short packet)                   */
    USB_XFER_RESULT_ABORTED,   /**< Transfer was aborted (SETUP packet, bus reset, stall, endpoint closed)           */
    USB_XFER_RESULT_OVERFLOW,  /**< Received packet did not fit the buffer, data over the buffer size were dropped   */
    USB_XFER_RESULT_CNT        /**< Count of transfer results                                                        */
}   usb_XferResult_t;


/** \brief State of the downstream port in the host mode */
typedef enum
{
    USB_HOST_PORT_DISCONNECTED = 0u, /**< No device attached                                                          */
    USB_HOST_PORT_CONNECTED,         /**< Device attached, the bus was not reset yet (the device answers address 0)  */
    USB_HOST_PORT_RESET,             /**< Reset signaling is driven, or released and the first start of frame awaited */
    USB_HOST_PORT_ENABLED,           /**< Bus reset done, start of frame generated - transfers are possible            */
    USB_HOST_PORT_CNT                /**< Count of port states                                                         */
}   usb_HostPortState_t;


/** \brief Token (type of transaction) started by the host on a channel */
typedef enum
{
    USB_TOKEN_SETUP = 0u, /**< SETUP transaction of a control transfer (8 bytes, DATA0) */
    USB_TOKEN_OUT,        /**< OUT transaction (data from the host to the device)        */
    USB_TOKEN_IN,         /**< IN transaction (data from the device to the host)         */
    USB_TOKEN_CNT         /**< Count of tokens                                           */
}   usb_Token_t;


/** \brief Data toggle (packet identifier DATA0 / DATA1) of a channel */
typedef enum
{
    USB_DATA_TOGGLE_DATA0 = 0u, /**< Next data packet has the identifier DATA0 */
    USB_DATA_TOGGLE_DATA1,      /**< Next data packet has the identifier DATA1 */
    USB_DATA_TOGGLE_CNT         /**< Count of data toggle values               */
}   usb_DataToggle_t;


/** \brief State of a host channel */
typedef enum
{
    USB_CHANNEL_STATE_CLOSED = 0u, /**< Channel is not initialized                   */
    USB_CHANNEL_STATE_IDLE,        /**< Channel is initialized, no transfer is running */
    USB_CHANNEL_STATE_BUSY,        /**< Transfer is running                          */
    USB_CHANNEL_STATE_CNT          /**< Count of channel states                      */
}   usb_ChannelState_t;


/** \brief Result of a host channel transfer reported by \ref usb_ChannelCallback_t */
typedef enum
{
    USB_CHANNEL_RESULT_OK = 0u, /**< Transfer finished (requested size transferred or short packet)                            */
    USB_CHANNEL_RESULT_NAK,     /**< Interrupt channel: the device answered NAK (no data) - the channel is idle, start it again */
    USB_CHANNEL_RESULT_STALL,   /**< The device answered STALL - the channel is idle (the caller clears the halt of the endpoint) */
    USB_CHANNEL_RESULT_ERROR,   /**< Three consecutive transaction errors (no answer, CRC, bit stuffing) - the channel is idle   */
    USB_CHANNEL_RESULT_ABORTED, /**< Device was disconnected (the transfer stopped by the caller is not reported)               */
    USB_CHANNEL_RESULT_CNT      /**< Count of channel transfer results                                                          */
}   usb_ChannelResult_t;


/** \brief Controller errors reported by \ref usb_ErrCallback_t */
typedef enum
{
    USB_ERROR_BUS = 0u,        /**< Bus error (no response, CRC, bit stuffing, unexpected packet)  */
    USB_ERROR_PMA_OVERRUN,     /**< Packet memory overrun / underrun                               */
    USB_ERROR_SETUP_LENGTH,    /**< SETUP packet with invalid length received - endpoint 0 stalled */
    USB_ERROR_CNT              /**< Count of controller errors                                     */
}   usb_ErrorId_t;


/** \brief Endpoint configuration */
typedef struct
{
    usb_EpAddr_t       Address;       /**< Endpoint address (direction and number, \ref USB_EP_ADDR_IN, \ref USB_EP_ADDR_OUT) */
    usb_EpType_t       Type;          /**< Endpoint transfer type (both directions of one endpoint number use the same type)  */
    usb_EpMaxPacket_t  MaxPacketSize; /**< Maximal packet size in bytes (control / bulk / interrupt: 8, 16, 32, 64)           */
    usb_EpBufferMode_t BufferMode;    /**< Packet buffering                                                                   */
}   usb_EpConfig_t;


/**
 * \brief Host channel configuration
 *
 * A channel is the combination of the target device and one of its endpoints. A control channel is used
 * in both directions (SETUP / OUT and IN), a bulk or interrupt channel in the direction of the endpoint address.
 */
typedef struct
{
    usb_ChannelId_t    ChannelId;     /**< Channel (index of the channel register, 0 .. 7)                                    */
    usb_EpAddr_t       Address;       /**< Endpoint address of the device (direction and number; control: direction ignored)  */
    usb_EpType_t       Type;          /**< Endpoint transfer type (control, bulk, interrupt)                                  */
    usb_DevAddr_t      DevAddress;    /**< Address of the device (0 - not yet addressed .. 127)                               */
    usb_Speed_t        DevSpeed;      /**< Speed of the device (low, full)                                                    */
    usb_EpMaxPacket_t  MaxPacketSize; /**< Maximal packet size of the endpoint (control / bulk: 8, 16, 32, 64; interrupt: 1 .. 64) */
}   usb_ChannelConfig_t;


/**
 * \brief SETUP packet (USB 2.0 specification chapter 9.3)
 *
 * Fields have the byte order of the host (little endian) - no conversion is needed.
 */
typedef struct
{
    usb_ReqType_t    RequestType; /**< bmRequestType: direction, type and recipient of the request */
    usb_ReqId_t      Request;     /**< bRequest: request identification                            */
    usb_ReqValue_t   Value;       /**< wValue: request specific value                              */
    usb_ReqIndex_t   Index;       /**< wIndex: request specific index                              */
    usb_ReqLength_t  Length;      /**< wLength: count of bytes of the data stage                   */
}   usb_SetupPacket_t;


/** \brief Bus event callback (bus reset, suspend, resume, start of frame) */
typedef void ( usb_EventCallback_t )( usb_PeriphId_t usbId );

/** \brief SETUP packet callback, the packet is valid only during the callback (copy it) */
typedef void ( usb_SetupCallback_t )( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket );

/**
 * \brief Transfer finished callback
 *
 * \param usbId      [in]: USB controller identification
 * \param epAddr     [in]: Endpoint address (direction and number)
 * \param xferCnt    [in]: Count of bytes transferred (sent / stored to the buffer)
 * \param xferResult [in]: Result of the transfer
 */
typedef void ( usb_XferCallback_t )( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult );

/** \brief Controller error callback */
typedef void ( usb_ErrCallback_t )( usb_PeriphId_t usbId, usb_ErrorId_t errorId );

/**
 * \brief Host channel transfer finished callback
 *
 * \param usbId    [in]: USB controller identification
 * \param chId     [in]: Channel
 * \param xferCnt  [in]: Count of bytes transferred (sent / stored to the buffer)
 * \param chResult [in]: Result of the transfer
 */
typedef void ( usb_ChannelCallback_t )( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t xferCnt, usb_ChannelResult_t chResult );


/**
 * \brief Event callbacks (all called from the context servicing the controller)
 *
 * Unused callback shall be set to USB_NULL_PTR. The start of frame interrupt (1 kHz) is enabled
 * only if SofCallback is set.
 *
 * - ResetCallback:   bus reset. Endpoints other than endpoint 0 were closed (the module reopens
 *                    endpoint 0), device address is 0. Endpoints are initialized again after the
 *                    host configures the device.
 * - SuspendCallback: no bus activity for 3 ms.
 * - ResumeCallback:  bus activity after suspend (resume signaling, also remote wakeup).
 * - SofCallback:     start of frame.
 * - SetupCallback:   SETUP packet received on endpoint 0. Running transfers of endpoint 0 were
 *                    aborted. Endpoint 0 answers NAK until the callback (or later) starts the data
 *                    or status stage by Usb_Set_EpTxStart() / Usb_Set_EpRxStart().
 * - XferCallback:    transfer started by Usb_Set_EpTxStart() / Usb_Set_EpRxStart() finished.
 * - ErrorCallback:   controller error.
 *
 * Host mode callbacks (the device mode callbacks above are not called in the host mode, the start of
 * frame and the error callbacks are):
 *
 * - ConnectCallback:    a device was attached to the port. The bus is not reset yet; the caller waits
 *                       for the debounce time (100 ms) and resets the port (Usb_Set_HostPortResetActive()).
 * - DisconnectCallback: the device was detached. Running transfers were reported as aborted, all channels
 *                       were closed.
 * - ChannelCallback:    transfer started by Usb_Set_ChannelTxStart() / Usb_Set_ChannelRxStart() finished.
 *                       The channel is idle in the callback, so the callback can start the next transfer.
 */
typedef struct
{
    usb_EventCallback_t   *ResetCallback;      /**< Bus reset. USB_NULL_PTR if not used.                  */
    usb_EventCallback_t   *SuspendCallback;    /**< Bus suspend. USB_NULL_PTR if not used.                */
    usb_EventCallback_t   *ResumeCallback;     /**< Bus resume. USB_NULL_PTR if not used.                 */
    usb_EventCallback_t   *SofCallback;        /**< Start of frame. USB_NULL_PTR if not used.             */
    usb_SetupCallback_t   *SetupCallback;      /**< SETUP packet received. USB_NULL_PTR if not used.      */
    usb_XferCallback_t    *XferCallback;       /**< Transfer finished. USB_NULL_PTR if not used.          */
    usb_ErrCallback_t     *ErrorCallback;      /**< Controller error. USB_NULL_PTR if not used.           */
    usb_EventCallback_t   *ConnectCallback;    /**< Host: device attached. USB_NULL_PTR if not used.      */
    usb_EventCallback_t   *DisconnectCallback; /**< Host: device detached. USB_NULL_PTR if not used.      */
    usb_ChannelCallback_t *ChannelCallback;    /**< Host: channel transfer finished. USB_NULL_PTR if not used. */
}   usb_Callbacks_t;


/** \brief List of DM (D-) pins available for the USB controller */
typedef enum
{
#ifdef GPIOA
    USB_DM_PIN_PA11   = USB_PIN_BIT_MASK_ENCODE( USB_PERIPH_FS  , GPIO_PORT_A   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_10  ), /**< USB DM pin connected to PA11 */
#endif
    USB_DM_PIN_UNUSED = USB_PIN_BIT_MASK_ENCODE( USB_PERIPH_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin */
}   usb_DmPin_t;


/** \brief List of DP (D+) pins available for the USB controller */
typedef enum
{
#ifdef GPIOA
    USB_DP_PIN_PA12   = USB_PIN_BIT_MASK_ENCODE( USB_PERIPH_FS  , GPIO_PORT_A   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_10  ), /**< USB DP pin connected to PA12 */
#endif
    USB_DP_PIN_UNUSED = USB_PIN_BIT_MASK_ENCODE( USB_PERIPH_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin */
}   usb_DpPin_t;


/** \brief USB controller configuration structure */
typedef struct
{
    usb_PeriphId_t           PeriphId;         /**< USB controller identification                                           */
    usb_Mode_t               Mode;             /**< Operation mode (device, host)                                           */
    usb_ClkSource_t          ClkSource;        /**< Source of the 48 MHz kernel clock                                       */
    usb_ServiceMode_t        ServiceMode;      /**< Servicing by interrupt / Usb_Task()                                     */
    usb_IrqPrio_t            IrqPriority;      /**< USB interrupt priority (interrupt servicing)                            */
    usb_EpMaxPacket_t        Ep0MaxPacketSize; /**< Maximal packet size of the control endpoint 0 (8, 16, 32, 64)           */
    const usb_Callbacks_t   *Callbacks;        /**< Event callbacks (copied). USB_NULL_PTR - no callbacks                   */
    usb_DmPin_t              DmPin;            /**< DM GPIO pin used by the controller (USB_DM_PIN_UNUSED - not configured) */
    usb_DpPin_t              DpPin;            /**< DP GPIO pin used by the controller (USB_DP_PIN_UNUSED - not configured) */
}   usb_BusConfig_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* USB_USB_TYPES_H */
