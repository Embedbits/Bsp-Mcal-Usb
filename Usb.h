/**
 * \author Mr.Nobody
 * \file Usb.h
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module common functionality header file.
 *
 * This file contains the common functionality used internally by the module
 * (USB_DRD_FS controller). Nothing from this file is available outside of the
 * module - the public interface is Usb_Port.h.
 *
 */

#ifndef USB_USB_H
#define USB_USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usb_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

#if defined(USB_DRD_FS)

/** \brief Endpoint number (index of the endpoint / channel register) */
typedef uint8_t usb_EpNum_t;

/** \brief Address in the packet memory (PMA) in bytes */
typedef uint16_t usb_PmaAddr_t;

/** \brief Size in the packet memory (PMA) in bytes */
typedef uint16_t usb_PmaSize_t;

/** \brief Type representing iteration count of busy-wait loops (register read-back) */
typedef uint32_t usb_TimeoutCnt_t;

/** \brief Type representing a register value */
typedef uint32_t usb_RegValue_t;


/** \brief Endpoint direction (index into the endpoint contexts) */
typedef enum
{
    USB_EP_DIR_OUT = 0u, /**< OUT endpoint (host to device, reception)   */
    USB_EP_DIR_IN,       /**< IN endpoint (device to host, transmission) */
    USB_EP_DIR_CNT       /**< Count of directions                        */
}   usb_EpDir_t;


/** \brief Runtime context of one endpoint direction */
typedef struct
{
    usb_FunctionState_t           Opened;        /**< Endpoint direction is initialized                        */
    usb_EpType_t                  Type;          /**< Transfer type                                            */
    usb_EpMaxPacket_t             MaxPacketSize; /**< Maximal packet size                                      */
    usb_PmaAddr_t                 PmaAddr;       /**< Address of the packet buffer in the packet memory        */
    usb_PmaSize_t                 PmaSize;       /**< Size of the packet buffer in the packet memory           */
    volatile usb_FlagState_t      Stalled;       /**< STALL handshake is requested                             */
    volatile usb_FunctionState_t  XferActive;    /**< Transfer is running                                      */
    const usb_Data_t * volatile   TxData;        /**< IN transfer: data to send                                */
    usb_Data_t * volatile         RxBuffer;      /**< OUT transfer: buffer for received data                   */
    volatile usb_DataCnt_t        XferSize;      /**< Requested size of the transfer                           */
    volatile usb_DataCnt_t        XferCnt;       /**< Count of bytes transferred                               */
    volatile usb_DataCnt_t        PacketLen;     /**< IN transfer: length of the packet given to the controller */
}   usb_EpContext_t;


/**
 * \brief Runtime context of a host channel
 *
 * The packet buffers and the transfer state of a channel are held by the endpoint contexts of the same
 * index: \c Ep[ channel ][ USB_EP_DIR_OUT ] for the SETUP / OUT transactions (transmit buffer) and
 * \c Ep[ channel ][ USB_EP_DIR_IN ] for the IN transactions (receive buffer).
 */
typedef struct
{
    usb_EpAddr_t                  Address;       /**< Endpoint address of the device                              */
    usb_DevAddr_t                 DevAddress;    /**< Address of the device                                       */
    usb_Speed_t                   DevSpeed;      /**< Speed of the device                                         */
    usb_Token_t                   Token;         /**< Token of the running transfer                               */
    uint8_t                       ErrorCnt;      /**< Count of the consecutive transaction errors                 */
}   usb_ChannelContext_t;


/** \brief Runtime context of a USB controller */
typedef struct
{
    usb_FunctionState_t           InitState;        /**< Controller is initialized                                    */
    usb_Mode_t                    Mode;             /**< Operation mode                                               */
    usb_Callbacks_t               Callbacks;        /**< Copy of the event callbacks                                  */
    usb_ServiceMode_t             ServiceMode;      /**< Interrupt / polling servicing                                */
    usb_ClkSource_t               ClkSource;        /**< Source of the kernel clock                                   */
    usb_EpMaxPacket_t             Ep0MaxPacketSize; /**< Maximal packet size of the endpoint 0                        */
    usb_FunctionState_t           TrimActive;       /**< HSI48 automatic trimming was activated by the module         */
    volatile usb_DeviceState_t    DeviceState;      /**< Device state                                                 */
    usb_DeviceState_t             StateBeforeSuspend; /**< Device state restored by the resume                        */
    volatile usb_DevAddr_t        DeviceAddress;    /**< Address applied to the controller                            */
    volatile usb_DevAddr_t        PendingAddress;   /**< Address applied after the status stage (0 - none)            */
    usb_SetupPacket_t             SetupPacket;      /**< Last received SETUP packet                                   */
    usb_FlagState_t               SetupValid;       /**< SETUP packet was received since the last bus reset           */
    volatile usb_HostPortState_t  HostPortState;    /**< Host mode: state of the downstream port                      */
    usb_Speed_t                   HostDevSpeed;     /**< Host mode: speed of the attached device                      */
    usb_ChannelContext_t          Channel[ USB_DRD_FS_EP_NBR ]; /**< Host mode: channel contexts                      */
    usb_EpContext_t               Ep[ USB_DRD_FS_EP_NBR ][ USB_EP_DIR_CNT ]; /**< Endpoint contexts                   */
}   usb_Context_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Count of endpoint numbers of the controller */
#define USB_EP_NUM_CNT                      ( USB_DRD_FS_EP_NBR )

/** Busy-wait iteration budget used for register read-back (common for all Usb module files) */
#define USB_TIMEOUT_RAW                     ( (usb_TimeoutCnt_t)0x84FCBu )

/** Number of register reads giving the controller start-up time after the power-up (>= 1 us) */
#define USB_STARTUP_READ_CNT                ( 64u )

/** Kernel clock frequency required by the controller in Hz */
#define USB_KERNEL_CLK_HZ                   ( 48000000u )

/**
 * Count of register reads giving the delay after a correct transfer interrupt before the received packet
 * is read from the packet memory (errata ES0561 2.15.1: 800 ns in full speed, 6.4 us in low speed).
 * Every read of a peripheral register takes at least 8 ns, so the count is sufficient on every clock configuration.
 */
#define USB_PMA_SETTLE_READ_CNT             ( 100u )

/** Count of the consecutive transaction errors of a channel after which its transfer fails (host mode) */
#define USB_HOST_ERROR_LIMIT                ( 3u )

/** Multiplier of the wait before a packet is read from a low speed device (6.4 us instead of 800 ns, errata ES0561 2.15.1) */
#define USB_PMA_SETTLE_LS_FACTOR            ( 8u )

/** Size of the buffer descriptor table in the packet memory (8 bytes per endpoint) */
#define USB_PMA_BTABLE_SIZE                 ( USB_EP_NUM_CNT * 8u )

/** Address field of a buffer descriptor (TXBD / RXBD) */
#define USB_PMA_DESC_ADDR_MASK              ( 0x0000FFFFu )

/** Packet memory is allocated in blocks of 4 bytes (word access, buffer address is word aligned) */
#define USB_PMA_ALIGN                       ( 4u )

/** Size of the packet memory in bytes */
#define USB_PMA_TOTAL_SIZE                  ( USB_DRD_PMA_SIZE )

/** Maximal count of transfer events served by one run of Usb_Task() */
#define USB_CTR_LOOP_MAX                    ( 64u )

/** Maximal packet size of control, bulk and interrupt endpoints at full speed */
#define USB_FS_MAX_PACKET_SIZE              ( 64u )

/** Length of the SETUP packet buffer descriptor in bytes */
#define USB_SETUP_BYTE_CNT                  ( USB_SETUP_PACKET_LEN )

/** Bits of ISTR which are cleared by writing 0 (write 1 has no effect) */
#define USB_ISTR_CLEARABLE_MASK             ( USB_ISTR_L1REQ | USB_ISTR_ESOF | USB_ISTR_SOF   | USB_ISTR_RESET | \
                                              USB_ISTR_SUSP  | USB_ISTR_WKUP | USB_ISTR_ERR   | USB_ISTR_PMAOVR | \
                                              USB_ISTR_THR512 | USB_ISTR_DDISC )

/** Control register bits enabling the interrupts always used by the module */
#define USB_CNTR_EVENT_IRQ_MASK             ( USB_CNTR_CTRM | USB_CNTR_PMAOVRM | USB_CNTR_ERRM | \
                                              USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM )

/** Control register bits which shall be zero when the controller accepts endpoint access */
#define USB_CNTR_NOT_ACTIVE_MASK            ( USB_CNTR_PDWN | USB_CNTR_USBRST )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Usb.c - common services for the files of the module */
usb_RequestState_t  Usb_Get_Context       ( usb_PeriphId_t usbId, usb_Context_t ** const usbContext );
usb_RequestState_t  Usb_Get_PeriphReg     ( usb_PeriphId_t usbId, USB_DRD_TypeDef ** const periphReg );
usb_RequestState_t  Usb_Check_Operational ( usb_PeriphId_t usbId );
usb_RequestState_t  Usb_Check_HostOperational( usb_PeriphId_t usbId );
void                Usb_Set_IstrFlagClear ( USB_DRD_TypeDef * const periphReg, usb_RegValue_t flagMask );
void                Usb_Set_EpContextReset( usb_EpContext_t * const epContext );
void                Usb_Handle_Error      ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_RegValue_t flagMask, usb_ErrorId_t errorId );
usb_RequestState_t  Usb_Verify_Reg        ( const volatile uint32_t * const regAddr, usb_RegValue_t regMask, usb_RegValue_t regValue );

/* Implemented in Usb_Pma.c - packet memory */
void                Usb_Pma_Write         ( const usb_Data_t * const srcData, usb_PmaAddr_t pmaAddr, usb_DataCnt_t dataLen );
void                Usb_Pma_Read          ( usb_Data_t * const dstData, usb_PmaAddr_t pmaAddr, usb_DataCnt_t dataLen );
void                Usb_Pma_Reset         ( void );
void                Usb_Pma_Wait_Settle   ( const USB_DRD_TypeDef * const periphReg );
usb_PmaSize_t       Usb_Pma_Get_BufferSize( usb_EpMaxPacket_t maxPacketSize );
usb_RequestState_t  Usb_Pma_Alloc         ( const usb_Context_t * const usbContext, usb_PmaSize_t bufSize, usb_PmaAddr_t * const pmaAddr );

/* Implemented in Usb_Ep.c - endpoints */
usb_RequestState_t  Usb_Ep_Get_Context    ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_EpContext_t ** const epContext );
usb_RequestState_t  Usb_Ep_Open           ( usb_PeriphId_t usbId, const usb_EpConfig_t * const epConfig );
usb_RequestState_t  Usb_Ep_Open_Control0  ( usb_PeriphId_t usbId );
usb_FlagState_t     Usb_Ep_Is_MaxPacketValid( usb_EpType_t epType, usb_EpMaxPacket_t maxPacketSize );
void                Usb_Ep_Set_RxSetupReady( usb_PeriphId_t usbId );
void                Usb_Ep_Close_All      ( usb_PeriphId_t usbId );

/* Implemented in Usb_Host.c - host mode */
usb_RequestState_t  Usb_Host_Init_Controller( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
void                Usb_Host_Service      ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );

/* Implemented in Usb_Xfer.c - data transfers */
void                Usb_Xfer_Service      ( usb_PeriphId_t usbId );
void                Usb_Xfer_Finish       ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_XferResult_t xferResult );
void                Usb_Xfer_Abort_All    ( usb_PeriphId_t usbId, usb_FlagState_t notify );

#endif /* USB_DRD_FS */

#ifdef __cplusplus
}
#endif

#endif /* USB_USB_H */
