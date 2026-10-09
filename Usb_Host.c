/**
 * \author Mr.Nobody
 * \file Usb_Host.c
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module host mode
 *
 * Host mode of the USB_DRD_FS controller: one downstream port (attach / detach detection, bus reset,
 * start of frame) and up to eight channels. A channel is the combination of the target device
 * (address, speed) and one of its endpoints (number, type, maximal packet size); it is the channel
 * register (CHEPnR) of the controller with its packet buffers: the transmit buffer for the SETUP and
 * OUT transactions, the receive buffer for the IN transactions. A control channel uses both buffers.
 *
 * Transfers of any size are split to packets of the maximal packet size, moved between the application
 * buffer and the packet memory in the servicing context (interrupt / Usb_Task()) and reported by one
 * callback. A transfer ends by a short packet or by the requested size (no zero length packet is added,
 * a zero length packet is a transfer of zero size).
 *
 * Behaviour of the controller used by the module (reference manual RM0492, USB chapter):
 * - After a transaction the status field of the channel register reports the answer of the device:
 *   disabled = ACK, NAK, STALL. A NAKed bulk or control transaction is retried by the controller at the
 *   start of the next frame - the module only acknowledges the event. A NAKed interrupt transaction is
 *   not retried, the channel is disabled and the caller starts it again after the polling interval.
 * - Data toggles are toggled by the controller after every acknowledged packet, a control channel sets
 *   both of them when a SETUP transaction is acknowledged (data and status stage use DATA1).
 * - Transaction errors (no answer, CRC, bit stuffing) are flagged in the channel register, the module
 *   starts the transaction again up to three times in a row.
 * - The channel registers are reset by the bus reset driven by the port.
 *
 * \note Register write verification (read-back within a timeout): the control register, the battery
 *       charging register, the transfer type, the device address, the endpoint address and the buffer
 *       addresses of the channels are verified. Exceptions: status and data toggle fields of the
 *       channel registers (changed by the controller, verified by the transfer events), flags of the
 *       interrupt status register, packet memory data.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usb.h"                            /* Self include                   */
#include "Usb_Port.h"                       /* Own port file include          */
#include "Usb_Types.h"                      /* Module types definitions       */
#include "Stm32_usb.h"                      /* USB RAL functionality          */
/* ============================== TYPEDEFS ================================== */

#if defined(USB_DRD_FS)

/* ======================== FORWARD DECLARATIONS ============================ */

static usb_RequestState_t  Usb_Host_Get_Channel        ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Context_t ** const usbContext, USB_DRD_TypeDef ** const periphReg );
static usb_RequestState_t  Usb_Host_Check_ChannelConfig( const usb_Context_t * const usbContext, const usb_ChannelConfig_t * const chConfig );
static usb_RequestState_t  Usb_Host_Verify_Channel     ( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_RegValue_t chepExpected, usb_PmaAddr_t outAddr, usb_PmaAddr_t inAddr, usb_FlagState_t useOut, usb_FlagState_t useIn );
static void                Usb_Host_Service_Channel    ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static void                Usb_Host_Handle_Connection  ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_RegValue_t istrValue );
static void                Usb_Host_Handle_Sof         ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static void                Usb_Host_Handle_In          ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_RegValue_t chepValue );
static void                Usb_Host_Handle_Out         ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_RegValue_t chepValue );
static void                Usb_Host_Handle_Error       ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_EpDir_t chDir, usb_RegValue_t chepValue );
static void                Usb_Host_Send_Packet        ( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_EpContext_t * const epContext, const usb_ChannelContext_t * const chContext );
static void                Usb_Host_Request_Packet     ( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_EpContext_t * const epContext );
static void                Usb_Host_Finish             ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, usb_ChannelId_t chId, usb_EpDir_t chDir, usb_ChannelResult_t chResult );
static void                Usb_Host_Close_Channel      ( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId );
static void                Usb_Host_Close_All          ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_FlagState_t notify );
static void                Usb_Host_Set_SetupBit       ( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId );
static void                Usb_Host_Wait_Settle        ( const USB_DRD_TypeDef * const periphReg, usb_Speed_t devSpeed );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Control register bits enabling the interrupts used in the host mode (correct transfer, packet memory overrun, device attach / detach) */
#define USB_CNTR_HOST_IRQ_MASK              ( USB_CNTR_CTRM | USB_CNTR_PMAOVRM | USB_CNTR_DCON )

/** Position of the three consecutive errors field of the IN transactions in the channel register */
#define USB_CHEP_THREE_ERR_RX_POS           ( 29u )

/** Position of the three consecutive errors field of the OUT / SETUP transactions in the channel register */
#define USB_CHEP_THREE_ERR_TX_POS           ( 27u )

/** Width mask of the three consecutive errors fields */
#define USB_CHEP_THREE_ERR_WIDTH_MASK       ( 0x3u )

/** Channel register bits cleared when a channel is (re)initialized (the remaining bits keep their value) */
#define USB_CHEP_INIT_CLEAR_MASK            ( USB_CHEP_DEVADDR | USB_CHEP_ADDR | USB_CHEP_LSEP | USB_CHEP_NAK | \
                                              USB_CHEP_KIND | USB_CHEP_SETUP | USB_CHEP_ERRTX | USB_CHEP_ERRRX )

/** Highest device address */
#define USB_HOST_DEV_ADDR_MAX               ( USB_DEV_ADDR_MAX )

/** Length of the SETUP packet */
#define USB_HOST_SETUP_LEN                  ( USB_SETUP_PACKET_LEN )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** Transfer type bits of the channel register, indexed by \ref usb_EpType_t */
static const usb_RegValue_t usb_HostTypeRegLut[ USB_EP_TYPE_CNT ] =
{
    [USB_EP_TYPE_CONTROL]     = USB_EP_CONTROL,
    [USB_EP_TYPE_ISOCHRONOUS] = USB_EP_ISOCHRONOUS,
    [USB_EP_TYPE_BULK]        = USB_EP_BULK,
    [USB_EP_TYPE_INTERRUPT]   = USB_EP_INTERRUPT,
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Reads the state of the downstream port
 *
 * \param usbId      [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param portState [out]: Port state (disconnected, connected, reset, enabled)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (also if the controller is not in the host mode).
 */
usb_RequestState_t Usb_Get_HostPortState( usb_PeriphId_t usbId, usb_HostPortState_t * const portState )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( USB_NULL_PTR != portState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_MODE_HOST       == usbContext->Mode        )    )
    {
        *portState = usbContext->HostPortState;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the reset signaling of the port (bus reset)
 *
 * The channels are closed (the controller resets their registers), running transfers are cancelled
 * without a callback. The caller keeps the reset active for at least 10 ms (the stack uses 50 ms),
 * releases it by \ref Usb_Set_HostPortResetInactive and waits for the port state
 * \ref USB_HOST_PORT_ENABLED (first start of frame) and for the recovery time (at least 10 ms) before
 * the device is addressed. The function does not wait.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (no device attached, reset already active).
 */
usb_RequestState_t Usb_Set_HostPortResetActive( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Check_HostOperational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( ( USB_REQUEST_OK            == retState                    ) &&
        ( USB_HOST_PORT_DISCONNECTED != usbContext->HostPortState  )    )
    {
        Usb_Host_Close_All( usbId, usbContext, periphReg, USB_FLAG_INACTIVE );

        periphReg->CNTR |= USB_CNTR_USBRST;

        retState = Usb_Verify_Reg( &periphReg->CNTR, USB_CNTR_USBRST, USB_CNTR_USBRST );

        usbContext->HostPortState = USB_HOST_PORT_RESET;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases the reset signaling of the port
 *
 * The controller starts to generate the start of frame packets. The port state changes to
 * \ref USB_HOST_PORT_ENABLED with the first start of frame (the start of frame interrupt is enabled
 * until then). The function does not wait.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (reset is not active).
 */
usb_RequestState_t Usb_Set_HostPortResetInactive( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Get_Context( usbId, &usbContext );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( ( USB_REQUEST_OK        == retState                           ) &&
        ( USB_FUNCTION_ACTIVE   == usbContext->InitState              ) &&
        ( USB_MODE_HOST         == usbContext->Mode                   ) &&
        ( USB_HOST_PORT_RESET   == usbContext->HostPortState          ) &&
        ( 0u                    != ( periphReg->CNTR & USB_CNTR_USBRST ) )    )
    {
        periphReg->CNTR &= ~USB_CNTR_USBRST;
        periphReg->CNTR |= USB_CNTR_SOFM;

        retState = Usb_Verify_Reg( &periphReg->CNTR, ( USB_CNTR_USBRST | USB_CNTR_SOFM ), USB_CNTR_SOFM );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes a host channel
 *
 * Allocates the packet buffers of the channel (a control channel has the transmit and the receive
 * buffer, a bulk or interrupt channel the buffer of its direction), configures the channel register
 * (transfer type, device address, endpoint address, low speed flag, buffer descriptors), sets the data
 * toggles to DATA0 and disables the channel. An open channel which is not busy can be initialized again
 * (for another device or endpoint, the buffers are allocated again). The default control channel is
 * typically initialized again for every device which is addressed.
 *
 * \note  The port shall be connected and the bus reset not driven (the channel registers are reset by
 *        the reset). Control, bulk and interrupt channels are supported, the isochronous type is rejected.
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chConfig [in]: Channel configuration
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (invalid parameter, no device attached,
 *         channel busy, no free packet memory).
 */
usb_RequestState_t Usb_Init_Channel( usb_PeriphId_t usbId, const usb_ChannelConfig_t * const chConfig )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    if( USB_NULL_PTR != chConfig )
    {
        retState = Usb_Host_Get_Channel( usbId, chConfig->ChannelId, &usbContext, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Host_Check_ChannelConfig( usbContext, chConfig );
    }

    if( USB_REQUEST_OK == retState )
    {
        const usb_ChannelId_t chId     = chConfig->ChannelId;
        const usb_PmaSize_t   bufSize  = Usb_Pma_Get_BufferSize( chConfig->MaxPacketSize );
        usb_FlagState_t       useOut   = USB_FLAG_INACTIVE;
        usb_FlagState_t       useIn    = USB_FLAG_INACTIVE;
        usb_PmaAddr_t         outAddr  = 0u;
        usb_PmaAddr_t         inAddr   = 0u;

        if( USB_EP_TYPE_CONTROL == chConfig->Type )
        {
            useOut = USB_FLAG_ACTIVE;
            useIn  = USB_FLAG_ACTIVE;
        }
        else if( USB_EP_ADDR_IS_IN( chConfig->Address ) )
        {
            useIn  = USB_FLAG_ACTIVE;
        }
        else
        {
            useOut = USB_FLAG_ACTIVE;
        }

        /* An open channel is initialized again - its buffers are released first */
        Usb_Set_EpContextReset( &usbContext->Ep[ chId ][ USB_EP_DIR_OUT ] );
        Usb_Set_EpContextReset( &usbContext->Ep[ chId ][ USB_EP_DIR_IN  ] );

        if( USB_FLAG_ACTIVE == useOut )
        {
            retState = Usb_Pma_Alloc( usbContext, bufSize, &outAddr );

            if( USB_REQUEST_OK == retState )
            {
                usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Opened  = USB_FUNCTION_ACTIVE;
                usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].PmaAddr = outAddr;
                usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].PmaSize = bufSize;
            }
        }

        if( ( USB_REQUEST_OK    == retState ) &&
            ( USB_FLAG_ACTIVE   == useIn    )    )
        {
            retState = Usb_Pma_Alloc( usbContext, bufSize, &inAddr );

            if( USB_REQUEST_OK == retState )
            {
                usbContext->Ep[ chId ][ USB_EP_DIR_IN ].Opened  = USB_FUNCTION_ACTIVE;
                usbContext->Ep[ chId ][ USB_EP_DIR_IN ].PmaAddr = inAddr;
                usbContext->Ep[ chId ][ USB_EP_DIR_IN ].PmaSize = bufSize;
            }
        }

        if( USB_REQUEST_OK == retState )
        {
            const usb_RegValue_t typeBits = usb_HostTypeRegLut[ chConfig->Type ];
            const usb_RegValue_t epNumber = (usb_RegValue_t)USB_EP_ADDR_NUMBER( chConfig->Address );
            usb_RegValue_t       regValue = USB_DRD_GET_CHEP( periphReg, chId ) & USB_CH_T_MASK;

            /* Transfer type, device address and endpoint address (a low speed device behind a full speed hub is flagged) */
            regValue &= ~USB_CHEP_INIT_CLEAR_MASK;
            regValue |= typeBits;
            regValue |= ( (usb_RegValue_t)chConfig->DevAddress << USB_CHEP_DEVADDR_Pos );
            regValue |= epNumber;

            if( ( USB_SPEED_LOW  == chConfig->DevSpeed      ) &&
                ( USB_SPEED_FULL == usbContext->HostDevSpeed )    )
            {
                regValue |= USB_CHEP_LSEP;
            }
            else
            {
                /* Device is connected directly or it is a full speed device */
            }

            USB_DRD_SET_CHEP( periphReg, chId, regValue | USB_CH_VTRX | USB_CH_VTTX );

            /* Buffers, data toggles DATA0, channel disabled until a transfer is started */
            if( USB_FLAG_ACTIVE == useOut )
            {
                USB_DRD_SET_CHEP_TX_ADDRESS( periphReg, chId, outAddr );
                USB_DRD_CLEAR_TX_DTOG( periphReg, chId );
            }

            if( USB_FLAG_ACTIVE == useIn )
            {
                USB_DRD_SET_CHEP_RX_ADDRESS( periphReg, chId, inAddr );
                USB_DRD_CLEAR_RX_DTOG( periphReg, chId );
            }

            USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_DIS );
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_DIS );

            retState = Usb_Host_Verify_Channel( periphReg, chId, ( typeBits | ( (usb_RegValue_t)chConfig->DevAddress << USB_CHEP_DEVADDR_Pos ) | epNumber ),
                                                outAddr, inAddr, useOut, useIn );
        }

        if( USB_REQUEST_OK == retState )
        {
            for( usb_DataCnt_t dirIdx = 0u; USB_EP_DIR_CNT > dirIdx; dirIdx ++ )
            {
                usbContext->Ep[ chId ][ dirIdx ].Type          = chConfig->Type;
                usbContext->Ep[ chId ][ dirIdx ].MaxPacketSize = chConfig->MaxPacketSize;
            }

            usbContext->Channel[ chId ].Address    = chConfig->Address;
            usbContext->Channel[ chId ].DevAddress = chConfig->DevAddress;
            usbContext->Channel[ chId ].DevSpeed   = chConfig->DevSpeed;
            usbContext->Channel[ chId ].Token      = USB_TOKEN_OUT;
            usbContext->Channel[ chId ].ErrorCnt   = 0u;
        }
        else
        {
            /* Channel could not be configured - its buffers are released */
            Usb_Set_EpContextReset( &usbContext->Ep[ chId ][ USB_EP_DIR_OUT ] );
            Usb_Set_EpContextReset( &usbContext->Ep[ chId ][ USB_EP_DIR_IN  ] );
        }
    }

    return ( retState );
}


/**
 * \brief Deinitializes a host channel
 *
 * The channel is disabled, a running transfer is cancelled without a callback, the packet buffers are released.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId  [in]: Channel
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (the channel is not initialized).
 */
usb_RequestState_t Usb_Deinit_Channel( usb_PeriphId_t usbId, usb_ChannelId_t chId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Host_Get_Channel( usbId, chId, &usbContext, &periphReg );

    if( ( USB_REQUEST_OK      == retState                                              ) &&
        ( ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Opened ) ||
          ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].Opened ) )    )
    {
        Usb_Host_Close_Channel( usbContext, periphReg, chId );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the state of a host channel
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId     [in]: Channel
 * \param chState [out]: Channel state (closed, idle, busy)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_ChannelState( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_ChannelState_t * const chState )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( ( USB_NULL_PTR != chState      ) &&
        ( USB_EP_NUM_CNT > chId        )    )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_MODE_HOST       == usbContext->Mode        )    )
    {
        const usb_EpContext_t * const outContext = &usbContext->Ep[ chId ][ USB_EP_DIR_OUT ];
        const usb_EpContext_t * const inContext  = &usbContext->Ep[ chId ][ USB_EP_DIR_IN  ];

        if( ( USB_FUNCTION_ACTIVE == outContext->XferActive ) ||
            ( USB_FUNCTION_ACTIVE == inContext->XferActive  )    )
        {
            *chState = USB_CHANNEL_STATE_BUSY;
        }
        else if( ( USB_FUNCTION_ACTIVE == outContext->Opened ) ||
                 ( USB_FUNCTION_ACTIVE == inContext->Opened  )    )
        {
            *chState = USB_CHANNEL_STATE_IDLE;
        }
        else
        {
            *chState = USB_CHANNEL_STATE_CLOSED;
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets the data toggle (packet identifier of the next data packet) of a channel
 *
 * The controller toggles the data toggle after every acknowledged packet. The caller sets DATA0 after the
 * halt of the endpoint was cleared by CLEAR_FEATURE(ENDPOINT_HALT) (a channel is initialized with DATA0).
 * The data toggles of a control channel are set by the controller when a SETUP transaction is acknowledged.
 *
 * \param usbId      [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId       [in]: Channel (not busy)
 * \param dataToggle [in]: Data toggle of the next packet
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_ChannelDataToggle( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataToggle_t dataToggle )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Host_Get_Channel( usbId, chId, &usbContext, &periphReg );

    if( ( USB_REQUEST_OK        == retState                                                 ) &&
        ( USB_DATA_TOGGLE_CNT   >  dataToggle                                               ) &&
        ( USB_FUNCTION_INACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].XferActive      ) &&
        ( USB_FUNCTION_INACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].XferActive      )    )
    {
        if( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Opened )
        {
            USB_DRD_CLEAR_TX_DTOG( periphReg, chId );

            if( USB_DATA_TOGGLE_DATA1 == dataToggle )
            {
                USB_DRD_TX_DTOG( periphReg, chId );
            }
        }

        if( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_IN ].Opened )
        {
            USB_DRD_CLEAR_RX_DTOG( periphReg, chId );

            if( USB_DATA_TOGGLE_DATA1 == dataToggle )
            {
                USB_DRD_RX_DTOG( periphReg, chId );
            }
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts a SETUP or OUT transfer (host to device) on a channel
 *
 * The data are sent in packets of the maximal packet size of the channel, the last packet is shorter if the
 * size is not a multiple of the packet size. A transfer of zero size sends one zero length packet (status
 * stage of a control read transfer). The end of the transfer (all packets acknowledged by the device) is
 * reported by the channel callback. The data have to stay valid until then.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId   [in]: Channel (control, or bulk / interrupt OUT)
 * \param token  [in]: \ref USB_TOKEN_SETUP (control channel, 8 bytes) or \ref USB_TOKEN_OUT
 * \param txData [in]: Data to send (can be USB_NULL_PTR if the size is zero)
 * \param txSize [in]: Count of bytes to send
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (channel not initialized or busy, port not
 *         enabled, invalid token or parameter).
 */
usb_RequestState_t Usb_Set_ChannelTxStart( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Token_t token, const usb_Data_t * const txData, usb_DataCnt_t txSize )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;
    usb_FlagState_t    dataValid  = USB_FLAG_INACTIVE;
    usb_FlagState_t    tokenValid = USB_FLAG_INACTIVE;

    retState = Usb_Host_Get_Channel( usbId, chId, &usbContext, &periphReg );

    if( USB_NULL_PTR != txData )
    {
        dataValid = USB_FLAG_ACTIVE;
    }
    else if( 0u == txSize )
    {
        dataValid = USB_FLAG_ACTIVE;
    }
    else
    {
        dataValid = USB_FLAG_INACTIVE;
    }

    if( USB_REQUEST_OK == retState )
    {
        if( USB_TOKEN_OUT == token )
        {
            tokenValid = USB_FLAG_ACTIVE;
        }
        else if( ( USB_TOKEN_SETUP                     == token                                          ) &&
                 ( USB_EP_TYPE_CONTROL                 == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Type  ) &&
                 ( USB_HOST_SETUP_LEN                  == txSize                                         )    )
        {
            tokenValid = USB_FLAG_ACTIVE;
        }
        else
        {
            tokenValid = USB_FLAG_INACTIVE;
        }
    }

    if( ( USB_REQUEST_OK          == retState                                                  ) &&
        ( USB_FLAG_ACTIVE         == dataValid                                                 ) &&
        ( USB_FLAG_ACTIVE         == tokenValid                                                ) &&
        ( USB_HOST_PORT_ENABLED   == usbContext->HostPortState                                 ) &&
        ( USB_FUNCTION_ACTIVE     == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Opened           ) &&
        ( USB_FUNCTION_INACTIVE   == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].XferActive       ) &&
        ( USB_FUNCTION_INACTIVE   == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].XferActive       )    )
    {
        usb_EpContext_t * const epContext = &usbContext->Ep[ chId ][ USB_EP_DIR_OUT ];

        epContext->TxData     = txData;
        epContext->XferSize   = txSize;
        epContext->XferCnt    = 0u;
        epContext->PacketLen  = 0u;
        epContext->XferActive = USB_FUNCTION_ACTIVE;

        usbContext->Channel[ chId ].Token    = token;
        usbContext->Channel[ chId ].ErrorCnt = 0u;

        Usb_Host_Send_Packet( periphReg, chId, epContext, &usbContext->Channel[ chId ] );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts an IN transfer (device to host) on a channel
 *
 * The received packets are stored to the buffer. The transfer ends when a short packet (shorter than the
 * maximal packet size, also a zero length packet) is received or the requested size is received - the
 * end is reported by the channel callback with the count of received bytes. A transfer of zero size
 * receives one zero length packet (status stage of a control write transfer). The buffer has to stay
 * valid until then and its size shall be a multiple of the packet size.
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId     [in]: Channel (control, or bulk / interrupt IN)
 * \param rxBuffer [in]: Buffer for the received data (can be USB_NULL_PTR if the size is zero)
 * \param rxSize   [in]: Size of the buffer in bytes
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (channel not initialized or busy, port not
 *         enabled, invalid parameter).
 */
usb_RequestState_t Usb_Set_ChannelRxStart( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;
    usb_FlagState_t    dataValid  = USB_FLAG_INACTIVE;

    retState = Usb_Host_Get_Channel( usbId, chId, &usbContext, &periphReg );

    if( USB_NULL_PTR != rxBuffer )
    {
        dataValid = USB_FLAG_ACTIVE;
    }
    else if( 0u == rxSize )
    {
        dataValid = USB_FLAG_ACTIVE;
    }
    else
    {
        dataValid = USB_FLAG_INACTIVE;
    }

    if( ( USB_REQUEST_OK          == retState                                                  ) &&
        ( USB_FLAG_ACTIVE         == dataValid                                                 ) &&
        ( USB_HOST_PORT_ENABLED   == usbContext->HostPortState                                 ) &&
        ( USB_FUNCTION_ACTIVE     == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].Opened           ) &&
        ( USB_FUNCTION_INACTIVE   == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].XferActive       ) &&
        ( USB_FUNCTION_INACTIVE   == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].XferActive       )    )
    {
        usb_EpContext_t * const epContext = &usbContext->Ep[ chId ][ USB_EP_DIR_IN ];

        epContext->RxBuffer   = rxBuffer;
        epContext->XferSize   = rxSize;
        epContext->XferCnt    = 0u;
        epContext->PacketLen  = 0u;
        epContext->XferActive = USB_FUNCTION_ACTIVE;

        usbContext->Channel[ chId ].Token    = USB_TOKEN_IN;
        usbContext->Channel[ chId ].ErrorCnt = 0u;

        Usb_Host_Request_Packet( periphReg, chId, epContext );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops a running transfer of a channel
 *
 * The channel is disabled and the transfer is cancelled without a callback. A transaction which is already
 * running on the bus finishes regularly, its result is dropped. The channel without a running transfer is
 * not changed.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId  [in]: Channel
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (the channel is not initialized).
 */
usb_RequestState_t Usb_Set_ChannelXferStop( usb_PeriphId_t usbId, usb_ChannelId_t chId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Host_Get_Channel( usbId, chId, &usbContext, &periphReg );

    if( ( USB_REQUEST_OK      == retState                                              ) &&
        ( ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Opened ) ||
          ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].Opened ) )    )
    {
        if( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].XferActive )
        {
            usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].XferActive = USB_FUNCTION_INACTIVE;

            USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_DIS );
        }

        if( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_IN ].XferActive )
        {
            usbContext->Ep[ chId ][ USB_EP_DIR_IN ].XferActive = USB_FUNCTION_INACTIVE;

            USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_DIS );
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the count of bytes transferred by the running or the last transfer of a channel
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param chId     [in]: Channel
 * \param xferCnt [out]: Count of bytes sent (SETUP / OUT) and stored to the buffer (IN) - the transfer
 *                       in the direction of the last started transfer
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_ChannelXferCount( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t * const xferCnt )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( ( USB_NULL_PTR != xferCnt ) &&
        ( USB_EP_NUM_CNT > chId   )    )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_MODE_HOST       == usbContext->Mode        )    )
    {
        if( USB_TOKEN_IN == usbContext->Channel[ chId ].Token )
        {
            *xferCnt = usbContext->Ep[ chId ][ USB_EP_DIR_IN ].XferCnt;
        }
        else
        {
            *xferCnt = usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].XferCnt;
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}

/* ====================== SERVICES OF THE MODULE FILES ====================== */

/**
 * \brief Powers up and configures the controller registers in the host mode
 *
 * The host mode is selected while the controller is powered down, then the controller is powered up in the
 * reset state, the reset is released after the start-up time, pending flags are cleared, the pull-down
 * resistors of the DP / DM lines are connected, the buffer descriptor table is cleared and the interrupts of
 * the transfers, the device attach / detach and the packet memory errors are enabled (start of frame
 * interrupt only if its callback is used). The bus stays idle until a device is attached and the port is reset.
 *
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was success.
 */
usb_RequestState_t Usb_Host_Init_Controller( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;
    usb_RegValue_t     irqMask  = USB_CNTR_HOST_IRQ_MASK;

    /* Host mode is selected in the power down state, then the controller is powered up in the reset state */
    periphReg->CNTR = ( USB_CNTR_HOST | USB_CNTR_USBRST | USB_CNTR_PDWN );
    periphReg->CNTR = ( USB_CNTR_HOST | USB_CNTR_USBRST );

    retState = Usb_Verify_Reg( &periphReg->CNTR, ( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_HOST ), ( USB_CNTR_USBRST | USB_CNTR_HOST ) );

    if( USB_REQUEST_OK == retState )
    {
        /* Analog start-up time of the controller (at least 1 us) */
        for( usb_TimeoutCnt_t readIdx = 0u; USB_STARTUP_READ_CNT > readIdx; readIdx ++ )
        {
            (void)periphReg->CNTR;
        }

        /* Pending flags cleared, pull-down resistors of the DP / DM lines connected, buffer descriptors cleared */
        periphReg->ISTR  = 0u;
        periphReg->DADDR = 0u;
        periphReg->BCDR  = USB_BCDR_DPPD;

        Usb_Pma_Reset();

        retState = Usb_Verify_Reg( &periphReg->BCDR, ( USB_BCDR_DPPU | USB_BCDR_DPPD ), USB_BCDR_DPPD );
    }

    if( USB_REQUEST_OK == retState )
    {
        if( USB_NULL_PTR != usbContext->Callbacks.SofCallback )
        {
            irqMask |= USB_CNTR_SOFM;
        }
        else
        {
            /* Start of frame interrupt is not needed */
        }

        /* Reset released (no reset signaling), interrupts enabled */
        periphReg->CNTR = ( USB_CNTR_HOST | irqMask );

        retState = Usb_Verify_Reg( &periphReg->CNTR, ( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_HOST | USB_CNTR_HOST_IRQ_MASK | USB_CNTR_SOFM ),
                                   ( USB_CNTR_HOST | irqMask ) );
    }

    if( USB_REQUEST_OK == retState )
    {
        usbContext->HostPortState = USB_HOST_PORT_DISCONNECTED;
        usbContext->HostDevSpeed  = USB_SPEED_FULL;
    }

    return ( retState );
}


/**
 * \brief Services the host mode events of a controller
 *
 * Called by the interrupt handler or by \ref Usb_Task. One correct transfer event (of the channel
 * presented by the controller) is served first, then the pending device attach / detach, the packet
 * memory error and the start of frame events whose interrupt is enabled.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
void Usb_Host_Service( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    usb_RegValue_t istrValue = periphReg->ISTR;

    if( 0u != ( istrValue & USB_ISTR_CTR ) )
    {
        Usb_Host_Service_Channel( usbId, usbContext, periphReg );

        istrValue = periphReg->ISTR;
    }
    else
    {
        /* No transfer event */
    }

    const usb_RegValue_t eventMask = istrValue & periphReg->CNTR;

    if( 0u != ( eventMask & USB_ISTR_DCON ) )
    {
        Usb_Host_Handle_Connection( usbId, usbContext, periphReg, istrValue );
    }

    if( 0u != ( eventMask & USB_ISTR_PMAOVR ) )
    {
        Usb_Handle_Error( usbId, usbContext, periphReg, USB_ISTR_PMAOVR, USB_ERROR_PMA_OVERRUN );
    }

    if( 0u != ( eventMask & USB_ISTR_SOF ) )
    {
        Usb_Host_Handle_Sof( usbId, usbContext, periphReg );
    }
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks the host operation of the controller and the channel identification, returns the context and the registers
 *
 * \param usbId       [in]: USB controller identification
 * \param chId        [in]: Channel
 * \param usbContext [out]: Context of the controller
 * \param periphReg  [out]: Controller registers
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the controller is operational in the host
 *         mode and the channel identification is valid.
 */
static usb_RequestState_t Usb_Host_Get_Channel( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Context_t ** const usbContext, USB_DRD_TypeDef ** const periphReg )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    retState = Usb_Check_HostOperational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_Context( usbId, usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, periphReg );
    }

    if( ( USB_REQUEST_OK == retState ) &&
        ( USB_EP_NUM_CNT >  chId     )    )
    {
        retState = USB_REQUEST_OK;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the configuration of a channel and the state of the port and of the channel
 *
 * \param usbContext [in]: Context of the controller
 * \param chConfig   [in]: Channel configuration
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the channel can be initialized.
 */
static usb_RequestState_t Usb_Host_Check_ChannelConfig( const usb_Context_t * const usbContext, const usb_ChannelConfig_t * const chConfig )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_FlagState_t    typeValid  = USB_FLAG_INACTIVE;
    usb_FlagState_t    sizeValid  = USB_FLAG_INACTIVE;
    usb_FlagState_t    reserved   = USB_FLAG_INACTIVE;

    if( ( USB_EP_TYPE_CONTROL   == chConfig->Type ) ||
        ( USB_EP_TYPE_BULK      == chConfig->Type ) ||
        ( USB_EP_TYPE_INTERRUPT == chConfig->Type )    )
    {
        typeValid = USB_FLAG_ACTIVE;
        sizeValid = Usb_Ep_Is_MaxPacketValid( chConfig->Type, chConfig->MaxPacketSize );
    }
    else
    {
        typeValid = USB_FLAG_INACTIVE;
    }

    if( 0u != ( chConfig->Address & ( (usb_EpAddr_t)( ~( USB_EP_ADDR_DIR_IN | USB_EP_ADDR_NUM_MASK ) ) ) ) )
    {
        reserved = USB_FLAG_ACTIVE;
    }
    else
    {
        reserved = USB_FLAG_INACTIVE;
    }

    if( ( USB_FLAG_ACTIVE            == typeValid                                                  ) &&
        ( USB_FLAG_ACTIVE            == sizeValid                                                  ) &&
        ( USB_FLAG_INACTIVE          == reserved                                                   ) &&
        ( USB_HOST_DEV_ADDR_MAX      >= chConfig->DevAddress                                       ) &&
        ( USB_SPEED_HIGH             >  chConfig->DevSpeed                                         ) &&
        ( USB_HOST_PORT_DISCONNECTED != usbContext->HostPortState                                  ) &&
        ( USB_FUNCTION_INACTIVE      == usbContext->Ep[ chConfig->ChannelId ][ USB_EP_DIR_OUT ].XferActive ) &&
        ( USB_FUNCTION_INACTIVE      == usbContext->Ep[ chConfig->ChannelId ][ USB_EP_DIR_IN  ].XferActive )    )
    {
        retState = USB_REQUEST_OK;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Verifies by read-back the transfer type, the device address, the endpoint address and the buffer addresses of a channel
 *
 * The status and data toggle fields of the channel register are changed by the controller during the
 * transfers, so they are not verified (they are verified by the transfer events).
 *
 * \param periphReg   [in]: USB controller registers
 * \param chId        [in]: Channel
 * \param chepExpected [in]: Expected transfer type, device address and endpoint address bits of the channel register
 * \param outAddr     [in]: Expected address of the transmit buffer
 * \param inAddr      [in]: Expected address of the receive buffer
 * \param useOut      [in]: Active - the transmit buffer is verified
 * \param useIn       [in]: Active - the receive buffer is verified
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the channel is configured as required.
 */
static usb_RequestState_t Usb_Host_Verify_Channel( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_RegValue_t chepExpected, usb_PmaAddr_t outAddr, usb_PmaAddr_t inAddr, usb_FlagState_t useOut, usb_FlagState_t useIn )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    for( usb_TimeoutCnt_t iterationCnt = 0u; USB_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const usb_RegValue_t chepValue = USB_DRD_GET_CHEP( periphReg, chId );
        const usb_RegValue_t chepMask  = ( USB_CH_UTYPE | USB_CHEP_DEVADDR | USB_CHEP_ADDR );
        usb_FlagState_t      outValid  = USB_FLAG_ACTIVE;
        usb_FlagState_t      inValid   = USB_FLAG_ACTIVE;

        if( USB_FLAG_ACTIVE == useOut )
        {
            if( outAddr == ( (USB_DRD_PMA_BUFF + chId)->TXBD & USB_PMA_DESC_ADDR_MASK ) )
            {
                outValid = USB_FLAG_ACTIVE;
            }
            else
            {
                outValid = USB_FLAG_INACTIVE;
            }
        }

        if( USB_FLAG_ACTIVE == useIn )
        {
            if( inAddr == ( (USB_DRD_PMA_BUFF + chId)->RXBD & USB_PMA_DESC_ADDR_MASK ) )
            {
                inValid = USB_FLAG_ACTIVE;
            }
            else
            {
                inValid = USB_FLAG_INACTIVE;
            }
        }

        if( ( chepExpected == ( chepValue & chepMask ) ) &&
            ( USB_FLAG_ACTIVE == outValid              ) &&
            ( USB_FLAG_ACTIVE == inValid               )    )
        {
            retState = USB_REQUEST_OK;
            break;
        }
        else
        {
            /* Configuration not applied yet */
        }
    }

    return ( retState );
}


/**
 * \brief Services one correct transfer event of a channel
 *
 * The controller presents the channel and the direction of the oldest completed transaction (ISTR.IDN,
 * ISTR.DIR): the direction bit is set for the IN transactions (reception), cleared for the OUT / SETUP
 * transactions (transmission). If a channel has both transactions pending, the IN transaction is presented
 * first.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
static void Usb_Host_Service_Channel( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    const usb_RegValue_t  istrValue = periphReg->ISTR;
    const usb_ChannelId_t chId      = (usb_ChannelId_t)( istrValue & USB_ISTR_IDN );

    if( ( 0u != ( istrValue & USB_ISTR_CTR ) ) &&
        ( USB_EP_NUM_CNT > chId              )    )
    {
        const usb_RegValue_t chepValue = USB_DRD_GET_CHEP( periphReg, chId );

        if( 0u != ( istrValue & USB_ISTR_DIR ) )
        {
            if( 0u != ( chepValue & USB_CH_VTRX ) )
            {
                Usb_Host_Handle_In( usbId, usbContext, periphReg, chId, chepValue );
            }
            else
            {
                /* Inconsistent event - the controller presents it again if it is still pending */
            }
        }
        else
        {
            if( 0u != ( chepValue & USB_CH_VTTX ) )
            {
                Usb_Host_Handle_Out( usbId, usbContext, periphReg, chId, chepValue );
            }
            else
            {
                /* Inconsistent event - the controller presents it again if it is still pending */
            }
        }
    }
    else
    {
        /* No pending correct transfer event or invalid channel identification */
    }
}


/**
 * \brief Handles the attach and the detach of a device
 *
 * Attach: the port is connected, the speed of the device is stored and the connect callback is called.
 * Detach: running transfers are reported as aborted, all channels are closed, the reset signaling is
 * released and the disconnect callback is called. The events caused by the reset signaling driven by the
 * module are ignored.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param istrValue  [in]: Interrupt status register value read when the event was found
 */
static void Usb_Host_Handle_Connection( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_RegValue_t istrValue )
{
    Usb_Set_IstrFlagClear( periphReg, USB_ISTR_DCON );

    if( ( USB_HOST_PORT_RESET == usbContext->HostPortState ) &&
        ( 0u != ( periphReg->CNTR & USB_CNTR_USBRST )      )    )
    {
        /* Event caused by the reset signaling of the module */
    }
    else if( 0u != ( istrValue & USB_ISTR_DCON_STAT ) )
    {
        if( 0u != ( istrValue & USB_ISTR_LS_DCONN ) )
        {
            usbContext->HostDevSpeed = USB_SPEED_LOW;
        }
        else
        {
            usbContext->HostDevSpeed = USB_SPEED_FULL;
        }

        if( USB_HOST_PORT_DISCONNECTED == usbContext->HostPortState )
        {
            usbContext->HostPortState = USB_HOST_PORT_CONNECTED;

            if( USB_NULL_PTR != usbContext->Callbacks.ConnectCallback )
            {
                usbContext->Callbacks.ConnectCallback( usbId );
            }
            else
            {
                /* Connect callback is not used */
            }
        }
        else
        {
            /* Device was attached already */
        }
    }
    else if( USB_HOST_PORT_DISCONNECTED != usbContext->HostPortState )
    {
        periphReg->CNTR &= ~USB_CNTR_USBRST;

        usbContext->HostPortState = USB_HOST_PORT_DISCONNECTED;

        Usb_Host_Close_All( usbId, usbContext, periphReg, USB_FLAG_ACTIVE );

        if( USB_NULL_PTR != usbContext->Callbacks.DisconnectCallback )
        {
            usbContext->Callbacks.DisconnectCallback( usbId );
        }
        else
        {
            /* Disconnect callback is not used */
        }
    }
    else
    {
        /* Detach of a device which was not attached */
    }
}


/**
 * \brief Handles the start of frame
 *
 * The first start of frame after the release of the reset signaling enables the port. The start of frame
 * interrupt is disabled again if the start of frame callback is not used.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
static void Usb_Host_Handle_Sof( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    Usb_Set_IstrFlagClear( periphReg, USB_ISTR_SOF );

    if( ( USB_HOST_PORT_RESET == usbContext->HostPortState )  &&
        ( 0u == ( periphReg->CNTR & USB_CNTR_USBRST )      )    )
    {
        usbContext->HostPortState = USB_HOST_PORT_ENABLED;

        if( USB_NULL_PTR == usbContext->Callbacks.SofCallback )
        {
            periphReg->CNTR &= ~USB_CNTR_SOFM;
        }
        else
        {
            /* Start of frame interrupt is needed by the callback */
        }
    }
    else
    {
        /* Port is not waiting for the first start of frame */
    }

    if( USB_NULL_PTR != usbContext->Callbacks.SofCallback )
    {
        usbContext->Callbacks.SofCallback( usbId );
    }
    else
    {
        /* Start of frame callback is not used */
    }
}


/**
 * \brief Handles the end of an IN transaction (reception) of a channel
 *
 * The status field of the channel register reports the answer of the device: disabled = data received and
 * acknowledged, NAK, STALL. A transaction ended by errors is flagged in the channel register.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param chId       [in]: Channel
 * \param chepValue  [in]: Channel register value read when the event was found
 */
static void Usb_Host_Handle_In( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_RegValue_t chepValue )
{
    usb_EpContext_t * const        epContext = &usbContext->Ep[ chId ][ USB_EP_DIR_IN ];
    usb_ChannelContext_t * const   chContext = &usbContext->Channel[ chId ];
    const usb_RegValue_t           statusRx  = chepValue & USB_CH_RX_STRX;

    if( USB_FUNCTION_INACTIVE == epContext->XferActive )
    {
        /* Event of a stopped transfer */
        USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, chId );
    }
    else if( 0u != ( chepValue & USB_CH_ERRRX ) )
    {
        Usb_Host_Handle_Error( usbId, usbContext, periphReg, chId, USB_EP_DIR_IN, chepValue );
    }
    else if( USB_CH_RX_ACK_SBUF == statusRx )
    {
        Usb_Host_Wait_Settle( periphReg, chContext->DevSpeed );

        const usb_DataCnt_t rxCnt    = USB_DRD_GET_CHEP_RX_CNT( periphReg, chId );
        const usb_DataCnt_t spaceCnt = epContext->XferSize - epContext->XferCnt;
        usb_DataCnt_t       copyCnt  = rxCnt;

        if( rxCnt > spaceCnt )
        {
            copyCnt = spaceCnt;
        }

        if( 0u < copyCnt )
        {
            Usb_Pma_Read( &epContext->RxBuffer[ epContext->XferCnt ], epContext->PmaAddr, copyCnt );
        }
        else
        {
            /* Zero length packet */
        }

        epContext->XferCnt += copyCnt;
        chContext->ErrorCnt = 0u;

        USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, chId );

        if( ( epContext->MaxPacketSize <= rxCnt                 ) &&
            ( epContext->XferSize      >  epContext->XferCnt    )    )
        {
            /* Full packet and the buffer has space for more - next packet is requested */
            Usb_Host_Request_Packet( periphReg, chId, epContext );
        }
        else
        {
            Usb_Host_Finish( usbId, usbContext, chId, USB_EP_DIR_IN, USB_CHANNEL_RESULT_OK );
        }
    }
    else if( USB_CH_RX_NAK == statusRx )
    {
        if( USB_EP_TYPE_INTERRUPT == epContext->Type )
        {
            /* A NAKed interrupt transaction is not retried - the caller polls the endpoint again */
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_DIS );
            USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, chId );

            Usb_Host_Finish( usbId, usbContext, chId, USB_EP_DIR_IN, USB_CHANNEL_RESULT_NAK );
        }
        else
        {
            /* The controller retries the transaction at the start of the next frame */
            USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, chId );
        }
    }
    else if( USB_CH_RX_STALL == statusRx )
    {
        USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_DIS );
        USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, chId );

        Usb_Host_Finish( usbId, usbContext, chId, USB_EP_DIR_IN, USB_CHANNEL_RESULT_STALL );
    }
    else
    {
        /* Transaction is still running (valid status) */
        USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, chId );
    }
}


/**
 * \brief Handles the end of an OUT or SETUP transaction (transmission) of a channel
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param chId       [in]: Channel
 * \param chepValue  [in]: Channel register value read when the event was found
 */
static void Usb_Host_Handle_Out( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_RegValue_t chepValue )
{
    usb_EpContext_t * const        epContext = &usbContext->Ep[ chId ][ USB_EP_DIR_OUT ];
    usb_ChannelContext_t * const   chContext = &usbContext->Channel[ chId ];
    const usb_RegValue_t           statusTx  = chepValue & USB_CH_TX_STTX;

    if( USB_FUNCTION_INACTIVE == epContext->XferActive )
    {
        /* Event of a stopped transfer */
        USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, chId );
    }
    else if( 0u != ( chepValue & USB_CH_ERRTX ) )
    {
        Usb_Host_Handle_Error( usbId, usbContext, periphReg, chId, USB_EP_DIR_OUT, chepValue );
    }
    else if( USB_CH_TX_ACK_SBUF == statusTx )
    {
        USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, chId );

        epContext->XferCnt += epContext->PacketLen;
        chContext->ErrorCnt = 0u;

        if( epContext->XferCnt >= epContext->XferSize )
        {
            Usb_Host_Finish( usbId, usbContext, chId, USB_EP_DIR_OUT, USB_CHANNEL_RESULT_OK );
        }
        else
        {
            Usb_Host_Send_Packet( periphReg, chId, epContext, chContext );
        }
    }
    else if( USB_CH_TX_NAK == statusTx )
    {
        if( USB_EP_TYPE_INTERRUPT == epContext->Type )
        {
            /* A NAKed interrupt transaction is not retried - the caller sends the data again */
            USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_DIS );
            USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, chId );

            Usb_Host_Finish( usbId, usbContext, chId, USB_EP_DIR_OUT, USB_CHANNEL_RESULT_NAK );
        }
        else
        {
            /* The controller retries the transaction at the start of the next frame */
            USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, chId );
        }
    }
    else if( USB_CH_TX_STALL == statusTx )
    {
        USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_DIS );
        USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, chId );

        Usb_Host_Finish( usbId, usbContext, chId, USB_EP_DIR_OUT, USB_CHANNEL_RESULT_STALL );
    }
    else
    {
        /* Transaction is still running (valid status) */
        USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, chId );
    }
}


/**
 * \brief Handles a transaction ended by an error (no answer, CRC, bit stuffing)
 *
 * The error flags are cleared and the transaction is started again. After three consecutive errors (or when the
 * controller flags three consecutive errors) the channel is disabled and the transfer fails.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param chId       [in]: Channel
 * \param chDir      [in]: Direction of the failed transaction
 * \param chepValue  [in]: Channel register value read when the event was found
 */
static void Usb_Host_Handle_Error( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_EpDir_t chDir, usb_RegValue_t chepValue )
{
    usb_EpContext_t * const      epContext = &usbContext->Ep[ chId ][ chDir ];
    usb_ChannelContext_t * const chContext = &usbContext->Channel[ chId ];
    usb_RegValue_t               threeErr  = 0u;

    if( USB_EP_DIR_IN == chDir )
    {
        threeErr = ( chepValue >> USB_CHEP_THREE_ERR_RX_POS ) & USB_CHEP_THREE_ERR_WIDTH_MASK;

        USB_DRD_CLEAR_CHEP_RX_ERR( periphReg, chId );
    }
    else
    {
        threeErr = ( chepValue >> USB_CHEP_THREE_ERR_TX_POS ) & USB_CHEP_THREE_ERR_WIDTH_MASK;

        USB_DRD_CLEAR_CHEP_TX_ERR( periphReg, chId );
    }

    chContext->ErrorCnt ++;

    if( ( USB_HOST_ERROR_LIMIT <= chContext->ErrorCnt ) ||
        ( 0u                   != threeErr            )    )
    {
        chContext->ErrorCnt = 0u;

        if( USB_EP_DIR_IN == chDir )
        {
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_DIS );
        }
        else
        {
            USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_DIS );
        }

        Usb_Host_Finish( usbId, usbContext, chId, chDir, USB_CHANNEL_RESULT_ERROR );
    }
    else if( USB_EP_DIR_IN == chDir )
    {
        Usb_Host_Request_Packet( periphReg, chId, epContext );
    }
    else
    {
        Usb_Host_Send_Packet( periphReg, chId, epContext, chContext );
    }
}


/**
 * \brief Gives the next packet of a SETUP / OUT transfer to the controller
 *
 * Copies the next packet (maximal packet size or the rest of the data) to the packet memory, sets its
 * length, marks the transaction as SETUP (first packet of a SETUP transfer) and enables the transmission.
 *
 * \param periphReg [in]: USB controller registers
 * \param chId      [in]: Channel
 * \param epContext [in]: Context of the transmit buffer of the channel
 * \param chContext [in]: Context of the channel
 */
static void Usb_Host_Send_Packet( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_EpContext_t * const epContext, const usb_ChannelContext_t * const chContext )
{
    const usb_DataCnt_t restLen   = epContext->XferSize - epContext->XferCnt;
    usb_DataCnt_t       packetLen = restLen;

    if( restLen > epContext->MaxPacketSize )
    {
        packetLen = epContext->MaxPacketSize;
    }
    else
    {
        packetLen = restLen;
    }

    if( 0u < packetLen )
    {
        Usb_Pma_Write( &epContext->TxData[ epContext->XferCnt ], epContext->PmaAddr, packetLen );
    }
    else
    {
        /* Zero length packet */
    }

    USB_DRD_SET_CHEP_TX_CNT( periphReg, chId, packetLen );

    epContext->PacketLen = packetLen;

    if( USB_TOKEN_SETUP == chContext->Token )
    {
        Usb_Host_Set_SetupBit( periphReg, chId );
    }
    else
    {
        /* Plain OUT transaction */
    }

    USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_VALID );
}


/**
 * \brief Requests the next packet of an IN transfer from the device
 *
 * Sets the expected length of the packet (maximal packet size or the rest of the buffer) and enables the
 * reception.
 *
 * \param periphReg [in]: USB controller registers
 * \param chId      [in]: Channel
 * \param epContext [in]: Context of the receive buffer of the channel
 */
static void Usb_Host_Request_Packet( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId, usb_EpContext_t * const epContext )
{
    const usb_DataCnt_t restLen   = epContext->XferSize - epContext->XferCnt;
    usb_DataCnt_t       packetLen = restLen;

    if( restLen > epContext->MaxPacketSize )
    {
        packetLen = epContext->MaxPacketSize;
    }
    else
    {
        packetLen = restLen;
    }

    USB_DRD_SET_CHEP_RX_CNT( periphReg, chId, packetLen );

    epContext->PacketLen = packetLen;

    USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_VALID );
}


/**
 * \brief Finishes a running transfer of a channel and reports it by the channel callback
 *
 * The transfer is marked as not running before the callback is called, so the callback can start a new
 * transfer of the channel.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param chId       [in]: Channel
 * \param chDir      [in]: Direction of the transfer
 * \param chResult   [in]: Result of the transfer
 */
static void Usb_Host_Finish( usb_PeriphId_t usbId, usb_Context_t * const usbContext, usb_ChannelId_t chId, usb_EpDir_t chDir, usb_ChannelResult_t chResult )
{
    usb_ChannelCallback_t * const channelCallback = usbContext->Callbacks.ChannelCallback;
    const usb_DataCnt_t           xferCnt         = usbContext->Ep[ chId ][ chDir ].XferCnt;

    usbContext->Ep[ chId ][ chDir ].XferActive = USB_FUNCTION_INACTIVE;

    if( USB_NULL_PTR != channelCallback )
    {
        channelCallback( usbId, chId, xferCnt, chResult );
    }
    else
    {
        /* Channel callback is not used */
    }
}


/**
 * \brief Closes a channel: the channel is disabled, its register cleared and its buffers released
 *
 * A running transfer is cancelled without a callback.
 *
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param chId       [in]: Channel
 */
static void Usb_Host_Close_Channel( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId )
{
    USB_DRD_SET_CHEP_TX_STATUS( periphReg, chId, USB_CH_TX_DIS );
    USB_DRD_SET_CHEP_RX_STATUS( periphReg, chId, USB_CH_RX_DIS );

    /* Transfer type, addresses, flags and the setup request cleared (VT flags written with 1 do not change) */
    USB_DRD_SET_CHEP( periphReg, chId, ( USB_CH_VTRX | USB_CH_VTTX ) );

    Usb_Set_EpContextReset( &usbContext->Ep[ chId ][ USB_EP_DIR_OUT ] );
    Usb_Set_EpContextReset( &usbContext->Ep[ chId ][ USB_EP_DIR_IN  ] );

    usbContext->Channel[ chId ].ErrorCnt = 0u;
}


/**
 * \brief Closes all open channels
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param notify     [in]: Active - every running transfer is reported by the channel callback as aborted,
 *                         inactive - transfers are cancelled silently
 */
static void Usb_Host_Close_All( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_FlagState_t notify )
{
    for( usb_ChannelId_t chId = 0u; USB_EP_NUM_CNT > chId; chId ++ )
    {
        for( usb_DataCnt_t dirIdx = 0u; USB_EP_DIR_CNT > dirIdx; dirIdx ++ )
        {
            if( ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ dirIdx ].XferActive ) &&
                ( USB_FLAG_ACTIVE     == notify                                      )    )
            {
                Usb_Host_Finish( usbId, usbContext, chId, (usb_EpDir_t)dirIdx, USB_CHANNEL_RESULT_ABORTED );
            }
            else
            {
                usbContext->Ep[ chId ][ dirIdx ].XferActive = USB_FUNCTION_INACTIVE;
            }
        }

        if( ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_OUT ].Opened ) ||
            ( USB_FUNCTION_ACTIVE == usbContext->Ep[ chId ][ USB_EP_DIR_IN  ].Opened )    )
        {
            Usb_Host_Close_Channel( usbContext, periphReg, chId );
        }
        else
        {
            /* Channel is not open */
        }
    }
}


/**
 * \brief Marks the next transmitted transaction of a control channel as SETUP
 *
 * The SETUP flag of the channel register is cleared by the controller when the SETUP transaction is
 * acknowledged. Only the fields which do not toggle are written, so the status and the data toggles keep
 * their values.
 *
 * \param periphReg [in]: USB controller registers
 * \param chId      [in]: Channel
 */
static void Usb_Host_Set_SetupBit( USB_DRD_TypeDef * const periphReg, usb_ChannelId_t chId )
{
    const usb_RegValue_t regValue = USB_DRD_GET_CHEP( periphReg, chId ) & USB_CHEP_REG_MASK;

    USB_DRD_SET_CHEP( periphReg, chId, ( regValue | USB_CHEP_SETUP | USB_CH_VTRX | USB_CH_VTTX ) );
}


/**
 * \brief Waits until the controller finished the packet memory accesses of a received packet
 *
 * Device errata ES0561 section 2.15.1 (bug AB#1030): 800 ns in full speed mode, 6.4 us in low speed mode.
 *
 * \param periphReg [in]: USB controller registers
 * \param devSpeed  [in]: Speed of the device which sent the packet
 */
static void Usb_Host_Wait_Settle( const USB_DRD_TypeDef * const periphReg, usb_Speed_t devSpeed )
{
    usb_DataCnt_t waitCnt = 1u;

    if( USB_SPEED_LOW == devSpeed )
    {
        waitCnt = USB_PMA_SETTLE_LS_FACTOR;
    }
    else
    {
        waitCnt = 1u;
    }

    for( usb_DataCnt_t waitIdx = 0u; waitCnt > waitIdx; waitIdx ++ )
    {
        Usb_Pma_Wait_Settle( periphReg );
    }
}

#endif /* USB_DRD_FS */
