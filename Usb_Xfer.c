/**
 * \author Mr.Nobody
 * \file Usb_Xfer.c
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module data transfers
 *
 * Transfers of any size over the endpoints. The module splits a transfer to packets of the
 * maximal packet size of the endpoint, moves the packets between the application buffer and
 * the packet memory in the servicing context (interrupt / Usb_Task()) and reports the end of
 * the transfer by one callback. A transfer ends by a short packet or by the requested size
 * (no zero length packet is added, a zero length packet is a transfer of zero size).
 *
 * Control endpoint 0:
 * - SETUP packet aborts the transfers of the endpoint 0 and is reported by the SETUP callback,
 *   the endpoint answers NAK until the data or the status stage is started by the application.
 * - The endpoint receives the next SETUP packet after the status stage: the module makes the
 *   endpoint 0 ready after a finished IN transfer (if no OUT transfer is running) and after
 *   a zero length OUT transfer (status stage).
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

static void Usb_Xfer_Send_Packet       ( USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum, usb_EpContext_t * const epContext );
static void Usb_Xfer_Handle_Setup      ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static void Usb_Xfer_Handle_Rx         ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum, usb_RegValue_t chepValue );
static void Usb_Xfer_Handle_Tx         ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum );
static void Usb_Xfer_Apply_PendingAddr ( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Count of bits of a byte */
#define USB_BYTE_BITS                       ( 8u )

/** Index of the byte holding bmRequestType in the SETUP packet */
#define USB_SETUP_IDX_REQUEST_TYPE          ( 0u )

/** Index of the byte holding bRequest in the SETUP packet */
#define USB_SETUP_IDX_REQUEST               ( 1u )

/** Index of the low byte of wValue in the SETUP packet */
#define USB_SETUP_IDX_VALUE                 ( 2u )

/** Index of the low byte of wIndex in the SETUP packet */
#define USB_SETUP_IDX_INDEX                 ( 4u )

/** Index of the low byte of wLength in the SETUP packet */
#define USB_SETUP_IDX_LENGTH                ( 6u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Starts an IN transfer (device to host)
 *
 * The data are sent in packets of the maximal packet size of the endpoint, the last packet is
 * shorter if the size is not a multiple of the packet size. A transfer of zero size sends one
 * zero length packet. The end of the transfer (all data acknowledged by the host) is reported by
 * the transfer callback. The data have to stay valid until then.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr [in]: Endpoint address, IN direction
 * \param txData [in]: Data to send (can be USB_NULL_PTR if the size is zero)
 * \param txSize [in]: Count of bytes to send
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (endpoint is not initialized,
 *         stalled or busy, invalid parameter).
 */
usb_RequestState_t Usb_Set_EpTxStart( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, const usb_Data_t * const txData, usb_DataCnt_t txSize )
{
    usb_RequestState_t retState  = USB_REQUEST_ERROR;
    USB_DRD_TypeDef *  periphReg = USB_NULL_PTR;
    usb_EpContext_t *  epContext = USB_NULL_PTR;
    usb_FlagState_t    dataValid = USB_FLAG_INACTIVE;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Ep_Get_Context( usbId, epAddr, &epContext );
    }

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

    if( ( USB_REQUEST_OK        == retState                 ) &&
        ( USB_FLAG_ACTIVE       == dataValid                ) &&
        ( USB_EP_ADDR_IS_IN( epAddr )                       ) &&
        ( USB_FUNCTION_ACTIVE   == epContext->Opened        ) &&
        ( USB_FLAG_INACTIVE     == epContext->Stalled       ) &&
        ( USB_FUNCTION_INACTIVE == epContext->XferActive    )    )
    {
        epContext->TxData     = txData;
        epContext->XferSize   = txSize;
        epContext->XferCnt    = 0u;
        epContext->PacketLen  = 0u;
        epContext->XferActive = USB_FUNCTION_ACTIVE;

        Usb_Xfer_Send_Packet( periphReg, (usb_EpNum_t)USB_EP_ADDR_NUMBER( epAddr ), epContext );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts an OUT transfer (host to device)
 *
 * The received packets are stored to the buffer. The transfer ends when a short packet (shorter
 * than the maximal packet size, also a zero length packet) is received or the requested size is
 * received - the end is reported by the transfer callback with the count of received bytes. Data
 * of a packet which does not fit the buffer are dropped and the transfer ends with the overflow
 * result. The buffer has to stay valid until then.
 *
 * A transfer of zero size on the control endpoint 0 is the status stage of a control read transfer
 * (only a zero length packet is accepted).
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr   [in]: Endpoint address, OUT direction
 * \param rxBuffer [in]: Buffer for the received data (can be USB_NULL_PTR if the size is zero)
 * \param rxSize   [in]: Size of the buffer in bytes
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (endpoint is not initialized,
 *         stalled or busy, invalid parameter).
 */
usb_RequestState_t Usb_Set_EpRxStart( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize )
{
    usb_RequestState_t retState  = USB_REQUEST_ERROR;
    USB_DRD_TypeDef *  periphReg = USB_NULL_PTR;
    usb_EpContext_t *  epContext = USB_NULL_PTR;
    usb_FlagState_t    dataValid = USB_FLAG_INACTIVE;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Ep_Get_Context( usbId, epAddr, &epContext );
    }

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

    if( ( USB_REQUEST_OK        == retState                 ) &&
        ( USB_FLAG_ACTIVE       == dataValid                ) &&
        ( USB_FUNCTION_ACTIVE   == epContext->Opened        ) &&
        ( USB_FLAG_INACTIVE     == epContext->Stalled       ) &&
        ( USB_FUNCTION_INACTIVE == epContext->XferActive    )    )
    {
        if( USB_EP_ADDR_IS_IN( epAddr ) )
        {
            retState = USB_REQUEST_ERROR;
        }
        else
        {
            const usb_EpNum_t epNum = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epAddr );

            epContext->RxBuffer   = rxBuffer;
            epContext->XferSize   = rxSize;
            epContext->XferCnt    = 0u;
            epContext->PacketLen  = 0u;
            epContext->XferActive = USB_FUNCTION_ACTIVE;

            if( USB_EP_TYPE_CONTROL == epContext->Type )
            {
                if( 0u == rxSize )
                {
                    /* Status stage: only zero length data packets are accepted */
                    USB_DRD_SET_CHEP_KIND( periphReg, epNum );
                }
                else
                {
                    USB_DRD_CLEAR_CHEP_KIND( periphReg, epNum );
                }
            }
            else
            {
                /* The endpoint kind bit of other transfer types selects double buffering */
            }

            USB_DRD_SET_CHEP_RX_STATUS( periphReg, epNum, USB_EP_RX_VALID );
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops a running transfer of an endpoint
 *
 * The endpoint answers NAK to the host (the endpoint 0 stays ready to receive a SETUP packet) and
 * the transfer is cancelled without a callback. An endpoint without a running transfer is not
 * changed.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr [in]: Endpoint address (direction and number)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_EpXferStop( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
{
    usb_RequestState_t retState  = USB_REQUEST_ERROR;
    USB_DRD_TypeDef *  periphReg = USB_NULL_PTR;
    usb_EpContext_t *  epContext = USB_NULL_PTR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Ep_Get_Context( usbId, epAddr, &epContext );
    }

    if( ( USB_REQUEST_OK      == retState          ) &&
        ( USB_FUNCTION_ACTIVE == epContext->Opened )    )
    {
        if( USB_FUNCTION_ACTIVE == epContext->XferActive )
        {
            const usb_EpNum_t epNum = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epAddr );

            epContext->XferActive = USB_FUNCTION_INACTIVE;

            if( USB_EP_ADDR_IS_IN( epAddr ) )
            {
                USB_DRD_SET_CHEP_TX_STATUS( periphReg, epNum, USB_EP_TX_NAK );
            }
            else if( 0u == epNum )
            {
                Usb_Ep_Set_RxSetupReady( usbId );
            }
            else
            {
                USB_DRD_SET_CHEP_RX_STATUS( periphReg, epNum, USB_EP_RX_NAK );
            }
        }
        else
        {
            /* No transfer is running */
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the count of bytes transferred by the running or the last transfer of an endpoint
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr   [in]: Endpoint address (direction and number)
 * \param xferCnt [out]: Count of bytes sent (IN) / stored to the buffer (OUT)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_EpXferCount( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t * const xferCnt )
{
    usb_RequestState_t retState  = USB_REQUEST_ERROR;
    usb_EpContext_t *  epContext = USB_NULL_PTR;

    if( USB_NULL_PTR != xferCnt )
    {
        retState = Usb_Ep_Get_Context( usbId, epAddr, &epContext );
    }

    if( ( USB_REQUEST_OK      == retState          ) &&
        ( USB_FUNCTION_ACTIVE == epContext->Opened )    )
    {
        *xferCnt = epContext->XferCnt;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}

/* ====================== SERVICES OF THE MODULE FILES ====================== */

/**
 * \brief Services one correct transfer event of an endpoint
 *
 * Called by the controller servicing when the correct transfer flag is set. The controller presents
 * the endpoint and the direction of the oldest completed transaction (ISTR.IDN, ISTR.DIR), this
 * transaction is served. The controller presents the next pending transaction after the flag of the
 * served one is cleared, so the interrupt is raised again (interrupt servicing) or the function is
 * called again (\ref Usb_Task). If an endpoint has both transactions pending, the OUT / SETUP
 * transaction is presented first.
 *
 * \param usbId [in]: USB controller identification
 */
void Usb_Xfer_Service( usb_PeriphId_t usbId )
{
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;
    usb_RequestState_t ctxState   = USB_REQUEST_ERROR;
    usb_RequestState_t regState   = USB_REQUEST_ERROR;

    ctxState = Usb_Get_Context( usbId, &usbContext );
    regState = Usb_Get_PeriphReg( usbId, &periphReg );

    if( ( USB_REQUEST_OK == ctxState ) &&
        ( USB_REQUEST_OK == regState )    )
    {
        const usb_RegValue_t istrValue = periphReg->ISTR;
        const usb_EpNum_t    epNum     = (usb_EpNum_t)( istrValue & USB_ISTR_IDN );

        if( ( 0u != ( istrValue & USB_ISTR_CTR ) ) &&
            ( USB_EP_NUM_CNT > epNum             )    )
        {
            const usb_RegValue_t chepValue = USB_DRD_GET_CHEP( periphReg, epNum );

            if( 0u != ( istrValue & USB_ISTR_DIR ) )
            {
                /* OUT or SETUP transaction */
                if( 0u != ( chepValue & USB_EP_VTRX ) )
                {
                    Usb_Xfer_Handle_Rx( usbId, usbContext, periphReg, epNum, chepValue );
                }
                else
                {
                    /* Inconsistent event - the controller presents it again if it is still pending */
                }
            }
            else
            {
                /* IN transaction */
                if( 0u != ( chepValue & USB_EP_VTTX ) )
                {
                    Usb_Xfer_Handle_Tx( usbId, usbContext, periphReg, epNum );
                }
                else
                {
                    /* Inconsistent event - the controller presents it again if it is still pending */
                }
            }
        }
        else
        {
            /* No pending correct transfer event or invalid endpoint identification */
        }
    }
    else
    {
        /* Invalid controller identification */
    }
}


/**
 * \brief Finishes a running transfer and reports it by the transfer callback
 *
 * The transfer is marked as not running before the callback is called, so the callback can start
 * a new transfer of the endpoint.
 *
 * \param usbId      [in]: USB controller identification
 * \param epAddr     [in]: Endpoint address (direction and number)
 * \param xferResult [in]: Result of the transfer
 */
void Usb_Xfer_Finish( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_XferResult_t xferResult )
{
    usb_Context_t *    usbContext = USB_NULL_PTR;
    usb_EpContext_t *  epContext  = USB_NULL_PTR;
    usb_RequestState_t ctxState   = USB_REQUEST_ERROR;
    usb_RequestState_t epState    = USB_REQUEST_ERROR;

    ctxState = Usb_Get_Context( usbId, &usbContext );
    epState  = Usb_Ep_Get_Context( usbId, epAddr, &epContext );

    if( ( USB_REQUEST_OK == ctxState ) &&
        ( USB_REQUEST_OK == epState  )    )
    {
        usb_XferCallback_t * const xferCallback = usbContext->Callbacks.XferCallback;
        const usb_DataCnt_t        xferCnt      = epContext->XferCnt;

        epContext->XferActive = USB_FUNCTION_INACTIVE;

        if( USB_NULL_PTR != xferCallback )
        {
            xferCallback( usbId, epAddr, xferCnt, xferResult );
        }
        else
        {
            /* Transfer callback is not used */
        }
    }
    else
    {
        /* Invalid controller identification or endpoint address */
    }
}


/**
 * \brief Aborts the running transfers of all endpoints
 *
 * \param usbId  [in]: USB controller identification
 * \param notify [in]: Active - every aborted transfer is reported by the transfer callback with
 *                     the aborted result, inactive - transfers are cancelled silently
 */
void Usb_Xfer_Abort_All( usb_PeriphId_t usbId, usb_FlagState_t notify )
{
    usb_Context_t *    usbContext = USB_NULL_PTR;
    usb_RequestState_t ctxState   = USB_REQUEST_ERROR;

    ctxState = Usb_Get_Context( usbId, &usbContext );

    if( USB_REQUEST_OK == ctxState )
    {
        for( usb_EpNum_t epNum = 0u; USB_EP_NUM_CNT > epNum; epNum ++ )
        {
            for( usb_DataCnt_t dirIdx = 0u; USB_EP_DIR_CNT > dirIdx; dirIdx ++ )
            {
                usb_EpContext_t * const epContext = &usbContext->Ep[ epNum ][ dirIdx ];

                if( USB_FUNCTION_ACTIVE == epContext->XferActive )
                {
                    if( USB_FLAG_ACTIVE == notify )
                    {
                        usb_EpAddr_t epAddr = USB_EP_ADDR_OUT( epNum );

                        if( USB_EP_DIR_IN == dirIdx )
                        {
                            epAddr = USB_EP_ADDR_IN( epNum );
                        }

                        Usb_Xfer_Finish( usbId, epAddr, USB_XFER_RESULT_ABORTED );
                    }
                    else
                    {
                        epContext->XferActive = USB_FUNCTION_INACTIVE;
                    }
                }
                else
                {
                    /* No transfer is running */
                }
            }
        }
    }
    else
    {
        /* Invalid controller identification */
    }
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Gives the next packet of an IN transfer to the controller
 *
 * Copies the next packet (maximal packet size or the rest of the data) to the packet memory,
 * sets its length and enables the transmission.
 *
 * \param periphReg [in]: USB controller registers
 * \param epNum     [in]: Endpoint number
 * \param epContext [in]: Context of the IN endpoint
 */
static void Usb_Xfer_Send_Packet( USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum, usb_EpContext_t * const epContext )
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

    USB_DRD_SET_CHEP_TX_CNT( periphReg, epNum, packetLen );

    epContext->PacketLen = packetLen;

    USB_DRD_SET_CHEP_TX_STATUS( periphReg, epNum, USB_EP_TX_VALID );
}


/**
 * \brief Handles a SETUP packet received by the endpoint 0
 *
 * The packet is read from the packet memory before the event flag is cleared (the SETUP flag of
 * the endpoint register stays frozen while the correct transfer flag is set). Transfers of the
 * endpoint 0 and a pending device address are cancelled, the SETUP callback is called last.
 * A SETUP packet of an invalid length stalls the endpoint 0.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the USB controller
 * \param periphReg  [in]: USB controller registers
 */
static void Usb_Xfer_Handle_Setup( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    /* Device errata bug AB#1030 (ES0561 2.15.1): the packet memory is complete some time after the interrupt */
    Usb_Pma_Wait_Settle( periphReg );

    const usb_DataCnt_t setupLen = USB_DRD_GET_CHEP_RX_CNT( periphReg, 0u );

    if( USB_SETUP_BYTE_CNT == setupLen )
    {
        usb_Data_t setupBytes[ USB_SETUP_BYTE_CNT ];

        Usb_Pma_Read( setupBytes, usbContext->Ep[ 0u ][ USB_EP_DIR_OUT ].PmaAddr, USB_SETUP_BYTE_CNT );

        USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, 0u );

        usbContext->SetupPacket.RequestType = setupBytes[ USB_SETUP_IDX_REQUEST_TYPE ];
        usbContext->SetupPacket.Request     = setupBytes[ USB_SETUP_IDX_REQUEST ];
        usbContext->SetupPacket.Value       = (usb_ReqValue_t)( (usb_ReqValue_t)setupBytes[ USB_SETUP_IDX_VALUE ] |
                                                                ( (usb_ReqValue_t)setupBytes[ USB_SETUP_IDX_VALUE + 1u ] << USB_BYTE_BITS ) );
        usbContext->SetupPacket.Index       = (usb_ReqIndex_t)( (usb_ReqIndex_t)setupBytes[ USB_SETUP_IDX_INDEX ] |
                                                                ( (usb_ReqIndex_t)setupBytes[ USB_SETUP_IDX_INDEX + 1u ] << USB_BYTE_BITS ) );
        usbContext->SetupPacket.Length      = (usb_ReqLength_t)( (usb_ReqLength_t)setupBytes[ USB_SETUP_IDX_LENGTH ] |
                                                                 ( (usb_ReqLength_t)setupBytes[ USB_SETUP_IDX_LENGTH + 1u ] << USB_BYTE_BITS ) );
        usbContext->SetupValid              = USB_FLAG_ACTIVE;

        /* A SETUP packet ends the previous control transfer and the stall of the endpoint 0 */
        usbContext->PendingAddress                       = 0u;
        usbContext->Ep[ 0u ][ USB_EP_DIR_OUT ].Stalled   = USB_FLAG_INACTIVE;
        usbContext->Ep[ 0u ][ USB_EP_DIR_IN  ].Stalled   = USB_FLAG_INACTIVE;

        if( USB_FUNCTION_ACTIVE == usbContext->Ep[ 0u ][ USB_EP_DIR_OUT ].XferActive )
        {
            Usb_Xfer_Finish( usbId, USB_EP_ADDR_OUT( 0u ), USB_XFER_RESULT_ABORTED );
        }
        else
        {
            /* No OUT transfer of the endpoint 0 is running */
        }

        if( USB_FUNCTION_ACTIVE == usbContext->Ep[ 0u ][ USB_EP_DIR_IN ].XferActive )
        {
            Usb_Xfer_Finish( usbId, USB_EP_ADDR_IN( 0u ), USB_XFER_RESULT_ABORTED );
        }
        else
        {
            /* No IN transfer of the endpoint 0 is running */
        }

        if( USB_NULL_PTR != usbContext->Callbacks.SetupCallback )
        {
            usbContext->Callbacks.SetupCallback( usbId, &usbContext->SetupPacket );
        }
        else
        {
            /* SETUP callback is not used */
        }
    }
    else
    {
        /* Invalid SETUP packet length - both directions of the endpoint 0 are stalled */
        USB_DRD_SET_CHEP_RX_STATUS( periphReg, 0u, USB_EP_RX_STALL );
        USB_DRD_SET_CHEP_TX_STATUS( periphReg, 0u, USB_EP_TX_STALL );
        USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, 0u );

        usbContext->Ep[ 0u ][ USB_EP_DIR_OUT ].Stalled = USB_FLAG_ACTIVE;
        usbContext->Ep[ 0u ][ USB_EP_DIR_IN  ].Stalled = USB_FLAG_ACTIVE;

        if( USB_NULL_PTR != usbContext->Callbacks.ErrorCallback )
        {
            usbContext->Callbacks.ErrorCallback( usbId, USB_ERROR_SETUP_LENGTH );
        }
        else
        {
            /* Error callback is not used */
        }
    }
}


/**
 * \brief Handles a received packet (OUT / SETUP transaction) of an endpoint
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the USB controller
 * \param periphReg  [in]: USB controller registers
 * \param epNum      [in]: Endpoint number
 * \param chepValue  [in]: Endpoint register value read when the event was found
 */
static void Usb_Xfer_Handle_Rx( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum, usb_RegValue_t chepValue )
{
    usb_EpContext_t * const epContext = &usbContext->Ep[ epNum ][ USB_EP_DIR_OUT ];

    if( ( 0u == epNum                      ) &&
        ( 0u != ( chepValue & USB_EP_SETUP ) )    )
    {
        Usb_Xfer_Handle_Setup( usbId, usbContext, periphReg );
    }
    else
    {
        /* Device errata bug AB#1030 (ES0561 2.15.1): the packet memory is complete some time after the interrupt */
        Usb_Pma_Wait_Settle( periphReg );

        const usb_DataCnt_t rxCnt = USB_DRD_GET_CHEP_RX_CNT( periphReg, epNum );

        if( USB_FUNCTION_ACTIVE == epContext->XferActive )
        {
            const usb_DataCnt_t spaceCnt = epContext->XferSize - epContext->XferCnt;
            usb_DataCnt_t       copyCnt  = rxCnt;
            usb_XferResult_t    result   = USB_XFER_RESULT_OK;

            if( rxCnt > spaceCnt )
            {
                /* Data over the buffer size are dropped */
                copyCnt = spaceCnt;
                result  = USB_XFER_RESULT_OVERFLOW;
            }
            else
            {
                copyCnt = rxCnt;
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

            USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, epNum );

            if( ( USB_XFER_RESULT_OK         == result                    ) &&
                ( epContext->MaxPacketSize   <= rxCnt                     ) &&
                ( epContext->XferSize        >  epContext->XferCnt        )    )
            {
                /* Full packet and the buffer has space for more - next packet is received */
                USB_DRD_SET_CHEP_RX_STATUS( periphReg, epNum, USB_EP_RX_VALID );
            }
            else
            {
                if( ( 0u == epNum                ) &&
                    ( 0u == epContext->XferSize  )    )
                {
                    /* Status stage finished - the endpoint 0 receives the next SETUP packet */
                    Usb_Ep_Set_RxSetupReady( usbId );
                }
                else
                {
                    /* Data stage or other endpoint */
                }

                Usb_Xfer_Finish( usbId, USB_EP_ADDR_OUT( epNum ), result );
            }
        }
        else
        {
            /* Packet received without a running transfer (status stage of a control read transfer) */
            USB_DRD_CLEAR_RX_CHEP_CTR( periphReg, epNum );

            if( ( 0u == epNum ) &&
                ( 0u == rxCnt ) )
            {
                Usb_Ep_Set_RxSetupReady( usbId );
            }
            else
            {
                /* Endpoint answers NAK until a transfer is started */
            }
        }
    }
}


/**
 * \brief Handles a transmitted packet (IN transaction) of an endpoint
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the USB controller
 * \param periphReg  [in]: USB controller registers
 * \param epNum      [in]: Endpoint number
 */
static void Usb_Xfer_Handle_Tx( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum )
{
    usb_EpContext_t * const epContext = &usbContext->Ep[ epNum ][ USB_EP_DIR_IN ];

    USB_DRD_CLEAR_TX_CHEP_CTR( periphReg, epNum );

    if( USB_FUNCTION_ACTIVE == epContext->XferActive )
    {
        epContext->XferCnt += epContext->PacketLen;

        if( epContext->XferCnt >= epContext->XferSize )
        {
            if( 0u == epNum )
            {
                /* Status stage of SET_ADDRESS acknowledged - the address is valid from now */
                Usb_Xfer_Apply_PendingAddr( usbContext, periphReg );

                if( USB_FUNCTION_INACTIVE == usbContext->Ep[ 0u ][ USB_EP_DIR_OUT ].XferActive )
                {
                    /* The host can send only the status packet or the next SETUP packet now */
                    Usb_Ep_Set_RxSetupReady( usbId );
                }
                else
                {
                    /* OUT transfer of the endpoint 0 is running */
                }
            }
            else
            {
                /* Not the control endpoint */
            }

            Usb_Xfer_Finish( usbId, USB_EP_ADDR_IN( epNum ), USB_XFER_RESULT_OK );
        }
        else
        {
            Usb_Xfer_Send_Packet( periphReg, epNum, epContext );
        }
    }
    else
    {
        /* Event of a cancelled transfer */
    }
}


/**
 * \brief Applies the device address requested by Usb_Set_DeviceAddress()
 *
 * \param usbContext [in]: Context of the USB controller
 * \param periphReg  [in]: USB controller registers
 */
static void Usb_Xfer_Apply_PendingAddr( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    if( 0u != usbContext->PendingAddress )
    {
        const usb_RegValue_t addrValue = (usb_RegValue_t)usbContext->PendingAddress | USB_DADDR_EF;

        periphReg->DADDR = addrValue;

        for( usb_TimeoutCnt_t iterationCnt = 0u; USB_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            if( addrValue == periphReg->DADDR )
            {
                usbContext->DeviceAddress = usbContext->PendingAddress;
                usbContext->DeviceState   = USB_DEVICE_STATE_ADDRESSED;
                break;
            }
            else
            {
                /* Address not applied yet */
            }
        }

        usbContext->PendingAddress = 0u;
    }
    else
    {
        /* No address change requested */
    }
}

#endif /* USB_DRD_FS */
