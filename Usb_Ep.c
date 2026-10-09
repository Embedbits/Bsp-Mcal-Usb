/**
 * \author Mr.Nobody
 * \file Usb_Ep.c
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module endpoints
 *
 * Initialization, deinitialization, stall and state of the endpoints. Every endpoint number
 * of the USB_DRD_FS controller has one channel / endpoint register (CHEPnR) shared by both
 * directions (transfer type, address), separate status and data toggle fields per direction
 * and one packet buffer per direction in the packet memory.
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

static usb_RequestState_t  Usb_Ep_Check_Config     ( const usb_Context_t * const usbContext, const usb_EpConfig_t * const epConfig );
static usb_RequestState_t  Usb_Ep_Verify_Config    ( USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum, usb_EpDir_t epDir, usb_RegValue_t typeBits, usb_PmaAddr_t pmaAddr );
static usb_EpDir_t         Usb_Ep_Get_Direction    ( usb_EpAddr_t epAddr );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Endpoint address bits other than the direction and the endpoint number (reserved, shall be zero) */
#define USB_EP_ADDR_RESERVED_MASK           ( (usb_EpAddr_t)( ~( USB_EP_ADDR_DIR_IN | USB_EP_ADDR_NUM_MASK ) ) )

/** Minimal packet size of control and bulk endpoints at full speed */
#define USB_FS_MIN_PACKET_SIZE              ( 8u )

/** Count of allowed packet sizes of control and bulk endpoints */
#define USB_FS_PACKET_SIZE_CNT              ( 4u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** Transfer type bits of the endpoint register, indexed by \ref usb_EpType_t */
static const usb_RegValue_t usb_EpTypeRegLut[ USB_EP_TYPE_CNT ] =
{
    [USB_EP_TYPE_CONTROL]     = USB_EP_CONTROL,
    [USB_EP_TYPE_ISOCHRONOUS] = USB_EP_ISOCHRONOUS,
    [USB_EP_TYPE_BULK]        = USB_EP_BULK,
    [USB_EP_TYPE_INTERRUPT]   = USB_EP_INTERRUPT,
};

/** Allowed maximal packet sizes of control and bulk endpoints at full speed */
static const usb_EpMaxPacket_t usb_EpMaxPacketLut[ USB_FS_PACKET_SIZE_CNT ] =
{
    USB_FS_MIN_PACKET_SIZE,
    ( 2u * USB_FS_MIN_PACKET_SIZE ),
    ( 4u * USB_FS_MIN_PACKET_SIZE ),
    USB_FS_MAX_PACKET_SIZE,
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Initializes an endpoint
 *
 * Allocates the packet buffer of the endpoint, configures the endpoint register (transfer type,
 * address, buffer descriptor) and sets the endpoint to NAK (endpoint 0 receives SETUP packets).
 * Both directions of one endpoint number have the same transfer type and are initialized by
 * separate calls. Endpoints other than endpoint 0 are initialized after the host configures
 * the device (endpoint 0 is initialized by the module after each bus reset).
 *
 * \note  Control, bulk and interrupt endpoints with single buffering are supported, the
 *        isochronous transfer type and double buffering are rejected.
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epConfig [in]: Endpoint configuration
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Init_Endpoint( usb_PeriphId_t usbId, const usb_EpConfig_t * const epConfig )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Ep_Open( usbId, epConfig );
    }
    else
    {
        /* Controller is not initialized or not powered */
    }

    return ( retState );
}


/**
 * \brief Deinitializes an endpoint
 *
 * The endpoint is disabled (no answer to the host), a running transfer is cancelled without
 * a callback and the packet buffer is released.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr [in]: Endpoint address (direction and number)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Deinit_Endpoint( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
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

    if( ( USB_REQUEST_OK        == retState           ) &&
        ( USB_FUNCTION_ACTIVE   == epContext->Opened  )    )
    {
        const usb_EpNum_t epNum = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epAddr );

        if( USB_EP_ADDR_IS_IN( epAddr ) )
        {
            USB_DRD_CLEAR_TX_DTOG( periphReg, epNum );
            USB_DRD_SET_CHEP_TX_STATUS( periphReg, epNum, USB_EP_TX_DIS );
        }
        else
        {
            USB_DRD_CLEAR_RX_DTOG( periphReg, epNum );
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, epNum, USB_EP_RX_DIS );
        }

        Usb_Set_EpContextReset( epContext );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the state of an endpoint
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr   [in]: Endpoint address (direction and number)
 * \param epState [out]: Endpoint state (closed, idle, busy, stalled)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_EpState( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_EpState_t * const epState )
{
    usb_RequestState_t retState  = USB_REQUEST_ERROR;
    usb_EpContext_t *  epContext = USB_NULL_PTR;

    if( USB_NULL_PTR != epState )
    {
        retState = Usb_Ep_Get_Context( usbId, epAddr, &epContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        if( USB_FUNCTION_ACTIVE != epContext->Opened )
        {
            *epState = USB_EP_STATE_CLOSED;
        }
        else if( USB_FLAG_ACTIVE == epContext->Stalled )
        {
            *epState = USB_EP_STATE_STALLED;
        }
        else if( USB_FUNCTION_ACTIVE == epContext->XferActive )
        {
            *epState = USB_EP_STATE_BUSY;
        }
        else
        {
            *epState = USB_EP_STATE_IDLE;
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stalls an endpoint
 *
 * The endpoint answers STALL to the host until the stall is cleared by
 * \ref Usb_Set_EpStallInactive (halt of a bulk / interrupt endpoint) or, on the control
 * endpoint 0, by the next SETUP packet. A running transfer is cancelled without a callback.
 * Both directions of the endpoint 0 are stalled by separate calls.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr [in]: Endpoint address (direction and number)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_EpStallActive( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
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
        const usb_EpNum_t epNum = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epAddr );

        if( USB_EP_ADDR_IS_IN( epAddr ) )
        {
            USB_DRD_SET_CHEP_TX_STATUS( periphReg, epNum, USB_EP_TX_STALL );
        }
        else
        {
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, epNum, USB_EP_RX_STALL );
        }

        epContext->XferActive = USB_FUNCTION_INACTIVE;
        epContext->Stalled    = USB_FLAG_ACTIVE;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears the stall of an endpoint and resets its data toggle
 *
 * The data toggle is reset to DATA0 (the host resets its toggle by the ClearFeature(ENDPOINT_HALT)
 * request, also after SetConfiguration / SetInterface). A stalled endpoint answers NAK afterwards
 * (the endpoint 0 receives SETUP packets). The status of an endpoint which is not stalled is not
 * changed, only the data toggle is reset.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param epAddr [in]: Endpoint address (direction and number)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_EpStallInactive( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
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
        const usb_EpNum_t epNum = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epAddr );

        if( USB_EP_ADDR_IS_IN( epAddr ) )
        {
            USB_DRD_CLEAR_TX_DTOG( periphReg, epNum );

            if( USB_FLAG_ACTIVE == epContext->Stalled )
            {
                USB_DRD_SET_CHEP_TX_STATUS( periphReg, epNum, USB_EP_TX_NAK );
            }
            else
            {
                /* Endpoint is not stalled - status is kept */
            }
        }
        else
        {
            USB_DRD_CLEAR_RX_DTOG( periphReg, epNum );

            if( USB_FLAG_ACTIVE == epContext->Stalled )
            {
                if( 0u == epNum )
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
                /* Endpoint is not stalled - status is kept */
            }
        }

        epContext->Stalled = USB_FLAG_INACTIVE;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}

/* ====================== SERVICES OF THE MODULE FILES ====================== */

/**
 * \brief Returns the context of an endpoint direction
 *
 * \param usbId        [in]: USB controller identification
 * \param epAddr       [in]: Endpoint address (direction and number)
 * \param epContext   [out]: Context of the endpoint direction
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the endpoint address and the
 *         controller identification are valid, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Ep_Get_Context( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_EpContext_t ** const epContext )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( ( USB_NULL_PTR != epContext                                          ) &&
        ( 0u           == ( epAddr & USB_EP_ADDR_RESERVED_MASK )             ) &&
        ( USB_EP_NUM_CNT > USB_EP_ADDR_NUMBER( epAddr )                      )    )
    {
        retState = Usb_Get_Context( usbId, &usbContext );

        if( USB_REQUEST_OK == retState )
        {
            const usb_EpDir_t epDir = Usb_Ep_Get_Direction( epAddr );

            *epContext = &usbContext->Ep[ USB_EP_ADDR_NUMBER( epAddr ) ][ epDir ];
        }
        else
        {
            /* Invalid controller identification */
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Opens an endpoint direction (configuration check, packet buffer, endpoint registers)
 *
 * The controller shall be operational (checked by the caller).
 *
 * \param usbId    [in]: USB controller identification
 * \param epConfig [in]: Endpoint configuration
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Ep_Open( usb_PeriphId_t usbId, const usb_EpConfig_t * const epConfig )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;
    usb_EpContext_t *  epContext  = USB_NULL_PTR;
    usb_PmaAddr_t      pmaAddr    = 0u;

    if( USB_NULL_PTR != epConfig )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Ep_Get_Context( usbId, epConfig->Address, &epContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Ep_Check_Config( usbContext, epConfig );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Pma_Alloc( usbContext, Usb_Pma_Get_BufferSize( epConfig->MaxPacketSize ), &pmaAddr );
    }

    if( USB_REQUEST_OK == retState )
    {
        const usb_EpNum_t    epNum    = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epConfig->Address );
        const usb_EpDir_t    epDir    = Usb_Ep_Get_Direction( epConfig->Address );
        const usb_RegValue_t typeBits = usb_EpTypeRegLut[ epConfig->Type ];
        usb_RegValue_t       regValue = USB_DRD_GET_CHEP( periphReg, epNum ) & USB_EP_T_MASK;

        /* Transfer type and endpoint address (the other direction of the endpoint number keeps its status) */
        regValue |= typeBits;

        USB_DRD_SET_CHEP( periphReg, epNum, regValue | USB_EP_VTRX | USB_EP_VTTX );
        USB_DRD_SET_CHEP_ADDRESS( periphReg, epNum, epNum );

        if( USB_EP_DIR_IN == epDir )
        {
            USB_DRD_SET_CHEP_TX_ADDRESS( periphReg, epNum, pmaAddr );
            USB_DRD_CLEAR_TX_DTOG( periphReg, epNum );
            USB_DRD_SET_CHEP_TX_STATUS( periphReg, epNum, USB_EP_TX_NAK );
        }
        else
        {
            USB_DRD_SET_CHEP_RX_ADDRESS( periphReg, epNum, pmaAddr );
            USB_DRD_SET_CHEP_RX_CNT( periphReg, epNum, epConfig->MaxPacketSize );
            USB_DRD_CLEAR_RX_DTOG( periphReg, epNum );
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, epNum, USB_EP_RX_NAK );
        }

        retState = Usb_Ep_Verify_Config( periphReg, epNum, epDir, typeBits, pmaAddr );

        if( USB_REQUEST_OK == retState )
        {
            Usb_Set_EpContextReset( epContext );

            epContext->Opened        = USB_FUNCTION_ACTIVE;
            epContext->Type          = epConfig->Type;
            epContext->MaxPacketSize = epConfig->MaxPacketSize;
            epContext->PmaAddr       = pmaAddr;
            epContext->PmaSize       = Usb_Pma_Get_BufferSize( epConfig->MaxPacketSize );

            if( ( 0u         == epNum         ) &&
                ( USB_EP_DIR_OUT == epDir     )    )
            {
                /* Endpoint 0 receives SETUP packets */
                Usb_Ep_Set_RxSetupReady( usbId );
            }
            else
            {
                /* Endpoint answers NAK until a transfer is started */
            }
        }
        else
        {
            /* Endpoint registers were not configured */
        }
    }
    else
    {
        /* Invalid configuration or no free packet memory */
    }

    return ( retState );
}


/**
 * \brief Opens the control endpoint 0 (both directions)
 *
 * \param usbId [in]: USB controller identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Ep_Open_Control0( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    usb_EpConfig_t     epConfig;

    retState = Usb_Get_Context( usbId, &usbContext );

    if( USB_REQUEST_OK == retState )
    {
        epConfig.Address       = USB_EP_ADDR_OUT( 0u );
        epConfig.Type          = USB_EP_TYPE_CONTROL;
        epConfig.MaxPacketSize = usbContext->Ep0MaxPacketSize;
        epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;

        retState = Usb_Ep_Open( usbId, &epConfig );
    }

    if( USB_REQUEST_OK == retState )
    {
        epConfig.Address = USB_EP_ADDR_IN( 0u );

        retState = Usb_Ep_Open( usbId, &epConfig );
    }

    return ( retState );
}


/**
 * \brief Makes the endpoint 0 ready to receive a SETUP packet or a zero length status packet
 *
 * The endpoint 0 receives (status OUT: only zero length data packets besides the SETUP packets
 * are accepted, longer data packets are answered by STALL). Called when a control transfer is
 * finished and the host can send only a SETUP packet next.
 *
 * \param usbId [in]: USB controller identification
 */
void Usb_Ep_Set_RxSetupReady( usb_PeriphId_t usbId )
{
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;
    usb_EpContext_t *  epContext  = USB_NULL_PTR;
    usb_RequestState_t regState   = USB_REQUEST_ERROR;
    usb_RequestState_t epState    = USB_REQUEST_ERROR;

    regState = Usb_Get_PeriphReg( usbId, &periphReg );
    epState  = Usb_Ep_Get_Context( usbId, USB_EP_ADDR_OUT( 0u ), &epContext );

    if( ( USB_REQUEST_OK == regState ) &&
        ( USB_REQUEST_OK == epState  )    )
    {
        if( USB_FUNCTION_ACTIVE == epContext->Opened )
        {
            USB_DRD_SET_CHEP_KIND( periphReg, 0u );
            USB_DRD_SET_CHEP_RX_STATUS( periphReg, 0u, USB_EP_RX_VALID );
        }
        else
        {
            /* Endpoint 0 is not initialized */
        }
    }
    else
    {
        /* Invalid controller identification */
    }
}


/**
 * \brief Closes all endpoints of the controller after a bus reset
 *
 * The endpoint registers were reset by the controller, so only the contexts are cleared.
 * Running transfers are reported as aborted by the transfer callback.
 *
 * \param usbId [in]: USB controller identification
 */
void Usb_Ep_Close_All( usb_PeriphId_t usbId )
{
    usb_Context_t *    usbContext = USB_NULL_PTR;
    usb_RequestState_t ctxState   = USB_REQUEST_ERROR;

    ctxState = Usb_Get_Context( usbId, &usbContext );

    if( USB_REQUEST_OK == ctxState )
    {
        Usb_Xfer_Abort_All( usbId, USB_FLAG_ACTIVE );

        for( usb_EpNum_t epNum = 0u; USB_EP_NUM_CNT > epNum; epNum ++ )
        {
            for( usb_DataCnt_t dirIdx = 0u; USB_EP_DIR_CNT > dirIdx; dirIdx ++ )
            {
                Usb_Set_EpContextReset( &usbContext->Ep[ epNum ][ dirIdx ] );
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
 * \brief Returns direction of an endpoint address
 *
 * \param epAddr [in]: Endpoint address
 *
 * \return Direction index of the endpoint contexts
 */
static usb_EpDir_t Usb_Ep_Get_Direction( usb_EpAddr_t epAddr )
{
    usb_EpDir_t epDir = USB_EP_DIR_OUT;

    if( USB_EP_ADDR_IS_IN( epAddr ) )
    {
        epDir = USB_EP_DIR_IN;
    }
    else
    {
        epDir = USB_EP_DIR_OUT;
    }

    return ( epDir );
}


/**
 * \brief Checks if the maximal packet size is valid for the transfer type at full speed
 *
 * \param epType        [in]: Endpoint transfer type
 * \param maxPacketSize [in]: Maximal packet size in bytes
 *
 * \return Active if the packet size is allowed (control, bulk: 8, 16, 32, 64; interrupt: 1 - 64)
 */
usb_FlagState_t Usb_Ep_Is_MaxPacketValid( usb_EpType_t epType, usb_EpMaxPacket_t maxPacketSize )
{
    usb_FlagState_t sizeValid = USB_FLAG_INACTIVE;

    if( USB_EP_TYPE_INTERRUPT == epType )
    {
        if( ( 0u                      < maxPacketSize ) &&
            ( USB_FS_MAX_PACKET_SIZE >= maxPacketSize )    )
        {
            sizeValid = USB_FLAG_ACTIVE;
        }
        else
        {
            sizeValid = USB_FLAG_INACTIVE;
        }
    }
    else
    {
        for( usb_DataCnt_t sizeIdx = 0u; USB_FS_PACKET_SIZE_CNT > sizeIdx; sizeIdx ++ )
        {
            if( usb_EpMaxPacketLut[ sizeIdx ] == maxPacketSize )
            {
                sizeValid = USB_FLAG_ACTIVE;
            }
            else
            {
                /* Not this size */
            }
        }
    }

    return ( sizeValid );
}


/**
 * \brief Checks the configuration of an endpoint against the capabilities of the controller
 *
 * \param usbContext [in]: Context of the USB controller
 * \param epConfig   [in]: Endpoint configuration
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the endpoint can be initialized.
 */
static usb_RequestState_t Usb_Ep_Check_Config( const usb_Context_t * const usbContext, const usb_EpConfig_t * const epConfig )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( ( USB_EP_TYPE_CNT      >  epConfig->Type          ) &&
        ( USB_EP_BUFFER_CNT    >  epConfig->BufferMode    ) &&
        ( USB_EP_TYPE_ISOCHRONOUS != epConfig->Type       ) &&
        ( USB_EP_BUFFER_SINGLE == epConfig->BufferMode    ) &&
        ( 0u == ( epConfig->Address & USB_EP_ADDR_RESERVED_MASK ) ) &&
        ( USB_EP_NUM_CNT       >  USB_EP_ADDR_NUMBER( epConfig->Address ) )    )
    {
        const usb_EpNum_t     epNum      = (usb_EpNum_t)USB_EP_ADDR_NUMBER( epConfig->Address );
        const usb_EpDir_t     epDir      = Usb_Ep_Get_Direction( epConfig->Address );
        const usb_EpContext_t * thisCtx  = &usbContext->Ep[ epNum ][ epDir ];
        const usb_EpContext_t * otherCtx = USB_NULL_PTR;
        const usb_FlagState_t sizeValid  = Usb_Ep_Is_MaxPacketValid( epConfig->Type, epConfig->MaxPacketSize );

        if( USB_EP_DIR_IN == epDir )
        {
            otherCtx = &usbContext->Ep[ epNum ][ USB_EP_DIR_OUT ];
        }
        else
        {
            otherCtx = &usbContext->Ep[ epNum ][ USB_EP_DIR_IN ];
        }

        retState = USB_REQUEST_OK;

        if( USB_FLAG_ACTIVE != sizeValid )
        {
            retState = USB_REQUEST_ERROR;
        }
        else if( USB_FUNCTION_ACTIVE == thisCtx->Opened )
        {
            /* Endpoint direction is initialized already */
            retState = USB_REQUEST_ERROR;
        }
        else if( ( USB_FUNCTION_ACTIVE == otherCtx->Opened ) &&
                 ( epConfig->Type      != otherCtx->Type   )    )
        {
            /* Both directions of an endpoint number share the transfer type */
            retState = USB_REQUEST_ERROR;
        }
        else if( ( 0u                  == epNum            ) &&
                 ( USB_EP_TYPE_CONTROL != epConfig->Type   )    )
        {
            /* Endpoint 0 is the control endpoint */
            retState = USB_REQUEST_ERROR;
        }
        else
        {
            /* Configuration is valid */
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Verifies by read-back the transfer type, the address and the buffer address of an endpoint
 *
 * The status and data toggle fields of the endpoint register are changed by the controller during
 * the transfers, so they are not verified (they are verified by the transfer events).
 *
 * \param periphReg [in]: USB controller registers
 * \param epNum     [in]: Endpoint number
 * \param epDir     [in]: Endpoint direction
 * \param typeBits  [in]: Expected transfer type bits of the endpoint register
 * \param pmaAddr   [in]: Expected address of the packet buffer
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the endpoint is configured as required.
 */
static usb_RequestState_t Usb_Ep_Verify_Config( USB_DRD_TypeDef * const periphReg, usb_EpNum_t epNum, usb_EpDir_t epDir, usb_RegValue_t typeBits, usb_PmaAddr_t pmaAddr )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    for( usb_TimeoutCnt_t iterationCnt = 0u; USB_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const usb_RegValue_t chepValue = USB_DRD_GET_CHEP( periphReg, epNum );
        usb_RegValue_t       descValue = 0u;

        if( USB_EP_DIR_IN == epDir )
        {
            descValue = (USB_DRD_PMA_BUFF + epNum)->TXBD;
        }
        else
        {
            descValue = (USB_DRD_PMA_BUFF + epNum)->RXBD;
        }

        if( ( typeBits == ( chepValue & USB_EP_UTYPE )           ) &&
            ( epNum    == ( chepValue & USB_CHEP_ADDR )          ) &&
            ( pmaAddr  == ( descValue & USB_PMA_DESC_ADDR_MASK ) )    )
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

#endif /* USB_DRD_FS */
