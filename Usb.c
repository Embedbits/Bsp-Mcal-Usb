/**
 * \author Mr.Nobody
 * \file Usb.c
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module common functionality
 *
 * Device and host mode of the USB_DRD_FS controller: initialization, device control (connect,
 * address, remote wakeup) and servicing of the bus events (reset, suspend, resume, start of frame,
 * errors) and of the endpoint transfer events by the interrupt or by Usb_Task().
 * The endpoints are handled by Usb_Ep.c, the data transfers by Usb_Xfer.c and the packet
 * memory by Usb_Pma.c. The host mode (port, channels, transfers) is in Usb_Host.c.
 *
 * \note Register write verification (read-back within a timeout): the control, device address and
 *       battery charging registers and the transfer type, address and buffer address of the endpoints
 *       are verified. Exceptions (changed by the controller or write-1-to-toggle / write-0-to-clear
 *       fields): status and data toggle fields of the endpoint registers (verified by the transfer
 *       events), valid transfer flags of the endpoint registers, flags of the interrupt status register
 *       (event flags), packet memory data.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usb.h"                            /* Self include                   */
#include "Usb_Port.h"                       /* Own port file include          */
#include "Usb_Types.h"                      /* Module types definitions       */
#include "Stm32_usb.h"                      /* USB RAL functionality          */
#include "Gpio_Port.h"                      /* GPIO handler functionality     */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
/* ============================== TYPEDEFS ================================== */

#if defined(USB_DRD_FS)

/** Structure type used for the USB controllers configuration array */
typedef struct
{
    USB_DRD_TypeDef *    PeriphReg;  /**< Controller registers                */
    nvic_PeriphIrqList_t PeriphNvic; /**< Controller NVIC configuration ID    */
    nvic_IsrCallback_t   PeriphIsr;  /**< Controller ISR callback routine     */
}   usb_PeriphConfigStruct_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static void                Usb_Fs_IsrHandler      ( void );
static void                Usb_Service            ( usb_PeriphId_t usbId );
static void                Usb_Handle_Reset       ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static void                Usb_Handle_Suspend     ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static void                Usb_Handle_Wakeup      ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static void                Usb_Handle_Sof         ( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );


static usb_RequestState_t  Usb_Init_Resources     ( usb_PeriphId_t usbId, const usb_BusConfig_t * const usbConfig );
static usb_RequestState_t  Usb_Init_Controller    ( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg );
static usb_RequestState_t  Usb_Init_Interrupt     ( usb_PeriphId_t usbId, usb_IrqPrio_t irqPriority );
static usb_RequestState_t  Usb_Release_Resources  ( usb_PeriphId_t usbId );
static usb_RequestState_t  Usb_Init_Gpio          ( usb_RegValue_t pinId );
static usb_RequestState_t  Usb_Check_PoweredMode  ( usb_PeriphId_t usbId, usb_Mode_t mode );

#endif /* USB_DRD_FS */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define USB_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define USB_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define USB_PATCH_VERSION           ( 0u )

/** Default maximal packet size of the control endpoint 0 */
#define USB_DEFAULT_EP0_MAX_PACKET  ( 64u )

/** Default priority of the USB interrupt */
#define USB_DEFAULT_IRQ_PRIORITY    ( 0u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

#if defined(USB_DRD_FS)

/** USB controllers configuration array */
static const usb_PeriphConfigStruct_t usb_PeriphConf[ USB_PERIPH_CNT ] =
{
    [USB_PERIPH_FS] = { .PeriphReg = USB_DRD_FS, .PeriphNvic = NVIC_PERIPH_IRQ_USB_DRD_FS, .PeriphIsr = Usb_Fs_IsrHandler },
};

/** RCC identification of the kernel clock sources, indexed by \ref usb_ClkSource_t */
static const rcc_PeriphId_t usb_ClkSourceLut[ USB_CLK_SOURCE_CNT ] =
{
    [USB_CLK_SOURCE_HSI48] = RCC_PERIPH_USB_HSI48,
    [USB_CLK_SOURCE_PLL1Q] = RCC_PERIPH_USB_PLL1Q,
#if defined(RCC_CR_PLL3ON)
    [USB_CLK_SOURCE_PLL3Q] = RCC_PERIPH_USB_PLL3Q,
#else
    [USB_CLK_SOURCE_PLL2Q] = RCC_PERIPH_USB_PLL2Q,
#endif
};

/** Runtime contexts of the USB controllers */
static usb_Context_t usb_Context[ USB_PERIPH_CNT ];

#endif /* USB_DRD_FS */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
usb_ModuleVersion_t Usb_Get_ModuleVersion( void )
{
    usb_ModuleVersion_t retVersion;

    retVersion.Major = USB_MAJOR_VERSION;
    retVersion.Minor = USB_MINOR_VERSION;
    retVersion.Patch = USB_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initialization of the configuration structure to default values.
 *
 * Default configuration: full-speed controller in device mode, HSI48 kernel clock, interrupt
 * servicing with priority 0, endpoint 0 of 64 bytes, no callbacks, pins PA11 (DM) and PA12 (DP).
 *
 * \param usbConfig [out]: Pointer to the configuration structure
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_DefaultConfig( usb_BusConfig_t * const usbConfig )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( USB_NULL_PTR != usbConfig )
    {
        usbConfig->PeriphId         = USB_PERIPH_FS;
        usbConfig->Mode             = USB_MODE_DEVICE;
        usbConfig->ClkSource        = USB_CLK_SOURCE_HSI48;
        usbConfig->ServiceMode      = USB_SERVICE_MODE_ISR;
        usbConfig->IrqPriority      = USB_DEFAULT_IRQ_PRIORITY;
        usbConfig->Ep0MaxPacketSize = USB_DEFAULT_EP0_MAX_PACKET;
        usbConfig->Callbacks        = USB_NULL_PTR;
#ifdef GPIOA
        usbConfig->DmPin            = USB_DM_PIN_PA11;
        usbConfig->DpPin            = USB_DP_PIN_PA12;
#else
        usbConfig->DmPin            = USB_DM_PIN_UNUSED;
        usbConfig->DpPin            = USB_DP_PIN_UNUSED;
#endif

        retState = USB_REQUEST_OK;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}

#if defined(USB_DRD_FS)

/**
 * \brief Initializes the USB controller
 *
 * A previous initialization of the controller is released first. The function activates the
 * clocks and the VDDUSB supply (RCC module), configures the DM / DP pins (GPIO module), resets
 * and powers up the controller, enables its interrupts and connects the interrupt handler (NVIC
 * module, interrupt servicing). The device is not connected to the bus (DP pull-up is off) - the
 * application connects it by \ref Usb_Set_DeviceConnect after the endpoint 0 callbacks are ready.
 * The endpoint 0 is initialized by the module after each bus reset.
 *
 * With the HSI48 kernel clock the automatic trimming of the oscillator by the USB start of frame
 * is activated (RCC module).
 *
 * Host mode (\ref USB_MODE_HOST): the controller is powered up in the host mode with the DP / DM
 * pull-down resistors, the bus is idle (no reset signaling, no start of frame) and the attachment of
 * a device is reported by the connect callback. The VBUS supply of the port is not controlled by the
 * module. The host mode functions are \ref Usb_Get_HostPortState, \ref Usb_Set_HostPortResetActive and
 * the channel functions \ref Usb_Init_Channel .. \ref Usb_Get_ChannelXferCount.
 *
 * \param usbConfig [in]: Pointer to the configuration structure. Must not be NULL.
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Init( usb_BusConfig_t * const usbConfig )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;
    usb_FlagState_t    ep0Valid = USB_FLAG_INACTIVE;

    if( USB_NULL_PTR != usbConfig )
    {
        ep0Valid = Usb_Ep_Is_MaxPacketValid( USB_EP_TYPE_CONTROL, usbConfig->Ep0MaxPacketSize );
    }

    if( ( USB_NULL_PTR        != usbConfig                  ) &&
        ( USB_FLAG_ACTIVE     == ep0Valid                   ) &&
        ( USB_PERIPH_CNT      >  usbConfig->PeriphId        ) &&
        ( USB_MODE_CNT        >  usbConfig->Mode            ) &&
        ( USB_CLK_SOURCE_CNT  >  usbConfig->ClkSource       ) &&
        ( USB_SERVICE_MODE_CNT > usbConfig->ServiceMode     )    )
    {
        usb_Context_t *   usbContext = USB_NULL_PTR;
        USB_DRD_TypeDef * periphReg  = USB_NULL_PTR;
        const usb_PeriphId_t usbId   = usbConfig->PeriphId;

        retState = Usb_Get_Context( usbId, &usbContext );

        if( USB_REQUEST_OK == retState )
        {
            retState = Usb_Get_PeriphReg( usbId, &periphReg );
        }

        /*---------------- Previous initialization of the controller ----------------*/
        if( ( USB_REQUEST_OK      == retState                ) &&
            ( USB_FUNCTION_ACTIVE == usbContext->InitState   )    )
        {
            retState = Usb_Deinit( usbId );
        }

        /*---------------------------- Module context ------------------------------*/
        if( USB_REQUEST_OK == retState )
        {
            for( usb_EpNum_t epNum = 0u; USB_EP_NUM_CNT > epNum; epNum ++ )
            {
                for( usb_DataCnt_t dirIdx = 0u; USB_EP_DIR_CNT > dirIdx; dirIdx ++ )
                {
                    Usb_Set_EpContextReset( &usbContext->Ep[ epNum ][ dirIdx ] );
                }
            }

            for( usb_ChannelId_t chId = 0u; USB_EP_NUM_CNT > chId; chId ++ )
            {
                usbContext->Channel[ chId ].Address    = 0u;
                usbContext->Channel[ chId ].DevAddress = 0u;
                usbContext->Channel[ chId ].DevSpeed   = USB_SPEED_FULL;
                usbContext->Channel[ chId ].Token      = USB_TOKEN_OUT;
                usbContext->Channel[ chId ].ErrorCnt   = 0u;
            }

            usbContext->Mode               = usbConfig->Mode;
            usbContext->ServiceMode        = usbConfig->ServiceMode;
            usbContext->ClkSource          = usbConfig->ClkSource;
            usbContext->Ep0MaxPacketSize   = usbConfig->Ep0MaxPacketSize;
            usbContext->TrimActive         = USB_FUNCTION_INACTIVE;
            usbContext->DeviceState        = USB_DEVICE_STATE_DISCONNECTED;
            usbContext->StateBeforeSuspend = USB_DEVICE_STATE_DISCONNECTED;
            usbContext->DeviceAddress      = 0u;
            usbContext->PendingAddress     = 0u;
            usbContext->SetupValid         = USB_FLAG_INACTIVE;
            usbContext->HostPortState      = USB_HOST_PORT_DISCONNECTED;
            usbContext->HostDevSpeed       = USB_SPEED_FULL;

            usbContext->Callbacks.ResetCallback      = USB_NULL_PTR;
            usbContext->Callbacks.SuspendCallback    = USB_NULL_PTR;
            usbContext->Callbacks.ResumeCallback     = USB_NULL_PTR;
            usbContext->Callbacks.SofCallback        = USB_NULL_PTR;
            usbContext->Callbacks.SetupCallback      = USB_NULL_PTR;
            usbContext->Callbacks.XferCallback       = USB_NULL_PTR;
            usbContext->Callbacks.ErrorCallback      = USB_NULL_PTR;
            usbContext->Callbacks.ConnectCallback    = USB_NULL_PTR;
            usbContext->Callbacks.DisconnectCallback = USB_NULL_PTR;
            usbContext->Callbacks.ChannelCallback    = USB_NULL_PTR;

            if( USB_NULL_PTR != usbConfig->Callbacks )
            {
                usbContext->Callbacks = *usbConfig->Callbacks;
            }
            else
            {
                /* Event callbacks are not used */
            }
        }

        /*------------- Clocks, supply, trimming and pins (other modules) -----------*/
        if( USB_REQUEST_OK == retState )
        {
            retState = Usb_Init_Resources( usbId, usbConfig );
        }

        /*---------------------------- Controller ----------------------------------*/
        if( ( USB_REQUEST_OK == retState             ) &&
            ( USB_MODE_HOST  == usbConfig->Mode      )    )
        {
            retState = Usb_Host_Init_Controller( usbContext, periphReg );
        }
        else if( USB_REQUEST_OK == retState )
        {
            retState = Usb_Init_Controller( usbContext, periphReg );
        }
        else
        {
            /* Initialization failed */
        }

        /*---------------------------- Interrupt -----------------------------------*/
        if( ( USB_REQUEST_OK        == retState                    ) &&
            ( USB_SERVICE_MODE_ISR  == usbConfig->ServiceMode      )    )
        {
            retState = Usb_Init_Interrupt( usbId, usbConfig->IrqPriority );
        }
        else
        {
            /* Initialization failed or the controller is serviced by Usb_Task() */
        }

        if( USB_REQUEST_OK == retState )
        {
            usbContext->InitState = USB_FUNCTION_ACTIVE;
        }
        else
        {
            /* Resources taken by the failed initialization are released (error is already reported) */
            (void)Usb_Release_Resources( usbId );
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes the USB controller
 *
 * The device is disconnected from the bus, the interrupt is disabled, the controller is stopped
 * and powered down, its reset is activated and the clocks and the supply are released. All steps
 * are executed, any failure is reported.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (also if the controller is not initialized).
 */
usb_RequestState_t Usb_Deinit( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    retState = Usb_Get_Context( usbId, &usbContext );

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   )    )
    {
        retState = Usb_Release_Resources( usbId );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of the USB module
 *
 * This function shall be called in the main loop of the application or the task scheduler
 * for the controllers serviced by polling (\ref USB_SERVICE_MODE_POLL). It handles the bus events
 * and the transfers and calls the callbacks, so it has to be called often enough to answer the
 * host in time (the SETUP packets of the enumeration within 50 ms, start of frame callback every 1 ms).
 */
void Usb_Task( void )
{
    for( usb_PeriphId_t usbId = (usb_PeriphId_t)0u; USB_PERIPH_CNT > usbId; usbId ++ )
    {
        usb_Context_t *    usbContext = USB_NULL_PTR;
        usb_RequestState_t ctxState   = USB_REQUEST_ERROR;

        ctxState = Usb_Get_Context( usbId, &usbContext );

        if( ( USB_REQUEST_OK        == ctxState                       ) &&
            ( USB_FUNCTION_ACTIVE   == usbContext->InitState          ) &&
            ( USB_SERVICE_MODE_POLL == usbContext->ServiceMode        )    )
        {
            USB_DRD_TypeDef * periphReg = USB_NULL_PTR;
            usb_DataCnt_t     loopCnt   = 0u;

            /* One call serves one transfer event - events pending meanwhile are served in the same task run */
            (void)Usb_Get_PeriphReg( usbId, &periphReg );

            Usb_Service( usbId );

            while( ( USB_CTR_LOOP_MAX >  loopCnt                         ) &&
                   ( 0u              != ( periphReg->ISTR & USB_ISTR_CTR ) )    )
            {
                Usb_Service( usbId );
                loopCnt ++;
            }
        }
        else
        {
            /* Controller is not serviced by polling */
        }
    }
}


/**
 * \brief Connects the device to the bus (DP pull-up)
 *
 * The host detects the device and starts the enumeration by a bus reset.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_DeviceConnect( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        periphReg->BCDR |= USB_BCDR_DPPU;

        retState = Usb_Verify_Reg( &periphReg->BCDR, USB_BCDR_DPPU, USB_BCDR_DPPU );
    }

    if( ( USB_REQUEST_OK                == retState                ) &&
        ( USB_DEVICE_STATE_DISCONNECTED == usbContext->DeviceState )    )
    {
        usbContext->DeviceState = USB_DEVICE_STATE_CONNECTED;
    }
    else
    {
        /* Connection failed or the device is connected already */
    }

    return ( retState );
}


/**
 * \brief Disconnects the device from the bus (DP pull-up off)
 *
 * The host detects the removal of the device. Running transfers are cancelled without
 * a callback, a pending device address is dropped.
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_DeviceDisconnect( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        periphReg->BCDR &= ~USB_BCDR_DPPU;

        retState = Usb_Verify_Reg( &periphReg->BCDR, USB_BCDR_DPPU, 0u );

        Usb_Xfer_Abort_All( usbId, USB_FLAG_INACTIVE );

        usbContext->PendingAddress = 0u;
        usbContext->DeviceState    = USB_DEVICE_STATE_DISCONNECTED;
    }
    else
    {
        /* Controller is not initialized or not powered */
    }

    return ( retState );
}


/**
 * \brief Sets the device address assigned by the host
 *
 * The default address 0 is applied immediately. Other address is applied by the module when
 * the status stage of the control transfer finishes (the next transfer started on the IN
 * endpoint 0 is finished) - the status stage has to be answered at the old address. Call it from
 * the SETUP callback of the SET_ADDRESS request (before or after starting the status stage).
 *
 * \param usbId   [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param devAddr [in]: Device address (0 - 127)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_DeviceAddress( usb_PeriphId_t usbId, usb_DevAddr_t devAddr )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( ( USB_REQUEST_OK  == retState ) &&
        ( USB_DEV_ADDR_MAX >= devAddr )    )
    {
        if( 0u == devAddr )
        {
            periphReg->DADDR = USB_DADDR_EF;

            retState = Usb_Verify_Reg( &periphReg->DADDR, ( USB_DADDR_ADD | USB_DADDR_EF ), USB_DADDR_EF );

            usbContext->PendingAddress = 0u;
            usbContext->DeviceAddress  = 0u;

            if( USB_DEVICE_STATE_ADDRESSED == usbContext->DeviceState )
            {
                usbContext->DeviceState = USB_DEVICE_STATE_DEFAULT;
            }
            else
            {
                /* Device state is not addressed */
            }
        }
        else
        {
            usbContext->PendingAddress = devAddr;
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the device address applied to the controller
 *
 * \param usbId    [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param devAddr [out]: Device address (0 - default address)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_DeviceAddress( usb_PeriphId_t usbId, usb_DevAddr_t * const devAddr )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( USB_NULL_PTR != devAddr )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_MODE_DEVICE     == usbContext->Mode        )    )
    {
        *devAddr = usbContext->DeviceAddress;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the state of the device
 *
 * \param usbId     [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param devState [out]: Device state (disconnected, connected, default, addressed, suspended)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_DeviceState( usb_PeriphId_t usbId, usb_DeviceState_t * const devState )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( USB_NULL_PTR != devState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_MODE_DEVICE     == usbContext->Mode        )    )
    {
        *devState = usbContext->DeviceState;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the number of the last frame
 *
 * \param usbId         [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param frameNumber  [out]: Frame number (11 bits, incremented every 1 ms)
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_FrameNumber( usb_PeriphId_t usbId, usb_FrameNumber_t * const frameNumber )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    if( USB_NULL_PTR != frameNumber )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        /* The frame number is available in both modes */
        retState = Usb_Check_PoweredMode( usbId, usbContext->Mode );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        *frameNumber = (usb_FrameNumber_t)( periphReg->FNR & USB_FNR_FN );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the speed of the bus
 *
 * Device mode: the USB_DRD_FS controller works at full speed. Host mode: speed of the attached device
 * (low or full speed), an error is returned while no device is attached.
 *
 * \param usbId  [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param speed [out]: Bus speed
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Get_Speed( usb_PeriphId_t usbId, usb_Speed_t * const speed )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( USB_NULL_PTR != speed )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Check_PoweredMode( usbId, usbContext->Mode );
    }

    if( ( USB_REQUEST_OK == retState                  ) &&
        ( USB_MODE_HOST  == usbContext->Mode          ) &&
        ( USB_HOST_PORT_DISCONNECTED != usbContext->HostPortState ) )
    {
        *speed = usbContext->HostDevSpeed;
    }
    else if( ( USB_REQUEST_OK == retState             ) &&
             ( USB_MODE_DEVICE == usbContext->Mode    )    )
    {
        *speed = USB_SPEED_FULL;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the resume signaling (remote wakeup) of the suspended device
 *
 * The signaling lasts until \ref Usb_Set_RemoteWakeupInactive (the application waits 1 - 15 ms).
 * The application sends the remote wakeup only if the host enabled it (DEVICE_REMOTE_WAKEUP).
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (also if the device is not suspended).
 */
usb_RequestState_t Usb_Set_RemoteWakeupActive( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( ( USB_REQUEST_OK              == retState                ) &&
        ( USB_DEVICE_STATE_SUSPENDED  == usbContext->DeviceState )    )
    {
        periphReg->CNTR |= USB_CNTR_L2RES;

        retState = Usb_Verify_Reg( &periphReg->CNTR, USB_CNTR_L2RES, USB_CNTR_L2RES );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops the resume signaling (remote wakeup)
 *
 * \param usbId [in]: USB controller identification, value from \ref usb_PeriphId_t
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_Set_RemoteWakeupInactive( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState  = USB_REQUEST_ERROR;
    USB_DRD_TypeDef *  periphReg = USB_NULL_PTR;

    retState = Usb_Check_Operational( usbId );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        periphReg->CNTR &= ~USB_CNTR_L2RES;

        retState = Usb_Verify_Reg( &periphReg->CNTR, USB_CNTR_L2RES, 0u );
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the last SETUP packet received since the bus reset
 *
 * \param usbId         [in]: USB controller identification, value from \ref usb_PeriphId_t
 * \param setupPacket  [out]: Copy of the SETUP packet
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR (no SETUP packet received yet).
 */
usb_RequestState_t Usb_Get_SetupPacket( usb_PeriphId_t usbId, usb_SetupPacket_t * const setupPacket )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;

    if( USB_NULL_PTR != setupPacket )
    {
        retState = Usb_Get_Context( usbId, &usbContext );
    }

    if( ( USB_REQUEST_OK      == retState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_FLAG_ACTIVE     == usbContext->SetupValid  )    )
    {
        *setupPacket = usbContext->SetupPacket;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes the GPIO pin used as the DM (D-) signal of the controller
 *
 * \param pinId [in]: Pin identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_InitDmGpio( usb_DmPin_t pinId )
{
    return ( Usb_Init_Gpio( (usb_RegValue_t)pinId ) );
}


/**
 * \brief Initializes the GPIO pin used as the DP (D+) signal of the controller
 *
 * \param pinId [in]: Pin identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was
 *         success, otherwise returns \ref USB_REQUEST_ERROR.
 */
usb_RequestState_t Usb_InitDpGpio( usb_DpPin_t pinId )
{
    return ( Usb_Init_Gpio( (usb_RegValue_t)pinId ) );
}

/* ====================== SERVICES OF THE MODULE FILES ====================== */

/**
 * \brief Returns the runtime context of a USB controller
 *
 * \param usbId       [in]: USB controller identification
 * \param usbContext [out]: Context of the controller
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the identification is valid.
 */
usb_RequestState_t Usb_Get_Context( usb_PeriphId_t usbId, usb_Context_t ** const usbContext )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( ( USB_NULL_PTR   != usbContext ) &&
        ( USB_PERIPH_CNT >  usbId      )    )
    {
        *usbContext = &usb_Context[ usbId ];
        retState    = USB_REQUEST_OK;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the registers of a USB controller
 *
 * \param usbId      [in]: USB controller identification
 * \param periphReg [out]: Controller registers
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the identification is valid.
 */
usb_RequestState_t Usb_Get_PeriphReg( usb_PeriphId_t usbId, USB_DRD_TypeDef ** const periphReg )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( ( USB_NULL_PTR   != periphReg ) &&
        ( USB_PERIPH_CNT >  usbId     )    )
    {
        *periphReg = usb_PeriphConf[ usbId ].PeriphReg;
        retState   = USB_REQUEST_OK;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that the controller is initialized and powered (the registers can be written)
 *
 * The endpoint registers are accessible when the controller is neither in the power down state
 * nor in the reset state.
 *
 * \param usbId [in]: USB controller identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the controller is operational.
 */
usb_RequestState_t Usb_Check_Operational( usb_PeriphId_t usbId )
{
    return ( Usb_Check_PoweredMode( usbId, USB_MODE_DEVICE ) );
}


/**
 * \brief Checks that the controller is initialized in the host mode and powered (the channel registers can be written)
 *
 * The channel registers are accessible when the controller is neither in the power down state nor in
 * the reset state (the reset state is also the bus reset driven by the port reset).
 *
 * \param usbId [in]: USB controller identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the controller is operational in the host mode.
 */
usb_RequestState_t Usb_Check_HostOperational( usb_PeriphId_t usbId )
{
    return ( Usb_Check_PoweredMode( usbId, USB_MODE_HOST ) );
}


/**
 * \brief Clears flags of the interrupt status register
 *
 * The flags are cleared by writing 0, writing 1 has no effect, so all other flags are written
 * as 1 and a flag set meanwhile is not lost.
 *
 * \param periphReg [in]: USB controller registers
 * \param flagMask  [in]: Flags to clear (USB_ISTR_x)
 */
void Usb_Set_IstrFlagClear( USB_DRD_TypeDef * const periphReg, usb_RegValue_t flagMask )
{
    periphReg->ISTR = ( ~flagMask ) & USB_ISTR_CLEARABLE_MASK;
}


/**
 * \brief Resets the context of an endpoint direction to the closed state
 *
 * \param epContext [in]: Endpoint context
 */
void Usb_Set_EpContextReset( usb_EpContext_t * const epContext )
{
    epContext->Opened        = USB_FUNCTION_INACTIVE;
    epContext->Type          = USB_EP_TYPE_CONTROL;
    epContext->MaxPacketSize = 0u;
    epContext->PmaAddr       = 0u;
    epContext->PmaSize       = 0u;
    epContext->Stalled       = USB_FLAG_INACTIVE;
    epContext->XferActive    = USB_FUNCTION_INACTIVE;
    epContext->TxData        = USB_NULL_PTR;
    epContext->RxBuffer      = USB_NULL_PTR;
    epContext->XferSize      = 0u;
    epContext->XferCnt       = 0u;
    epContext->PacketLen     = 0u;
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks that the controller is initialized in the given mode and powered
 *
 * \param usbId [in]: USB controller identification
 * \param mode  [in]: Required operation mode
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the controller is initialized in
 *         the mode and neither in the power down state nor in the reset state.
 */
static usb_RequestState_t Usb_Check_PoweredMode( usb_PeriphId_t usbId, usb_Mode_t mode )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Get_Context( usbId, &usbContext );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( ( USB_REQUEST_OK      == retState                          ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState             ) &&
        ( mode                == usbContext->Mode                  ) &&
        ( 0u                  == ( periphReg->CNTR & USB_CNTR_NOT_ACTIVE_MASK ) )    )
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
 * \brief Verifies by read-back that bits of a register have the required value
 *
 * \param regAddr  [in]: Register
 * \param regMask  [in]: Verified bits
 * \param regValue [in]: Required value of the verified bits
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the register had the required
 *         value within the timeout.
 */
usb_RequestState_t Usb_Verify_Reg( const volatile uint32_t * const regAddr, usb_RegValue_t regMask, usb_RegValue_t regValue )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    for( usb_TimeoutCnt_t iterationCnt = 0u; USB_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const usb_RegValue_t readValue = *regAddr;

        if( regValue == ( readValue & regMask ) )
        {
            retState = USB_REQUEST_OK;
            break;
        }
        else
        {
            /* Register not changed yet */
        }
    }

    return ( retState );
}


/**
 * \brief Initializes a DM / DP GPIO pin from its encoded identification
 *
 * \param pinId [in]: Encoded pin identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if the pin is valid and was configured.
 */
static usb_RequestState_t Usb_Init_Gpio( usb_RegValue_t pinId )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( USB_PERIPH_CNT > USB_BIT_MASK_DECODE_PERIPH( pinId ) )
    {
        gpio_Config_t       pinConfig     = { 0u };
        gpio_RequestState_t gpioInitState = GPIO_REQUEST_ERROR;

        pinConfig.PortId         = USB_BIT_MASK_DECODE_PORT( pinId );
        pinConfig.PinId          = USB_BIT_MASK_DECODE_PIN( pinId );
        pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
        pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
        pinConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
        pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        pinConfig.PinAltFunction = USB_BIT_MASK_DECODE_AF( pinId );

        gpioInitState = Gpio_Init( &pinConfig );

        if( GPIO_REQUEST_ERROR != gpioInitState )
        {
            retState = USB_REQUEST_OK;
        }
        else
        {
            retState = USB_REQUEST_ERROR;
        }
    }
    else
    {
        /* Unused pin */
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Activates the clock and the supply of the controller, the HSI48 trimming and configures the pins
 *
 * \param usbId     [in]: USB controller identification
 * \param usbConfig [in]: Controller configuration
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was success.
 */
static usb_RequestState_t Usb_Init_Resources( usb_PeriphId_t usbId, const usb_BusConfig_t * const usbConfig )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    const rcc_PeriphId_t rccId    = usb_ClkSourceLut[ usbConfig->ClkSource ];

    retState = Usb_Get_Context( usbId, &usbContext );

    /*------------------------- Kernel and bus clock ------------------------*/
    if( USB_REQUEST_OK == retState )
    {
        rcc_FunctionState_t rccActivation = RCC_FUNCTION_INACTIVE;
        rcc_RequestState_t  rccState      = RCC_REQUEST_ERROR;

        rccState = Rcc_Get_PeriphState( rccId, &rccActivation );

        if( RCC_REQUEST_OK != rccState )
        {
            retState = USB_REQUEST_ERROR;
        }
        else if( RCC_FUNCTION_INACTIVE == rccActivation )
        {
            rccState = Rcc_Set_PeriphActive( rccId );

            if( RCC_REQUEST_OK != rccState )
            {
                retState = USB_REQUEST_ERROR;
            }
        }
        else
        {
            /* Clock is active already */
        }
    }

    /*------------------ Kernel clock frequency (48 MHz required) -----------*/
    if( USB_REQUEST_OK == retState )
    {
        rcc_FreqHz_t       kernelClk = 0u;
        rcc_RequestState_t rccState  = RCC_REQUEST_ERROR;

        rccState = Rcc_Get_PeriphClk( rccId, &kernelClk );

        if( ( RCC_REQUEST_OK    != rccState  ) ||
            ( USB_KERNEL_CLK_HZ != kernelClk )    )
        {
            retState = USB_REQUEST_ERROR;
        }
    }

#if defined(PWR_USBSCR_USB33SV)
    /*----------------------------- VDDUSB supply ---------------------------*/
    if( USB_REQUEST_OK == retState )
    {
        const rcc_RequestState_t supplyState = Rcc_Set_PwrSupplyActive( RCC_PWR_SUPPLY_VDDUSB );

        if( RCC_REQUEST_OK != supplyState )
        {
            retState = USB_REQUEST_ERROR;
        }
    }
#endif /* PWR_USBSCR_USB33SV */

    /*----------------------- HSI48 automatic trimming ----------------------*/
    if( ( USB_REQUEST_OK         == retState                ) &&
        ( USB_CLK_SOURCE_HSI48   == usbConfig->ClkSource    )    )
    {
        const rcc_RequestState_t trimState = Rcc_Set_Hsi48TrimActive( RCC_HSI48_TRIM_SRC_USB_SOF );

        if( RCC_REQUEST_OK != trimState )
        {
            retState = USB_REQUEST_ERROR;
        }
        else
        {
            usbContext->TrimActive = USB_FUNCTION_ACTIVE;
        }
    }
    else
    {
        /* Initialization failed or the kernel clock is not trimmed */
    }

    /*------------------------ Peripheral reset -----------------------------*/
    if( USB_REQUEST_OK == retState )
    {
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( rccId );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( rccId );

        if( ( RCC_REQUEST_OK != rstActState   ) ||
            ( RCC_REQUEST_OK != rstInactState )    )
        {
            retState = USB_REQUEST_ERROR;
        }
    }

    /*------------------------------- Pins ----------------------------------*/
    if( ( USB_REQUEST_OK    == retState                 ) &&
        ( USB_DM_PIN_UNUSED != usbConfig->DmPin         )    )
    {
        retState = Usb_InitDmGpio( usbConfig->DmPin );
    }

    if( ( USB_REQUEST_OK    == retState                 ) &&
        ( USB_DP_PIN_UNUSED != usbConfig->DpPin         )    )
    {
        retState = Usb_InitDpGpio( usbConfig->DpPin );
    }

    return ( retState );
}


/**
 * \brief Powers up and configures the controller registers (device mode, interrupts enabled)
 *
 * The controller is powered up in the reset state, the reset is released after the start-up
 * time, pending flags are cleared, the device is disconnected (pull-up off), the address is 0
 * and the buffer descriptor table is cleared. Then the interrupts of the bus events and of the
 * transfers are enabled (start of frame interrupt only if its callback is used).
 *
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was success.
 */
static usb_RequestState_t Usb_Init_Controller( usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;
    usb_RegValue_t     irqMask  = USB_CNTR_EVENT_IRQ_MASK;

    /* Power-up (PDWN = 0) in the reset state */
    periphReg->CNTR = USB_CNTR_USBRST;

    retState = Usb_Verify_Reg( &periphReg->CNTR, ( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_HOST ), USB_CNTR_USBRST );

    if( USB_REQUEST_OK == retState )
    {
        /* Analog start-up time of the controller (at least 1 us) */
        for( usb_TimeoutCnt_t readIdx = 0u; USB_STARTUP_READ_CNT > readIdx; readIdx ++ )
        {
            (void)periphReg->CNTR;
        }

        /* Reset released, device mode, interrupts masked */
        periphReg->CNTR = 0u;

        retState = Usb_Verify_Reg( &periphReg->CNTR, ( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_HOST ), 0u );
    }

    if( USB_REQUEST_OK == retState )
    {
        /* Pending flags cleared, device disconnected, default address, buffer descriptors cleared */
        periphReg->ISTR = 0u;
        periphReg->BCDR = 0u;
        periphReg->DADDR = 0u;

        Usb_Pma_Reset();

        retState = Usb_Verify_Reg( &periphReg->BCDR, USB_BCDR_DPPU, 0u );
    }

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Verify_Reg( &periphReg->DADDR, ( USB_DADDR_ADD | USB_DADDR_EF ), 0u );
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

        periphReg->CNTR = irqMask;

        retState = Usb_Verify_Reg( &periphReg->CNTR, ( USB_CNTR_EVENT_IRQ_MASK | USB_CNTR_SOFM ), irqMask );
    }

    return ( retState );
}


/**
 * \brief Connects the interrupt handler of the controller in the NVIC and enables the interrupt
 *
 * \param usbId       [in]: USB controller identification
 * \param irqPriority [in]: Priority of the interrupt
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if request was success.
 */
static usb_RequestState_t Usb_Init_Interrupt( usb_PeriphId_t usbId, usb_IrqPrio_t irqPriority )
{
    usb_RequestState_t retState = USB_REQUEST_ERROR;

    if( USB_PERIPH_CNT > usbId )
    {
        const nvic_RequestState_t handlerState = Nvic_Set_PeriphIrq_Handler( usb_PeriphConf[ usbId ].PeriphNvic,
                                                                              usb_PeriphConf[ usbId ].PeriphIsr );
        const nvic_RequestState_t prioState    = Nvic_Set_PeriphIrq_Prio( usb_PeriphConf[ usbId ].PeriphNvic, irqPriority );
        const nvic_RequestState_t activeState  = Nvic_Set_PeriphIrq_Active( usb_PeriphConf[ usbId ].PeriphNvic );

        if( ( NVIC_REQUEST_OK == handlerState ) &&
            ( NVIC_REQUEST_OK == prioState    ) &&
            ( NVIC_REQUEST_OK == activeState  )    )
        {
            retState = USB_REQUEST_OK;
        }
        else
        {
            retState = USB_REQUEST_ERROR;
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases everything taken by the initialization of the controller
 *
 * The pull-up is switched off, the interrupt disabled, the controller stopped and powered down,
 * the peripheral reset, the kernel clock, the trimming and the supply released. All steps are
 * executed, the failure of any step is reported.
 *
 * \param usbId [in]: USB controller identification
 *
 * \return State of request execution. Returns \ref USB_REQUEST_OK if all steps succeeded.
 */
static usb_RequestState_t Usb_Release_Resources( usb_PeriphId_t usbId )
{
    usb_RequestState_t retState   = USB_REQUEST_ERROR;
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;

    retState = Usb_Get_Context( usbId, &usbContext );

    if( USB_REQUEST_OK == retState )
    {
        retState = Usb_Get_PeriphReg( usbId, &periphReg );
    }

    if( USB_REQUEST_OK == retState )
    {
        const rcc_PeriphId_t rccId = usb_ClkSourceLut[ usbContext->ClkSource ];
        rcc_RequestState_t   rstActState   = RCC_REQUEST_ERROR;
        rcc_RequestState_t   rstInactState = RCC_REQUEST_ERROR;
        rcc_RequestState_t   clkState      = RCC_REQUEST_ERROR;
        rcc_RequestState_t   trimState     = RCC_REQUEST_OK;
        rcc_RequestState_t   supplyState   = RCC_REQUEST_OK;

        rcc_FunctionState_t  clkActivation = RCC_FUNCTION_INACTIVE;
        const rcc_RequestState_t clkReadState = Rcc_Get_PeriphState( rccId, &clkActivation );

        /* -1- Interrupt and bus connection */
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( usb_PeriphConf[ usbId ].PeriphNvic );

        Usb_Xfer_Abort_All( usbId, USB_FLAG_INACTIVE );

        /* -2- Controller: disconnected, stopped (reset state), powered down (registers of a controller without clock are not accessible) */
        if( ( RCC_REQUEST_OK      == clkReadState  ) &&
            ( RCC_FUNCTION_ACTIVE == clkActivation )    )
        {
            periphReg->BCDR &= ~( USB_BCDR_DPPU | USB_BCDR_DPPD );
            periphReg->CNTR  = USB_CNTR_USBRST;
            periphReg->ISTR  = 0u;
            periphReg->CNTR  = ( USB_CNTR_USBRST | USB_CNTR_PDWN );
        }
        else
        {
            /* Controller has no clock - nothing to stop */
        }

        /* -3- Trimming of the oscillator, peripheral reset and clock */
        if( USB_FUNCTION_ACTIVE == usbContext->TrimActive )
        {
            trimState = Rcc_Set_Hsi48TrimInactive();
        }

        rstActState   = Rcc_Set_ResetActive( rccId );
        rstInactState = Rcc_Set_ResetInactive( rccId );
        clkState      = Rcc_Set_PeriphInactive( rccId );

#if defined(PWR_USBSCR_USB33SV)
        /* -4- VDDUSB supply */
        supplyState = Rcc_Set_PwrSupplyInactive( RCC_PWR_SUPPLY_VDDUSB );
#endif /* PWR_USBSCR_USB33SV */

        usbContext->InitState          = USB_FUNCTION_INACTIVE;
        usbContext->Mode               = USB_MODE_DEVICE;
        usbContext->TrimActive         = USB_FUNCTION_INACTIVE;
        usbContext->DeviceState        = USB_DEVICE_STATE_DISCONNECTED;
        usbContext->HostPortState      = USB_HOST_PORT_DISCONNECTED;
        usbContext->PendingAddress     = 0u;
        usbContext->DeviceAddress      = 0u;
        usbContext->SetupValid         = USB_FLAG_INACTIVE;

        if( ( NVIC_REQUEST_OK == nvicState     ) &&
            ( RCC_REQUEST_OK  == trimState     ) &&
            ( RCC_REQUEST_OK  == rstActState   ) &&
            ( RCC_REQUEST_OK  == rstInactState ) &&
            ( RCC_REQUEST_OK  == clkState      ) &&
            ( RCC_REQUEST_OK  == supplyState   )    )
        {
            retState = USB_REQUEST_OK;
        }
        else
        {
            retState = USB_REQUEST_ERROR;
        }
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Services the bus events and the transfer events of a controller
 *
 * Called by the interrupt handler or by \ref Usb_Task. Only events with enabled interrupt are
 * served. One correct transfer event (of the endpoint presented by the controller) is served
 * first, then the pending bus events. The controller keeps the interrupt request while a flag is
 * pending, so the remaining transfer events are served by the next call.
 *
 * \param usbId [in]: USB controller identification
 */
static void Usb_Service( usb_PeriphId_t usbId )
{
    usb_Context_t *    usbContext = USB_NULL_PTR;
    USB_DRD_TypeDef *  periphReg  = USB_NULL_PTR;
    usb_RequestState_t ctxState   = USB_REQUEST_ERROR;
    usb_RequestState_t regState   = USB_REQUEST_ERROR;

    ctxState = Usb_Get_Context( usbId, &usbContext );
    regState = Usb_Get_PeriphReg( usbId, &periphReg );

    if( ( USB_REQUEST_OK      == ctxState                ) &&
        ( USB_REQUEST_OK      == regState                ) &&
        ( USB_FUNCTION_ACTIVE == usbContext->InitState   ) &&
        ( USB_MODE_HOST       == usbContext->Mode        )    )
    {
        Usb_Host_Service( usbId, usbContext, periphReg );
    }
    else if( ( USB_REQUEST_OK      == ctxState                ) &&
             ( USB_REQUEST_OK      == regState                ) &&
             ( USB_FUNCTION_ACTIVE == usbContext->InitState   )    )
    {
        usb_RegValue_t istrValue = periphReg->ISTR;

        if( 0u != ( istrValue & USB_ISTR_CTR ) )
        {
            Usb_Xfer_Service( usbId );

            istrValue = periphReg->ISTR;
        }
        else
        {
            /* No transfer event */
        }

        const usb_RegValue_t eventMask = istrValue & periphReg->CNTR;

        if( 0u != ( eventMask & USB_ISTR_RESET ) )
        {
            Usb_Handle_Reset( usbId, usbContext, periphReg );
        }
        else
        {
            if( 0u != ( eventMask & USB_ISTR_PMAOVR ) )
            {
                Usb_Handle_Error( usbId, usbContext, periphReg, USB_ISTR_PMAOVR, USB_ERROR_PMA_OVERRUN );
            }

            if( 0u != ( eventMask & USB_ISTR_ERR ) )
            {
                Usb_Handle_Error( usbId, usbContext, periphReg, USB_ISTR_ERR, USB_ERROR_BUS );
            }

            if( 0u != ( eventMask & USB_ISTR_WKUP ) )
            {
                Usb_Handle_Wakeup( usbId, usbContext, periphReg );
            }

            if( 0u != ( eventMask & USB_ISTR_SUSP ) )
            {
                Usb_Handle_Suspend( usbId, usbContext, periphReg );
            }

            if( 0u != ( eventMask & USB_ISTR_SOF ) )
            {
                Usb_Handle_Sof( usbId, usbContext, periphReg );
            }
        }
    }
    else
    {
        /* Controller is not initialized */
    }
}


/**
 * \brief Handles the bus reset
 *
 * Running transfers are reported as aborted, all endpoints are closed (their registers were reset
 * by the controller), the function is enabled with the default address 0 and the endpoint 0 is
 * initialized again. The reset callback is called last.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
static void Usb_Handle_Reset( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    /* Reception and transmission are disabled until the flag is cleared */
    Usb_Set_IstrFlagClear( periphReg, USB_ISTR_RESET );

    Usb_Ep_Close_All( usbId );

    usbContext->PendingAddress = 0u;
    usbContext->DeviceAddress  = 0u;
    usbContext->SetupValid     = USB_FLAG_INACTIVE;

    periphReg->DADDR = USB_DADDR_EF;

    if( USB_DEVICE_STATE_DISCONNECTED != usbContext->DeviceState )
    {
        usbContext->DeviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else
    {
        /* Pull-up is off - the reset is not a bus event of the device */
    }

    const usb_RequestState_t ep0State = Usb_Ep_Open_Control0( usbId );

    if( USB_REQUEST_OK != ep0State )
    {
        /* Endpoint 0 could not be initialized - the host can not enumerate the device */
        if( USB_NULL_PTR != usbContext->Callbacks.ErrorCallback )
        {
            usbContext->Callbacks.ErrorCallback( usbId, USB_ERROR_BUS );
        }
        else
        {
            /* Error callback is not used */
        }
    }
    else
    {
        /* Endpoint 0 is ready to receive the SETUP packets */
    }

    if( USB_NULL_PTR != usbContext->Callbacks.ResetCallback )
    {
        usbContext->Callbacks.ResetCallback( usbId );
    }
    else
    {
        /* Reset callback is not used */
    }
}


/**
 * \brief Handles the bus suspend (no activity for 3 ms)
 *
 * The controller is put to the suspend state (flag cleared after the suspend state is requested).
 * A suspend detected while the device is not connected is ignored.
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
static void Usb_Handle_Suspend( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    if( ( USB_DEVICE_STATE_DISCONNECTED != usbContext->DeviceState ) &&
        ( USB_DEVICE_STATE_SUSPENDED    != usbContext->DeviceState )    )
    {
        periphReg->CNTR |= USB_CNTR_SUSPEN;

        Usb_Set_IstrFlagClear( periphReg, USB_ISTR_SUSP );

        periphReg->CNTR |= USB_CNTR_SUSPRDY;

        usbContext->StateBeforeSuspend = usbContext->DeviceState;
        usbContext->DeviceState        = USB_DEVICE_STATE_SUSPENDED;

        if( USB_NULL_PTR != usbContext->Callbacks.SuspendCallback )
        {
            usbContext->Callbacks.SuspendCallback( usbId );
        }
        else
        {
            /* Suspend callback is not used */
        }
    }
    else
    {
        Usb_Set_IstrFlagClear( periphReg, USB_ISTR_SUSP );
    }
}


/**
 * \brief Handles the wakeup (bus activity after the suspend)
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
static void Usb_Handle_Wakeup( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    periphReg->CNTR &= ~USB_CNTR_SUSPRDY;
    periphReg->CNTR &= ~USB_CNTR_SUSPEN;

    Usb_Set_IstrFlagClear( periphReg, USB_ISTR_WKUP );

    if( USB_DEVICE_STATE_SUSPENDED == usbContext->DeviceState )
    {
        usbContext->DeviceState = usbContext->StateBeforeSuspend;

        if( USB_NULL_PTR != usbContext->Callbacks.ResumeCallback )
        {
            usbContext->Callbacks.ResumeCallback( usbId );
        }
        else
        {
            /* Resume callback is not used */
        }
    }
    else
    {
        /* Device was not suspended */
    }
}


/**
 * \brief Handles the start of frame
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 */
static void Usb_Handle_Sof( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg )
{
    Usb_Set_IstrFlagClear( periphReg, USB_ISTR_SOF );

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
 * \brief Handles a controller error flag
 *
 * \param usbId      [in]: USB controller identification
 * \param usbContext [in]: Context of the controller
 * \param periphReg  [in]: Controller registers
 * \param flagMask   [in]: Error flag to clear (USB_ISTR_x)
 * \param errorId    [in]: Error reported by the callback
 */
void Usb_Handle_Error( usb_PeriphId_t usbId, usb_Context_t * const usbContext, USB_DRD_TypeDef * const periphReg, usb_RegValue_t flagMask, usb_ErrorId_t errorId )
{
    Usb_Set_IstrFlagClear( periphReg, flagMask );

    if( USB_NULL_PTR != usbContext->Callbacks.ErrorCallback )
    {
        usbContext->Callbacks.ErrorCallback( usbId, errorId );
    }
    else
    {
        /* Error callback is not used */
    }
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief Interrupt handler of the full-speed USB controller
 */
static void Usb_Fs_IsrHandler( void )
{
    Usb_Service( USB_PERIPH_FS );
}

#endif /* USB_DRD_FS */
