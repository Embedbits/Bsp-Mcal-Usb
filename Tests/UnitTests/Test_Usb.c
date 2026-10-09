/**
 * \author Mr.Nobody
 * \file Test_Usb.c
 * \ingroup Usb
 * \brief Unit tests of Universal Serial Bus (USB) module.
 *
 * All Usb sources are compiled unchanged with real LL drivers. USB_DRD_FS registers and the
 * packet memory are emulated by RegMem, RCC, NVIC and GPIO modules are mocked by CMock.
 *
 * \note Emulated registers are plain RAM, so the tests emulate the behaviour of the controller:
 *       - Flags of the interrupt status register (rc_w0) are cleared by writing the inverted mask,
 *         the emulated ISTR contains the other clearable flags set - the tests preset ISTR before
 *         every call of the interrupt handler.
 *       - Toggle fields of the endpoint registers (status, data toggle) are changed by writing 1,
 *         the emulated register holds the written toggle pattern (current ^ required status).
 *       - Events of the controller (received / sent packet) are emulated by the tests: packet
 *         memory, buffer descriptor, endpoint register flags and ISTR are preset and the interrupt
 *         handler registered in NVIC is called.
 *
 * The lines without the USB_DRD_FS controller (STM32H5E4 / E5 / F4 / F5) are covered by the tests
 * of the unsupported functions, the other tests are skipped there.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Usb_Port.h"                       /* Module under test              */
#include "Usb.h"                            /* Internal API (packet memory)   */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "Stm32_usb.h"                      /* USB registers definition       */
#include <string.h>                         /* Test data handling             */
/* ============================= TYPEDEFS =================================== */

/** Record of the transfer callback calls */
typedef struct
{
    uint32_t          CallCnt;  /**< Count of calls                      */
    usb_PeriphId_t    UsbId;    /**< Controller identification           */
    usb_EpAddr_t      EpAddr;   /**< Endpoint address                    */
    usb_DataCnt_t     XferCnt;  /**< Count of transferred bytes          */
    usb_XferResult_t  Result;   /**< Result of the transfer              */
}   utUsb_XferRecord_t;

/** Record of the host channel callback calls */
typedef struct
{
    uint32_t             CallCnt;  /**< Count of calls                      */
    usb_PeriphId_t       UsbId;    /**< Controller identification           */
    usb_ChannelId_t      ChId;     /**< Channel                             */
    usb_DataCnt_t        XferCnt;  /**< Count of transferred bytes          */
    usb_ChannelResult_t  Result;   /**< Result of the transfer              */
}   utUsbHost_ChRecord_t;

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(USB_DRD_FS)
static nvic_RequestState_t  Ut_Usb_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static gpio_RequestState_t  Ut_Usb_GpioInitStub         ( gpio_Config_t * gpioConfig, int callCnt );
static void                 Ut_Usb_ResetCallback        ( usb_PeriphId_t usbId );
static void                 Ut_Usb_SuspendCallback      ( usb_PeriphId_t usbId );
static void                 Ut_Usb_ResumeCallback       ( usb_PeriphId_t usbId );
static void                 Ut_Usb_SofCallback          ( usb_PeriphId_t usbId );
static void                 Ut_Usb_SetupCallback        ( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket );
static void                 Ut_Usb_XferCallback         ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult );
static void                 Ut_Usb_ErrorCallback        ( usb_PeriphId_t usbId, usb_ErrorId_t errorId );
static usb_BusConfig_t      Ut_Usb_Get_Config           ( usb_ServiceMode_t serviceMode, usb_ClkSource_t clkSource );
static rcc_PeriphId_t       Ut_Usb_Get_RccId            ( usb_ClkSource_t clkSource );
static void                 Ut_Usb_Expect_ClockState    ( rcc_PeriphId_t rccId, rcc_FunctionState_t clockState );
static void                 Ut_Usb_Expect_KernelClk     ( rcc_PeriphId_t rccId, rcc_FreqHz_t clockHz );
static void                 Ut_Usb_Expect_InitResources ( usb_ClkSource_t clkSource );
static void                 Ut_Usb_Expect_InitIrq       ( void );
static void                 Ut_Usb_Expect_Release       ( usb_ClkSource_t clkSource, usb_FlagState_t trimActive, rcc_FunctionState_t clockState );
static void                 Ut_Usb_Init                 ( usb_ServiceMode_t serviceMode );
static void                 Ut_Usb_Init_Reset           ( void );
static void                 Ut_Usb_RunIsr               ( uint32_t istrValue );
static volatile uint32_t *  Ut_Usb_Get_ChepReg          ( usb_EpNum_t epNum );
static uint32_t             Ut_Usb_Get_TxAddr           ( usb_EpNum_t epNum );
static uint32_t             Ut_Usb_Get_RxAddr           ( usb_EpNum_t epNum );
static uint32_t             Ut_Usb_Get_TxCnt            ( usb_EpNum_t epNum );
static void                 Ut_Usb_Pma_Set              ( uint32_t pmaAddr, const uint8_t * data, uint32_t dataLen );
static void                 Ut_Usb_Pma_Get              ( uint32_t pmaAddr, uint8_t * data, uint32_t dataLen );
static void                 Ut_Usb_Fill                 ( uint8_t * data, uint32_t dataLen, uint8_t seed );
static void                 Ut_Usb_Receive              ( usb_EpNum_t epNum, const uint8_t * data, uint32_t dataLen, usb_FlagState_t isSetup );
static void                 Ut_Usb_TxDone               ( usb_EpNum_t epNum );
static void                 Ut_Usb_Open_Ep              ( usb_EpAddr_t epAddr, usb_EpType_t epType, usb_EpMaxPacket_t maxPacket );
static void                 Ut_UsbHost_ConnectCallback    ( usb_PeriphId_t usbId );
static void                 Ut_UsbHost_DisconnectCallback ( usb_PeriphId_t usbId );
static void                 Ut_UsbHost_ChannelCallback    ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t xferCnt, usb_ChannelResult_t chResult );
static usb_BusConfig_t      Ut_UsbHost_Get_Config         ( usb_ServiceMode_t serviceMode, const usb_Callbacks_t * callbacks );
static void                 Ut_UsbHost_Init               ( usb_ServiceMode_t serviceMode );
static void                 Ut_UsbHost_Connect            ( usb_FlagState_t lowSpeed );
static void                 Ut_UsbHost_Enable             ( void );
static void                 Ut_UsbHost_Init_Enabled       ( void );
static void                 Ut_UsbHost_Open_Channel       ( usb_ChannelId_t chId, usb_EpAddr_t epAddr, usb_EpType_t epType, usb_EpMaxPacket_t maxPacket );
static usb_ChannelState_t   Ut_UsbHost_Get_ChannelState   ( usb_ChannelId_t chId );
static void                 Ut_UsbHost_EventIn            ( usb_ChannelId_t chId, usb_RegValue_t statusRx, const uint8_t * data, uint32_t dataLen, usb_RegValue_t flags );
static void                 Ut_UsbHost_EventOut           ( usb_ChannelId_t chId, usb_RegValue_t statusTx, usb_RegValue_t flags );
#endif /* USB_DRD_FS */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Interrupt priority used by tests */
#define UT_USB_IRQ_PRIO                     ( 3u )

/** Packet memory address of the first buffer (behind the buffer descriptor table) */
#define UT_USB_PMA_FIRST                    ( 64u )

/** Size of the endpoint 0 packet buffer */
#define UT_USB_EP0_MPS                      ( 64u )

/** Size of the buffers of the test data */
#define UT_USB_BUF_SIZE                     ( 256u )

/** Maximal count of transfer callback records */
#define UT_USB_XFER_HISTORY                 ( 8u )

/** Count of storages of values returned by the RCC mock through pointers */
#define UT_USB_RET_STORAGE_CNT              ( 8u )

/** Mask of the count field of the buffer descriptor */
#define UT_USB_BD_COUNT_MASK                ( 0x03FF0000u )

/** Position of the count field of the buffer descriptor */
#define UT_USB_BD_COUNT_POS                 ( 16u )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(USB_DRD_FS)

/** Interrupt handler registered in NVIC */
static nvic_IsrCallback_t   utUsb_Isr;

/** Count of the reset callback calls */
static uint32_t             utUsb_ResetCnt;

/** Count of the suspend callback calls */
static uint32_t             utUsb_SuspendCnt;

/** Count of the resume callback calls */
static uint32_t             utUsb_ResumeCnt;

/** Count of the start of frame callback calls */
static uint32_t             utUsb_SofCnt;

/** Count of the SETUP callback calls */
static uint32_t             utUsb_SetupCnt;

/** SETUP packet passed to the last SETUP callback */
static usb_SetupPacket_t    utUsb_LastSetup;

/** Count of the error callback calls */
static uint32_t             utUsb_ErrorCnt;

/** Error passed to the last error callback */
static usb_ErrorId_t        utUsb_LastError;

/** Records of the transfer callback calls (the last record is the last call) */
static utUsb_XferRecord_t   utUsb_XferHistory[ UT_USB_XFER_HISTORY ];

/** Count of the transfer callback calls */
static uint32_t             utUsb_XferCnt;

/** Callbacks used by the tests */
static const usb_Callbacks_t utUsb_Callbacks =
{
    .ResetCallback   = Ut_Usb_ResetCallback,
    .SuspendCallback = Ut_Usb_SuspendCallback,
    .ResumeCallback  = Ut_Usb_ResumeCallback,
    .SofCallback     = Ut_Usb_SofCallback,
    .SetupCallback   = Ut_Usb_SetupCallback,
    .XferCallback    = Ut_Usb_XferCallback,
    .ErrorCallback   = Ut_Usb_ErrorCallback,
};

/** Callbacks without the start of frame callback */
static const usb_Callbacks_t utUsb_CallbacksNoSof =
{
    .ResetCallback   = Ut_Usb_ResetCallback,
    .SuspendCallback = Ut_Usb_SuspendCallback,
    .ResumeCallback  = Ut_Usb_ResumeCallback,
    .SofCallback     = USB_NULL_PTR,
    .SetupCallback   = Ut_Usb_SetupCallback,
    .XferCallback    = Ut_Usb_XferCallback,
    .ErrorCallback   = Ut_Usb_ErrorCallback,
};

/** Size of the transfer started by the transfer callback (0 - no restart) */
static usb_DataCnt_t        utUsb_RestartSize;

/** Endpoint of the transfer started by the transfer callback */
static usb_EpAddr_t         utUsb_RestartEpAddr;

/** Result of starting the transfer from the transfer callback */
static usb_RequestState_t   utUsb_RestartResult;

/** Configuration of the GPIO pins captured by the GPIO stub */
static gpio_Config_t        utUsb_GpioConfig[ 2 ];

/** Count of GPIO initializations captured by the GPIO stub */
static uint32_t             utUsb_GpioCnt;

/** Storage of the clock states returned by the RCC mock */
static rcc_FunctionState_t  utUsb_StateStorage[ UT_USB_RET_STORAGE_CNT ];

/** Index of the next free storage of the clock states */
static uint32_t             utUsb_StateIdx;

/** Storage of the clock frequencies returned by the RCC mock */
static rcc_FreqHz_t         utUsb_ClkStorage[ UT_USB_RET_STORAGE_CNT ];

/** Index of the next free storage of the clock frequencies */
static uint32_t             utUsb_ClkIdx;

/** Test data buffers */
static uint8_t              utUsb_TxData[ UT_USB_BUF_SIZE ];
static uint8_t              utUsb_RxData[ UT_USB_BUF_SIZE ];

/** Count of the host connect callback calls */
static uint32_t             utUsbHost_ConnectCnt;

/** Count of the host disconnect callback calls */
static uint32_t             utUsbHost_DisconnectCnt;

/** Records of the host channel callback calls (the last record is the last call) */
static utUsbHost_ChRecord_t utUsbHost_ChHistory[ UT_USB_XFER_HISTORY ];

/** Count of the host channel callback calls */
static uint32_t             utUsbHost_ChCnt;

/** Size of the IN transfer started by the channel callback (0 - no restart) */
static usb_DataCnt_t        utUsbHost_RestartSize;

/** Channel of the IN transfer started by the channel callback */
static usb_ChannelId_t      utUsbHost_RestartCh;

/** Result of starting the transfer from the channel callback */
static usb_RequestState_t   utUsbHost_RestartResult;

/** Callbacks of the host mode tests */
static const usb_Callbacks_t utUsbHost_Callbacks =
{
    .SofCallback        = Ut_Usb_SofCallback,
    .ErrorCallback      = Ut_Usb_ErrorCallback,
    .ConnectCallback    = Ut_UsbHost_ConnectCallback,
    .DisconnectCallback = Ut_UsbHost_DisconnectCallback,
    .ChannelCallback    = Ut_UsbHost_ChannelCallback,
};

#endif /* USB_DRD_FS */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

#if defined(USB_DRD_FS)
    utUsb_Isr        = NULL;
    utUsb_ResetCnt   = 0u;
    utUsb_SuspendCnt = 0u;
    utUsb_ResumeCnt  = 0u;
    utUsb_SofCnt     = 0u;
    utUsb_SetupCnt   = 0u;
    utUsb_ErrorCnt   = 0u;
    utUsb_LastError  = USB_ERROR_CNT;
    utUsb_XferCnt    = 0u;

    utUsb_RestartSize   = 0u;
    utUsb_RestartEpAddr = 0u;
    utUsb_RestartResult = USB_REQUEST_ERROR;
    utUsb_GpioCnt       = 0u;
    utUsb_StateIdx      = 0u;
    utUsb_ClkIdx        = 0u;

    utUsbHost_ConnectCnt    = 0u;
    utUsbHost_DisconnectCnt = 0u;
    utUsbHost_ChCnt         = 0u;
    utUsbHost_RestartSize   = 0u;
    utUsbHost_RestartCh     = 0u;
    utUsbHost_RestartResult = USB_REQUEST_ERROR;
    (void)memset( utUsbHost_ChHistory, 0, sizeof( utUsbHost_ChHistory ) );

    (void)memset( &utUsb_LastSetup, 0, sizeof( utUsb_LastSetup ) );
    (void)memset( utUsb_XferHistory, 0, sizeof( utUsb_XferHistory ) );
    (void)memset( utUsb_RxData, 0xEE, sizeof( utUsb_RxData ) );

    Ut_Usb_Fill( utUsb_TxData, UT_USB_BUF_SIZE, 1u );

    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Usb_NvicSetHandlerStub );
#endif /* USB_DRD_FS */
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Usb_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Usb_Get_ModuleVersion_ReturnsVersion( void )
{
    usb_ModuleVersion_t version = Usb_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   Usb_Get_DefaultConfig() fills default configuration.
 *
 * \par Expected results
 * - USB_REQUEST_OK, full-speed controller, device mode, HSI48, interrupt servicing, priority 0,
 *   endpoint 0 of 64 bytes, no callbacks, pins PA11 / PA12.
 * - NULL pointer: USB_REQUEST_ERROR.
 */
void Ut_Usb_Get_DefaultConfig_ReturnsDefaults( void )
{
    usb_BusConfig_t config;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( USB_PERIPH_FS,         config.PeriphId );
    TEST_ASSERT_EQUAL( USB_MODE_DEVICE,       config.Mode );
    TEST_ASSERT_EQUAL( USB_CLK_SOURCE_HSI48,  config.ClkSource );
    TEST_ASSERT_EQUAL( USB_SERVICE_MODE_ISR,  config.ServiceMode );
    TEST_ASSERT_EQUAL_UINT32( 0u,             config.IrqPriority );
    TEST_ASSERT_EQUAL_UINT16( 64u,            config.Ep0MaxPacketSize );
    TEST_ASSERT_NULL( config.Callbacks );
    TEST_ASSERT_EQUAL( USB_DM_PIN_PA11,       config.DmPin );
    TEST_ASSERT_EQUAL( USB_DP_PIN_PA12,       config.DpPin );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DefaultConfig( NULL ) );
}

#if defined(USB_DRD_FS)

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Usb_Init() powers up the controller and connects the interrupt.
 *
 * \details Initializes with callbacks including the start of frame callback, interrupt servicing.
 *
 * \par Expected results
 * - USB_REQUEST_OK, RCC / GPIO / NVIC called in the order of the initialization.
 * - CNTR: transfer, bus event and start of frame interrupts enabled, no power down, no reset.
 * - Pull-up off (BCDR.DPPU = 0), address 0 (DADDR = 0), no pending flag (ISTR = 0).
 * - Handler registered in NVIC, device state disconnected, address 0.
 */
void Ut_Usb_Init_Isr_PowersUpAndConnectsIrq( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;
    usb_DevAddr_t     devAddr  = 0xFFu;

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_BITS_HIGH( USB_CNTR_CTRM | USB_CNTR_PMAOVRM | USB_CNTR_ERRM | USB_CNTR_WKUPM |
                           USB_CNTR_SUSPM | USB_CNTR_RESETM | USB_CNTR_SOFM, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_HOST, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_BCDR_DPPU, USB_DRD_FS->BCDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->DADDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->ISTR );
    TEST_ASSERT_NOT_NULL( utUsb_Isr );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DISCONNECTED, devState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceAddress( USB_PERIPH_FS, &devAddr ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, devAddr );
}


/**
 * \brief   Usb_Init() with polling servicing does not touch the NVIC.
 *
 * \par Expected results
 * - USB_REQUEST_OK, no NVIC call (mock would fail on an unexpected call), handler not registered.
 */
void Ut_Usb_Init_Poll_NoInterrupt( void )
{
    Ut_Usb_Init( USB_SERVICE_MODE_POLL );

    TEST_ASSERT_NULL( utUsb_Isr );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_CTRM | USB_CNTR_RESETM, USB_DRD_FS->CNTR );
}


/**
 * \brief   Usb_Init() enables the start of frame interrupt only if its callback is used.
 *
 * \par Expected results
 * - Without start of frame callback CNTR.SOFM is cleared.
 */
void Ut_Usb_Init_NoSofCallback_SofInterruptDisabled( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    config.Callbacks = &utUsb_CallbacksNoSof;

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Ut_Usb_Expect_InitIrq();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );

    TEST_ASSERT_BITS_LOW( USB_CNTR_SOFM, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_CTRM | USB_CNTR_RESETM, USB_DRD_FS->CNTR );
}


/**
 * \brief   Usb_Init() with a PLL kernel clock does not trim HSI48.
 *
 * \par Expected results
 * - USB_REQUEST_OK, RCC activation of the PLL1Q kernel clock, no HSI48 trimming call.
 */
void Ut_Usb_Init_PllClock_NoTrimming( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_PLL1Q );

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_PLL1Q );
    Ut_Usb_Expect_InitIrq();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Init() keeps an already active kernel clock.
 *
 * \par Expected results
 * - USB_REQUEST_OK, no RCC activation of the clock.
 */
void Ut_Usb_Init_ClockActive_KeepsClock( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_POLL, USB_CLK_SOURCE_PLL1Q );

    Ut_Usb_Expect_ClockState( RCC_PERIPH_USB_PLL1Q, RCC_FUNCTION_ACTIVE );
    Ut_Usb_Expect_KernelClk( RCC_PERIPH_USB_PLL1Q, 48000000u );
#if defined(PWR_USBSCR_USB33SV)
    Rcc_Set_PwrSupplyActive_ExpectAndReturn( RCC_PWR_SUPPLY_VDDUSB, RCC_REQUEST_OK );
#endif
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_USB_PLL1Q, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_USB_PLL1Q, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Init() configures the DM and DP pins as alternate function 10.
 *
 * \details GPIO initialization is captured by a stub. Unused pins are refused.
 *
 * \par Expected results
 * - Port A, pins 11 (DM) and 12 (DP), alternate function mode, alternate function 10.
 * - Unused pin: USB_REQUEST_ERROR without GPIO call.
 */
void Ut_Usb_InitGpio_DmDp_AlternateFunction10( void )
{
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_InitDmGpio( USB_DM_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_InitDpGpio( USB_DP_PIN_UNUSED ) );

    Gpio_Init_Stub( Ut_Usb_GpioInitStub );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_InitDmGpio( USB_DM_PIN_PA11 ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_InitDpGpio( USB_DP_PIN_PA12 ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_GpioCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A,             utUsb_GpioConfig[ 0 ].PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_11,          utUsb_GpioConfig[ 0 ].PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, utUsb_GpioConfig[ 0 ].PinMode );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_10,        utUsb_GpioConfig[ 0 ].PinAltFunction );
    TEST_ASSERT_EQUAL( GPIO_PORT_A,             utUsb_GpioConfig[ 1 ].PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_12,          utUsb_GpioConfig[ 1 ].PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, utUsb_GpioConfig[ 1 ].PinMode );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_10,        utUsb_GpioConfig[ 1 ].PinAltFunction );
}


/**
 * \brief   Failure of the GPIO initialization is reported by the pin functions.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR when the GPIO module reports an error.
 */
void Ut_Usb_InitGpio_GpioFailure_ReturnsError( void )
{
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_InitDpGpio( USB_DP_PIN_PA12 ) );
}


/**
 * \brief   Usb_Init() rejects invalid configuration without any access.
 *
 * \details NULL pointer, controller, mode, clock source, servicing and endpoint 0 packet size out of range.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR, no RCC / NVIC / GPIO call, registers untouched.
 */
void Ut_Usb_Init_InvalidConfig_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( NULL ) );

    config.PeriphId = USB_PERIPH_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );
    config.Mode = USB_MODE_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );
    config.ClkSource = USB_CLK_SOURCE_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );
    config.ServiceMode = USB_SERVICE_MODE_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );
    config.Ep0MaxPacketSize = 12u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->CNTR );
}


/**
 * \brief   Usb_Init() fails when the clock state can not be read and releases what was taken.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR, the release sequence is called (clock without activation: registers untouched).
 */
void Ut_Usb_Init_RccStateFailure_ReturnsErrorAndReleases( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    Rcc_Get_PeriphState_ExpectAndReturn( RCC_PERIPH_USB_HSI48, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_INACTIVE, RCC_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->CNTR );
}


/**
 * \brief   Usb_Init() fails when the kernel clock can not be activated.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR, release sequence called.
 */
void Ut_Usb_Init_ClockActivationFailure_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    Ut_Usb_Expect_ClockState( RCC_PERIPH_USB_HSI48, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_ERROR );
    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_INACTIVE, RCC_FUNCTION_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Init() fails when the kernel clock is not 48 MHz.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR before the supply, trimming and controller are touched, release sequence called.
 */
void Ut_Usb_Init_WrongKernelClock_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_PLL1Q );

    Ut_Usb_Expect_ClockState( RCC_PERIPH_USB_PLL1Q, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_USB_PLL1Q, RCC_REQUEST_OK );
    Ut_Usb_Expect_KernelClk( RCC_PERIPH_USB_PLL1Q, 47000000u );
    Ut_Usb_Expect_Release( USB_CLK_SOURCE_PLL1Q, USB_FLAG_INACTIVE, RCC_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    /* Clock active: the release stopped the controller */
    TEST_ASSERT_EQUAL_HEX32( USB_CNTR_USBRST | USB_CNTR_PDWN, USB_DRD_FS->CNTR );
}


/**
 * \brief   Usb_Init() fails when the HSI48 trimming can not be activated.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR, the release sequence does not stop the trimming (it was not started).
 */
void Ut_Usb_Init_TrimFailure_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    Ut_Usb_Expect_ClockState( RCC_PERIPH_USB_HSI48, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_OK );
    Ut_Usb_Expect_KernelClk( RCC_PERIPH_USB_HSI48, 48000000u );
#if defined(PWR_USBSCR_USB33SV)
    Rcc_Set_PwrSupplyActive_ExpectAndReturn( RCC_PWR_SUPPLY_VDDUSB, RCC_REQUEST_OK );
#endif
    Rcc_Set_Hsi48TrimActive_ExpectAndReturn( RCC_HSI48_TRIM_SRC_USB_SOF, RCC_REQUEST_ERROR );
    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_INACTIVE, RCC_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Init() fails when the GPIO pin can not be configured.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR, the DP pin is not configured, release sequence (with trimming) called.
 */
void Ut_Usb_Init_GpioFailure_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    Ut_Usb_Expect_ClockState( RCC_PERIPH_USB_HSI48, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_OK );
    Ut_Usb_Expect_KernelClk( RCC_PERIPH_USB_HSI48, 48000000u );
#if defined(PWR_USBSCR_USB33SV)
    Rcc_Set_PwrSupplyActive_ExpectAndReturn( RCC_PWR_SUPPLY_VDDUSB, RCC_REQUEST_OK );
#endif
    Rcc_Set_Hsi48TrimActive_ExpectAndReturn( RCC_HSI48_TRIM_SRC_USB_SOF, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_ACTIVE, RCC_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Init() fails when the NVIC interrupt can not be activated.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR, release sequence called, controller stopped.
 */
void Ut_Usb_Init_NvicFailure_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_USB_DRD_FS, UT_USB_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_USB_DRD_FS, NVIC_REQUEST_ERROR );
    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_ACTIVE, RCC_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( USB_CNTR_USBRST | USB_CNTR_PDWN, USB_DRD_FS->CNTR );
}


/**
 * \brief   Usb_Init() of an initialized controller releases the previous initialization.
 *
 * \par Expected results
 * - USB_REQUEST_OK, release sequence called before the new initialization.
 */
void Ut_Usb_Init_Reinit_ReleasesPrevious( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_ACTIVE, RCC_FUNCTION_ACTIVE );
    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Ut_Usb_Expect_InitIrq();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Deinit() releases the controller.
 *
 * \par Expected results
 * - USB_REQUEST_OK, NVIC deactivated, trimming stopped, reset pulse, clocks and supply released.
 * - CNTR: reset and power down, pull-up off, no pending flag.
 * - Controller is not initialized afterwards: getters and second Usb_Deinit() return error.
 */
void Ut_Usb_Deinit_ReleasesResources( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );

    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_ACTIVE, RCC_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit( USB_PERIPH_FS ) );

    TEST_ASSERT_EQUAL_HEX32( USB_CNTR_USBRST | USB_CNTR_PDWN, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_BCDR_DPPU, USB_DRD_FS->BCDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->ISTR );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit( USB_PERIPH_FS ) );
}


/**
 * \brief   Usb_Deinit() reports a failure of the release steps.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR if the clock disable fails (all steps are executed).
 * - Invalid controller: USB_REQUEST_ERROR.
 */
void Ut_Usb_Deinit_ClockFailure_ReturnsError( void )
{
    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    Ut_Usb_Expect_ClockState( RCC_PERIPH_USB_HSI48, RCC_FUNCTION_ACTIVE );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_USB_DRD_FS, NVIC_REQUEST_OK );
    Rcc_Set_Hsi48TrimInactive_ExpectAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_USB_HSI48, RCC_REQUEST_ERROR );
#if defined(PWR_USBSCR_USB33SV)
    Rcc_Set_PwrSupplyInactive_ExpectAndReturn( RCC_PWR_SUPPLY_VDDUSB, RCC_REQUEST_OK );
#endif

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit( USB_PERIPH_CNT ) );
}

/* ========================= SERVICING BY POLLING =========================== */

/**
 * \brief   Usb_Task() services the controller in polling mode.
 *
 * \details Controller initialized for polling, bus reset flag is set.
 *
 * \par Expected results
 * - Reset is handled: callback called, device address enabled.
 */
void Ut_Usb_Task_PollMode_ServicesBusEvents( void )
{
    Ut_Usb_Init( USB_SERVICE_MODE_POLL );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );

    USB_DRD_FS->ISTR = USB_ISTR_RESET;
    Usb_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_ResetCnt );
    TEST_ASSERT_EQUAL_HEX32( USB_DADDR_EF, USB_DRD_FS->DADDR );
}


/**
 * \brief   Usb_Task() does not service a controller serviced by the interrupt.
 *
 * \par Expected results
 * - No callback is called, the reset flag is not cleared.
 */
void Ut_Usb_Task_IsrMode_DoesNothing( void )
{
    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    USB_DRD_FS->ISTR = USB_ISTR_RESET;
    Usb_Task();

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_ResetCnt );
    TEST_ASSERT_BITS_HIGH( USB_ISTR_RESET, USB_DRD_FS->ISTR );
}


/**
 * \brief   Usb_Task() without initialized controller does nothing.
 *
 * \par Expected results
 * - No register access, no callback.
 */
void Ut_Usb_Task_NotInitialized_DoesNothing( void )
{
    USB_DRD_FS->ISTR = USB_ISTR_RESET;
    Usb_Task();

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_ResetCnt );
    TEST_ASSERT_EQUAL_HEX32( USB_ISTR_RESET, USB_DRD_FS->ISTR );
}

/* ============================ DEVICE CONTROL ============================== */

/**
 * \brief   Device is connected and disconnected.
 *
 * \par Expected results
 * - Connect: BCDR.DPPU set, state connected. Disconnect: DPPU cleared, state disconnected.
 * - Not initialized controller: USB_REQUEST_ERROR and no register change.
 */
void Ut_Usb_DeviceConnect_PullUpAndState( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceDisconnect( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->BCDR );

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_HIGH( USB_BCDR_DPPU, USB_DRD_FS->BCDR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_CONNECTED, devState );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceDisconnect( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_LOW( USB_BCDR_DPPU, USB_DRD_FS->BCDR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DISCONNECTED, devState );
}


/**
 * \brief   Device control functions reject invalid arguments.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for controller out of range, NULL pointers and address out of range.
 */
void Ut_Usb_DeviceControl_InvalidArguments_ReturnsError( void )
{
    usb_DeviceState_t  devState = USB_DEVICE_STATE_CNT;
    usb_DevAddr_t      devAddr  = 0u;
    usb_FrameNumber_t  frame    = 0u;
    usb_Speed_t        speed    = USB_SPEED_CNT;
    usb_SetupPacket_t  setup;

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceConnect( USB_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceDisconnect( USB_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceAddress( USB_PERIPH_CNT, 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceAddress( USB_PERIPH_FS, USB_DEV_ADDR_MAX + 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceAddress( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceAddress( USB_PERIPH_CNT, &devAddr ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceState( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceState( USB_PERIPH_CNT, &devState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_FrameNumber( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_FrameNumber( USB_PERIPH_CNT, &frame ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_Speed( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_Speed( USB_PERIPH_CNT, &speed ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_SetupPacket( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_SetupPacket( USB_PERIPH_FS, &setup ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_RemoteWakeupActive( USB_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_RemoteWakeupInactive( USB_PERIPH_CNT ) );
}


/**
 * \brief   Frame number and speed are read.
 *
 * \par Expected results
 * - Frame number is the FN field of FNR (other bits masked), speed is full speed.
 */
void Ut_Usb_Get_FrameNumberAndSpeed_ReadsController( void )
{
    usb_FrameNumber_t frame = 0u;
    usb_Speed_t       speed = USB_SPEED_CNT;

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    USB_DRD_FS->FNR = 0x2000u | 0x0123u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_FrameNumber( USB_PERIPH_FS, &frame ) );
    TEST_ASSERT_EQUAL_UINT16( 0x0123u, frame );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_Speed( USB_PERIPH_FS, &speed ) );
    TEST_ASSERT_EQUAL( USB_SPEED_FULL, speed );
}


/**
 * \brief   Device address 0 is applied immediately, other address after the status stage.
 *
 * \details Bus reset, SET_ADDRESS(5) handling: address set, status stage (zero length IN packet
 *          on endpoint 0) started and finished.
 *
 * \par Expected results
 * - After Usb_Set_DeviceAddress(5) the controller still answers at the default address.
 * - After the status stage: DADDR = 5 + EF, address 5 read back, state addressed.
 * - Usb_Set_DeviceAddress(0): DADDR = EF, state default.
 */
void Ut_Usb_Set_DeviceAddress_AppliedAfterStatusStage( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;
    usb_DevAddr_t     devAddr  = 0u;

    Ut_Usb_Init_Reset();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceAddress( USB_PERIPH_FS, 5u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_DADDR_EF, USB_DRD_FS->DADDR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), NULL, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_DADDR_EF, USB_DRD_FS->DADDR );

    Ut_Usb_TxDone( 0u );

    TEST_ASSERT_EQUAL_HEX32( 5u | USB_DADDR_EF, USB_DRD_FS->DADDR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceAddress( USB_PERIPH_FS, &devAddr ) );
    TEST_ASSERT_EQUAL_UINT8( 5u, devAddr );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_ADDRESSED, devState );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceAddress( USB_PERIPH_FS, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_DADDR_EF, USB_DRD_FS->DADDR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DEFAULT, devState );
}


/**
 * \brief   A pending device address is dropped by a SETUP packet and by the disconnect.
 *
 * \par Expected results
 * - After a new SETUP packet the status stage does not apply the address.
 */
void Ut_Usb_Set_DeviceAddress_SetupDropsPendingAddress( void )
{
    const uint8_t setupBytes[ USB_SETUP_PACKET_LEN ] = { 0x00u, 0x05u, 0x05u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };

    Ut_Usb_Init_Reset();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceAddress( USB_PERIPH_FS, 5u ) );
    Ut_Usb_Receive( 0u, setupBytes, USB_SETUP_PACKET_LEN, USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), NULL, 0u ) );
    Ut_Usb_TxDone( 0u );

    TEST_ASSERT_EQUAL_HEX32( USB_DADDR_EF, USB_DRD_FS->DADDR );
}

/* ============================ BUS EVENTS ================================== */

/**
 * \brief   Bus reset initializes endpoint 0 and enables the function.
 *
 * \details Device is connected, the reset flag is set and the handler is called.
 *
 * \par Expected results
 * - Reset flag cleared (written 0), DADDR = EF, device state default.
 * - Endpoint 0 initialized in both directions: control type, packet buffers right behind the buffer
 *   descriptor table, endpoint idle, reset callback called once.
 */
void Ut_Usb_Isr_BusReset_OpensEndpoint0( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;
    usb_EpState_t     epState  = USB_EP_STATE_CNT;

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_RESET );

    TEST_ASSERT_BITS_LOW( USB_ISTR_RESET, USB_DRD_FS->ISTR );
    TEST_ASSERT_EQUAL_HEX32( USB_DADDR_EF, USB_DRD_FS->DADDR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DEFAULT, devState );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_OUT( 0u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_CONTROL, *Ut_Usb_Get_ChepReg( 0u ) & USB_EP_UTYPE );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST, Ut_Usb_Get_RxAddr( 0u ) );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST + UT_USB_EP0_MPS, Ut_Usb_Get_TxAddr( 0u ) );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_ResetCnt );
}


/**
 * \brief   Bus reset aborts the running transfers and closes the endpoints.
 *
 * \details A bulk IN transfer and a bulk OUT transfer are running when the reset arrives.
 *
 * \par Expected results
 * - Both transfers reported by the transfer callback as aborted (in the order of the endpoint numbers).
 * - Bulk endpoints closed, address 0.
 */
void Ut_Usb_Isr_BusReset_AbortsTransfersAndClosesEndpoints( void )
{
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 100u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 100u ) );

    Ut_Usb_RunIsr( USB_ISTR_RESET );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_IN( 1u ), utUsb_XferHistory[ 0 ].EpAddr );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_ABORTED, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_OUT( 2u ), utUsb_XferHistory[ 1 ].EpAddr );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_ABORTED, utUsb_XferHistory[ 1 ].Result );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_CLOSED, epState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_CLOSED, epState );
}


/**
 * \brief   Suspend and resume are handled.
 *
 * \details Device is connected and reset, suspend flag is set, then the wakeup flag.
 *
 * \par Expected results
 * - Suspend: CNTR.SUSPEN and SUSPRDY set, flag cleared, state suspended, callback called.
 * - Resume: SUSPEN and SUSPRDY cleared, state restored (default), callback called.
 */
void Ut_Usb_Isr_SuspendResume_StatesAndCallbacks( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;

    Ut_Usb_Init_Reset();

    Ut_Usb_RunIsr( USB_ISTR_SUSP );

    TEST_ASSERT_BITS_HIGH( USB_CNTR_SUSPEN | USB_CNTR_SUSPRDY, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_ISTR_SUSP, USB_DRD_FS->ISTR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_SUSPENDED, devState );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_SuspendCnt );

    Ut_Usb_RunIsr( USB_ISTR_WKUP );

    TEST_ASSERT_BITS_LOW( USB_CNTR_SUSPEN | USB_CNTR_SUSPRDY, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_ISTR_WKUP, USB_DRD_FS->ISTR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DEFAULT, devState );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_ResumeCnt );
}


/**
 * \brief   Suspend of a disconnected device is ignored.
 *
 * \par Expected results
 * - Flag cleared, no state change, no callback, controller not put to the suspend state.
 */
void Ut_Usb_Isr_SuspendDisconnected_Ignored( void )
{
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    Ut_Usb_RunIsr( USB_ISTR_SUSP );

    TEST_ASSERT_BITS_LOW( USB_CNTR_SUSPEN | USB_CNTR_SUSPRDY, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_ISTR_SUSP, USB_DRD_FS->ISTR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DISCONNECTED, devState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_SuspendCnt );
}


/**
 * \brief   Remote wakeup is signaled only by a suspended device.
 *
 * \par Expected results
 * - Not suspended: USB_REQUEST_ERROR, CNTR.L2RES stays cleared.
 * - Suspended: L2RES set by Usb_Set_RemoteWakeupActive(), cleared by Usb_Set_RemoteWakeupInactive().
 */
void Ut_Usb_RemoteWakeup_OnlyWhenSuspended( void )
{
    Ut_Usb_Init_Reset();

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_RemoteWakeupActive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_LOW( USB_CNTR_L2RES, USB_DRD_FS->CNTR );

    Ut_Usb_RunIsr( USB_ISTR_SUSP );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_RemoteWakeupActive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_L2RES, USB_DRD_FS->CNTR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_RemoteWakeupInactive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_LOW( USB_CNTR_L2RES, USB_DRD_FS->CNTR );
}


/**
 * \brief   Start of frame and error flags are reported by callbacks.
 *
 * \par Expected results
 * - SOF: callback called, flag cleared. Bus error: callback with USB_ERROR_BUS.
 * - Packet memory overrun: callback with USB_ERROR_PMA_OVERRUN.
 */
void Ut_Usb_Isr_SofAndErrors_Callbacks( void )
{
    Ut_Usb_Init_Reset();

    Ut_Usb_RunIsr( USB_ISTR_SOF );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_SofCnt );
    TEST_ASSERT_BITS_LOW( USB_ISTR_SOF, USB_DRD_FS->ISTR );

    Ut_Usb_RunIsr( USB_ISTR_ERR );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_ErrorCnt );
    TEST_ASSERT_EQUAL( USB_ERROR_BUS, utUsb_LastError );
    TEST_ASSERT_BITS_LOW( USB_ISTR_ERR, USB_DRD_FS->ISTR );

    Ut_Usb_RunIsr( USB_ISTR_PMAOVR );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_ErrorCnt );
    TEST_ASSERT_EQUAL( USB_ERROR_PMA_OVERRUN, utUsb_LastError );
    TEST_ASSERT_BITS_LOW( USB_ISTR_PMAOVR, USB_DRD_FS->ISTR );
}


/**
 * \brief   Events with disabled interrupt are not served and events without callback are tolerated.
 *
 * \details Initialization without the start of frame callback (SOFM cleared), SOF flag set.
 *
 * \par Expected results
 * - SOF callback not called (event not enabled), no fault.
 */
void Ut_Usb_Isr_DisabledEvent_NotServed( void )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );

    config.Callbacks = &utUsb_CallbacksNoSof;

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Ut_Usb_Expect_InitIrq();
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );

    Ut_Usb_RunIsr( USB_ISTR_SOF );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_SofCnt );
    TEST_ASSERT_BITS_HIGH( USB_ISTR_SOF, USB_DRD_FS->ISTR );
}


/**
 * \brief   Bus events are tolerated without callbacks.
 *
 * \details Initialization without callbacks, bus reset, suspend, wakeup and error flags.
 *
 * \par Expected results
 * - Events are handled (flags cleared, states changed), no callback is called.
 */
void Ut_Usb_Isr_NoCallbacks_EventsHandled( void )
{
    usb_BusConfig_t   config   = Ut_Usb_Get_Config( USB_SERVICE_MODE_ISR, USB_CLK_SOURCE_HSI48 );
    usb_DeviceState_t devState = USB_DEVICE_STATE_CNT;

    config.Callbacks = NULL;

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Ut_Usb_Expect_InitIrq();
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_RESET );
    Ut_Usb_RunIsr( USB_ISTR_SUSP );
    Ut_Usb_RunIsr( USB_ISTR_WKUP );
    Ut_Usb_RunIsr( USB_ISTR_ERR );
    Ut_Usb_RunIsr( USB_ISTR_PMAOVR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_DEFAULT, devState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_ResetCnt + utUsb_SuspendCnt + utUsb_ResumeCnt + utUsb_ErrorCnt );
}

/* =============================== ENDPOINTS ================================ */

/**
 * \brief   Endpoints are initialized with automatically allocated packet buffers.
 *
 * \details After the bus reset (endpoint 0 uses the buffers at 64 and 128) a bulk IN, a bulk OUT and
 *          an interrupt IN endpoint are initialized.
 *
 * \par Expected results
 * - Buffers allocated one behind another from address 192 with the (rounded) packet size.
 * - Endpoint registers: transfer type and address, buffer addresses in the descriptor table.
 * - Receive buffer size of the OUT endpoint in the descriptor (64 bytes: 32-byte blocks, 1 + 1).
 * - Endpoint state idle.
 */
void Ut_Usb_Init_Endpoint_AllocatesPacketMemory( void )
{
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();

    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 3u ), USB_EP_TYPE_INTERRUPT, 10u );

    TEST_ASSERT_EQUAL_UINT32( 192u, Ut_Usb_Get_TxAddr( 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 256u, Ut_Usb_Get_RxAddr( 2u ) );
    TEST_ASSERT_EQUAL_UINT32( 320u, Ut_Usb_Get_TxAddr( 3u ) );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_BULK,      *Ut_Usb_Get_ChepReg( 1u ) & USB_EP_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 1u,               *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_BULK,      *Ut_Usb_Get_ChepReg( 2u ) & USB_EP_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 2u,               *Ut_Usb_Get_ChepReg( 2u ) & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_INTERRUPT, *Ut_Usb_Get_ChepReg( 3u ) & USB_EP_UTYPE );

    /* Receive buffer of 64 bytes: 32-byte blocks (BLSIZE = 1), NUM_BLOCK = 1 */
    TEST_ASSERT_BITS_HIGH( USB_CNTRX_BLSIZE, (USB_DRD_PMA_BUFF + 2u)->RXBD );
    TEST_ASSERT_EQUAL_UINT32( 1u, ( (USB_DRD_PMA_BUFF + 2u)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_OUT( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_CLOSED, epState );
}


/**
 * \brief   Receive buffer descriptor of a small endpoint uses 2-byte blocks.
 *
 * \par Expected results
 * - 10 bytes interrupt OUT endpoint: BLSIZE = 0, NUM_BLOCK = 5, buffer of 12 bytes reserved.
 */
void Ut_Usb_Init_Endpoint_SmallReceiveBuffer_UsesSmallBlocks( void )
{
    Ut_Usb_Init_Reset();

    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 1u ), USB_EP_TYPE_INTERRUPT, 10u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_INTERRUPT, 8u );

    TEST_ASSERT_BITS_LOW( USB_CNTRX_BLSIZE, (USB_DRD_PMA_BUFF + 1u)->RXBD );
    TEST_ASSERT_EQUAL_UINT32( 5u, ( (USB_DRD_PMA_BUFF + 1u)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );
    TEST_ASSERT_EQUAL_UINT32( 192u, Ut_Usb_Get_RxAddr( 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 192u + 12u, Ut_Usb_Get_TxAddr( 1u ) );
}


/**
 * \brief   Endpoint initialization rejects invalid and unsupported configurations.
 *
 * \details Isochronous type, double buffering, packet size not allowed, endpoint 0 not control, endpoint
 *          number out of range, reserved address bits, NULL pointer.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for each case, the endpoint stays closed.
 */
void Ut_Usb_Init_Endpoint_InvalidConfig_ReturnsError( void )
{
    usb_EpConfig_t epConfig;
    usb_EpState_t  epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();

    epConfig.Address       = USB_EP_ADDR_IN( 1u );
    epConfig.Type          = USB_EP_TYPE_BULK;
    epConfig.MaxPacketSize = 64u;
    epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_CNT, &epConfig ) );

    epConfig.Type = USB_EP_TYPE_ISOCHRONOUS;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.Type = USB_EP_TYPE_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.Type       = USB_EP_TYPE_BULK;
    epConfig.BufferMode = USB_EP_BUFFER_DOUBLE;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;
    epConfig.MaxPacketSize = 48u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.MaxPacketSize = 128u;
    epConfig.Type          = USB_EP_TYPE_INTERRUPT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.MaxPacketSize = 0u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.MaxPacketSize = 64u;
    epConfig.Type          = USB_EP_TYPE_BULK;
    epConfig.Address       = USB_EP_ADDR_IN( USB_EP_NUM_CNT );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.Address = (usb_EpAddr_t)( USB_EP_ADDR_IN( 1u ) | 0x10u );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.Address = USB_EP_ADDR_IN( 0u );
    epConfig.Type    = USB_EP_TYPE_BULK;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_CLOSED, epState );
}


/**
 * \brief   An endpoint direction is initialized once and both directions share the transfer type.
 *
 * \par Expected results
 * - Second initialization of the same direction: USB_REQUEST_ERROR.
 * - OUT direction with another transfer type than the IN direction: USB_REQUEST_ERROR.
 * - Not initialized controller: USB_REQUEST_ERROR.
 */
void Ut_Usb_Init_Endpoint_DuplicateAndTypeMismatch_ReturnsError( void )
{
    usb_EpConfig_t epConfig;

    epConfig.Address       = USB_EP_ADDR_IN( 1u );
    epConfig.Type          = USB_EP_TYPE_BULK;
    epConfig.MaxPacketSize = 64u;
    epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    Ut_Usb_Init_Reset();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.Address = USB_EP_ADDR_OUT( 1u );
    epConfig.Type    = USB_EP_TYPE_INTERRUPT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );

    epConfig.Type = USB_EP_TYPE_BULK;
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );
}


/**
 * \brief   Deinitialized endpoint releases its packet buffer for the next endpoint.
 *
 * \par Expected results
 * - Buffer of the closed endpoint (192) is reused by the next initialized endpoint of the same size.
 * - Closed endpoint: state closed, second deinitialization and unknown endpoint: USB_REQUEST_ERROR.
 */
void Ut_Usb_Deinit_Endpoint_ReleasesPacketMemory( void )
{
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();

    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit_Endpoint( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_CLOSED, epState );

    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 3u ), USB_EP_TYPE_BULK, 64u );
    TEST_ASSERT_EQUAL_UINT32( 192u, Ut_Usb_Get_TxAddr( 3u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Endpoint( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Endpoint( USB_PERIPH_FS, USB_EP_ADDR_IN( USB_EP_NUM_CNT ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Endpoint( USB_PERIPH_CNT, USB_EP_ADDR_IN( 1u ) ) );
}


/**
 * \brief   Endpoint states are reported.
 *
 * \par Expected results
 * - Idle after initialization, busy during a transfer, stalled after stall, error for invalid arguments.
 */
void Ut_Usb_Get_EpState_ReportsBusyAndStalled( void )
{
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_BUSY, epState );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallActive( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_STALLED, epState );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpState( USB_PERIPH_CNT, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpState( USB_PERIPH_FS, (usb_EpAddr_t)0x71u, &epState ) );
}


/**
 * \brief   Endpoint is stalled and the stall is cleared.
 *
 * \details Bulk IN endpoint: stall cancels a running transfer, the endpoint refuses new transfers, the unstall
 *          resets the data toggle and the endpoint answers NAK. The endpoint register toggle fields hold the
 *          written toggle patterns (the emulated register is plain RAM).
 *
 * \par Expected results
 * - Stall: STAT_TX toggle pattern NAK ^ STALL, transfer cancelled, new transfer refused.
 * - Unstall: STAT_TX toggle pattern STALL ^ NAK, transfer accepted again.
 * - Unstall of an endpoint which is not stalled only resets the data toggle (toggle bit written), status is not written.
 */
void Ut_Usb_EpStall_SetAndClear_TogglePatterns( void )
{
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );

    /* Controller state before the stall: STAT_TX = VALID */
    *Ut_Usb_Get_ChepReg( 1u ) = ( *Ut_Usb_Get_ChepReg( 1u ) & ~USB_CHEP_TX_STTX ) | USB_EP_TX_VALID;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallActive( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_VALID ^ USB_EP_TX_STALL, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );

    /* Controller state before the unstall: STAT_TX = STALL, DTOG_TX = 0 */
    *Ut_Usb_Get_ChepReg( 1u ) = ( *Ut_Usb_Get_ChepReg( 1u ) & ~( USB_CHEP_TX_STTX | USB_CHEP_DTOG_TX ) ) | USB_EP_TX_STALL;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallInactive( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_STALL ^ USB_EP_TX_NAK, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );

    /* Not stalled endpoint: only the data toggle is reset (toggle written because DTOG_TX = 1, status is not written) */
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 2u ), USB_EP_TYPE_BULK, 64u );
    *Ut_Usb_Get_ChepReg( 2u ) = ( *Ut_Usb_Get_ChepReg( 2u ) & ~USB_CHEP_TX_STTX ) | USB_EP_TX_NAK | USB_CHEP_DTOG_TX;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallInactive( USB_PERIPH_FS, USB_EP_ADDR_IN( 2u ) ) );
    TEST_ASSERT_BITS_HIGH( USB_CHEP_DTOG_TX, *Ut_Usb_Get_ChepReg( 2u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, *Ut_Usb_Get_ChepReg( 2u ) & USB_CHEP_TX_STTX );
}


/**
 * \brief   OUT endpoint is stalled and unstalled.
 *
 * \par Expected results
 * - Stall: STAT_RX toggle pattern NAK ^ STALL. Unstall: STAT_RX toggle pattern STALL ^ NAK.
 * - Endpoint 0: unstall makes the endpoint ready for the SETUP packet (VALID, status out).
 * - Not initialized endpoint / controller: USB_REQUEST_ERROR.
 */
void Ut_Usb_EpStall_OutEndpoints_TogglePatterns( void )
{
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 1u ), USB_EP_TYPE_BULK, 64u );

    *Ut_Usb_Get_ChepReg( 1u ) = ( *Ut_Usb_Get_ChepReg( 1u ) & ~USB_CHEP_RX_STRX ) | USB_EP_RX_NAK;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallActive( USB_PERIPH_FS, USB_EP_ADDR_OUT( 1u ) ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_NAK ^ USB_EP_RX_STALL, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_RX_STRX );

    *Ut_Usb_Get_ChepReg( 1u ) = ( *Ut_Usb_Get_ChepReg( 1u ) & ~USB_CHEP_RX_STRX ) | USB_EP_RX_STALL;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallInactive( USB_PERIPH_FS, USB_EP_ADDR_OUT( 1u ) ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_STALL ^ USB_EP_RX_NAK, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_RX_STRX );

    /* Endpoint 0 */
    *Ut_Usb_Get_ChepReg( 0u ) = ( *Ut_Usb_Get_ChepReg( 0u ) & ~( USB_CHEP_RX_STRX | USB_EP_KIND ) ) | USB_EP_RX_STALL;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallActive( USB_PERIPH_FS, USB_EP_ADDR_OUT( 0u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallInactive( USB_PERIPH_FS, USB_EP_ADDR_OUT( 0u ) ) );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpStallActive( USB_PERIPH_FS, USB_EP_ADDR_OUT( 5u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpStallInactive( USB_PERIPH_FS, USB_EP_ADDR_OUT( 5u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpStallActive( USB_PERIPH_CNT, USB_EP_ADDR_OUT( 1u ) ) );
}


/**
 * \brief   Packet memory allocation finds the first free area and reports no space.
 *
 * \details The allocation function is tested directly with prepared contexts.
 *
 * \par Expected results
 * - Free memory: first address behind the descriptor table. A hole between two buffers is used if it fits.
 * - Memory full: USB_REQUEST_ERROR. NULL pointers / zero size: USB_REQUEST_ERROR.
 */
void Ut_Usb_Pma_Alloc_FirstFitAndNoSpace( void )
{
    static usb_Context_t usbContext;
    usb_PmaAddr_t        pmaAddr = 0u;

    (void)memset( &usbContext, 0, sizeof( usbContext ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Pma_Alloc( &usbContext, 64u, &pmaAddr ) );
    TEST_ASSERT_EQUAL_UINT16( UT_USB_PMA_FIRST, pmaAddr );

    /* Buffers at 64 - 128 and 192 - 256: the hole fits 64 bytes, not 128 */
    usbContext.Ep[ 0 ][ USB_EP_DIR_OUT ].Opened = USB_FUNCTION_ACTIVE;
    usbContext.Ep[ 0 ][ USB_EP_DIR_OUT ].PmaAddr = 64u;
    usbContext.Ep[ 0 ][ USB_EP_DIR_OUT ].PmaSize = 64u;
    usbContext.Ep[ 1 ][ USB_EP_DIR_IN  ].Opened = USB_FUNCTION_ACTIVE;
    usbContext.Ep[ 1 ][ USB_EP_DIR_IN  ].PmaAddr = 192u;
    usbContext.Ep[ 1 ][ USB_EP_DIR_IN  ].PmaSize = 64u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Pma_Alloc( &usbContext, 64u, &pmaAddr ) );
    TEST_ASSERT_EQUAL_UINT16( 128u, pmaAddr );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Pma_Alloc( &usbContext, 128u, &pmaAddr ) );
    TEST_ASSERT_EQUAL_UINT16( 256u, pmaAddr );

    /* Buffer up to the end of the memory */
    usbContext.Ep[ 2 ][ USB_EP_DIR_IN ].Opened = USB_FUNCTION_ACTIVE;
    usbContext.Ep[ 2 ][ USB_EP_DIR_IN ].PmaAddr = 256u;
    usbContext.Ep[ 2 ][ USB_EP_DIR_IN ].PmaSize = USB_DRD_PMA_SIZE - 256u;

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Pma_Alloc( &usbContext, 128u, &pmaAddr ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Pma_Alloc( &usbContext, 64u, &pmaAddr ) );
    TEST_ASSERT_EQUAL_UINT16( 128u, pmaAddr );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Pma_Alloc( NULL, 64u, &pmaAddr ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Pma_Alloc( &usbContext, 64u, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Pma_Alloc( &usbContext, 0u, &pmaAddr ) );
}


/**
 * \brief   Size of the packet buffer follows the block structure of the buffer descriptor.
 *
 * \par Expected results
 * - Up to 62 bytes rounded to 4 bytes, above 62 bytes rounded to 32 bytes.
 */
void Ut_Usb_Pma_Get_BufferSize_RoundsToBlocks( void )
{
    TEST_ASSERT_EQUAL_UINT16( 8u,  Usb_Pma_Get_BufferSize( 8u ) );
    TEST_ASSERT_EQUAL_UINT16( 12u, Usb_Pma_Get_BufferSize( 10u ) );
    TEST_ASSERT_EQUAL_UINT16( 64u, Usb_Pma_Get_BufferSize( 62u ) );
    TEST_ASSERT_EQUAL_UINT16( 64u, Usb_Pma_Get_BufferSize( 63u ) );
    TEST_ASSERT_EQUAL_UINT16( 64u, Usb_Pma_Get_BufferSize( 64u ) );
    TEST_ASSERT_EQUAL_UINT16( 96u, Usb_Pma_Get_BufferSize( 65u ) );
}


/**
 * \brief   The wait before a received packet is read only waits - it changes no register.
 *
 * \note    Device errata bug AB#1030 (ES0561 2.15.1 "Buffer description table update completes after CTR
 *          interrupt triggers"): the packet memory is read at least 800 ns after the interrupt. The wait
 *          is a fixed count of register reads, the duration is not measurable with the emulated registers.
 *
 * \par Expected results
 * - Control and status registers keep their values.
 * - The received packet is delivered after the wait (see the OUT transfer tests).
 */
void Ut_Usb_Pma_Wait_Settle_KeepsRegisters( void )
{
    USB_DRD_FS->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
    USB_DRD_FS->ISTR = USB_ISTR_CTR | USB_ISTR_SOF;

    Usb_Pma_Wait_Settle( USB_DRD_FS );

    TEST_ASSERT_EQUAL_HEX32( USB_CNTR_CTRM | USB_CNTR_RESETM, USB_DRD_FS->CNTR );
    TEST_ASSERT_EQUAL_HEX32( USB_ISTR_CTR | USB_ISTR_SOF, USB_DRD_FS->ISTR );
}

/* =============================== TRANSFERS ================================ */

/**
 * \brief   IN transfer of a single short packet.
 *
 * \details Bulk IN endpoint, 10 bytes. The controller sends the packet (emulated).
 *
 * \par Expected results
 * - Data copied to the packet buffer (unaligned length), packet length 10 in the descriptor, STAT_TX toggle pattern
 *   NAK ^ VALID, endpoint busy.
 * - After the transaction: one callback (endpoint, 10 bytes, ok), endpoint idle, transfer count read back.
 */
void Ut_Usb_Set_EpTxStart_ShortPacket_CompletesAfterTransaction( void )
{
    uint8_t         pmaData[ UT_USB_BUF_SIZE ];
    usb_DataCnt_t   xferCnt = 0u;
    usb_EpState_t   epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    *Ut_Usb_Get_ChepReg( 1u ) = ( *Ut_Usb_Get_ChepReg( 1u ) & ~USB_CHEP_TX_STTX ) | USB_EP_TX_NAK;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );

    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 1u ), pmaData, 10u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( utUsb_TxData, pmaData, 10u );
    TEST_ASSERT_EQUAL_UINT32( 10u, Ut_Usb_Get_TxCnt( 1u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_NAK ^ USB_EP_TX_VALID, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_BUSY, epState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );

    Ut_Usb_TxDone( 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, utUsb_XferHistory[ 0 ].UsbId );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_IN( 1u ), utUsb_XferHistory[ 0 ].EpAddr );
    TEST_ASSERT_EQUAL_UINT32( 10u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_BITS_LOW( USB_EP_VTTX, *Ut_Usb_Get_ChepReg( 1u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpXferCount( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &xferCnt ) );
    TEST_ASSERT_EQUAL_UINT32( 10u, xferCnt );
}


/**
 * \brief   IN transfer of several packets ends by a short packet.
 *
 * \details 150 bytes over a 64 byte endpoint (64 + 64 + 22), data from an unaligned source address.
 *
 * \par Expected results
 * - Each packet is copied to the packet buffer when the previous one was acknowledged.
 * - One callback with 150 bytes after the third transaction.
 */
void Ut_Usb_Set_EpTxStart_MultiPacket_SendsPacketsAndCompletesOnce( void )
{
    uint8_t pmaData[ UT_USB_BUF_SIZE ];

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &utUsb_TxData[ 1 ], 150u ) );
    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 1u ), pmaData, 64u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( &utUsb_TxData[ 1 ], pmaData, 64u );
    TEST_ASSERT_EQUAL_UINT32( 64u, Ut_Usb_Get_TxCnt( 1u ) );

    Ut_Usb_TxDone( 1u );
    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 1u ), pmaData, 64u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( &utUsb_TxData[ 65 ], pmaData, 64u );
    TEST_ASSERT_EQUAL_UINT32( 64u, Ut_Usb_Get_TxCnt( 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );

    Ut_Usb_TxDone( 1u );
    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 1u ), pmaData, 22u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( &utUsb_TxData[ 129 ], pmaData, 22u );
    TEST_ASSERT_EQUAL_UINT32( 22u, Ut_Usb_Get_TxCnt( 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );

    Ut_Usb_TxDone( 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 150u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 0 ].Result );
}


/**
 * \brief   IN transfer of a multiple of the packet size ends without a zero length packet.
 *
 * \par Expected results
 * - 128 bytes: two packets of 64 bytes, callback after the second one. A transfer of zero size sends one zero length packet.
 */
void Ut_Usb_Set_EpTxStart_MultipleOfPacketSize_NoZeroLengthPacket( void )
{
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 128u ) );
    Ut_Usb_TxDone( 1u );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );
    Ut_Usb_TxDone( 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 128u, utUsb_XferHistory[ 0 ].XferCnt );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), NULL, 0u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, Ut_Usb_Get_TxCnt( 1u ) );
    Ut_Usb_TxDone( 1u );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferHistory[ 1 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 1 ].Result );
}


/**
 * \brief   IN transfer start is refused in invalid states.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for a running transfer, OUT address, closed endpoint, NULL data with size, controller out of range,
 *   not initialized controller.
 */
void Ut_Usb_Set_EpTxStart_InvalidState_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 1u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), NULL, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 1u ), utUsb_TxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 4u ), utUsb_TxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_CNT, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );
}


/**
 * \brief   OUT transfer ends by a short packet.
 *
 * \details Bulk OUT endpoint, buffer of 100 bytes, the host sends a full packet (64) and a short packet (20).
 *
 * \par Expected results
 * - STAT_RX toggle pattern NAK ^ VALID after the start, the endpoint is re-armed after the full packet (no callback).
 * - After the short packet: one callback (endpoint, 84 bytes, ok), data in the buffer, endpoint idle.
 */
void Ut_Usb_Set_EpRxStart_ShortPacket_CompletesWithReceivedData( void )
{
    uint8_t       hostData[ UT_USB_BUF_SIZE ];
    usb_DataCnt_t xferCnt = 0u;

    Ut_Usb_Fill( hostData, UT_USB_BUF_SIZE, 100u );
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    *Ut_Usb_Get_ChepReg( 2u ) = ( *Ut_Usb_Get_ChepReg( 2u ) & ~USB_CHEP_RX_STRX ) | USB_EP_RX_NAK;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 100u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_NAK ^ USB_EP_RX_VALID, *Ut_Usb_Get_ChepReg( 2u ) & USB_CHEP_RX_STRX );

    Ut_Usb_Receive( 2u, hostData, 64u, USB_FLAG_INACTIVE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpXferCount( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), &xferCnt ) );
    TEST_ASSERT_EQUAL_UINT32( 64u, xferCnt );

    Ut_Usb_Receive( 2u, &hostData[ 64 ], 20u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_OUT( 2u ), utUsb_XferHistory[ 0 ].EpAddr );
    TEST_ASSERT_EQUAL_UINT32( 84u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( hostData, utUsb_RxData, 84u );
    TEST_ASSERT_EQUAL_HEX8( 0xEEu, utUsb_RxData[ 84 ] );
    TEST_ASSERT_BITS_LOW( USB_EP_VTRX, *Ut_Usb_Get_ChepReg( 2u ) );
}


/**
 * \brief   OUT transfer ends when the requested size is received.
 *
 * \par Expected results
 * - 128 bytes buffer, two full packets: callback after the second one with 128 bytes, ok.
 */
void Ut_Usb_Set_EpRxStart_BufferFilled_CompletesOnSize( void )
{
    uint8_t hostData[ UT_USB_BUF_SIZE ];

    Ut_Usb_Fill( hostData, UT_USB_BUF_SIZE, 7u );
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 128u ) );

    Ut_Usb_Receive( 2u, hostData, 64u, USB_FLAG_INACTIVE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );
    Ut_Usb_Receive( 2u, &hostData[ 64 ], 64u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 128u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( hostData, utUsb_RxData, 128u );
}


/**
 * \brief   OUT packet which does not fit the buffer is reported as overflow.
 *
 * \par Expected results
 * - Buffer of 10 bytes, packet of 20 bytes: 10 bytes stored, bytes behind the buffer untouched, callback with
 *   10 bytes and the overflow result.
 */
void Ut_Usb_Set_EpRxStart_PacketLargerThanBuffer_ReportsOverflow( void )
{
    uint8_t hostData[ UT_USB_BUF_SIZE ];

    Ut_Usb_Fill( hostData, UT_USB_BUF_SIZE, 50u );
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 10u ) );
    Ut_Usb_Receive( 2u, hostData, 20u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 10u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OVERFLOW, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( hostData, utUsb_RxData, 10u );
    TEST_ASSERT_EQUAL_HEX8( 0xEEu, utUsb_RxData[ 10 ] );
}


/**
 * \brief   OUT transfer receives a zero length packet.
 *
 * \par Expected results
 * - Callback with 0 bytes, ok. Zero size transfer without a buffer is accepted.
 */
void Ut_Usb_Set_EpRxStart_ZeroLengthPacket_CompletesWithZeroBytes( void )
{
    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 64u ) );
    Ut_Usb_Receive( 2u, utUsb_TxData, 0u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 0 ].Result );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), NULL, 0u ) );
}


/**
 * \brief   OUT transfer start is refused in invalid states.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for a running transfer, IN address, closed endpoint, NULL buffer with size, invalid controller.
 */
void Ut_Usb_Set_EpRxStart_InvalidState_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 10u ) );

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), NULL, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 2u ), utUsb_RxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 4u ), utUsb_RxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_CNT, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 10u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 10u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 10u ) );
}


/**
 * \brief   A running transfer is stopped without a callback.
 *
 * \par Expected results
 * - IN endpoint: STAT_TX toggle pattern VALID ^ NAK, endpoint idle, no callback, new transfer accepted.
 * - OUT endpoint: STAT_RX toggle pattern VALID ^ NAK. Stop without a running transfer: USB_REQUEST_OK, nothing changed.
 * - Closed endpoint / invalid controller: USB_REQUEST_ERROR.
 */
void Ut_Usb_Set_EpXferStop_CancelsTransferSilently( void )
{
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_Usb_Open_Ep( USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 100u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ), utUsb_RxData, 100u ) );

    *Ut_Usb_Get_ChepReg( 1u ) = ( *Ut_Usb_Get_ChepReg( 1u ) & ~USB_CHEP_TX_STTX ) | USB_EP_TX_VALID;
    *Ut_Usb_Get_ChepReg( 2u ) = ( *Ut_Usb_Get_ChepReg( 2u ) & ~USB_CHEP_RX_STRX ) | USB_EP_RX_VALID;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpXferStop( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_VALID ^ USB_EP_TX_NAK, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpXferStop( USB_PERIPH_FS, USB_EP_ADDR_OUT( 2u ) ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_VALID ^ USB_EP_RX_NAK, *Ut_Usb_Get_ChepReg( 2u ) & USB_CHEP_RX_STRX );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpXferStop( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 100u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpXferStop( USB_PERIPH_FS, USB_EP_ADDR_IN( 5u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpXferStop( USB_PERIPH_CNT, USB_EP_ADDR_IN( 1u ) ) );
}


/**
 * \brief   Transfer count is read for opened endpoints only.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for NULL pointer, closed endpoint and invalid controller.
 */
void Ut_Usb_Get_EpXferCount_InvalidArguments_ReturnsError( void )
{
    usb_DataCnt_t xferCnt = 0u;

    Ut_Usb_Init_Reset();

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpXferCount( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &xferCnt ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpXferCount( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpXferCount( USB_PERIPH_CNT, USB_EP_ADDR_IN( 0u ), &xferCnt ) );
}


/**
 * \brief   The transfer callback can start the next transfer of the endpoint.
 *
 * \details The callback of the finished IN transfer starts a new IN transfer of the same endpoint (stream of
 *          packets from the interrupt context).
 *
 * \par Expected results
 * - The transfer is finished before the callback is called, so the callback can start the next one.
 * - The module does not touch the endpoint after the callback: the new transfer runs and completes.
 */
void Ut_Usb_XferCallback_StartsNextTransfer_RunsIndependently( void )
{
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();
    Ut_Usb_Open_Ep( USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    utUsb_RestartEpAddr = USB_EP_ADDR_IN( 1u );
    utUsb_RestartSize   = 5u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), utUsb_TxData, 10u ) );
    Ut_Usb_TxDone( 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, utUsb_RestartResult );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_BUSY, epState );
    TEST_ASSERT_EQUAL_UINT32( 5u, Ut_Usb_Get_TxCnt( 1u ) );

    Ut_Usb_TxDone( 1u );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 5u, utUsb_XferHistory[ 1 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
}

/* ======================= CONTROL ENDPOINT 0 TRANSFERS ===================== */

/**
 * \brief   SETUP packet is decoded and reported.
 *
 * \details GET_DESCRIPTOR(device) SETUP packet received by the endpoint 0.
 *
 * \par Expected results
 * - SETUP callback called once with the decoded fields (little endian), the receive flag cleared.
 * - Usb_Get_SetupPacket() returns the same packet.
 */
void Ut_Usb_Isr_SetupPacket_DecodedAndReported( void )
{
    const uint8_t     setupBytes[ USB_SETUP_PACKET_LEN ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x09u, 0x04u, 0x12u, 0x00u };
    usb_SetupPacket_t setup;

    Ut_Usb_Init_Reset();

    Ut_Usb_Receive( 0u, setupBytes, USB_SETUP_PACKET_LEN, USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_SetupCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x80u, utUsb_LastSetup.RequestType );
    TEST_ASSERT_EQUAL_HEX8( 0x06u, utUsb_LastSetup.Request );
    TEST_ASSERT_EQUAL_HEX16( 0x0100u, utUsb_LastSetup.Value );
    TEST_ASSERT_EQUAL_HEX16( 0x0409u, utUsb_LastSetup.Index );
    TEST_ASSERT_EQUAL_HEX16( 0x0012u, utUsb_LastSetup.Length );
    TEST_ASSERT_BITS_LOW( USB_EP_VTRX, *Ut_Usb_Get_ChepReg( 0u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_SetupPacket( USB_PERIPH_FS, &setup ) );
    TEST_ASSERT_EQUAL_HEX16( 0x0409u, setup.Index );
    TEST_ASSERT_EQUAL_HEX16( 0x0012u, setup.Length );
}


/**
 * \brief   SETUP packet of an invalid length stalls the endpoint 0.
 *
 * \par Expected results
 * - Error callback with USB_ERROR_SETUP_LENGTH, no SETUP callback, endpoint 0 stalled in both directions.
 */
void Ut_Usb_Isr_SetupPacketInvalidLength_StallsEndpoint0( void )
{
    const uint8_t setupBytes[ USB_SETUP_PACKET_LEN ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x00u, 0x00u, 0x12u, 0x00u };
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();

    Ut_Usb_Receive( 0u, setupBytes, 7u, USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_SetupCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_ErrorCnt );
    TEST_ASSERT_EQUAL( USB_ERROR_SETUP_LENGTH, utUsb_LastError );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_OUT( 0u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_STALLED, epState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_STALLED, epState );
}


/**
 * \brief   Control read transfer: data stage over several packets, then the status stage.
 *
 * \details GET_DESCRIPTOR of 130 bytes: SETUP, IN data stage started from the SETUP handling (64 + 64 + 2 bytes), the
 *          host sends the zero length status packet (OUT), then the next SETUP packet.
 *
 * \par Expected results
 * - IN transfer callback after the last data packet: endpoint 0x80, 130 bytes, ok. The endpoint 0 is ready to receive
 *   the status packet (status out kind bit).
 * - Status stage: OUT transfer of zero size completes with 0 bytes, the endpoint is ready for the next SETUP packet.
 */
void Ut_Usb_ControlRead_DataAndStatusStages( void )
{
    const uint8_t setupBytes[ USB_SETUP_PACKET_LEN ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x00u, 0x00u, 0x82u, 0x00u };

    Ut_Usb_Init_Reset();

    Ut_Usb_Receive( 0u, setupBytes, USB_SETUP_PACKET_LEN, USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), utUsb_TxData, 130u ) );
    Ut_Usb_TxDone( 0u );
    Ut_Usb_TxDone( 0u );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, Ut_Usb_Get_TxCnt( 0u ) );

    /* Status OUT is expected after the last data packet: kind bit cleared before, set by the module */
    *Ut_Usb_Get_ChepReg( 0u ) &= ~USB_EP_KIND;
    Ut_Usb_TxDone( 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_IN( 0u ), utUsb_XferHistory[ 0 ].EpAddr );
    TEST_ASSERT_EQUAL_UINT32( 130u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );

    /* Status stage armed by the application, zero length OUT packet from the host */
    *Ut_Usb_Get_ChepReg( 0u ) &= ~USB_EP_KIND;
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 0u ), NULL, 0u ) );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );

    *Ut_Usb_Get_ChepReg( 0u ) &= ~USB_EP_KIND;
    Ut_Usb_Receive( 0u, NULL, 0u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_OUT( 0u ), utUsb_XferHistory[ 1 ].EpAddr );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferHistory[ 1 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_OK, utUsb_XferHistory[ 1 ].Result );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );
}


/**
 * \brief   Control write transfer: OUT data stage, then the IN status stage.
 *
 * \details SET_CONFIGURATION-like request with a 12 byte data stage.
 *
 * \par Expected results
 * - Data stage: the kind (status out) bit is cleared by the OUT transfer of non-zero size, 12 bytes stored.
 * - Status stage: zero length IN packet; after it the endpoint 0 is ready for the next SETUP packet.
 */
void Ut_Usb_ControlWrite_DataAndStatusStages( void )
{
    const uint8_t setupBytes[ USB_SETUP_PACKET_LEN ] = { 0x00u, 0x09u, 0x01u, 0x00u, 0x00u, 0x00u, 0x0Cu, 0x00u };

    Ut_Usb_Init_Reset();

    Ut_Usb_Receive( 0u, setupBytes, USB_SETUP_PACKET_LEN, USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 0u ), utUsb_RxData, 12u ) );
    TEST_ASSERT_BITS_LOW( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );

    Ut_Usb_Receive( 0u, utUsb_TxData, 12u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_UINT32( 12u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( utUsb_TxData, utUsb_RxData, 12u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), NULL, 0u ) );
    *Ut_Usb_Get_ChepReg( 0u ) &= ~USB_EP_KIND;
    Ut_Usb_TxDone( 0u );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_IN( 0u ), utUsb_XferHistory[ 1 ].EpAddr );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferHistory[ 1 ].XferCnt );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );
}


/**
 * \brief   A SETUP packet aborts the running transfers of the endpoint 0.
 *
 * \details IN data stage is running when the host sends a new SETUP packet.
 *
 * \par Expected results
 * - Transfer callback with the aborted result (before the SETUP callback), SETUP callback called, endpoint 0 idle.
 */
void Ut_Usb_Isr_SetupPacket_AbortsRunningEndpoint0Transfers( void )
{
    const uint8_t setupBytes[ USB_SETUP_PACKET_LEN ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x00u, 0x00u, 0x82u, 0x00u };
    usb_EpState_t epState = USB_EP_STATE_CNT;

    Ut_Usb_Init_Reset();

    Ut_Usb_Receive( 0u, setupBytes, USB_SETUP_PACKET_LEN, USB_FLAG_ACTIVE );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), utUsb_TxData, 130u ) );
    Ut_Usb_TxDone( 0u );

    Ut_Usb_Receive( 0u, setupBytes, USB_SETUP_PACKET_LEN, USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_XferCnt );
    TEST_ASSERT_EQUAL_HEX8( USB_EP_ADDR_IN( 0u ), utUsb_XferHistory[ 0 ].EpAddr );
    TEST_ASSERT_EQUAL_UINT32( 64u, utUsb_XferHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_XFER_RESULT_ABORTED, utUsb_XferHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsb_SetupCnt );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
}


/**
 * \brief   Zero length packet without a running transfer on the endpoint 0 makes it ready for the SETUP packet.
 *
 * \details Status packet of a control read transfer arrives while the application did not start the OUT transfer.
 *
 * \par Expected results
 * - No callback, receive flag cleared, the kind (status out) bit set.
 */
void Ut_Usb_Isr_UnsolicitedStatusPacket_ReArmsEndpoint0( void )
{
    Ut_Usb_Init_Reset();

    *Ut_Usb_Get_ChepReg( 0u ) &= ~USB_EP_KIND;
    Ut_Usb_Receive( 0u, NULL, 0u, USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );
    TEST_ASSERT_BITS_HIGH( USB_EP_KIND, *Ut_Usb_Get_ChepReg( 0u ) );
}


/**
 * \brief   Events of an interrupt without a valid flag or endpoint are tolerated.
 *
 * \details Correct transfer flag with endpoint identification out of range, then with the IN direction and no
 *          transmit flag in the endpoint register.
 *
 * \par Expected results
 * - No callback, no fault, the handler returns.
 */
void Ut_Usb_Isr_InconsistentTransferEvent_Ignored( void )
{
    Ut_Usb_Init_Reset();

    Ut_Usb_RunIsr( USB_ISTR_CTR | USB_ISTR_IDN );
    Ut_Usb_RunIsr( USB_ISTR_CTR | 1u );
    Ut_Usb_RunIsr( USB_ISTR_CTR | USB_ISTR_DIR | 1u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsb_XferCnt );
}

/* ============================ HOST: INITIALIZATION ======================== */

/**
 * \brief   Usb_Init() in the host mode powers up the controller with the pull-down resistors.
 *
 * \par Expected results
 * - USB_REQUEST_OK, CNTR: host mode, correct transfer, attach / detach, packet memory and start of frame
 *   interrupts enabled, no power down, no reset signaling, no suspend / wake-up / error interrupt.
 * - BCDR: DP / DM pull-down resistors on, pull-up off; no pending flag.
 * - Port disconnected, handler registered in NVIC, the device mode functions return an error.
 */
void Ut_UsbHost_Init_PowersUpHostMode( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;
    usb_DeviceState_t   devState  = USB_DEVICE_STATE_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_BITS_HIGH( USB_CNTR_HOST | USB_CNTR_CTRM | USB_CNTR_PMAOVRM | USB_CNTR_DCON | USB_CNTR_SOFM, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_ERRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM, USB_DRD_FS->CNTR );
    TEST_ASSERT_EQUAL_HEX32( USB_BCDR_DPPD, USB_DRD_FS->BCDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->ISTR );
    TEST_ASSERT_NOT_NULL( utUsb_Isr );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_DISCONNECTED, portState );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
}


/**
 * \brief   The start of frame interrupt is enabled in the host mode only if its callback is used.
 *
 * \par Expected results
 * - Without the start of frame callback CNTR.SOFM is cleared.
 */
void Ut_UsbHost_Init_NoSofCallback_SofInterruptDisabled( void )
{
    usb_Callbacks_t callbacks = utUsbHost_Callbacks;
    usb_BusConfig_t config;

    callbacks.SofCallback = USB_NULL_PTR;
    config                = Ut_UsbHost_Get_Config( USB_SERVICE_MODE_ISR, &callbacks );

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Ut_Usb_Expect_InitIrq();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );

    TEST_ASSERT_BITS_LOW( USB_CNTR_SOFM, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_HOST | USB_CNTR_CTRM | USB_CNTR_DCON, USB_DRD_FS->CNTR );
}


/**
 * \brief   The host mode controller served by polling does not touch the NVIC and is served by Usb_Task().
 *
 * \par Expected results
 * - No NVIC call (mock would fail on an unexpected call), handler not registered.
 * - The attach of a device is reported by Usb_Task().
 */
void Ut_UsbHost_Init_Poll_ServicedByTask( void )
{
    Ut_UsbHost_Init( USB_SERVICE_MODE_POLL );

    TEST_ASSERT_NULL( utUsb_Isr );

    USB_DRD_FS->ISTR = USB_ISTR_DCON | USB_ISTR_DCON_STAT;
    Usb_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ConnectCnt );
}


/**
 * \brief   Usb_Init() rejects an invalid operation mode.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for a mode out of range.
 */
void Ut_UsbHost_Init_InvalidMode_ReturnsError( void )
{
    usb_BusConfig_t config = Ut_UsbHost_Get_Config( USB_SERVICE_MODE_ISR, &utUsbHost_Callbacks );

    config.Mode = USB_MODE_CNT;

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );
}


/**
 * \brief   Usb_Deinit() releases the controller in the host mode.
 *
 * \par Expected results
 * - USB_REQUEST_OK, CNTR: reset and power down, pull-down off, no pending flag.
 * - The host functions return an error afterwards.
 */
void Ut_UsbHost_Deinit_ReleasesResources( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    Ut_Usb_Expect_Release( USB_CLK_SOURCE_HSI48, USB_FLAG_ACTIVE, RCC_FUNCTION_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit( USB_PERIPH_FS ) );

    TEST_ASSERT_EQUAL_HEX32( USB_CNTR_USBRST | USB_CNTR_PDWN, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_BCDR_DPPD | USB_BCDR_DPPU, USB_DRD_FS->BCDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, USB_DRD_FS->ISTR );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
}


/**
 * \brief   The host functions are not available in the device mode.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR of the port and channel functions of a device mode controller.
 */
void Ut_UsbHost_DeviceMode_HostFunctionsReturnError( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;
    usb_ChannelState_t  chState   = USB_CHANNEL_STATE_CNT;
    usb_DataCnt_t       xferCnt   = 0u;
    usb_ChannelConfig_t chConfig  = { 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 0u, USB_SPEED_FULL, 8u };

    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Channel( USB_PERIPH_FS, 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelState( USB_PERIPH_FS, 0u, &chState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 0u, USB_DATA_TOGGLE_DATA0 ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_OUT, NULL, 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelXferCount( USB_PERIPH_FS, 0u, &xferCnt ) );
}


/**
 * \brief   The port and channel functions reject invalid parameters.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for NULL pointers, an invalid controller and an invalid channel.
 */
void Ut_UsbHost_Functions_InvalidParameters_ReturnError( void )
{
    usb_ChannelState_t chState = USB_CHANNEL_STATE_CNT;
    usb_DataCnt_t      xferCnt = 0u;
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    Ut_UsbHost_Init_Enabled();

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_HostPortState( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_HostPortState( USB_PERIPH_CNT, &portState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetActive( USB_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetInactive( USB_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelState( USB_PERIPH_FS, 0u, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelState( USB_PERIPH_FS, USB_EP_NUM_CNT, &chState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelXferCount( USB_PERIPH_FS, 0u, NULL ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelXferCount( USB_PERIPH_FS, USB_EP_NUM_CNT, &xferCnt ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Channel( USB_PERIPH_FS, USB_EP_NUM_CNT ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelXferStop( USB_PERIPH_FS, USB_EP_NUM_CNT ) );
}

/* ============================ HOST: PORT ================================== */

/**
 * \brief   The attach of a full speed device is reported.
 *
 * \par Expected results
 * - Port connected, speed full, connect callback called once (a repeated attach event does not call it again).
 * - The attach event flag is cleared.
 */
void Ut_UsbHost_Isr_Connect_FullSpeedDevice( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;
    usb_Speed_t         speed     = USB_SPEED_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_CONNECTED, portState );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_Speed( USB_PERIPH_FS, &speed ) );
    TEST_ASSERT_EQUAL( USB_SPEED_FULL, speed );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ConnectCnt );
    TEST_ASSERT_BITS_LOW( USB_ISTR_DCON, USB_DRD_FS->ISTR );
}


/**
 * \brief   The attach of a low speed device is reported with the low speed.
 *
 * \par Expected results
 * - Port connected, speed low, connect callback called.
 */
void Ut_UsbHost_Isr_Connect_LowSpeedDevice( void )
{
    usb_Speed_t speed = USB_SPEED_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    Ut_UsbHost_Connect( USB_FLAG_ACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_Speed( USB_PERIPH_FS, &speed ) );
    TEST_ASSERT_EQUAL( USB_SPEED_LOW, speed );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ConnectCnt );
}


/**
 * \brief   The attach event is ignored while the module drives the reset and repeated attach events are not reported twice.
 *
 * \par Expected results
 * - Repeated attach: one connect callback.
 * - Detach event while the reset is driven: port stays in the reset state, no disconnect callback.
 * - The same event after the reset was released is the detach of the device.
 */
void Ut_UsbHost_Isr_ResetDriven_ConnectionEventsIgnored( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );
    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );
    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ConnectCnt );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_DCON );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_RESET, portState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_DisconnectCnt );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_DCON );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_DISCONNECTED, portState );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_DisconnectCnt );
}


/**
 * \brief   The detach of the device aborts the running transfers and closes the channels.
 *
 * \par Expected results
 * - Port disconnected, disconnect callback called, reset signaling released.
 * - The running transfer is reported as aborted, the channel is closed and its register cleared.
 * - The speed cannot be read without a device.
 */
void Ut_UsbHost_Isr_Disconnect_AbortsTransfersAndClosesChannels( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;
    usb_Speed_t         speed     = USB_SPEED_CNT;

    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    Ut_Usb_RunIsr( USB_ISTR_DCON );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_DISCONNECTED, portState );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_DisconnectCnt );
    TEST_ASSERT_BITS_LOW( USB_CNTR_USBRST, USB_DRD_FS->CNTR );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChHistory[ 0 ].ChId );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_ABORTED, utUsbHost_ChHistory[ 0 ].Result );

    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_CLOSED, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_ADDR | USB_CHEP_DEVADDR | USB_CH_UTYPE, *Ut_Usb_Get_ChepReg( 1u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_Speed( USB_PERIPH_FS, &speed ) );
}


/**
 * \brief   A detach event without an attached device is ignored.
 *
 * \par Expected results
 * - No disconnect callback, port stays disconnected.
 */
void Ut_UsbHost_Isr_Disconnect_NotAttached_Ignored( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    Ut_Usb_RunIsr( USB_ISTR_DCON );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_DISCONNECTED, portState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_DisconnectCnt );
}


/**
 * \brief   The reset signaling is driven and released, the port is enabled by the first start of frame.
 *
 * \par Expected results
 * - Reset active: CNTR.USBRST set, port state reset, a second request returns an error.
 * - Reset inactive: CNTR.USBRST cleared, start of frame interrupt enabled, the port stays in the reset state
 *   until the first start of frame, then it is enabled and the start of frame callback is called.
 */
void Ut_UsbHost_PortReset_ActiveInactive_EnablesPortAfterFirstSof( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );
    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_USBRST, USB_DRD_FS->CNTR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_RESET, portState );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_LOW( USB_CNTR_USBRST, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_SOFM, USB_DRD_FS->CNTR );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_RESET, portState );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_SOF );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_ENABLED, portState );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_SofCnt );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_SOFM, USB_DRD_FS->CNTR );

    /* The port can be reset again */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
}


/**
 * \brief   Without the start of frame callback the start of frame interrupt is disabled after the port is enabled.
 *
 * \par Expected results
 * - Port enabled by the first start of frame, CNTR.SOFM cleared.
 */
void Ut_UsbHost_PortReset_NoSofCallback_SofInterruptDisabledWhenEnabled( void )
{
    usb_Callbacks_t     callbacks = utUsbHost_Callbacks;
    usb_BusConfig_t     config;
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    callbacks.SofCallback = USB_NULL_PTR;
    config                = Ut_UsbHost_Get_Config( USB_SERVICE_MODE_ISR, &callbacks );

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );
    Ut_Usb_Expect_InitIrq();
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );

    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_SOFM, USB_DRD_FS->CNTR );

    Ut_Usb_RunIsr( USB_ISTR_SOF );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_ENABLED, portState );
    TEST_ASSERT_BITS_LOW( USB_CNTR_SOFM, USB_DRD_FS->CNTR );
}


/**
 * \brief   The port reset requires an attached device and an active reset.
 *
 * \par Expected results
 * - Reset without a device: USB_REQUEST_ERROR, CNTR.USBRST stays cleared.
 * - Release of a reset which is not driven: USB_REQUEST_ERROR.
 */
void Ut_UsbHost_PortReset_InvalidStates_ReturnError( void )
{
    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_LOW( USB_CNTR_USBRST, USB_DRD_FS->CNTR );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );

    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );
}


/**
 * \brief   The port reset closes the channels without a callback (the controller resets their registers).
 *
 * \par Expected results
 * - The open channel with a running transfer is closed, no channel callback is called.
 */
void Ut_UsbHost_PortReset_ClosesChannelsSilently( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, utUsb_RxData, 8u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );

    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_CLOSED, Ut_UsbHost_Get_ChannelState( 0u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
}


/**
 * \brief   Frame number and the packet memory error work in the host mode.
 *
 * \par Expected results
 * - The frame number is read from the frame number register.
 * - The packet memory overrun is reported by the error callback.
 */
void Ut_UsbHost_FrameNumberAndPmaOverrun( void )
{
    usb_FrameNumber_t frameNumber = 0u;

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    USB_DRD_FS->FNR = 0x123u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_FrameNumber( USB_PERIPH_FS, &frameNumber ) );
    TEST_ASSERT_EQUAL_UINT16( 0x123u, frameNumber );

    Ut_Usb_RunIsr( USB_ISTR_PMAOVR );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsb_ErrorCnt );
    TEST_ASSERT_EQUAL( USB_ERROR_PMA_OVERRUN, utUsb_LastError );
}

/* ============================ HOST: CHANNELS ============================== */

/**
 * \brief   A control channel is configured with both packet buffers.
 *
 * \par Expected results
 * - Channel register: control type, endpoint 0, device address 0, no low speed flag.
 * - Transmit and receive buffers allocated behind the buffer descriptor table (8 bytes each), channel idle.
 */
void Ut_UsbHost_Channel_Init_Control_ConfiguresRegisterAndBuffers( void )
{
    const usb_ChannelConfig_t chConfig = { 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 0u, USB_SPEED_FULL, 8u };
    usb_RegValue_t            regValue = 0u;

    Ut_UsbHost_Init_Enabled();

    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_CLOSED, Ut_UsbHost_Get_ChannelState( 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );

    regValue = *Ut_Usb_Get_ChepReg( 0u );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_CONTROL, regValue & USB_CH_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 0u, regValue & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, regValue & USB_CHEP_DEVADDR );
    TEST_ASSERT_BITS_LOW( USB_CHEP_LSEP | USB_CHEP_SETUP | USB_CHEP_KIND, regValue );

    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST, Ut_Usb_Get_TxAddr( 0u ) );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST + 8u, Ut_Usb_Get_RxAddr( 0u ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 0u ) );
}


/**
 * \brief   Bulk and interrupt channels get the buffer of their direction only.
 *
 * \par Expected results
 * - Bulk IN of the device 5: receive buffer, device address and endpoint number in the register.
 * - Bulk OUT: transmit buffer. Interrupt IN: small receive buffer. Buffers do not overlap.
 */
void Ut_UsbHost_Channel_Init_BulkAndInterrupt_ConfigureOneBuffer( void )
{
    const usb_ChannelConfig_t bulkIn  = { 1u, USB_EP_ADDR_IN ( 1u ), USB_EP_TYPE_BULK,      5u, USB_SPEED_FULL, 64u };
    const usb_ChannelConfig_t bulkOut = { 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK,      5u, USB_SPEED_FULL, 32u };
    const usb_ChannelConfig_t intIn   = { 3u, USB_EP_ADDR_IN ( 3u ), USB_EP_TYPE_INTERRUPT, 6u, USB_SPEED_FULL, 1u  };

    Ut_UsbHost_Init_Enabled();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &bulkIn ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &bulkOut ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &intIn ) );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_BULK, *Ut_Usb_Get_ChepReg( 1u ) & USB_CH_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 1u, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_HEX32( 5u << USB_CHEP_DEVADDR_Pos, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_DEVADDR );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST, Ut_Usb_Get_RxAddr( 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, Ut_Usb_Get_TxAddr( 1u ) );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_BULK, *Ut_Usb_Get_ChepReg( 2u ) & USB_CH_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 2u, *Ut_Usb_Get_ChepReg( 2u ) & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST + 64u, Ut_Usb_Get_TxAddr( 2u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, Ut_Usb_Get_RxAddr( 2u ) );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_INTERRUPT, *Ut_Usb_Get_ChepReg( 3u ) & USB_CH_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 3u, *Ut_Usb_Get_ChepReg( 3u ) & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST + 64u + 32u, Ut_Usb_Get_RxAddr( 3u ) );
}


/**
 * \brief   The low speed flag is set for a low speed device behind a full speed hub only.
 *
 * \par Expected results
 * - Full speed device on the root port (hub): a low speed channel has the low speed flag, a full speed channel not.
 * - Low speed device on the root port: the low speed channel has no flag (the controller works at low speed).
 */
void Ut_UsbHost_Channel_Init_LowSpeed_FlagOnlyBehindFullSpeedHub( void )
{
    usb_ChannelConfig_t chConfig = { 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_INTERRUPT, 3u, USB_SPEED_LOW, 8u };

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );
    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_BITS_HIGH( USB_CHEP_LSEP, *Ut_Usb_Get_ChepReg( 1u ) );

    chConfig.DevSpeed = USB_SPEED_FULL;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_LSEP, *Ut_Usb_Get_ChepReg( 1u ) );

    /* Low speed device on the root port */
    Ut_UsbHost_Connect( USB_FLAG_ACTIVE );

    chConfig.DevSpeed = USB_SPEED_LOW;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_LSEP, *Ut_Usb_Get_ChepReg( 1u ) );
}


/**
 * \brief   An invalid channel configuration is rejected without a change.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for: channel out of range, isochronous / unknown type, unsupported packet size, device address
 *   above 127, high speed device, reserved bits of the endpoint address.
 * - Without a device attached the channel cannot be initialized.
 * - The channel stays closed.
 */
void Ut_UsbHost_Channel_Init_InvalidConfig_ReturnsError( void )
{
    usb_ChannelConfig_t chConfig = { 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 3u, USB_SPEED_FULL, 64u };

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );    /* no device attached */

    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );

    chConfig.ChannelId = USB_EP_NUM_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.ChannelId = 1u;

    chConfig.Type = USB_EP_TYPE_ISOCHRONOUS;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.Type = USB_EP_TYPE_CNT;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.Type = USB_EP_TYPE_BULK;

    chConfig.MaxPacketSize = 12u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.Type = USB_EP_TYPE_INTERRUPT;
    chConfig.MaxPacketSize = 0u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.MaxPacketSize = 65u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.Type          = USB_EP_TYPE_BULK;
    chConfig.MaxPacketSize = 64u;

    chConfig.DevAddress = USB_DEV_ADDR_MAX + 1u;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.DevAddress = 3u;

    chConfig.DevSpeed = USB_SPEED_HIGH;
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.DevSpeed = USB_SPEED_FULL;

    chConfig.Address = (usb_EpAddr_t)( USB_EP_ADDR_IN( 1u ) | 0x10u );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    chConfig.Address = USB_EP_ADDR_IN( 1u );

    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_CLOSED, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
}


/**
 * \brief   An idle channel is initialized again, a busy channel is not.
 *
 * \par Expected results
 * - Initialization of a channel with a running transfer: USB_REQUEST_ERROR, the transfer keeps running.
 * - After the transfer is stopped the channel is retargeted: new device address and packet size, buffer reallocated.
 */
void Ut_UsbHost_Channel_Init_Again_RetargetsIdleChannel( void )
{
    usb_ChannelConfig_t chConfig = { 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 3u, USB_SPEED_FULL, 64u };

    Ut_UsbHost_Init_Enabled();

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_BUSY, Ut_UsbHost_Get_ChannelState( 1u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 1u ) );

    chConfig.DevAddress    = 7u;
    chConfig.MaxPacketSize = 32u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL_HEX32( 7u << USB_CHEP_DEVADDR_Pos, *Ut_Usb_Get_ChepReg( 1u ) & USB_CHEP_DEVADDR );
    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST, Ut_Usb_Get_RxAddr( 1u ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 1u ) );
}


/**
 * \brief   Usb_Deinit_Channel() closes a channel and releases its buffers.
 *
 * \par Expected results
 * - The channel register is cleared, the channel is closed, a second request returns an error.
 * - The released buffer is allocated again by the next channel.
 */
void Ut_UsbHost_Channel_Deinit_ReleasesChannel( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit_Channel( USB_PERIPH_FS, 1u ) );

    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_CLOSED, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_ADDR | USB_CHEP_DEVADDR | USB_CH_UTYPE, *Ut_Usb_Get_ChepReg( 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Channel( USB_PERIPH_FS, 1u ) );

    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_IN( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL_UINT32( UT_USB_PMA_FIRST, Ut_Usb_Get_RxAddr( 2u ) );
}


/**
 * \brief   The data toggle of a channel is set by Usb_Set_ChannelDataToggle().
 *
 * \note    The emulated register is plain RAM: a toggle written to one data toggle bit is lost by the next
 *          write of the register (writing 0 does not toggle in the controller), so the toggles of both
 *          directions of a control channel cannot be seen together - the last written one is checked.
 *
 * \par Expected results
 * - DATA1 toggles the data toggle bit of the direction of the channel (bulk IN: receive, bulk OUT: transmit).
 * - DATA0 of a channel whose toggle is already DATA0 writes nothing.
 * - An invalid toggle value and a busy channel return an error.
 */
void Ut_UsbHost_Channel_DataToggle_TogglesBits( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 1u, USB_DATA_TOGGLE_DATA0 ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_DTOG_RX | USB_CHEP_DTOG_TX, *Ut_Usb_Get_ChepReg( 1u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 1u, USB_DATA_TOGGLE_DATA1 ) );
    TEST_ASSERT_BITS_HIGH( USB_CHEP_DTOG_RX, *Ut_Usb_Get_ChepReg( 1u ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_DTOG_TX, *Ut_Usb_Get_ChepReg( 1u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 2u, USB_DATA_TOGGLE_DATA1 ) );
    TEST_ASSERT_BITS_HIGH( USB_CHEP_DTOG_TX, *Ut_Usb_Get_ChepReg( 2u ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_DTOG_RX, *Ut_Usb_Get_ChepReg( 2u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 0u, USB_DATA_TOGGLE_DATA1 ) );
    TEST_ASSERT_BITS_HIGH( USB_CHEP_DTOG_RX, *Ut_Usb_Get_ChepReg( 0u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 1u, USB_DATA_TOGGLE_CNT ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 1u, USB_DATA_TOGGLE_DATA0 ) );
}

/* ============================ HOST: TRANSFERS ============================= */

/**
 * \brief   A SETUP transaction is sent on a control channel.
 *
 * \par Expected results
 * - The 8 bytes are written to the transmit buffer, count 8, SETUP flag set, transmission enabled, channel busy.
 * - After the acknowledge (SETUP flag cleared by the controller) the channel callback reports 8 bytes, result ok;
 *   the channel is idle and the correct transfer flag cleared.
 */
void Ut_UsbHost_Xfer_Setup_SentAndAcknowledged( void )
{
    const uint8_t setupBytes[ 8 ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x00u, 0x00u, 0x12u, 0x00u };
    uint8_t       pmaBytes[ 8 ];

    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_SETUP, setupBytes, 8u ) );

    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 0u ), pmaBytes, 8u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( setupBytes, pmaBytes, 8u );
    TEST_ASSERT_EQUAL_UINT32( 8u, Ut_Usb_Get_TxCnt( 0u ) );
    TEST_ASSERT_BITS_HIGH( USB_CHEP_SETUP, *Ut_Usb_Get_ChepReg( 0u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_TX_VALID, *Ut_Usb_Get_ChepReg( 0u ) & USB_CH_TX_STTX );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_BUSY, Ut_UsbHost_Get_ChannelState( 0u ) );

    Ut_UsbHost_EventOut( 0u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChHistory[ 0 ].ChId );
    TEST_ASSERT_EQUAL_UINT32( 8u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );
    TEST_ASSERT_BITS_LOW( USB_CH_VTTX | USB_CHEP_SETUP, *Ut_Usb_Get_ChepReg( 0u ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 0u ) );
}


/**
 * \brief   A control read transfer: SETUP stage, IN data stage in packets of 8 bytes, OUT status stage.
 *
 * \par Expected results
 * - Data stage: packets 8, 8 and the short packet 2 - the next packet is requested with the length of the rest
 *   after every full packet, the transfer ends by the short packet with 18 bytes, the data are stored.
 * - Status stage: a zero length OUT transaction ends with 0 bytes.
 */
void Ut_UsbHost_Xfer_ControlRead_SetupDataStatusStages( void )
{
    const uint8_t setupBytes[ 8 ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x00u, 0x00u, 0x12u, 0x00u };

    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );

    /* Setup stage */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_SETUP, setupBytes, 8u ) );
    Ut_UsbHost_EventOut( 0u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );

    /* Data stage */
    Ut_Usb_Fill( utUsb_TxData, 18u, 0x40u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, utUsb_RxData, 18u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 0u ) & USB_CH_RX_STRX );
    TEST_ASSERT_EQUAL_UINT32( 4u, ( (USB_DRD_PMA_BUFF + 0)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );    /* 8 bytes expected */

    Ut_UsbHost_EventIn( 0u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 0 ], 8u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 0u ) & USB_CH_RX_STRX );
    TEST_ASSERT_EQUAL_UINT32( 4u, ( (USB_DRD_PMA_BUFF + 0)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );    /* 8 bytes of 10 expected */

    Ut_UsbHost_EventIn( 0u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 8 ], 8u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, ( (USB_DRD_PMA_BUFF + 0)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );    /* 2 bytes expected */

    Ut_UsbHost_EventIn( 0u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 16 ], 2u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 18u, utUsbHost_ChHistory[ 1 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 1 ].Result );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( utUsb_TxData, utUsb_RxData, 18u );

    /* Status stage */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_OUT, NULL, 0u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, Ut_Usb_Get_TxCnt( 0u ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_SETUP, *Ut_Usb_Get_ChepReg( 0u ) );

    Ut_UsbHost_EventOut( 0u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 3u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChHistory[ 2 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 2 ].Result );
}


/**
 * \brief   A control write transfer ends with a zero length IN status stage.
 *
 * \par Expected results
 * - A zero size IN transfer expects a zero length packet and ends with 0 bytes.
 */
void Ut_UsbHost_Xfer_In_ZeroLengthStatusStage( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, NULL, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 0u ) & USB_CH_RX_STRX );

    Ut_UsbHost_EventIn( 0u, USB_CH_RX_ACK_SBUF, NULL, 0u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );
}


/**
 * \brief   An OUT transfer of several packets.
 *
 * \par Expected results
 * - 150 bytes in packets of 64, 64 and 22 bytes: every acknowledge sends the next packet (data and count in the
 *   transmit buffer), the transferred count follows, the end is reported after the last acknowledge.
 */
void Ut_UsbHost_Xfer_Out_MultiPacket( void )
{
    uint8_t       pmaBytes[ 64 ];
    usb_DataCnt_t xferCnt = 0u;

    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    Ut_Usb_Fill( utUsb_TxData, 150u, 3u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, utUsb_TxData, 150u ) );

    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 2u ), pmaBytes, 64u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( &utUsb_TxData[ 0 ], pmaBytes, 64u );
    TEST_ASSERT_EQUAL_UINT32( 64u, Ut_Usb_Get_TxCnt( 2u ) );
    TEST_ASSERT_BITS_LOW( USB_CHEP_SETUP, *Ut_Usb_Get_ChepReg( 2u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_ChannelXferCount( USB_PERIPH_FS, 2u, &xferCnt ) );
    TEST_ASSERT_EQUAL_UINT32( 64u, xferCnt );
    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 2u ), pmaBytes, 64u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( &utUsb_TxData[ 64 ], pmaBytes, 64u );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_TX_VALID, *Ut_Usb_Get_ChepReg( 2u ) & USB_CH_TX_STTX );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 22u, Ut_Usb_Get_TxCnt( 2u ) );
    Ut_Usb_Pma_Get( Ut_Usb_Get_TxAddr( 2u ), pmaBytes, 22u );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( &utUsb_TxData[ 128 ], pmaBytes, 22u );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 150u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 2u ) );
}


/**
 * \brief   An IN transfer ends by a short packet.
 *
 * \par Expected results
 * - 64 + 10 bytes received of 256 requested: the first full packet requests the next one, the short packet ends the
 *   transfer with 74 bytes and the data are stored.
 */
void Ut_UsbHost_Xfer_In_ShortPacketEndsTransfer( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    Ut_Usb_Fill( utUsb_TxData, 74u, 9u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 256u ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, ( (USB_DRD_PMA_BUFF + 1)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );    /* 64 bytes expected: 32 byte blocks */
    TEST_ASSERT_BITS_HIGH( USB_CNTRX_BLSIZE, (USB_DRD_PMA_BUFF + 1)->RXBD );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 0 ], 64u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 1u ) & USB_CH_RX_STRX );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 64 ], 10u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 74u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( utUsb_TxData, utUsb_RxData, 74u );
    TEST_ASSERT_BITS_LOW( USB_CH_VTRX, *Ut_Usb_Get_ChepReg( 1u ) );
}


/**
 * \brief   An IN transfer ends when the requested size is received (no zero length packet is expected).
 *
 * \par Expected results
 * - 128 bytes requested, two full packets: the transfer ends with the second one.
 */
void Ut_UsbHost_Xfer_In_RequestedSizeEndsTransfer( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    Ut_Usb_Fill( utUsb_TxData, 128u, 5u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 128u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 0 ], 64u, 0u );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 64 ], 64u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 128u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( utUsb_TxData, utUsb_RxData, 128u );
}


/**
 * \brief   A NAK answer of a bulk channel is retried by the controller.
 *
 * \par Expected results
 * - After the NAK event the transfer keeps running (no callback), the event flag is cleared.
 * - The data received after the retry end the transfer.
 */
void Ut_UsbHost_Xfer_Bulk_Nak_RetriedByController( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    Ut_Usb_Fill( utUsb_TxData, 16u, 2u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_NAK, NULL, 0u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_BUSY, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_BITS_LOW( USB_CH_VTRX, *Ut_Usb_Get_ChepReg( 1u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, utUsb_TxData, 16u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );

    /* The same for the OUT direction */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, utUsb_TxData, 16u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_NAK, USB_CHEP_NAK );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_BUSY, Ut_UsbHost_Get_ChannelState( 2u ) );
    TEST_ASSERT_BITS_LOW( USB_CH_VTTX, *Ut_Usb_Get_ChepReg( 2u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_ACK_SBUF, USB_CHEP_NAK );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 16u, utUsbHost_ChHistory[ 1 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 1 ].Result );
}


/**
 * \brief   A NAK answer of an interrupt channel ends the transfer (the caller polls again).
 *
 * \par Expected results
 * - NAK of an interrupt IN and OUT transaction: callback with the NAK result and no data, channel idle.
 * - The channel can be started again.
 */
void Ut_UsbHost_Xfer_Interrupt_Nak_ReportedAndChannelIdle( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 3u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_INTERRUPT, 8u );
    Ut_UsbHost_Open_Channel( 4u, USB_EP_ADDR_OUT( 1u ), USB_EP_TYPE_INTERRUPT, 8u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 3u, utUsb_RxData, 8u ) );

    Ut_UsbHost_EventIn( 3u, USB_CH_RX_NAK, NULL, 0u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, utUsbHost_ChHistory[ 0 ].ChId );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_NAK, utUsbHost_ChHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 3u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 3u, utUsb_RxData, 8u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 4u, USB_TOKEN_OUT, utUsb_TxData, 4u ) );

    Ut_UsbHost_EventOut( 4u, USB_CH_TX_NAK, 0u );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 4u, utUsbHost_ChHistory[ 1 ].ChId );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_NAK, utUsbHost_ChHistory[ 1 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 4u ) );
}


/**
 * \brief   A STALL answer ends the transfer.
 *
 * \par Expected results
 * - STALL of an IN and of an OUT transaction: callback with the STALL result, channel idle, flags cleared.
 */
void Ut_UsbHost_Xfer_Stall_ReportedAndChannelIdle( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_STALL, NULL, 0u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_STALL, utUsbHost_ChHistory[ 0 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_BITS_LOW( USB_CH_VTRX, *Ut_Usb_Get_ChepReg( 1u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, utUsb_TxData, 10u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_STALL, 0u );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_STALL, utUsbHost_ChHistory[ 1 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 2u ) );
    TEST_ASSERT_BITS_LOW( USB_CH_VTTX, *Ut_Usb_Get_ChepReg( 2u ) );
}


/**
 * \brief   A transaction with an error is started again, the third error in a row fails the transfer.
 *
 * \par Expected results
 * - IN: errors are retried (no callback, the packet is requested again), a good packet resets the error count
 *   (two errors before and two errors after it do not fail the transfer), three errors in a row end the transfer
 *   with the error result; the error flags are cleared.
 * - OUT: the same, the packet is sent again.
 */
void Ut_UsbHost_Xfer_Error_RetriedThenFails( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    Ut_Usb_Fill( utUsb_TxData, 128u, 7u );

    /* IN: two errors, a good packet, two errors, the last good packet */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 128u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );
    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_BITS_LOW( USB_CH_ERRRX, *Ut_Usb_Get_ChepReg( 1u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 1u ) & USB_CH_RX_STRX );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 0 ], 64u, 0u );
    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );
    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, &utUsb_TxData[ 64 ], 64u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 128u, utUsbHost_ChHistory[ 0 ].XferCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, utUsbHost_ChHistory[ 0 ].Result );

    /* IN: three errors in a row */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );
    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_ERROR, utUsbHost_ChHistory[ 1 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 1u ) );

    /* OUT: three errors in a row, the packet is sent again after each of the first two */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, utUsb_TxData, 64u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_VALID, USB_CH_ERRTX );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_BITS_LOW( USB_CH_ERRTX, *Ut_Usb_Get_ChepReg( 2u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_TX_VALID, *Ut_Usb_Get_ChepReg( 2u ) & USB_CH_TX_STTX );
    TEST_ASSERT_EQUAL_UINT32( 64u, Ut_Usb_Get_TxCnt( 2u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_VALID, USB_CH_ERRTX );
    Ut_UsbHost_EventOut( 2u, USB_CH_TX_VALID, USB_CH_ERRTX );

    TEST_ASSERT_EQUAL_UINT32( 3u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChHistory[ 2 ].ChId );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_ERROR, utUsbHost_ChHistory[ 2 ].Result );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 2u ) );
}


/**
 * \brief   Three consecutive errors flagged by the controller fail the transfer at once.
 *
 * \par Expected results
 * - IN with the three errors field set: callback with the error result after the first event.
 * - OUT with the three errors field set: the same.
 */
void Ut_UsbHost_Xfer_ThreeErrorsFlag_FailsAtOnce( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_VALID, NULL, 0u, USB_CH_ERRRX | ( 1u << 29u ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_ERROR, utUsbHost_ChHistory[ 0 ].Result );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, utUsb_TxData, 8u ) );

    Ut_UsbHost_EventOut( 2u, USB_CH_TX_VALID, USB_CH_ERRTX | ( 2u << 27u ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_ERROR, utUsbHost_ChHistory[ 1 ].Result );
}


/**
 * \brief   A transfer is stopped without a callback.
 *
 * \par Expected results
 * - Channel idle, channel disabled, no callback; the late event of the transaction is dropped.
 * - Stopping an idle channel is accepted.
 */
void Ut_UsbHost_Xfer_Stop_CancelsSilently( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, utUsb_TxData, 64u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 2u ) );

    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_IDLE, Ut_UsbHost_Get_ChannelState( 2u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 1u ) & USB_CH_RX_STRX );    /* toggle pattern VALID -> disabled */
    TEST_ASSERT_EQUAL_HEX32( USB_CH_TX_VALID, *Ut_Usb_Get_ChepReg( 2u ) & USB_CH_TX_STTX );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, utUsb_TxData, 8u, 0u );
    Ut_UsbHost_EventOut( 2u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_BITS_LOW( USB_CH_VTRX, *Ut_Usb_Get_ChepReg( 1u ) );
    TEST_ASSERT_BITS_LOW( USB_CH_VTTX, *Ut_Usb_Get_ChepReg( 2u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 5u ) );
}


/**
 * \brief   Invalid transfer requests are rejected.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR for: port not enabled, closed channel, busy channel, wrong direction of the channel, SETUP on a
 *   bulk channel, SETUP of another length than 8, IN token for the transmit function, data pointer without data.
 */
void Ut_UsbHost_Xfer_InvalidRequests_ReturnError( void )
{
    const uint8_t setupBytes[ 8 ] = { 0u };

    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );
    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );

    /* Port is connected but not enabled yet */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_SETUP, setupBytes, 8u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, utUsb_RxData, 8u ) );

    Ut_UsbHost_Enable();

    /* The port reset closed the channels */
    Ut_UsbHost_Open_Channel( 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 8u );
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 5u, USB_TOKEN_OUT, utUsb_TxData, 8u ) );    /* closed channel */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 5u, utUsb_RxData, 8u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, USB_EP_NUM_CNT, USB_TOKEN_OUT, utUsb_TxData, 8u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 1u, USB_TOKEN_OUT, utUsb_TxData, 8u ) );      /* IN channel */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 2u, utUsb_RxData, 8u ) );                      /* OUT channel */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_SETUP, setupBytes, 8u ) );       /* SETUP on bulk */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_SETUP, setupBytes, 7u ) );       /* SETUP length */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_IN, setupBytes, 8u ) );          /* IN token */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 2u, USB_TOKEN_OUT, NULL, 8u ) );               /* no data */
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, NULL, 8u ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 8u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 8u ) );                      /* busy */

    /* A control channel with a running transfer accepts neither direction */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, utUsb_RxData, 8u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_SETUP, setupBytes, 8u ) );
}


/**
 * \brief   The channel callback can start the next transfer of the channel.
 *
 * \par Expected results
 * - The channel is idle in the callback, the transfer started by the callback is accepted and running.
 */
void Ut_UsbHost_Xfer_CallbackStartsNextTransfer( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );

    utUsbHost_RestartSize = 64u;
    utUsbHost_RestartCh   = 1u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    Ut_UsbHost_EventIn( 1u, USB_CH_RX_ACK_SBUF, utUsb_TxData, 5u, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, utUsbHost_RestartResult );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_BUSY, Ut_UsbHost_Get_ChannelState( 1u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_CH_RX_VALID, *Ut_Usb_Get_ChepReg( 1u ) & USB_CH_RX_STRX );
}


/**
 * \brief   Inconsistent and stale transfer events are ignored.
 *
 * \par Expected results
 * - Event without the matching flag in the channel register, event of a channel out of range and event of a channel
 *   without a running transfer: no callback; the flag of the stale event is cleared.
 */
void Ut_UsbHost_Isr_InconsistentOrStaleEvent_Ignored( void )
{
    Ut_UsbHost_Init_Enabled();
    Ut_UsbHost_Open_Channel( 1u, USB_EP_ADDR_IN( 1u ), USB_EP_TYPE_BULK, 64u );
    Ut_UsbHost_Open_Channel( 2u, USB_EP_ADDR_OUT( 2u ), USB_EP_TYPE_BULK, 64u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 1u, utUsb_RxData, 64u ) );

    /* Flags of the channel register do not match the direction of the event (the status macros of the emulation set them) */
    *Ut_Usb_Get_ChepReg( 1u ) &= ~( USB_CH_VTRX | USB_CH_VTTX );

    Ut_Usb_RunIsr( USB_ISTR_CTR | USB_ISTR_DIR | 1u );
    Ut_Usb_RunIsr( USB_ISTR_CTR | 1u );

    /* Channel number out of range */
    Ut_Usb_RunIsr( USB_ISTR_CTR | USB_ISTR_DIR | 9u );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
    TEST_ASSERT_EQUAL( USB_CHANNEL_STATE_BUSY, Ut_UsbHost_Get_ChannelState( 1u ) );

    /* Event of a channel without a running transfer */
    Ut_UsbHost_EventOut( 2u, USB_CH_TX_ACK_SBUF, 0u );

    TEST_ASSERT_BITS_LOW( USB_CH_VTTX, *Ut_Usb_Get_ChepReg( 2u ) );

    Ut_UsbHost_EventIn( 2u, USB_CH_RX_ACK_SBUF, NULL, 0u, 0u );

    TEST_ASSERT_BITS_LOW( USB_CH_VTRX, *Ut_Usb_Get_ChepReg( 2u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsbHost_ChCnt );
}

#endif /* USB_DRD_FS */

#if defined(USB_DRD_FS)

/* ========================== LOCAL FUNCTIONS =============================== */

/** Stub of the NVIC handler registration - stores the USB interrupt handler */
static nvic_RequestState_t Ut_Usb_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_USB_DRD_FS, irqId );

    utUsb_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** Stub of the GPIO initialization - stores the pin configuration */
static gpio_RequestState_t Ut_Usb_GpioInitStub( gpio_Config_t * gpioConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( gpioConfig );
    TEST_ASSERT_TRUE( 2u > utUsb_GpioCnt );

    utUsb_GpioConfig[ utUsb_GpioCnt ] = *gpioConfig;
    utUsb_GpioCnt ++;

    return ( GPIO_REQUEST_OK );
}


/** Reset callback - counts calls */
static void Ut_Usb_ResetCallback( usb_PeriphId_t usbId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsb_ResetCnt ++;
}


/** Suspend callback - counts calls */
static void Ut_Usb_SuspendCallback( usb_PeriphId_t usbId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsb_SuspendCnt ++;
}


/** Resume callback - counts calls */
static void Ut_Usb_ResumeCallback( usb_PeriphId_t usbId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsb_ResumeCnt ++;
}


/** Start of frame callback - counts calls */
static void Ut_Usb_SofCallback( usb_PeriphId_t usbId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsb_SofCnt ++;
}


/** SETUP callback - counts calls and stores the packet */
static void Ut_Usb_SetupCallback( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );
    TEST_ASSERT_NOT_NULL( setupPacket );

    utUsb_SetupCnt ++;
    utUsb_LastSetup = *setupPacket;
}


/** Transfer callback - records the call */
static void Ut_Usb_XferCallback( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult )
{
    if( UT_USB_XFER_HISTORY > utUsb_XferCnt )
    {
        utUsb_XferHistory[ utUsb_XferCnt ].CallCnt = utUsb_XferCnt + 1u;
        utUsb_XferHistory[ utUsb_XferCnt ].UsbId   = usbId;
        utUsb_XferHistory[ utUsb_XferCnt ].EpAddr  = epAddr;
        utUsb_XferHistory[ utUsb_XferCnt ].XferCnt = xferCnt;
        utUsb_XferHistory[ utUsb_XferCnt ].Result  = xferResult;
    }

    utUsb_XferCnt ++;

    if( 0u < utUsb_RestartSize )
    {
        /* The endpoint is free in the callback - the application starts the next transfer */
        const usb_DataCnt_t restartSize = utUsb_RestartSize;

        utUsb_RestartSize   = 0u;
        utUsb_RestartResult = Usb_Set_EpTxStart( usbId, utUsb_RestartEpAddr, &utUsb_TxData[ 10 ], restartSize );
    }
}


/** Error callback - counts calls and stores the error */
static void Ut_Usb_ErrorCallback( usb_PeriphId_t usbId, usb_ErrorId_t errorId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsb_ErrorCnt ++;
    utUsb_LastError = errorId;
}


/** Default configuration for the given servicing and clock source with the test callbacks */
static usb_BusConfig_t Ut_Usb_Get_Config( usb_ServiceMode_t serviceMode, usb_ClkSource_t clkSource )
{
    usb_BusConfig_t config;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DefaultConfig( &config ) );

    config.ServiceMode = serviceMode;
    config.ClkSource   = clkSource;
    config.IrqPriority = UT_USB_IRQ_PRIO;
    config.Callbacks   = &utUsb_Callbacks;

    return ( config );
}


/** RCC identification of the kernel clock of the clock source */
static rcc_PeriphId_t Ut_Usb_Get_RccId( usb_ClkSource_t clkSource )
{
    rcc_PeriphId_t rccId = RCC_PERIPH_USB_HSI48;

    if( USB_CLK_SOURCE_PLL1Q == clkSource )
    {
        rccId = RCC_PERIPH_USB_PLL1Q;
    }
    else if( USB_CLK_SOURCE_HSI48 == clkSource )
    {
        rccId = RCC_PERIPH_USB_HSI48;
    }
    else
    {
#if defined(RCC_CR_PLL3ON)
        rccId = RCC_PERIPH_USB_PLL3Q;
#else
        rccId = RCC_PERIPH_USB_PLL2Q;
#endif
    }

    return ( rccId );
}


/** Expects the read of the activation state of a kernel clock (every expectation has its own storage of the returned value) */
static void Ut_Usb_Expect_ClockState( rcc_PeriphId_t rccId, rcc_FunctionState_t clockState )
{
    TEST_ASSERT_TRUE( UT_USB_RET_STORAGE_CNT > utUsb_StateIdx );

    utUsb_StateStorage[ utUsb_StateIdx ] = clockState;

    Rcc_Get_PeriphState_ExpectAndReturn( rccId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &utUsb_StateStorage[ utUsb_StateIdx ] );

    utUsb_StateIdx ++;
}


/** Expects the read of the frequency of a kernel clock (every expectation has its own storage of the returned value) */
static void Ut_Usb_Expect_KernelClk( rcc_PeriphId_t rccId, rcc_FreqHz_t clockHz )
{
    TEST_ASSERT_TRUE( UT_USB_RET_STORAGE_CNT > utUsb_ClkIdx );

    utUsb_ClkStorage[ utUsb_ClkIdx ] = clockHz;

    Rcc_Get_PeriphClk_ExpectAndReturn( rccId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &utUsb_ClkStorage[ utUsb_ClkIdx ] );

    utUsb_ClkIdx ++;
}


/** Expects the calls of the clock, supply, trimming, reset and pin initialization of Usb_Init() */
static void Ut_Usb_Expect_InitResources( usb_ClkSource_t clkSource )
{
    const rcc_PeriphId_t rccId = Ut_Usb_Get_RccId( clkSource );

    Ut_Usb_Expect_ClockState( rccId, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Ut_Usb_Expect_KernelClk( rccId, 48000000u );
#if defined(PWR_USBSCR_USB33SV)
    Rcc_Set_PwrSupplyActive_ExpectAndReturn( RCC_PWR_SUPPLY_VDDUSB, RCC_REQUEST_OK );
#endif
    if( USB_CLK_SOURCE_HSI48 == clkSource )
    {
        Rcc_Set_Hsi48TrimActive_ExpectAndReturn( RCC_HSI48_TRIM_SRC_USB_SOF, RCC_REQUEST_OK );
    }
    Rcc_Set_ResetActive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
}


/** Expects the NVIC calls of Usb_Init() with interrupt servicing (handler registration is captured by the stub) */
static void Ut_Usb_Expect_InitIrq( void )
{
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_USB_DRD_FS, UT_USB_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_USB_DRD_FS, NVIC_REQUEST_OK );
}


/** Expects the calls of the release of the controller (Usb_Deinit() and the failed Usb_Init()) */
static void Ut_Usb_Expect_Release( usb_ClkSource_t clkSource, usb_FlagState_t trimActive, rcc_FunctionState_t clockState )
{
    const rcc_PeriphId_t rccId = Ut_Usb_Get_RccId( clkSource );

    Ut_Usb_Expect_ClockState( rccId, clockState );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_USB_DRD_FS, NVIC_REQUEST_OK );
    if( USB_FLAG_ACTIVE == trimActive )
    {
        Rcc_Set_Hsi48TrimInactive_ExpectAndReturn( RCC_REQUEST_OK );
    }
    Rcc_Set_ResetActive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( rccId, RCC_REQUEST_OK );
#if defined(PWR_USBSCR_USB33SV)
    Rcc_Set_PwrSupplyInactive_ExpectAndReturn( RCC_PWR_SUPPLY_VDDUSB, RCC_REQUEST_OK );
#endif
}


/** Initializes the controller (HSI48 kernel clock, test callbacks) */
static void Ut_Usb_Init( usb_ServiceMode_t serviceMode )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( serviceMode, USB_CLK_SOURCE_HSI48 );

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );

    if( USB_SERVICE_MODE_ISR == serviceMode )
    {
        Ut_Usb_Expect_InitIrq();
    }

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
}


/** Initializes the controller, connects the device and delivers the bus reset (endpoint 0 initialized, callbacks cleared) */
static void Ut_Usb_Init_Reset( void )
{
    Ut_Usb_Init( USB_SERVICE_MODE_ISR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_RESET );

    utUsb_ResetCnt = 0u;
}


/** Presets the interrupt status register and calls the interrupt handler registered in NVIC */
static void Ut_Usb_RunIsr( uint32_t istrValue )
{
    TEST_ASSERT_NOT_NULL( utUsb_Isr );

    USB_DRD_FS->ISTR = istrValue;

    utUsb_Isr();
}


/** Endpoint register of an endpoint number */
static volatile uint32_t * Ut_Usb_Get_ChepReg( usb_EpNum_t epNum )
{
    return ( &USB_DRD_FS->CHEP0R + epNum );
}


/** Address of the transmit buffer from the buffer descriptor table */
static uint32_t Ut_Usb_Get_TxAddr( usb_EpNum_t epNum )
{
    return ( (USB_DRD_PMA_BUFF + epNum)->TXBD & 0xFFFFu );
}


/** Address of the receive buffer from the buffer descriptor table */
static uint32_t Ut_Usb_Get_RxAddr( usb_EpNum_t epNum )
{
    return ( (USB_DRD_PMA_BUFF + epNum)->RXBD & 0xFFFFu );
}


/** Count of bytes of the transmit packet from the buffer descriptor table */
static uint32_t Ut_Usb_Get_TxCnt( usb_EpNum_t epNum )
{
    return ( ( (USB_DRD_PMA_BUFF + epNum)->TXBD & UT_USB_BD_COUNT_MASK ) >> UT_USB_BD_COUNT_POS );
}


/** Writes bytes to the packet memory */
static void Ut_Usb_Pma_Set( uint32_t pmaAddr, const uint8_t * data, uint32_t dataLen )
{
    volatile uint8_t * const pmaBytes = (volatile uint8_t *)(uintptr_t)( USB_DRD_PMAADDR + pmaAddr );

    for( uint32_t byteIdx = 0u; dataLen > byteIdx; byteIdx ++ )
    {
        pmaBytes[ byteIdx ] = data[ byteIdx ];
    }
}


/** Reads bytes from the packet memory */
static void Ut_Usb_Pma_Get( uint32_t pmaAddr, uint8_t * data, uint32_t dataLen )
{
    const volatile uint8_t * const pmaBytes = (const volatile uint8_t *)(uintptr_t)( USB_DRD_PMAADDR + pmaAddr );

    for( uint32_t byteIdx = 0u; dataLen > byteIdx; byteIdx ++ )
    {
        data[ byteIdx ] = pmaBytes[ byteIdx ];
    }
}


/** Fills the buffer by a counting pattern */
static void Ut_Usb_Fill( uint8_t * data, uint32_t dataLen, uint8_t seed )
{
    for( uint32_t byteIdx = 0u; dataLen > byteIdx; byteIdx ++ )
    {
        data[ byteIdx ] = (uint8_t)( seed + byteIdx );
    }
}


/**
 * Emulates the reception of a packet by the controller: the data are stored to the receive buffer, the count
 * to the buffer descriptor, the flags to the endpoint register and the interrupt status register, then
 * the interrupt handler is called.
 */
static void Ut_Usb_Receive( usb_EpNum_t epNum, const uint8_t * data, uint32_t dataLen, usb_FlagState_t isSetup )
{
    const uint32_t rxAddr   = Ut_Usb_Get_RxAddr( epNum );
    uint32_t       chepFlag = USB_EP_VTRX;

    if( 0u < dataLen )
    {
        Ut_Usb_Pma_Set( rxAddr, data, dataLen );
    }

    (USB_DRD_PMA_BUFF + epNum)->RXBD = ( (USB_DRD_PMA_BUFF + epNum)->RXBD & ~UT_USB_BD_COUNT_MASK ) | ( dataLen << UT_USB_BD_COUNT_POS );

    if( USB_FLAG_ACTIVE == isSetup )
    {
        chepFlag |= USB_EP_SETUP;
    }
    else
    {
        /* SETUP flag is read-only in the controller (cleared with the receive flag), plain RAM keeps it */
        *Ut_Usb_Get_ChepReg( epNum ) &= ~USB_EP_SETUP;
    }

    *Ut_Usb_Get_ChepReg( epNum ) |= chepFlag;

    Ut_Usb_RunIsr( USB_ISTR_CTR | USB_ISTR_DIR | epNum );
}


/** Emulates the end of the transmission of a packet: the flag is set in the endpoint register and the handler is called */
static void Ut_Usb_TxDone( usb_EpNum_t epNum )
{
    *Ut_Usb_Get_ChepReg( epNum ) |= USB_EP_VTTX;

    Ut_Usb_RunIsr( USB_ISTR_CTR | epNum );
}


/** Initializes an endpoint and checks the result */
static void Ut_Usb_Open_Ep( usb_EpAddr_t epAddr, usb_EpType_t epType, usb_EpMaxPacket_t maxPacket )
{
    usb_EpConfig_t epConfig;

    epConfig.Address       = epAddr;
    epConfig.Type          = epType;
    epConfig.MaxPacketSize = maxPacket;
    epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );
}

/** Host connect callback - counts calls */
static void Ut_UsbHost_ConnectCallback( usb_PeriphId_t usbId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsbHost_ConnectCnt ++;
}


/** Host disconnect callback - counts calls */
static void Ut_UsbHost_DisconnectCallback( usb_PeriphId_t usbId )
{
    TEST_ASSERT_EQUAL( USB_PERIPH_FS, usbId );

    utUsbHost_DisconnectCnt ++;
}


/** Host channel callback - records the call, starts the IN transfer if requested */
static void Ut_UsbHost_ChannelCallback( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t xferCnt, usb_ChannelResult_t chResult )
{
    if( UT_USB_XFER_HISTORY > utUsbHost_ChCnt )
    {
        utUsbHost_ChHistory[ utUsbHost_ChCnt ].CallCnt = utUsbHost_ChCnt + 1u;
        utUsbHost_ChHistory[ utUsbHost_ChCnt ].UsbId   = usbId;
        utUsbHost_ChHistory[ utUsbHost_ChCnt ].ChId    = chId;
        utUsbHost_ChHistory[ utUsbHost_ChCnt ].XferCnt = xferCnt;
        utUsbHost_ChHistory[ utUsbHost_ChCnt ].Result  = chResult;
    }

    utUsbHost_ChCnt ++;

    if( 0u < utUsbHost_RestartSize )
    {
        /* The channel is idle in the callback - the application starts the next transfer */
        const usb_DataCnt_t restartSize = utUsbHost_RestartSize;

        utUsbHost_RestartSize   = 0u;
        utUsbHost_RestartResult = Usb_Set_ChannelRxStart( usbId, utUsbHost_RestartCh, utUsb_RxData, restartSize );
    }
}


/** Configuration of the host mode with the given servicing and callbacks */
static usb_BusConfig_t Ut_UsbHost_Get_Config( usb_ServiceMode_t serviceMode, const usb_Callbacks_t * callbacks )
{
    usb_BusConfig_t config = Ut_Usb_Get_Config( serviceMode, USB_CLK_SOURCE_HSI48 );

    config.Mode      = USB_MODE_HOST;
    config.Callbacks = callbacks;

    return ( config );
}


/** Initializes the controller in the host mode (HSI48 kernel clock, host test callbacks) */
static void Ut_UsbHost_Init( usb_ServiceMode_t serviceMode )
{
    usb_BusConfig_t config = Ut_UsbHost_Get_Config( serviceMode, &utUsbHost_Callbacks );

    Ut_Usb_Expect_InitResources( USB_CLK_SOURCE_HSI48 );

    if( USB_SERVICE_MODE_ISR == serviceMode )
    {
        Ut_Usb_Expect_InitIrq();
    }

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
}


/** Delivers the attach of a device (full or low speed) to the interrupt handler */
static void Ut_UsbHost_Connect( usb_FlagState_t lowSpeed )
{
    uint32_t istrValue = USB_ISTR_DCON | USB_ISTR_DCON_STAT;

    if( USB_FLAG_ACTIVE == lowSpeed )
    {
        istrValue |= USB_ISTR_LS_DCONN;
    }

    Ut_Usb_RunIsr( istrValue );
}


/** Resets the port and delivers the first start of frame (port enabled) */
static void Ut_UsbHost_Enable( void )
{
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );

    Ut_Usb_RunIsr( USB_ISTR_SOF );
}


/** Initializes the host mode, attaches a full speed device and enables the port */
static void Ut_UsbHost_Init_Enabled( void )
{
    Ut_UsbHost_Init( USB_SERVICE_MODE_ISR );
    Ut_UsbHost_Connect( USB_FLAG_INACTIVE );
    Ut_UsbHost_Enable();

    utUsb_SofCnt = 0u;
}


/** Initializes a channel of the device with address 1 (full speed) and checks the result */
static void Ut_UsbHost_Open_Channel( usb_ChannelId_t chId, usb_EpAddr_t epAddr, usb_EpType_t epType, usb_EpMaxPacket_t maxPacket )
{
    usb_ChannelConfig_t chConfig;

    chConfig.ChannelId     = chId;
    chConfig.Address       = epAddr;
    chConfig.Type          = epType;
    chConfig.DevAddress    = 1u;
    chConfig.DevSpeed      = USB_SPEED_FULL;
    chConfig.MaxPacketSize = maxPacket;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
}


/** State of a channel (the read is checked) */
static usb_ChannelState_t Ut_UsbHost_Get_ChannelState( usb_ChannelId_t chId )
{
    usb_ChannelState_t chState = USB_CHANNEL_STATE_CNT;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_ChannelState( USB_PERIPH_FS, chId, &chState ) );

    return ( chState );
}


/**
 * Emulates the end of an IN transaction: the data are stored to the receive buffer, the count to the buffer
 * descriptor, the answer of the device (status) and the flags to the channel register and the interrupt
 * status register, then the interrupt handler is called.
 */
static void Ut_UsbHost_EventIn( usb_ChannelId_t chId, usb_RegValue_t statusRx, const uint8_t * data, uint32_t dataLen, usb_RegValue_t flags )
{
    const uint32_t rxAddr = Ut_Usb_Get_RxAddr( chId );

    if( 0u < dataLen )
    {
        Ut_Usb_Pma_Set( rxAddr, data, dataLen );
    }

    (USB_DRD_PMA_BUFF + chId)->RXBD = ( (USB_DRD_PMA_BUFF + chId)->RXBD & ~UT_USB_BD_COUNT_MASK ) | ( dataLen << UT_USB_BD_COUNT_POS );

    *Ut_Usb_Get_ChepReg( chId ) = ( *Ut_Usb_Get_ChepReg( chId ) & ~USB_CH_RX_STRX ) | statusRx | USB_CH_VTRX | flags;

    Ut_Usb_RunIsr( USB_ISTR_CTR | USB_ISTR_DIR | chId );
}


/**
 * Emulates the end of an OUT or SETUP transaction: the answer of the device (status) and the flags are set
 * in the channel register (the SETUP flag is cleared by the controller when the transaction is acknowledged),
 * then the interrupt handler is called.
 */
static void Ut_UsbHost_EventOut( usb_ChannelId_t chId, usb_RegValue_t statusTx, usb_RegValue_t flags )
{
    usb_RegValue_t regValue = *Ut_Usb_Get_ChepReg( chId );

    regValue &= ~USB_CH_TX_STTX;

    if( USB_CH_TX_ACK_SBUF == statusTx )
    {
        regValue &= ~USB_CHEP_SETUP;
    }

    *Ut_Usb_Get_ChepReg( chId ) = regValue | statusTx | USB_CH_VTTX | flags;

    Ut_Usb_RunIsr( USB_ISTR_CTR | chId );
}

#endif /* USB_DRD_FS */
