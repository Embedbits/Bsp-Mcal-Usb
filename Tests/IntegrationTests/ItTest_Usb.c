/**
 * \author Mr.Nobody
 * \file ItTest_Usb.c
 * \ingroup Usb
 * \brief Integration tests of Universal Serial Bus (USB) module on target.
 *
 * Usb module runs on the target with real dependent modules (no mocks). Every test case runs after
 * system reset of the target. Target specific code (entry point, initialization, fault handlers) is
 * in ItTarget_Usb.c.
 *
 * Tests without a host (registers of the real controller, endpoint status toggling, packet memory,
 * pull-up) run with any board. The enumeration test needs the USB connector of the board (PA11 / PA12)
 * connected to a USB host (PC) - it is ignored if no host starts the enumeration within a few seconds.
 * The test firmware answers the enumeration through the Usb module only (vendor specific device
 * 1209:0001 with two bulk endpoints, no driver needed - the host shows an unknown device).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usb_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* Kernel clock check             */
#include "Tim_Port.h"                       /* Reference time base            */
#include "Stm32_usb.h"                      /* USB registers                  */
#include "Stm32_crs.h"                      /* CRS registers                  */
#include "Stm32_pwr.h"                      /* PWR registers                  */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 It_Usb_Init                 ( usb_ClkSource_t clkSource, const usb_Callbacks_t * const callbacks );
static void                 It_Usb_Init_RefTime         ( void );
static uint32_t             It_Usb_Get_ElapsedMs        ( uint32_t startCnt );
static volatile uint32_t *  It_Usb_Get_ChepReg          ( uint32_t epNum );
static void                 It_Usb_Open_Ep              ( usb_EpAddr_t epAddr, usb_EpType_t epType, usb_EpMaxPacket_t maxPacket );

static void                 It_Usb_ResetCallback        ( usb_PeriphId_t usbId );
static void                 It_Usb_SetupCallback        ( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket );
static void                 It_Usb_XferCallback         ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult );
static void                 It_Usb_Send_Control         ( const uint8_t * data, uint32_t dataLen, uint32_t requestedLen );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Reference time base (free running timer, 1 MHz) */
#define IT_USB_REF_TIM                      ( TIM_PERIPH_2 )

/** Count of timer ticks in one millisecond */
#define IT_USB_TICKS_PER_MS                 ( 1000u )

/** Time in which the host shall start the enumeration after the connection (first SETUP packet) */
#define IT_USB_HOST_DETECT_MS               ( 3000u )

/** Time in which the host shall finish the enumeration (device configured) after the first bus reset */
#define IT_USB_ENUMERATION_MS               ( 10000u )

/** Bound of the loops waiting for the host [loops]: the longest wait (IT_USB_ENUMERATION_MS at 250 MHz, at least
 *  10 clocks per loop) takes less than 250 million loops, so the bound never ends a wait earlier */
#define IT_USB_WAIT_LOOPS                   ( 1000000000u )

/** Kernel clock required by the controller */
#define IT_USB_KERNEL_CLK_HZ                ( 48000000u )

/** Address of the bulk IN endpoint of the test device */
#define IT_USB_EP_BULK_IN                   ( USB_EP_ADDR_IN( 1u ) )

/** Address of the bulk OUT endpoint of the test device */
#define IT_USB_EP_BULK_OUT                  ( USB_EP_ADDR_OUT( 1u ) )

/*------------------------- Standard request codes ----------------------------*/

#define IT_USB_REQ_GET_STATUS               ( 0x00u )
#define IT_USB_REQ_CLEAR_FEATURE            ( 0x01u )
#define IT_USB_REQ_SET_ADDRESS              ( 0x05u )
#define IT_USB_REQ_GET_DESCRIPTOR           ( 0x06u )
#define IT_USB_REQ_GET_CONFIGURATION        ( 0x08u )
#define IT_USB_REQ_SET_CONFIGURATION        ( 0x09u )
#define IT_USB_REQ_GET_INTERFACE            ( 0x0Au )
#define IT_USB_REQ_SET_INTERFACE            ( 0x0Bu )

/** Descriptor types (high byte of wValue of GET_DESCRIPTOR) */
#define IT_USB_DESC_DEVICE                  ( 0x01u )
#define IT_USB_DESC_CONFIGURATION           ( 0x02u )

/** Request recipient endpoint (bmRequestType bits 4:0) and the mask of the recipient */
#define IT_USB_RECIPIENT_MASK               ( 0x1Fu )
#define IT_USB_RECIPIENT_ENDPOINT           ( 0x02u )

/** Feature selector ENDPOINT_HALT */
#define IT_USB_FEATURE_ENDPOINT_HALT        ( 0x00u )

/** Size of the buffer for data stage replies */
#define IT_USB_REPLY_SIZE                   ( 2u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Count of the bus resets reported by the callback */
static volatile uint32_t    itUsb_ResetCnt;

/** Count of the SETUP packets received (the reset events alone do not prove a host - an open bus with the pull-up on produces them too) */
static volatile uint32_t    itUsb_SetupCnt;

/** Device is configured by the host (SET_CONFIGURATION with the value 1) */
static volatile uint32_t    itUsb_Configured;

/** Host read the whole configuration descriptor (hosts without a driver for the device do not configure it) */
static volatile uint32_t    itUsb_ConfigRead;

/** Count of requests answered by STALL */
static volatile uint32_t    itUsb_StallCnt;

/** Count of the control transfers with a data stage finished */
static volatile uint32_t    itUsb_ControlDoneCnt;

/** Value of the active configuration */
static volatile uint8_t     itUsb_ConfigValue;

/** Buffer for the short data stage replies (valid until the transfer finishes) */
static uint8_t              itUsb_Reply[ IT_USB_REPLY_SIZE ];

/** Device descriptor of the test device (vendor specific class, VID 0x1209 PID 0x0001 of pid.codes for tests) */
static const uint8_t        itUsb_DeviceDescriptor[] =
{
    0x12u,                  /* bLength                                   */
    IT_USB_DESC_DEVICE,     /* bDescriptorType                           */
    0x00u, 0x02u,           /* bcdUSB 2.00                               */
    0xFFu,                  /* bDeviceClass: vendor specific             */
    0x00u,                  /* bDeviceSubClass                           */
    0x00u,                  /* bDeviceProtocol                           */
    0x40u,                  /* bMaxPacketSize0                           */
    0x09u, 0x12u,           /* idVendor 0x1209                           */
    0x01u, 0x00u,           /* idProduct 0x0001                          */
    0x00u, 0x01u,           /* bcdDevice 1.00                            */
    0x00u,                  /* iManufacturer                             */
    0x00u,                  /* iProduct                                  */
    0x00u,                  /* iSerialNumber                             */
    0x01u                   /* bNumConfigurations                        */
};

/** Configuration descriptor: one vendor specific interface with a bulk IN and a bulk OUT endpoint */
static const uint8_t        itUsb_ConfigDescriptor[] =
{
    0x09u, IT_USB_DESC_CONFIGURATION, 0x20u, 0x00u, 0x01u, 0x01u, 0x00u, 0x80u, 0x32u,   /* configuration (32 bytes) */
    0x09u, 0x04u, 0x00u, 0x00u, 0x02u, 0xFFu, 0x00u, 0x00u, 0x00u,                       /* interface 0              */
    0x07u, 0x05u, 0x81u, 0x02u, 0x40u, 0x00u, 0x00u,                                     /* endpoint 1 IN, bulk 64   */
    0x07u, 0x05u, 0x01u, 0x02u, 0x40u, 0x00u, 0x00u                                      /* endpoint 1 OUT, bulk 64  */
};

/** Callbacks of the enumeration test */
static const usb_Callbacks_t itUsb_Callbacks =
{
    .ResetCallback   = It_Usb_ResetCallback,
    .SuspendCallback = USB_NULL_PTR,
    .ResumeCallback  = USB_NULL_PTR,
    .SofCallback     = USB_NULL_PTR,
    .SetupCallback   = It_Usb_SetupCallback,
    .XferCallback    = It_Usb_XferCallback,
    .ErrorCallback   = USB_NULL_PTR,
};

/* ======================= SETUP / TEARDOWN ================================= */

void setUp( void )
{
    itUsb_ResetCnt       = 0u;
    itUsb_SetupCnt       = 0u;
    itUsb_Configured     = 0u;
    itUsb_ConfigRead     = 0u;
    itUsb_StallCnt       = 0u;
    itUsb_ControlDoneCnt = 0u;
    itUsb_ConfigValue    = 0u;
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* =============================== TESTS ==================================== */

/**
 * \brief   Usb_Get_ModuleVersion() returns version of the module on target.
 *
 * \par Expected results
 * - Version is 1.0.0 (Major 1, Minor 0, Patch 0).
 */
void It_Usb_Get_ModuleVersion_ReturnsVersion( void )
{
    usb_ModuleVersion_t version = Usb_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}


/**
 * \brief   Usb_Init() powers up the real controller and Usb_Deinit() powers it down.
 *
 * \details Initialization with the HSI48 kernel clock, polling servicing, no host needed.
 *
 * \par Expected results
 * - Kernel clock 48 MHz, controller out of reset and power down, interrupts enabled in CNTR, pull-up off.
 * - HSI48 trimming by the clock recovery system running, VDDUSB supply valid (where it exists).
 * - After Usb_Deinit(): the clock of the controller is released (the controller was put in reset and power down before; the registers
 *   of a peripheral without a clock are not accessible and read 0).
 */
void It_Usb_Init_PowersUpAndDeinitPowersDown( void )
{
    rcc_FreqHz_t        kernelClk = 0u;
    rcc_FunctionState_t clkState  = RCC_FUNCTION_ACTIVE;

    It_Usb_Init( USB_CLK_SOURCE_HSI48, USB_NULL_PTR );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphClk( RCC_PERIPH_USB_HSI48, &kernelClk ) );
    TEST_ASSERT_EQUAL_UINT32( IT_USB_KERNEL_CLK_HZ, kernelClk );

    TEST_ASSERT_BITS_LOW( USB_CNTR_PDWN | USB_CNTR_USBRST | USB_CNTR_HOST, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_HIGH( USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_BCDR_DPPU, USB_DRD_FS->BCDR );

    TEST_ASSERT_BITS_HIGH( CRS_CR_AUTOTRIMEN | CRS_CR_CEN, CRS->CR );
#if defined(PWR_USBSCR_USB33SV)
    TEST_ASSERT_BITS_HIGH( PWR_USBSCR_USB33SV, PWR->USBSCR );
#endif

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit( USB_PERIPH_FS ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( RCC_PERIPH_USB_HSI48, &clkState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clkState );
}


/**
 * \brief   Soft connect switches the DP pull-up of the real controller.
 *
 * \par Expected results
 * - BCDR.DPPU set by Usb_Set_DeviceConnect(), cleared by Usb_Set_DeviceDisconnect().
 */
void It_Usb_DeviceConnect_SwitchesPullUp( void )
{
    It_Usb_Init( USB_CLK_SOURCE_HSI48, USB_NULL_PTR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_HIGH( USB_BCDR_DPPU, USB_DRD_FS->BCDR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceDisconnect( USB_PERIPH_FS ) );
    TEST_ASSERT_BITS_LOW( USB_BCDR_DPPU, USB_DRD_FS->BCDR );
}


/**
 * \brief   Endpoint registers of the real controller follow the status toggling of the module.
 *
 * \details Bulk IN, bulk OUT and interrupt IN endpoint are initialized (without a host), stalled and
 *          unstalled. The status fields of the endpoint registers toggle by writing 1 in the real
 *          controller, so the real registers prove the toggle handling.
 *
 * \par Expected results
 * - Initialized: transfer type and address set, STAT_TX / STAT_RX = NAK, data toggles cleared.
 * - Stall: STALL, unstall: NAK. Deinitialization: DISABLED.
 * - Receive buffer descriptor of the 64 byte OUT endpoint: 32-byte blocks (BLSIZE = 1, NUM_BLOCK = 1).
 */
void It_Usb_Endpoints_StatusToggleOnRealController( void )
{
    It_Usb_Init( USB_CLK_SOURCE_HSI48, USB_NULL_PTR );

    It_Usb_Open_Ep( IT_USB_EP_BULK_IN, USB_EP_TYPE_BULK, 64u );
    It_Usb_Open_Ep( IT_USB_EP_BULK_OUT, USB_EP_TYPE_BULK, 64u );
    It_Usb_Open_Ep( USB_EP_ADDR_IN( 2u ), USB_EP_TYPE_INTERRUPT, 8u );

    TEST_ASSERT_EQUAL_HEX32( USB_EP_BULK, *It_Usb_Get_ChepReg( 1u ) & USB_EP_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( 1u,          *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_ADDR );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_NAK, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_NAK, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_RX_STRX );
    TEST_ASSERT_BITS_LOW( USB_CHEP_DTOG_TX | USB_CHEP_DTOG_RX, *It_Usb_Get_ChepReg( 1u ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_INTERRUPT, *It_Usb_Get_ChepReg( 2u ) & USB_EP_UTYPE );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_NAK,    *It_Usb_Get_ChepReg( 2u ) & USB_CHEP_TX_STTX );

    TEST_ASSERT_BITS_HIGH( USB_CNTRX_BLSIZE, (USB_DRD_PMA_BUFF + 1u)->RXBD );
    TEST_ASSERT_EQUAL_UINT32( 1u, ( (USB_DRD_PMA_BUFF + 1u)->RXBD & USB_CNTRX_NBLK_MSK ) >> 26u );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallActive( USB_PERIPH_FS, IT_USB_EP_BULK_IN ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_STALL, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallActive( USB_PERIPH_FS, IT_USB_EP_BULK_OUT ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_STALL, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_RX_STRX );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallInactive( USB_PERIPH_FS, IT_USB_EP_BULK_IN ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_NAK, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpStallInactive( USB_PERIPH_FS, IT_USB_EP_BULK_OUT ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_NAK, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_RX_STRX );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit_Endpoint( USB_PERIPH_FS, IT_USB_EP_BULK_IN ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_DIS, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit_Endpoint( USB_PERIPH_FS, IT_USB_EP_BULK_OUT ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_RX_DIS, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_RX_STRX );
}


/**
 * \brief   Transmit data reach the packet memory of the real controller by 32-bit accesses.
 *
 * \details Bulk IN endpoint, 13 bytes (not a multiple of the word size) from an unaligned address are given
 *          to the controller (no host - the packet stays in the packet memory), then the transfer is stopped.
 *
 * \par Expected results
 * - Packet memory holds the 13 bytes, the packet length is 13, STAT_TX = VALID, endpoint busy.
 * - After Usb_Set_EpXferStop(): STAT_TX = NAK, endpoint idle.
 */
void It_Usb_Set_EpTxStart_DataInPacketMemory( void )
{
    static const uint8_t txData[ 14u ] = { 0xFFu, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u };
    const volatile uint8_t * pmaBytes = USB_NULL_PTR;
    usb_EpState_t            epState  = USB_EP_STATE_CNT;
    uint32_t                 pmaAddr  = 0u;

    It_Usb_Init( USB_CLK_SOURCE_HSI48, USB_NULL_PTR );
    It_Usb_Open_Ep( IT_USB_EP_BULK_IN, USB_EP_TYPE_BULK, 64u );

    pmaAddr  = (USB_DRD_PMA_BUFF + 1u)->TXBD & 0xFFFFu;
    pmaBytes = (const volatile uint8_t *)(uintptr_t)( USB_DRD_PMAADDR + pmaAddr );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpTxStart( USB_PERIPH_FS, IT_USB_EP_BULK_IN, &txData[ 1 ], 13u ) );

    for( uint32_t byteIdx = 0u; 13u > byteIdx; byteIdx++ )
    {
        TEST_ASSERT_EQUAL_HEX8( txData[ 1u + byteIdx ], pmaBytes[ byteIdx ] );
    }

    TEST_ASSERT_EQUAL_UINT32( 13u, ( (USB_DRD_PMA_BUFF + 1u)->TXBD >> 16u ) & 0x3FFu );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_VALID, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, IT_USB_EP_BULK_IN, &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_BUSY, epState );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_EpXferStop( USB_PERIPH_FS, IT_USB_EP_BULK_IN ) );
    TEST_ASSERT_EQUAL_HEX32( USB_EP_TX_NAK, *It_Usb_Get_ChepReg( 1u ) & USB_CHEP_TX_STTX );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, IT_USB_EP_BULK_IN, &epState ) );
    TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
}


/**
 * \brief   The host enumerates the device and configures it.
 *
 * \details The device connects to the bus and answers the standard requests of the host (descriptors, address,
 *          configuration) through the Usb module, serviced by Usb_Task(). The test needs the USB connector of the
 *          board connected to a host (PC). It is ignored if no SETUP packet (the first request of a host) arrives within a few
 *          seconds. The bus reset events do not prove a host: without a host (open bus, no VBUS) the suspend handling of the
 *          controller with the pull-up on produces them as well.
 *
 * \par Expected results
 * - SETUP packet within 3 s after the connection (otherwise the test is ignored: no host connected).
 * - Within 10 s after the reset: address assigned (device state addressed) and the whole configuration
 *   descriptor read by the host or SET_CONFIGURATION(1) received (hosts configure the device only if a driver
 *   is loaded for it; the bulk endpoints are initialized and idle after SET_CONFIGURATION).
 */
void It_Usb_Enumeration_HostConfiguresDevice( void )
{
    usb_DeviceState_t devState  = USB_DEVICE_STATE_CNT;
    usb_EpState_t     epState   = USB_EP_STATE_CNT;
    uint32_t          startCnt  = 0u;
    uint32_t          elapsedMs = 0u;

    It_Usb_Init_RefTime();
    It_Usb_Init( USB_CLK_SOURCE_HSI48, &itUsb_Callbacks );

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_USB_REF_TIM, &startCnt ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );

    for( uint32_t loopIdx = 0u;
         ( IT_USB_WAIT_LOOPS > loopIdx ) &&
         ( 0u == itUsb_SetupCnt ) &&
         ( IT_USB_HOST_DETECT_MS > elapsedMs );
         loopIdx++ )
    {
        Usb_Task();
        elapsedMs = It_Usb_Get_ElapsedMs( startCnt );
    }

    if( 0u == itUsb_SetupCnt )
    {
        TEST_IGNORE_MESSAGE( "No USB host connected (connect the board USB connector to a PC)" );
    }
    else
    {
        /* Host connected */
    }

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_USB_REF_TIM, &startCnt ) );
    elapsedMs = 0u;

    for( uint32_t loopIdx = 0u;
         ( IT_USB_WAIT_LOOPS > loopIdx ) &&
         ( 0u == itUsb_Configured ) &&
         ( 0u == itUsb_ConfigRead ) &&
         ( IT_USB_ENUMERATION_MS > elapsedMs );
         loopIdx++ )
    {
        Usb_Task();
        elapsedMs = It_Usb_Get_ElapsedMs( startCnt );
    }

    TEST_ASSERT_TRUE_MESSAGE( ( 0u != itUsb_Configured ) ||
                              ( 0u != itUsb_ConfigRead  ),
                              "Host did not read the configuration descriptor" );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_DEVICE_STATE_ADDRESSED, devState );
    TEST_ASSERT_TRUE( 0u < itUsb_ControlDoneCnt );

    if( 0u != itUsb_Configured )
    {
        TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, IT_USB_EP_BULK_IN, &epState ) );
        TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
        TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_EpState( USB_PERIPH_FS, IT_USB_EP_BULK_OUT, &epState ) );
        TEST_ASSERT_EQUAL( USB_EP_STATE_IDLE, epState );
    }
    else
    {
        /* The host does not configure a device without a driver (bulk endpoints are initialized by SET_CONFIGURATION) */
    }
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes the controller with polling servicing and checks the result.
 *
 * \param clkSource [in]: Kernel clock source
 * \param callbacks [in]: Event callbacks (USB_NULL_PTR - none)
 */
static void It_Usb_Init( usb_ClkSource_t clkSource, const usb_Callbacks_t * const callbacks )
{
    usb_BusConfig_t config;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DefaultConfig( &config ) );

    config.ClkSource    = clkSource;
    config.ServiceMode  = USB_SERVICE_MODE_POLL;
    config.Callbacks    = callbacks;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &config ) );
}


/** \brief Initializes and starts the reference time base (free running timer, 1 MHz) */
static void It_Usb_Init_RefTime( void )
{
    tim_PeriphConfig_t timConfig;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_DefaultConfig( &timConfig ) );
    timConfig.PeriphId         = IT_USB_REF_TIM;
    timConfig.TimerFrequency   = 1000000u;
    timConfig.RefreshFrequency = 1u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Init( &timConfig ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Period( IT_USB_REF_TIM, 0xFFFFFFFFu ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Start( IT_USB_REF_TIM ) );
}


/**
 * \brief Time elapsed from a timer value in milliseconds.
 *
 * \param startCnt [in]: Timer value at the start
 *
 * \return Elapsed time in milliseconds
 */
static uint32_t It_Usb_Get_ElapsedMs( uint32_t startCnt )
{
    tim_Counter_t counter = 0u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_USB_REF_TIM, &counter ) );

    return ( ( (uint32_t)counter - startCnt ) / IT_USB_TICKS_PER_MS );
}


/** \brief Endpoint register of an endpoint number */
static volatile uint32_t * It_Usb_Get_ChepReg( uint32_t epNum )
{
    return ( &USB_DRD_FS->CHEP0R + epNum );
}


/** \brief Initializes an endpoint and checks the result */
static void It_Usb_Open_Ep( usb_EpAddr_t epAddr, usb_EpType_t epType, usb_EpMaxPacket_t maxPacket )
{
    usb_EpConfig_t epConfig;

    epConfig.Address       = epAddr;
    epConfig.Type          = epType;
    epConfig.MaxPacketSize = maxPacket;
    epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );
}


/** \brief Bus reset callback - counts the resets (the module initialized the endpoint 0) */
static void It_Usb_ResetCallback( usb_PeriphId_t usbId )
{
    (void)usbId;

    itUsb_ResetCnt ++;
    itUsb_Configured  = 0u;
    itUsb_ConfigRead  = 0u;
    itUsb_ConfigValue = 0u;
}


/**
 * \brief Sends the data stage of a control read transfer (shortened to the requested length).
 *
 * \param data         [in]: Data
 * \param dataLen      [in]: Count of available bytes
 * \param requestedLen [in]: wLength of the request
 */
static void It_Usb_Send_Control( const uint8_t * data, uint32_t dataLen, uint32_t requestedLen )
{
    uint32_t           sendLen  = dataLen;
    usb_RequestState_t reqState = USB_REQUEST_ERROR;

    if( requestedLen < dataLen )
    {
        sendLen = requestedLen;
    }
    else
    {
        /* The whole data is sent */
    }

    reqState = Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 0u ), data, sendLen );

    if( USB_REQUEST_OK != reqState )
    {
        itUsb_StallCnt ++;
    }
    else
    {
        /* Data stage started */
    }
}


/**
 * \brief SETUP callback - the test device answers the standard requests of the enumeration.
 *
 * \param usbId       [in]: USB controller identification
 * \param setupPacket [in]: Received SETUP packet
 */
static void It_Usb_SetupCallback( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket )
{
    const uint8_t  recipient   = (uint8_t)( setupPacket->RequestType & IT_USB_RECIPIENT_MASK );
    const uint8_t  descType    = (uint8_t)( setupPacket->Value >> 8u );
    usb_FlagState_t handled    = USB_FLAG_INACTIVE;

    itUsb_SetupCnt ++;

    if( IT_USB_REQ_GET_DESCRIPTOR == setupPacket->Request )
    {
        if( IT_USB_DESC_DEVICE == descType )
        {
            It_Usb_Send_Control( itUsb_DeviceDescriptor, sizeof( itUsb_DeviceDescriptor ), setupPacket->Length );
            handled = USB_FLAG_ACTIVE;
        }
        else if( IT_USB_DESC_CONFIGURATION == descType )
        {
            It_Usb_Send_Control( itUsb_ConfigDescriptor, sizeof( itUsb_ConfigDescriptor ), setupPacket->Length );
            handled = USB_FLAG_ACTIVE;

            if( sizeof( itUsb_ConfigDescriptor ) <= setupPacket->Length )
            {
                itUsb_ConfigRead = 1u;
            }
            else
            {
                /* The host reads only the beginning of the descriptor */
            }
        }
        else
        {
            /* Strings, device qualifier and others are not supported (stall) */
        }
    }
    else if( IT_USB_REQ_SET_ADDRESS == setupPacket->Request )
    {
        const usb_DevAddr_t      devAddr   = (usb_DevAddr_t)( setupPacket->Value & USB_DEV_ADDR_MAX );
        const usb_RequestState_t addrState = Usb_Set_DeviceAddress( usbId, devAddr );

        if( USB_REQUEST_OK == addrState )
        {
            (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 0u ), USB_NULL_PTR, 0u );
            handled = USB_FLAG_ACTIVE;
        }
        else
        {
            /* Address not set (stall) */
        }
    }
    else if( IT_USB_REQ_SET_CONFIGURATION == setupPacket->Request )
    {
        if( 1u == setupPacket->Value )
        {
            usb_EpConfig_t     epConfig;
            usb_RequestState_t initState = USB_REQUEST_ERROR;

            epConfig.Address       = IT_USB_EP_BULK_IN;
            epConfig.Type          = USB_EP_TYPE_BULK;
            epConfig.MaxPacketSize = 64u;
            epConfig.BufferMode    = USB_EP_BUFFER_SINGLE;

            (void)Usb_Deinit_Endpoint( usbId, IT_USB_EP_BULK_IN );
            (void)Usb_Deinit_Endpoint( usbId, IT_USB_EP_BULK_OUT );

            initState = Usb_Init_Endpoint( usbId, &epConfig );

            if( USB_REQUEST_OK == initState )
            {
                epConfig.Address = IT_USB_EP_BULK_OUT;

                initState = Usb_Init_Endpoint( usbId, &epConfig );

                if( USB_REQUEST_OK == initState )
                {
                    itUsb_ConfigValue = 1u;
                    itUsb_Configured  = 1u;

                    (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 0u ), USB_NULL_PTR, 0u );
                    handled = USB_FLAG_ACTIVE;
                }
                else
                {
                    /* Bulk OUT endpoint not initialized (stall) */
                }
            }
            else
            {
                /* Bulk IN endpoint not initialized (stall) */
            }
        }
        else if( 0u == setupPacket->Value )
        {
            itUsb_ConfigValue = 0u;
            itUsb_Configured  = 0u;

            (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 0u ), USB_NULL_PTR, 0u );
            handled = USB_FLAG_ACTIVE;
        }
        else
        {
            /* Unknown configuration (stall) */
        }
    }
    else if( IT_USB_REQ_GET_CONFIGURATION == setupPacket->Request )
    {
        itUsb_Reply[ 0 ] = itUsb_ConfigValue;
        It_Usb_Send_Control( itUsb_Reply, 1u, setupPacket->Length );
        handled = USB_FLAG_ACTIVE;
    }
    else if( IT_USB_REQ_GET_STATUS == setupPacket->Request )
    {
        itUsb_Reply[ 0 ] = 0u;
        itUsb_Reply[ 1 ] = 0u;
        It_Usb_Send_Control( itUsb_Reply, IT_USB_REPLY_SIZE, setupPacket->Length );
        handled = USB_FLAG_ACTIVE;
    }
    else if( ( IT_USB_REQ_CLEAR_FEATURE == setupPacket->Request                 ) &&
             ( IT_USB_RECIPIENT_ENDPOINT == recipient                          ) &&
             ( IT_USB_FEATURE_ENDPOINT_HALT == setupPacket->Value              )    )
    {
        (void)Usb_Set_EpStallInactive( usbId, (usb_EpAddr_t)( setupPacket->Index & 0xFFu ) );
        (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 0u ), USB_NULL_PTR, 0u );
        handled = USB_FLAG_ACTIVE;
    }
    else if( IT_USB_REQ_GET_INTERFACE == setupPacket->Request )
    {
        itUsb_Reply[ 0 ] = 0u;
        It_Usb_Send_Control( itUsb_Reply, 1u, setupPacket->Length );
        handled = USB_FLAG_ACTIVE;
    }
    else if( ( IT_USB_REQ_SET_INTERFACE == setupPacket->Request ) &&
             ( 0u == setupPacket->Value                         )    )
    {
        (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 0u ), USB_NULL_PTR, 0u );
        handled = USB_FLAG_ACTIVE;
    }
    else
    {
        /* Not supported request (stall) */
    }

    if( USB_FLAG_ACTIVE != handled )
    {
        (void)Usb_Set_EpStallActive( usbId, USB_EP_ADDR_IN( 0u ) );
        (void)Usb_Set_EpStallActive( usbId, USB_EP_ADDR_OUT( 0u ) );

        itUsb_StallCnt ++;
    }
    else
    {
        /* Request is answered */
    }

}


/**
 * \brief Transfer callback - counts the finished data stages of the control transfers.
 *
 * \param usbId      [in]: USB controller identification
 * \param epAddr     [in]: Endpoint address
 * \param xferCnt    [in]: Count of transferred bytes
 * \param xferResult [in]: Result of the transfer
 */
static void It_Usb_XferCallback( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult )
{
    (void)usbId;

    if( ( USB_EP_ADDR_IN( 0u ) == epAddr                ) &&
        ( 0u                   <  xferCnt               ) &&
        ( USB_XFER_RESULT_OK   == xferResult            )    )
    {
        itUsb_ControlDoneCnt ++;
    }
    else
    {
        /* Status stage or aborted transfer */
    }
}
