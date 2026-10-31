/**
 * \author Mr.Nobody
 * \file ItTest_UsbHost.c
 * \ingroup Usb
 * \brief Integration tests of the host mode of Universal Serial Bus (USB) module on target.
 *
 * The Usb module runs on the target with real dependent modules (no mocks). Every test case runs after
 * system reset of the target. Target specific code (entry point, initialization, fault handlers) is
 * in ItTarget_Usb.c.
 *
 * The controller test without a device (power-up and release of the host mode) runs with any board. The device
 * test needs a full speed USB device (a flash drive, a hub) connected to the user USB connector (PA11 / PA12)
 * with the 5 V supply of VBUS - it is ignored if no device is attached within a few seconds:
 * - NUCLEO-H503RB: VBUS is switched by the board (USB_FS_PWR_EN, PC10) after the solder bridges of the user
 *   USB connector CN3 are set for the host (SB1, SB7, SB14 ON, SB4, SB8 OFF - UM3121 Table 14). ST does not
 *   recommend the host mode on CN3 (UM3121 Table 19). Check the VBUS LED LD6 and a USB-C to USB-A adapter
 *   for the device; or use an external USB-A connector on PA11 / PA12 with 5 V from the board.
 * - other boards: the 5 V supply of the device is provided externally.
 *
 * The device test runs the whole chain on the Usb module without the USB stack: attach, debounce, port reset,
 * first start of frame, GET_DESCRIPTOR(device) with the default address, SET_ADDRESS and GET_DESCRIPTOR(device)
 * with the new address.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usb_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* Kernel clock check             */
#include "Tim_Port.h"                       /* Reference time base            */
#include "Gpio_Port.h"                      /* VBUS switch                    */
#include "Stm32_usb.h"                      /* USB registers                  */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 It_UsbHost_Init_RefTime     ( void );
static uint32_t             It_UsbHost_Get_TimeMs       ( void );
static void                 It_UsbHost_Wait_Ms          ( uint32_t waitMs );
static void                 It_UsbHost_Set_Vbus         ( gpio_FunctionState_t vbusState );
static void                 It_UsbHost_Init             ( void );
static usb_ChannelResult_t  It_UsbHost_Wait_Channel     ( uint32_t timeoutMs );
static usb_ChannelResult_t  It_UsbHost_Control          ( const uint8_t * setupBytes, uint8_t * dataBuffer, uint32_t dataLen );
static usb_RequestState_t   It_UsbHost_Wait_Attach      ( uint32_t timeoutMs );
static usb_RequestState_t   It_UsbHost_Reset_Port       ( void );

static void                 It_UsbHost_ConnectCallback    ( usb_PeriphId_t usbId );
static void                 It_UsbHost_DisconnectCallback ( usb_PeriphId_t usbId );
static void                 It_UsbHost_ChannelCallback    ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t xferCnt, usb_ChannelResult_t chResult );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Reference time base (free running timer, 1 MHz) */
#define IT_USBHOST_REF_TIM                  ( TIM_PERIPH_2 )

/** Count of timer ticks in one millisecond */
#define IT_USBHOST_TICKS_PER_MS             ( 1000u )

/** Time in which a device shall be attached (the device tests are ignored otherwise) */
#define IT_USBHOST_ATTACH_WAIT_MS           ( 5000u )

/** Debounce time of the attach (USB 2.0 specification 7.1.7.3) */
#define IT_USBHOST_DEBOUNCE_MS              ( 100u )

/** Time of the reset signaling of the port (the root port reset is at least 10 ms, 50 ms is used by the stacks) */
#define IT_USBHOST_RESET_MS                 ( 50u )

/** Recovery time of the device after the reset (at least 10 ms) */
#define IT_USBHOST_RECOVERY_MS              ( 20u )

/** Time in which the port shall be enabled (first start of frame) after the reset is released */
#define IT_USBHOST_ENABLE_WAIT_MS           ( 100u )

/** Time in which a stage of a control transfer shall finish (USB 2.0 specification 9.2.6.1: 500 ms for the data stage) */
#define IT_USBHOST_STAGE_TIMEOUT_MS         ( 500u )

/** Time given to the device after SET_ADDRESS (at least 2 ms) */
#define IT_USBHOST_SET_ADDRESS_WAIT_MS      ( 5u )

/** Address assigned to the device */
#define IT_USBHOST_DEV_ADDRESS              ( 5u )

/** Control channel */
#define IT_USBHOST_CTRL_CH                  ( 0u )

/** Length of the SETUP packet */
#define IT_USBHOST_SETUP_LEN                ( 8u )

/** Length of the device descriptor */
#define IT_USBHOST_DEV_DESC_LEN             ( 18u )

/** Size of the first read of the device descriptor on the default address (up to the maximal packet size of the endpoint 0) */
#define IT_USBHOST_DEV_DESC_FIRST_LEN       ( 8u )

/** Index of bMaxPacketSize0 in the device descriptor */
#define IT_USBHOST_DESC_IDX_MAX_PACKET      ( 7u )

/** Descriptor type: device */
#define IT_USBHOST_DESC_DEVICE              ( 0x01u )

/** Maximal count of the packets of a stage waited by the busy loops */
#define IT_USBHOST_LOOP_GUARD               ( 4000000u )

/** Bound of the loops waiting for the reference time [loops]: the longest wait (IT_USBHOST_ATTACH_WAIT_MS at 250 MHz,
 *  at least 10 clocks per loop) takes less than 125 million loops, so the bound never ends a wait earlier */
#define IT_USBHOST_WAIT_LOOPS               ( 1000000000u )

#if defined(IT_BOARD_NUCLEO_H503RB)
/** VBUS switch of the user USB connector (USB_FS_PWR_EN, UM3121 Table 8), active LOW (measured on the board: PA9, the VBUS sense, follows PC10 inverted) */
#define IT_USBHOST_VBUS_AVAILABLE           ( 1u )
#define IT_USBHOST_VBUS_PORT                ( GPIO_PORT_C )
#define IT_USBHOST_VBUS_PIN                 ( GPIO_PIN_ID_10 )
#define IT_USBHOST_VBUS_ACTIVE_LEVEL        ( GPIO_PIN_LEVEL_LOW )
#else
#define IT_USBHOST_VBUS_AVAILABLE           ( 0u )
#endif

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Count of the device attach events */
static volatile uint32_t            itUsbHost_ConnectCnt;

/** Count of the device detach events */
static volatile uint32_t            itUsbHost_DisconnectCnt;

/** Channel transfer finished */
static volatile uint32_t            itUsbHost_ChDone;

/** Result of the last channel transfer */
static volatile usb_ChannelResult_t itUsbHost_ChResult;

/** Count of bytes of the last channel transfer */
static volatile usb_DataCnt_t       itUsbHost_ChCnt;

/** Speed of the attached device */
static usb_Speed_t                  itUsbHost_DevSpeed;

/** Callbacks of the host mode */
static const usb_Callbacks_t        itUsbHost_Callbacks =
{
    .ConnectCallback    = It_UsbHost_ConnectCallback,
    .DisconnectCallback = It_UsbHost_DisconnectCallback,
    .ChannelCallback    = It_UsbHost_ChannelCallback,
};

/* ======================= SETUP / TEARDOWN ================================= */

void setUp( void )
{
    itUsbHost_ConnectCnt    = 0u;
    itUsbHost_DisconnectCnt = 0u;
    itUsbHost_ChDone        = 0u;
    itUsbHost_ChResult      = USB_CHANNEL_RESULT_CNT;
    itUsbHost_ChCnt         = 0u;
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* ======================== TEST CASES ====================================== */

/**
 * \brief   The controller powers up in the host mode and is released.
 *
 * \par Expected results
 * - Usb_Init() in the host mode: kernel clock 48 MHz, CNTR.HOST set, no power down, no reset signaling,
 *   the pull-down resistors of the DP / DM lines on, the port reports no device or an attached device.
 * - Usb_Deinit(): the clock of the controller is released (the registers of a peripheral without a clock are not accessible and read 0).
 */
void It_UsbHost_Init_PowersUpHostModeAndDeinit( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;
    rcc_FunctionState_t clkState  = RCC_FUNCTION_ACTIVE;

    It_UsbHost_Init_RefTime();
    It_UsbHost_Init();

    TEST_ASSERT_BITS_HIGH( USB_CNTR_HOST, USB_DRD_FS->CNTR );
    TEST_ASSERT_BITS_LOW( USB_CNTR_PDWN | USB_CNTR_USBRST, USB_DRD_FS->CNTR );
    /* Bit 15 of BCDR is the pull-up (DPPU) in the device mode and the pull-down (DPPD) in the host mode - the same bit */
    TEST_ASSERT_BITS_HIGH( USB_BCDR_DPPD, USB_DRD_FS->BCDR );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_TRUE( ( USB_HOST_PORT_DISCONNECTED == portState ) ||
                      ( USB_HOST_PORT_CONNECTED    == portState )    );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit( USB_PERIPH_FS ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( RCC_PERIPH_USB_HSI48, &clkState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clkState );
}


/**
 * \brief   A device is attached, the port is reset and the device descriptor is read.
 *
 * \par Expected results
 * - Attach reported, the debounce, the reset and the first start of frame (port enabled) follow.
 * - GET_DESCRIPTOR(device) of the first 8 bytes on the address 0: bMaxPacketSize0 is 8, 16, 32 or 64.
 * - SET_ADDRESS accepted, GET_DESCRIPTOR(device) of 18 bytes on the new address returns a valid device descriptor.
 * - Ignored if no device is attached.
 */
void It_UsbHost_Device_AttachResetAndReadDeviceDescriptor( void )
{
    const uint8_t       getDescFirst[ IT_USBHOST_SETUP_LEN ] = { 0x80u, 0x06u, 0x00u, IT_USBHOST_DESC_DEVICE, 0x00u, 0x00u, IT_USBHOST_DEV_DESC_FIRST_LEN, 0x00u };
    const uint8_t       getDescFull [ IT_USBHOST_SETUP_LEN ] = { 0x80u, 0x06u, 0x00u, IT_USBHOST_DESC_DEVICE, 0x00u, 0x00u, IT_USBHOST_DEV_DESC_LEN, 0x00u };
    const uint8_t       setAddress  [ IT_USBHOST_SETUP_LEN ] = { 0x00u, 0x05u, IT_USBHOST_DEV_ADDRESS, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };
    uint8_t             descriptor[ IT_USBHOST_DEV_DESC_LEN ] = { 0u };
    usb_ChannelConfig_t chConfig;
    usb_HostPortState_t portState   = USB_HOST_PORT_CNT;
    usb_RequestState_t  attachState = USB_REQUEST_ERROR;

    It_UsbHost_Init_RefTime();
    It_UsbHost_Set_Vbus( GPIO_FUNCTION_ACTIVE );
    It_UsbHost_Init();

    attachState = It_UsbHost_Wait_Attach( IT_USBHOST_ATTACH_WAIT_MS );

    if( USB_REQUEST_OK != attachState )
    {
        It_UsbHost_Set_Vbus( GPIO_FUNCTION_INACTIVE );

        TEST_IGNORE_MESSAGE( "No USB device attached (connect a full speed device to the user USB connector, VBUS 5 V)" );
    }
    else
    {
        /* Device attached */
    }

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_Speed( USB_PERIPH_FS, &itUsbHost_DevSpeed ) );

    /* Debounce, reset signaling, first start of frame, recovery time */
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, It_UsbHost_Reset_Port() );
    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_HOST_PORT_ENABLED, portState );

    /* Default control pipe: device address 0, packet size 8 */
    chConfig.ChannelId     = IT_USBHOST_CTRL_CH;
    chConfig.Address       = USB_EP_ADDR_OUT( 0u );
    chConfig.Type          = USB_EP_TYPE_CONTROL;
    chConfig.DevAddress    = 0u;
    chConfig.DevSpeed      = itUsbHost_DevSpeed;
    chConfig.MaxPacketSize = 8u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, It_UsbHost_Control( getDescFirst, descriptor, IT_USBHOST_DEV_DESC_FIRST_LEN ) );

    TEST_ASSERT_EQUAL_UINT8( IT_USBHOST_DESC_DEVICE, descriptor[ 1 ] );
    TEST_ASSERT_TRUE( ( 8u  == descriptor[ IT_USBHOST_DESC_IDX_MAX_PACKET ] ) ||
                      ( 16u == descriptor[ IT_USBHOST_DESC_IDX_MAX_PACKET ] ) ||
                      ( 32u == descriptor[ IT_USBHOST_DESC_IDX_MAX_PACKET ] ) ||
                      ( 64u == descriptor[ IT_USBHOST_DESC_IDX_MAX_PACKET ] )    );

    /* Address assignment (no data stage), then the control pipe of the addressed device */
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, It_UsbHost_Control( setAddress, NULL, 0u ) );

    It_UsbHost_Wait_Ms( IT_USBHOST_SET_ADDRESS_WAIT_MS );

    chConfig.DevAddress    = IT_USBHOST_DEV_ADDRESS;
    chConfig.MaxPacketSize = descriptor[ IT_USBHOST_DESC_IDX_MAX_PACKET ];

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL( USB_CHANNEL_RESULT_OK, It_UsbHost_Control( getDescFull, descriptor, IT_USBHOST_DEV_DESC_LEN ) );

    TEST_ASSERT_EQUAL_UINT32( IT_USBHOST_DEV_DESC_LEN, itUsbHost_ChCnt );
    TEST_ASSERT_EQUAL_UINT8( IT_USBHOST_DEV_DESC_LEN, descriptor[ 0 ] );
    TEST_ASSERT_EQUAL_UINT8( IT_USBHOST_DESC_DEVICE, descriptor[ 1 ] );
    TEST_ASSERT_TRUE( 0u < descriptor[ 17 ] );                      /* bNumConfigurations */

    TEST_ASSERT_EQUAL_UINT32( 0u, itUsbHost_DisconnectCnt );

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Deinit( USB_PERIPH_FS ) );

    It_UsbHost_Set_Vbus( GPIO_FUNCTION_INACTIVE );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/** Starts the free running reference timer (1 MHz) */
static void It_UsbHost_Init_RefTime( void )
{
    tim_PeriphConfig_t timConfig;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_DefaultConfig( &timConfig ) );
    timConfig.PeriphId         = IT_USBHOST_REF_TIM;
    timConfig.TimerFrequency   = 1000000u;
    timConfig.RefreshFrequency = 1u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Init( &timConfig ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Period( IT_USBHOST_REF_TIM, 0xFFFFFFFFu ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Start( IT_USBHOST_REF_TIM ) );
}


/** Time since the start of the reference timer in milliseconds */
static uint32_t It_UsbHost_Get_TimeMs( void )
{
    tim_Counter_t counter = 0u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_USBHOST_REF_TIM, &counter ) );

    return ( (uint32_t)counter / IT_USBHOST_TICKS_PER_MS );
}


/** Waits for the given time, the controller is served meanwhile */
static void It_UsbHost_Wait_Ms( uint32_t waitMs )
{
    const uint32_t startMs   = It_UsbHost_Get_TimeMs();
    uint32_t       elapsedMs = It_UsbHost_Get_TimeMs() - startMs;

    for( uint32_t loopIdx = 0u;
         ( IT_USBHOST_WAIT_LOOPS > loopIdx ) &&
         ( waitMs > elapsedMs );
         loopIdx++ )
    {
        Usb_Task();

        elapsedMs = It_UsbHost_Get_TimeMs() - startMs;
    }
}


/** Switches the VBUS supply of the port (board specific, nothing is done if the board has no switch) */
static void It_UsbHost_Set_Vbus( gpio_FunctionState_t vbusState )
{
#if ( 1u == IT_USBHOST_VBUS_AVAILABLE )
    gpio_Config_t vbusConfig = { 0u };

    vbusConfig.PortId         = IT_USBHOST_VBUS_PORT;
    vbusConfig.PinId          = IT_USBHOST_VBUS_PIN;
    vbusConfig.PinMode        = GPIO_PIN_MODE_OUTPUT;
    vbusConfig.PinPull        = GPIO_PIN_PULL_NONE;
    vbusConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
    vbusConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    vbusConfig.PinActiveLevel = IT_USBHOST_VBUS_ACTIVE_LEVEL;

    TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Init( &vbusConfig ) );

    if( GPIO_FUNCTION_ACTIVE == vbusState )
    {
        TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Set_PinStateActive( IT_USBHOST_VBUS_PORT, IT_USBHOST_VBUS_PIN, IT_USBHOST_VBUS_ACTIVE_LEVEL ) );
    }
    else
    {
        TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Set_PinStateInactive( IT_USBHOST_VBUS_PORT, IT_USBHOST_VBUS_PIN, IT_USBHOST_VBUS_ACTIVE_LEVEL ) );
    }
#else
    (void)vbusState;
#endif
}


/** Initializes the controller in the host mode served by Usb_Task() */
static void It_UsbHost_Init( void )
{
    usb_BusConfig_t usbConfig;
    rcc_FreqHz_t    kernelClk = 0u;

    itUsbHost_ConnectCnt    = 0u;
    itUsbHost_DisconnectCnt = 0u;
    itUsbHost_ChDone        = 0u;
    itUsbHost_ChResult      = USB_CHANNEL_RESULT_CNT;
    itUsbHost_ChCnt         = 0u;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DefaultConfig( &usbConfig ) );

    usbConfig.Mode        = USB_MODE_HOST;
    usbConfig.ServiceMode = USB_SERVICE_MODE_POLL;
    usbConfig.Callbacks   = &itUsbHost_Callbacks;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Init( &usbConfig ) );
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphClk( RCC_PERIPH_USB_HSI48, &kernelClk ) );
    TEST_ASSERT_EQUAL_UINT32( 48000000u, kernelClk );
}


/** Waits for the finish of a channel transfer, returns its result (error result on a time out) */
static usb_ChannelResult_t It_UsbHost_Wait_Channel( uint32_t timeoutMs )
{
    const uint32_t      startMs   = It_UsbHost_Get_TimeMs();
    uint32_t            elapsedMs = It_UsbHost_Get_TimeMs() - startMs;
    usb_ChannelResult_t result    = USB_CHANNEL_RESULT_ERROR;

    for( uint32_t loopIdx = 0u;
         ( IT_USBHOST_WAIT_LOOPS > loopIdx ) &&
         ( 0u == itUsbHost_ChDone ) &&
         ( timeoutMs > elapsedMs );
         loopIdx++ )
    {
        Usb_Task();

        elapsedMs = It_UsbHost_Get_TimeMs() - startMs;
    }

    if( 0u != itUsbHost_ChDone )
    {
        result = itUsbHost_ChResult;
    }
    else
    {
        /* Time out */
    }

    itUsbHost_ChDone = 0u;

    return ( result );
}


/**
 * Control transfer on the control channel: SETUP stage, data stage (direction by the request, up to dataLen bytes)
 * and status stage (opposite direction, zero length). Returns the first result which is not ok. The count of bytes of the
 * data stage is in itUsbHost_ChCnt.
 */
static usb_ChannelResult_t It_UsbHost_Control( const uint8_t * setupBytes, uint8_t * dataBuffer, uint32_t dataLen )
{
    usb_FlagState_t       dataIn      = USB_FLAG_INACTIVE;
    usb_ChannelResult_t   chResult    = USB_CHANNEL_RESULT_OK;
    usb_DataCnt_t         dataCnt     = 0u;
    usb_RequestState_t    reqState    = USB_REQUEST_ERROR;

    if( 0u != ( setupBytes[ 0 ] & USB_EP_ADDR_DIR_IN ) )
    {
        dataIn = USB_FLAG_ACTIVE;
    }
    else
    {
        dataIn = USB_FLAG_INACTIVE;
    }

    itUsbHost_ChDone = 0u;

    reqState = Usb_Set_ChannelTxStart( USB_PERIPH_FS,
                                       IT_USBHOST_CTRL_CH,
                                       USB_TOKEN_SETUP,
                                       setupBytes,
                                       IT_USBHOST_SETUP_LEN );

    if( USB_REQUEST_OK != reqState )
    {
        chResult = USB_CHANNEL_RESULT_ERROR;
    }
    else
    {
        chResult = It_UsbHost_Wait_Channel( IT_USBHOST_STAGE_TIMEOUT_MS );
    }

    if( ( USB_CHANNEL_RESULT_OK == chResult ) &&
        ( 0u < dataLen                      )    )
    {
        if( USB_FLAG_ACTIVE == dataIn )
        {
            reqState = Usb_Set_ChannelRxStart( USB_PERIPH_FS, IT_USBHOST_CTRL_CH, dataBuffer, dataLen );

            if( USB_REQUEST_OK == reqState )
            {
                chResult = It_UsbHost_Wait_Channel( IT_USBHOST_STAGE_TIMEOUT_MS );
            }
            else
            {
                chResult = USB_CHANNEL_RESULT_ERROR;
            }
        }
        else
        {
            reqState = Usb_Set_ChannelTxStart( USB_PERIPH_FS, IT_USBHOST_CTRL_CH, USB_TOKEN_OUT, dataBuffer, dataLen );

            if( USB_REQUEST_OK == reqState )
            {
                chResult = It_UsbHost_Wait_Channel( IT_USBHOST_STAGE_TIMEOUT_MS );
            }
            else
            {
                chResult = USB_CHANNEL_RESULT_ERROR;
            }
        }

        dataCnt = itUsbHost_ChCnt;
    }
    else
    {
        /* No data stage */
    }

    if( USB_CHANNEL_RESULT_OK == chResult )
    {
        /* Status stage: the direction is opposite to the data stage, a request without a data stage has an IN status */
        if( ( USB_FLAG_ACTIVE == dataIn ) &&
            ( 0u < dataLen             )    )
        {
            reqState = Usb_Set_ChannelTxStart( USB_PERIPH_FS, IT_USBHOST_CTRL_CH, USB_TOKEN_OUT, NULL, 0u );

            if( USB_REQUEST_OK == reqState )
            {
                chResult = It_UsbHost_Wait_Channel( IT_USBHOST_STAGE_TIMEOUT_MS );
            }
            else
            {
                chResult = USB_CHANNEL_RESULT_ERROR;
            }
        }
        else
        {
            reqState = Usb_Set_ChannelRxStart( USB_PERIPH_FS, IT_USBHOST_CTRL_CH, NULL, 0u );

            if( USB_REQUEST_OK == reqState )
            {
                chResult = It_UsbHost_Wait_Channel( IT_USBHOST_STAGE_TIMEOUT_MS );
            }
            else
            {
                chResult = USB_CHANNEL_RESULT_ERROR;
            }
        }
    }
    else
    {
        /* Transfer failed - no status stage */
    }

    itUsbHost_ChCnt = dataCnt;

    return ( chResult );
}


/** Waits until a device is attached */
static usb_RequestState_t It_UsbHost_Wait_Attach( uint32_t timeoutMs )
{
    const uint32_t     startMs   = It_UsbHost_Get_TimeMs();
    uint32_t           elapsedMs = It_UsbHost_Get_TimeMs() - startMs;
    usb_RequestState_t retState  = USB_REQUEST_ERROR;

    for( uint32_t loopIdx = 0u;
         ( IT_USBHOST_WAIT_LOOPS > loopIdx ) &&
         ( 0u == itUsbHost_ConnectCnt ) &&
         ( timeoutMs > elapsedMs );
         loopIdx++ )
    {
        Usb_Task();

        elapsedMs = It_UsbHost_Get_TimeMs() - startMs;
    }

    if( 0u != itUsbHost_ConnectCnt )
    {
        retState = USB_REQUEST_OK;
    }
    else
    {
        retState = USB_REQUEST_ERROR;
    }

    return ( retState );
}


/** Debounce of the attach, reset signaling, wait for the first start of frame and the recovery time */
static usb_RequestState_t It_UsbHost_Reset_Port( void )
{
    usb_RequestState_t  retState  = USB_REQUEST_ERROR;
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;

    It_UsbHost_Wait_Ms( IT_USBHOST_DEBOUNCE_MS );

    retState = Usb_Set_HostPortResetActive( USB_PERIPH_FS );

    if( USB_REQUEST_OK == retState )
    {
        It_UsbHost_Wait_Ms( IT_USBHOST_RESET_MS );

        retState = Usb_Set_HostPortResetInactive( USB_PERIPH_FS );
    }
    else
    {
        /* Reset signaling not started */
    }

    if( USB_REQUEST_OK == retState )
    {
        const uint32_t startMs   = It_UsbHost_Get_TimeMs();
        uint32_t       elapsedMs = It_UsbHost_Get_TimeMs() - startMs;

        for( uint32_t loopIdx = 0u;
             ( IT_USBHOST_WAIT_LOOPS > loopIdx ) &&
             ( USB_HOST_PORT_ENABLED != portState ) &&
             ( IT_USBHOST_ENABLE_WAIT_MS > elapsedMs );
             loopIdx++ )
        {
            Usb_Task();
            retState = Usb_Get_HostPortState( USB_PERIPH_FS, &portState );

            elapsedMs = It_UsbHost_Get_TimeMs() - startMs;
        }

        if( USB_HOST_PORT_ENABLED != portState )
        {
            retState = USB_REQUEST_ERROR;
        }
        else
        {
            /* Port enabled */
        }
    }
    else
    {
        /* Reset signaling not finished */
    }

    if( USB_REQUEST_OK == retState )
    {
        It_UsbHost_Wait_Ms( IT_USBHOST_RECOVERY_MS );
    }
    else
    {
        /* Port not enabled */
    }

    return ( retState );
}


/** Attach callback - counts the events */
static void It_UsbHost_ConnectCallback( usb_PeriphId_t usbId )
{
    (void)usbId;

    itUsbHost_ConnectCnt ++;
}


/** Detach callback - counts the events */
static void It_UsbHost_DisconnectCallback( usb_PeriphId_t usbId )
{
    (void)usbId;

    itUsbHost_DisconnectCnt ++;
}


/** Channel callback - stores the result of the transfer */
static void It_UsbHost_ChannelCallback( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t xferCnt, usb_ChannelResult_t chResult )
{
    (void)usbId;
    (void)chId;

    itUsbHost_ChCnt    = xferCnt;
    itUsbHost_ChResult = chResult;
    itUsbHost_ChDone   = 1u;
}
