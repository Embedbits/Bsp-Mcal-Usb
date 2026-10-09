/**
 * \author Mr.Nobody
 * \file Test_UsbUnsupported.c
 * \ingroup Usb
 * \brief Unit tests of Universal Serial Bus (USB) module on the devices without the USB_DRD_FS controller.
 *
 * STM32H5E4, STM32H5E5, STM32H5F4 and STM32H5F5 have the USB OTG FS / HS controller. The module keeps
 * the interface of the other devices and returns errors (Usb_Unsupported.c). The unit tests of the
 * supported controller are in Test_Usb.c (the test set selects the test source by the device).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Usb_Port.h"                       /* Module under test              */
/* ========================== TEST FIXTURE ================================== */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
}


void tearDown( void )
{
    /* Nothing to verify */
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
 * - USB_REQUEST_OK and the default values, NULL pointer: USB_REQUEST_ERROR.
 */
void Ut_Usb_Get_DefaultConfig_ReturnsDefaults( void )
{
    usb_BusConfig_t config;

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( USB_PERIPH_FS,         config.PeriphId );
    TEST_ASSERT_EQUAL( USB_MODE_DEVICE,       config.Mode );
    TEST_ASSERT_EQUAL( USB_CLK_SOURCE_HSI48,  config.ClkSource );
    TEST_ASSERT_EQUAL( USB_SERVICE_MODE_ISR,  config.ServiceMode );
    TEST_ASSERT_EQUAL_UINT16( 64u,            config.Ep0MaxPacketSize );
    TEST_ASSERT_NULL( config.Callbacks );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DefaultConfig( NULL ) );
}

/* ============================ UNSUPPORTED LINES =========================== */
/**
 * \brief   Lines without the USB_DRD_FS controller keep the interface and return errors.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR from every function, Usb_Task() returns.
 */
void Ut_Usb_UnsupportedLine_AllFunctionsReturnError( void )
{
    usb_BusConfig_t      config;
    usb_EpConfig_t       epConfig = { 0 };
    usb_DevAddr_t        devAddr  = 0u;
    usb_DeviceState_t    devState = USB_DEVICE_STATE_CNT;
    usb_FrameNumber_t    frame    = 0u;
    usb_Speed_t          speed    = USB_SPEED_CNT;
    usb_SetupPacket_t    setup;
    usb_EpState_t        epState  = USB_EP_STATE_CNT;
    usb_DataCnt_t        xferCnt  = 0u;
    usb_Data_t           data[ 4 ] = { 0u };

    TEST_ASSERT_EQUAL( USB_REQUEST_OK, Usb_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init( &config ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit( USB_PERIPH_FS ) );
    Usb_Task();
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceConnect( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceDisconnect( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_DeviceAddress( USB_PERIPH_FS, 1u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceAddress( USB_PERIPH_FS, &devAddr ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_DeviceState( USB_PERIPH_FS, &devState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_FrameNumber( USB_PERIPH_FS, &frame ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_Speed( USB_PERIPH_FS, &speed ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_RemoteWakeupActive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_RemoteWakeupInactive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_SetupPacket( USB_PERIPH_FS, &setup ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Endpoint( USB_PERIPH_FS, &epConfig ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Endpoint( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpState( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &epState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpStallActive( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpStallInactive( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpTxStart( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), data, 4u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 1u ), data, 4u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_EpXferStop( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ) ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_EpXferCount( USB_PERIPH_FS, USB_EP_ADDR_IN( 1u ), &xferCnt ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_InitDmGpio( USB_DM_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_InitDpGpio( USB_DP_PIN_UNUSED ) );
}


/**
 * \brief   Lines without the USB_DRD_FS controller keep the host mode interface and return errors.
 *
 * \par Expected results
 * - USB_REQUEST_ERROR from every host port and channel function.
 */
void Ut_Usb_UnsupportedLine_HostFunctionsReturnError( void )
{
    usb_HostPortState_t portState = USB_HOST_PORT_CNT;
    usb_ChannelState_t  chState   = USB_CHANNEL_STATE_CNT;
    usb_ChannelConfig_t chConfig  = { 0u };
    usb_DataCnt_t       xferCnt   = 0u;
    usb_Data_t          data[ 4 ] = { 0u };

    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_HostPortState( USB_PERIPH_FS, &portState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetActive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_HostPortResetInactive( USB_PERIPH_FS ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Init_Channel( USB_PERIPH_FS, &chConfig ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Deinit_Channel( USB_PERIPH_FS, 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelState( USB_PERIPH_FS, 0u, &chState ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelDataToggle( USB_PERIPH_FS, 0u, USB_DATA_TOGGLE_DATA0 ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_OUT, data, 4u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelRxStart( USB_PERIPH_FS, 0u, data, 4u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Set_ChannelXferStop( USB_PERIPH_FS, 0u ) );
    TEST_ASSERT_EQUAL( USB_REQUEST_ERROR, Usb_Get_ChannelXferCount( USB_PERIPH_FS, 0u, &xferCnt ) );
}
