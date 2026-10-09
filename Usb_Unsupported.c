/**
 * \author Mr.Nobody
 * \file Usb_Unsupported.c
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module for the devices without the USB_DRD_FS controller
 *
 * STM32H5E4, STM32H5E5, STM32H5F4 and STM32H5F5 have the USB OTG FS / HS controller instead of
 * the USB_DRD_FS controller. The public interface is kept (it is identical for all devices),
 * the functions return the error value of the module until the OTG controller is supported.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usb_Port.h"                       /* Own port file include          */
#include "Usb_Types.h"                      /* Module types definitions       */
/* ============================== TYPEDEFS ================================== */

#if !defined(USB_DRD_FS)

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Initialization of the USB controller is not supported
 *
 * \param usbConfig [in]: Pointer to the configuration structure (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Init( usb_BusConfig_t * const usbConfig )
{
    (void)usbConfig;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Deinitialization of the USB controller is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Deinit( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Main task of the USB module - nothing to do without the supported controller
 */
void Usb_Task( void )
{
    return;
}


/**
 * \brief Connection of the device is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_DeviceConnect( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Disconnection of the device is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_DeviceDisconnect( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Setting of the device address is not supported
 *
 * \param usbId   [in]: USB controller identification (not used)
 * \param devAddr [in]: Device address (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_DeviceAddress( usb_PeriphId_t usbId, usb_DevAddr_t devAddr )
{
    (void)usbId;
    (void)devAddr;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the device address is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param devAddr [out]: Device address (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_DeviceAddress( usb_PeriphId_t usbId, usb_DevAddr_t * const devAddr )
{
    (void)usbId;
    (void)devAddr;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the device state is not supported
 *
 * \param usbId     [in]: USB controller identification (not used)
 * \param devState [out]: Device state (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_DeviceState( usb_PeriphId_t usbId, usb_DeviceState_t * const devState )
{
    (void)usbId;
    (void)devState;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the frame number is not supported
 *
 * \param usbId        [in]: USB controller identification (not used)
 * \param frameNumber [out]: Frame number (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_FrameNumber( usb_PeriphId_t usbId, usb_FrameNumber_t * const frameNumber )
{
    (void)usbId;
    (void)frameNumber;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the bus speed is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param speed [out]: Bus speed (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_Speed( usb_PeriphId_t usbId, usb_Speed_t * const speed )
{
    (void)usbId;
    (void)speed;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Remote wakeup is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_RemoteWakeupActive( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Remote wakeup is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_RemoteWakeupInactive( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the SETUP packet is not supported
 *
 * \param usbId        [in]: USB controller identification (not used)
 * \param setupPacket [out]: SETUP packet (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_SetupPacket( usb_PeriphId_t usbId, usb_SetupPacket_t * const setupPacket )
{
    (void)usbId;
    (void)setupPacket;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Initialization of an endpoint is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param epConfig [in]: Endpoint configuration (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Init_Endpoint( usb_PeriphId_t usbId, const usb_EpConfig_t * const epConfig )
{
    (void)usbId;
    (void)epConfig;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Deinitialization of an endpoint is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param epAddr [in]: Endpoint address (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Deinit_Endpoint( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
{
    (void)usbId;
    (void)epAddr;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the endpoint state is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param epAddr   [in]: Endpoint address (not used)
 * \param epState [out]: Endpoint state (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_EpState( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_EpState_t * const epState )
{
    (void)usbId;
    (void)epAddr;
    (void)epState;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Stall of an endpoint is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param epAddr [in]: Endpoint address (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_EpStallActive( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
{
    (void)usbId;
    (void)epAddr;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Unstall of an endpoint is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param epAddr [in]: Endpoint address (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_EpStallInactive( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
{
    (void)usbId;
    (void)epAddr;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief IN transfer is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param epAddr [in]: Endpoint address (not used)
 * \param txData [in]: Data to send (not used)
 * \param txSize [in]: Count of bytes to send (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_EpTxStart( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, const usb_Data_t * const txData, usb_DataCnt_t txSize )
{
    (void)usbId;
    (void)epAddr;
    (void)txData;
    (void)txSize;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief OUT transfer is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param epAddr   [in]: Endpoint address (not used)
 * \param rxBuffer [in]: Buffer for the received data (not used)
 * \param rxSize   [in]: Size of the buffer (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_EpRxStart( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize )
{
    (void)usbId;
    (void)epAddr;
    (void)rxBuffer;
    (void)rxSize;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Stop of a transfer is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param epAddr [in]: Endpoint address (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_EpXferStop( usb_PeriphId_t usbId, usb_EpAddr_t epAddr )
{
    (void)usbId;
    (void)epAddr;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the transferred count is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param epAddr   [in]: Endpoint address (not used)
 * \param xferCnt [out]: Count of transferred bytes (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_EpXferCount( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t * const xferCnt )
{
    (void)usbId;
    (void)epAddr;
    (void)xferCnt;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the host port state is not supported
 *
 * \param usbId      [in]: USB controller identification (not used)
 * \param portState [out]: Port state (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_HostPortState( usb_PeriphId_t usbId, usb_HostPortState_t * const portState )
{
    (void)usbId;
    (void)portState;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reset of the host port is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_HostPortResetActive( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Release of the host port reset is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_HostPortResetInactive( usb_PeriphId_t usbId )
{
    (void)usbId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Initialization of a host channel is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param chConfig [in]: Channel configuration (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Init_Channel( usb_PeriphId_t usbId, const usb_ChannelConfig_t * const chConfig )
{
    (void)usbId;
    (void)chConfig;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Deinitialization of a host channel is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 * \param chId  [in]: Channel (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Deinit_Channel( usb_PeriphId_t usbId, usb_ChannelId_t chId )
{
    (void)usbId;
    (void)chId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the host channel state is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param chId     [in]: Channel (not used)
 * \param chState [out]: Channel state (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_ChannelState( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_ChannelState_t * const chState )
{
    (void)usbId;
    (void)chId;
    (void)chState;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Setting of the host channel data toggle is not supported
 *
 * \param usbId      [in]: USB controller identification (not used)
 * \param chId       [in]: Channel (not used)
 * \param dataToggle [in]: Data toggle (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_ChannelDataToggle( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataToggle_t dataToggle )
{
    (void)usbId;
    (void)chId;
    (void)dataToggle;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief SETUP / OUT transfer of a host channel is not supported
 *
 * \param usbId  [in]: USB controller identification (not used)
 * \param chId   [in]: Channel (not used)
 * \param token  [in]: Token (not used)
 * \param txData [in]: Data to send (not used)
 * \param txSize [in]: Count of bytes to send (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_ChannelTxStart( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Token_t token, const usb_Data_t * const txData, usb_DataCnt_t txSize )
{
    (void)usbId;
    (void)chId;
    (void)token;
    (void)txData;
    (void)txSize;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief IN transfer of a host channel is not supported
 *
 * \param usbId    [in]: USB controller identification (not used)
 * \param chId     [in]: Channel (not used)
 * \param rxBuffer [in]: Buffer for the received data (not used)
 * \param rxSize   [in]: Size of the buffer (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_ChannelRxStart( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize )
{
    (void)usbId;
    (void)chId;
    (void)rxBuffer;
    (void)rxSize;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Stop of a host channel transfer is not supported
 *
 * \param usbId [in]: USB controller identification (not used)
 * \param chId  [in]: Channel (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Set_ChannelXferStop( usb_PeriphId_t usbId, usb_ChannelId_t chId )
{
    (void)usbId;
    (void)chId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Reading of the transferred count of a host channel is not supported
 *
 * \param usbId   [in]: USB controller identification (not used)
 * \param chId    [in]: Channel (not used)
 * \param xferCnt [out]: Count of transferred bytes (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_Get_ChannelXferCount( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t * const xferCnt )
{
    (void)usbId;
    (void)chId;
    (void)xferCnt;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Initialization of the DM pin is not supported
 *
 * \param pinId [in]: Pin identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_InitDmGpio( usb_DmPin_t pinId )
{
    (void)pinId;

    return ( USB_REQUEST_ERROR );
}


/**
 * \brief Initialization of the DP pin is not supported
 *
 * \param pinId [in]: Pin identification (not used)
 *
 * \return Always \ref USB_REQUEST_ERROR
 */
usb_RequestState_t Usb_InitDpGpio( usb_DpPin_t pinId )
{
    (void)pinId;

    return ( USB_REQUEST_ERROR );
}

#endif /* !USB_DRD_FS */
