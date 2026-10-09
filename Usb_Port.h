/**
 * \author Mr.Nobody
 * \file Usb_Port.h
 * \ingroup Usb
 * \brief Universal Serial Bus (USB) MCAL module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef USB_USB_PORT_H
#define USB_USB_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Usb_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

usb_ModuleVersion_t     Usb_Get_ModuleVersion           ( void );

usb_RequestState_t      Usb_Init                        ( usb_BusConfig_t * const usbConfig );
usb_RequestState_t      Usb_Deinit                      ( usb_PeriphId_t usbId );
void                    Usb_Task                        ( void );

usb_RequestState_t      Usb_Get_DefaultConfig           ( usb_BusConfig_t * const usbConfig );

/*---------------------------- Device control --------------------------------*/

usb_RequestState_t      Usb_Set_DeviceConnect           ( usb_PeriphId_t usbId );
usb_RequestState_t      Usb_Set_DeviceDisconnect        ( usb_PeriphId_t usbId );

usb_RequestState_t      Usb_Set_DeviceAddress           ( usb_PeriphId_t usbId, usb_DevAddr_t devAddr );
usb_RequestState_t      Usb_Get_DeviceAddress           ( usb_PeriphId_t usbId, usb_DevAddr_t * const devAddr );

usb_RequestState_t      Usb_Get_DeviceState             ( usb_PeriphId_t usbId, usb_DeviceState_t * const devState );
usb_RequestState_t      Usb_Get_FrameNumber             ( usb_PeriphId_t usbId, usb_FrameNumber_t * const frameNumber );
usb_RequestState_t      Usb_Get_Speed                   ( usb_PeriphId_t usbId, usb_Speed_t * const speed );

usb_RequestState_t      Usb_Set_RemoteWakeupActive      ( usb_PeriphId_t usbId );
usb_RequestState_t      Usb_Set_RemoteWakeupInactive    ( usb_PeriphId_t usbId );

usb_RequestState_t      Usb_Get_SetupPacket             ( usb_PeriphId_t usbId, usb_SetupPacket_t * const setupPacket );

/*------------------------------- Endpoints ----------------------------------*/

usb_RequestState_t      Usb_Init_Endpoint               ( usb_PeriphId_t usbId, const usb_EpConfig_t * const epConfig );
usb_RequestState_t      Usb_Deinit_Endpoint             ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr );

usb_RequestState_t      Usb_Get_EpState                 ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_EpState_t * const epState );

usb_RequestState_t      Usb_Set_EpStallActive           ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr );
usb_RequestState_t      Usb_Set_EpStallInactive         ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr );

/*------------------------------- Transfers ----------------------------------*/

usb_RequestState_t      Usb_Set_EpTxStart               ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, const usb_Data_t * const txData, usb_DataCnt_t txSize );
usb_RequestState_t      Usb_Set_EpRxStart               ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize );
usb_RequestState_t      Usb_Set_EpXferStop              ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr );
usb_RequestState_t      Usb_Get_EpXferCount             ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t * const xferCnt );

/*----------------------------- Host mode port --------------------------------*/

usb_RequestState_t      Usb_Get_HostPortState           ( usb_PeriphId_t usbId, usb_HostPortState_t * const portState );

usb_RequestState_t      Usb_Set_HostPortResetActive     ( usb_PeriphId_t usbId );
usb_RequestState_t      Usb_Set_HostPortResetInactive   ( usb_PeriphId_t usbId );

/*---------------------------- Host mode channels ------------------------------*/

usb_RequestState_t      Usb_Init_Channel                ( usb_PeriphId_t usbId, const usb_ChannelConfig_t * const chConfig );
usb_RequestState_t      Usb_Deinit_Channel              ( usb_PeriphId_t usbId, usb_ChannelId_t chId );

usb_RequestState_t      Usb_Get_ChannelState            ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_ChannelState_t * const chState );

usb_RequestState_t      Usb_Set_ChannelDataToggle       ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataToggle_t dataToggle );

usb_RequestState_t      Usb_Set_ChannelTxStart          ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Token_t token, const usb_Data_t * const txData, usb_DataCnt_t txSize );
usb_RequestState_t      Usb_Set_ChannelRxStart          ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize );
usb_RequestState_t      Usb_Set_ChannelXferStop         ( usb_PeriphId_t usbId, usb_ChannelId_t chId );
usb_RequestState_t      Usb_Get_ChannelXferCount        ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t * const xferCnt );

/*--------------------- GPIO configuration functionality ---------------------*/

usb_RequestState_t      Usb_InitDmGpio                  ( usb_DmPin_t pinId );
usb_RequestState_t      Usb_InitDpGpio                  ( usb_DpPin_t pinId );

#ifdef __cplusplus
}
#endif

#endif /* USB_USB_PORT_H */
