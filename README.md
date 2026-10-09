# USB Peripheral Driver

This module provides an abstraction layer for **USB controllers** on STM32 MCUs - initialization, device control,
endpoints, data transfers and bus event callbacks. It is the low-level layer of a USB stack: the interface maps to the
USBX device and host controller drivers one to one, but it can be used directly by an application as well.

The public interface (`Usb_Port.h`, `Usb_Types.h`) is **identical for all supported families and controllers**. The
implementation for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the
released versions), so a middleware or an application written against the interface does not change when the MCU
family changes.

---

## Features

- Initialization and deinitialization of the controller: kernel clock (48 MHz, HSI48 with automatic trimming by the
  USB start of frame, or a PLL output), supply, pins, interrupt - all through the MCAL modules of the platform
  (RCC, GPIO, NVIC), the module itself touches only the registers of the USB controller
- **Device mode**: soft connect / disconnect, device address (applied after the status stage), device state, frame
  number, remote wakeup, bus reset / suspend / resume handling
- **Host mode**: one downstream port (attach / detach with the speed of the device, bus reset driven by the caller, start of
  frame) and up to eight channels - control, bulk and interrupt transfers (SETUP / OUT / IN) of any size with NAK retry by the
  controller, STALL and error reporting; the interface maps to the USBX host controller driver
- Endpoints: control, bulk and interrupt, both directions, automatic allocation of the packet memory, stall / unstall
  (unstall resets the data toggle), endpoint state
- Transfers of any size: the module splits the data to packets, ends the transfer by a short packet or the requested
  size and reports it by one callback (count, result ok / aborted / overflow)
- Event callbacks: bus reset, suspend, resume, start of frame, SETUP packet, transfer finished, error
- Servicing by the USB interrupt or by polling (`Usb_Task()`), callbacks are called only from the servicing context
- Parameter validation, hardware state checks and register write read-back in every function, single return point,
  Doxygen contract of every function
- Unit tests with emulated registers (no hardware needed) and integration tests on the target

Planned (separate stories): isochronous endpoints, double buffering, LPM and battery charging detection, suspend / resume of
the host port, OTG controllers.

---

## Supported families

| Family | Branch | Controller | Status |
|---|---|---|---|
| STM32H5 (H503, H523, H533, H543, H553, H562, H563, H573) | `Dev/STM32H5` | `USB_DRD_FS` (full speed, 8 endpoints / channels, 2 KB packet memory) | device and host mode - host mode verified on the NUCLEO-H503RB with a full speed hub and a flash drive (USBX + FileX), device mode: the controller tests pass on the board, the enumeration by a PC host is not verified yet |
| STM32H5 (H5E4, H5E5, H5F4, H5F5) | `Dev/STM32H5` | USB OTG FS / HS | interface kept, functions return an error |

---

## Public interface in short

| Group | Functions |
|---|---|
| Module management | `Usb_Get_ModuleVersion`, `Usb_Init`, `Usb_Deinit`, `Usb_Task`, `Usb_Get_DefaultConfig` |
| Device control | `Usb_Set_DeviceConnect` / `Disconnect`, `Usb_Set_DeviceAddress`, `Usb_Get_DeviceAddress`, `Usb_Get_DeviceState`, `Usb_Get_FrameNumber`, `Usb_Get_Speed`, `Usb_Set_RemoteWakeupActive` / `Inactive`, `Usb_Get_SetupPacket` |
| Endpoints | `Usb_Init_Endpoint`, `Usb_Deinit_Endpoint`, `Usb_Get_EpState`, `Usb_Set_EpStallActive` / `Inactive` |
| Transfers | `Usb_Set_EpTxStart`, `Usb_Set_EpRxStart`, `Usb_Set_EpXferStop`, `Usb_Get_EpXferCount` |
| Host port | `Usb_Get_HostPortState`, `Usb_Set_HostPortResetActive` / `Inactive` |
| Host channels | `Usb_Init_Channel`, `Usb_Deinit_Channel`, `Usb_Get_ChannelState`, `Usb_Set_ChannelDataToggle`, `Usb_Set_ChannelTxStart` / `RxStart` / `XferStop`, `Usb_Get_ChannelXferCount` |
| Pins | `Usb_InitDmGpio`, `Usb_InitDpGpio` |

Every function returns `USB_REQUEST_OK` / `USB_REQUEST_ERROR` and checks its parameters before it touches a register.
The detailed description of each function is in the Doxygen contract of the source files, the usage and the
family specifics are in the README of the family branch.

---

## Quick start (device)

```c
static const usb_Callbacks_t callbacks =
{
    .ResetCallback = Device_ResetCallback,   /* bus reset: endpoint 0 is open again               */
    .SetupCallback = Device_SetupCallback,   /* SETUP packet: answer by Usb_Set_EpTxStart / stall */
    .XferCallback  = Device_XferCallback,    /* transfer finished: start the next one             */
};

usb_BusConfig_t usbConfig;

(void)Usb_Get_DefaultConfig( &usbConfig );   /* full speed controller, HSI48, interrupt, PA11 / PA12 */
usbConfig.Callbacks = &callbacks;

(void)Usb_Init( &usbConfig );                /* controller powered, not visible for the host */
(void)Usb_Set_DeviceConnect( USB_PERIPH_FS );/* the host detects the device and resets the bus */
```

---

## USBX

The interface maps to the functions of the USBX device controller driver (create / destroy / reset / stall endpoint,
transfer request / abort, set address, frame number, change state, tasks run) and of the USBX host controller driver
(port status and reset, create / destroy / reset endpoint, transfer run / abort, tasks run), so a USBX controller driver is a
thin layer over this module. See the README of the family branch for the table.

---

## Repository layout (family branches; master holds this README and the license)

| Path | Content |
|---|---|
| `Usb_Port.h`, `Usb_Types.h` | Public interface |
| `Usb*.c`, `Usb.h` | Implementation (family branches) |
| `CMakeLists.txt` | Library `Usb_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
