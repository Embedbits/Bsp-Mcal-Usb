# USB Peripheral Driver

This module provides an abstraction layer for **USB controllers** on STM32 MCUs. It is the low-level layer of a
USB stack (e.g. the USBX device and host controller drivers): endpoint / channel based, with multi-packet
transfers, and independent of the packet memory / FIFO handling of the controller.

The public interface (`Usb_Port.h`, `Usb_Types.h`) is the same for all supported controllers and families - an
application or a middleware written against it does not change when the controller changes. Everything which
depends on the controller (registers, packet memory, clock tree, interrupt) is hidden in the module.

---

## Contents

- [Features](#features)
- [Supported devices](#supported-devices)
- [STM32H5 specifics](#stm32h5-specifics)
- [Architecture](#architecture)
- [Initialization and deinitialization](#initialization-and-deinitialization)
- [Configuration](#configuration)
- [Callbacks and servicing](#callbacks-and-servicing)
- [Device mode](#device-mode)
- [Host mode](#host-mode)
- [Packet memory](#packet-memory)
- [Public API](#public-api)
- [Usage](#usage)
- [USBX device controller driver](#usbx-device-controller-driver)
- [Error handling](#error-handling)
- [Testing](#testing)
- [Errata and known limitations](#errata-and-known-limitations)
- [Troubleshooting](#troubleshooting)
- [CMake Integration](#cmake-integration)

---

## Features

- Initialization and deinitialization of the controller (kernel clock, supply, pins, interrupt)
- Soft connect / disconnect (DP pull-up), device address, device state, frame number, remote wakeup
- Endpoints: control, bulk and interrupt, both directions, automatic allocation of the packet memory
- Stall / unstall of endpoints (unstall resets the data toggle), endpoint state
- Transfers of any size: `Usb_Set_EpTxStart()` / `Usb_Set_EpRxStart()` split the data to packets and report
  the end of the transfer by one callback (count, result ok / aborted / overflow)
- Event callbacks: bus reset, suspend, resume, start of frame, SETUP packet, transfer finished, error
- Servicing by the USB interrupt or by polling (`Usb_Task()`)
- **Host mode**: one downstream port (attach / detach with the device speed, bus reset driven by the caller, start
  of frame) and up to 8 channels (control, bulk and interrupt, IN / OUT / SETUP) with transfers of any size, NAK
  retry by the controller, STALL and error reporting
- No dynamic memory, no blocking waits except the bounded register read-back of the initialization
- Every register write is verified by read-back (exceptions are documented in `Usb.c`), every input parameter
  is validated, the API returns `USB_REQUEST_OK` / `USB_REQUEST_ERROR`

Not implemented yet (separate stories, the interface already contains the enumerations and rejects them):
isochronous endpoints and double buffering (device and host), LPM, battery charging detection, suspend / resume of
the host port, OTG controllers.

---

## Supported devices

| Devices | Controller | Status |
|---|---|---|
| STM32H503, H523, H533, H543, H553, H562, H563, H573 | `USB_DRD_FS` - full-speed host / device, 8 endpoints / channels, 2 KB packet memory | device and host mode supported |
| STM32H5E4, H5E5, H5F4, H5F5 | USB OTG FS / HS (different IP) | interface kept, every function returns `USB_REQUEST_ERROR` (`Usb_Unsupported.c`) |

The device differences are handled by preprocessor conditions identical to the ST CMSIS / LL headers
(`USB_DRD_FS`, `PWR_USBSCR_USB33SV`, `RCC_CR_PLL3ON`), the module compiles without warnings for all of them.

---

## STM32H5 specifics

- The 48 MHz kernel clock is selected by `usb_BusConfig_t::ClkSource` (HSI48, PLL1Q, PLL2Q / PLL3Q) and
  checked against 48 MHz. With HSI48 the module activates the automatic trimming by the USB start of
  frame (RCC module, clock recovery system CRS) - without it the free running HSI48 is not accurate enough
  for the USB timing.
- The APB2 clock of the controller must be at least 12 MHz (reference manual), the packet memory is accessed by
  32-bit words.
- The VDDUSB supply is validated through the RCC module (`PWR_USBSCR.USB33SV`). STM32H503 has no isolated
  VDDUSB domain (no `USB33SV`) - the step is skipped there.
- Pins: PA11 (DM / D-), PA12 (DP / D+), alternate function 10 (`usb_DmPin_t`, `usb_DpPin_t`). Both pins can be
  left unconfigured (`USB_DM_PIN_UNUSED`) if the application configures them.
- DP pull-up (soft connect) is a part of the battery charging register (`USB_BCDR.DPPU`).
- Packet memory: buffer descriptor table of 8 endpoints (64 bytes), packet buffers are allocated behind it
  (first fit). The receive buffer size follows the block structure of the descriptor (up to 62 bytes in
  2-byte blocks, larger buffers in 32-byte blocks).

---

## Architecture

```text
   Application / USBX controller driver
                  |  Usb_Port.h, Usb_Types.h
   +--------------+-------------------------------------------+
   |  Usb module                                              |
   |    Usb.c        init / deinit, device control, bus events|
   |    Usb_Ep.c     endpoint open / close / stall            |
   |    Usb_Xfer.c   SETUP decoding, IN / OUT transfers       |
   |    Usb_Host.c   host port, channels, host transfers      |
   |    Usb_Pma.c    packet memory access and allocation      |
   +-------+----------------+---------------+-----------------+
           |                |               |
          Rcc              Gpio            Nvic        (MCAL modules)
           |
   Ral: CMSIS + ST LL USB (registers of USB_DRD_FS, packet memory)
```

| File | Content |
|---|---|
| `Usb_Port.h` | Public functions (the only header used outside of the module) |
| `Usb_Types.h` | Public types, enumerations, endpoint address macros, pin encoding |
| `Usb.h` | Private declarations shared by the source files (not exported by CMake) |
| `Usb.c` | Version, default configuration, init / deinit, `Usb_Task()`, connect, address, state, speed, remote wakeup, GPIO, servicing of bus events (device mode), interrupt handler |
| `Usb_Ep.c` | Endpoint contexts, open / close, stall, state, packet size validation |
| `Usb_Xfer.c` | SETUP packet handling, transmit and receive of packets, multi-packet transfers, device address latch |
| `Usb_Host.c` | Host mode: controller power-up, attach / detach, port reset, channels, SETUP / OUT / IN transfers, servicing of the host events |
| `Usb_Pma.c` | Copy to / from the packet memory, first-fit allocation of the packet buffers, errata wait |
| `Usb_Unsupported.c` | The same functions for the devices without `USB_DRD_FS` (return error) |

Rules followed by the module (MCAL requirements GEN-01 .. GEN-08):

- Only the LL driver of the own peripheral is used; clocks, supply and trimming are taken through the RCC module,
  pins through the GPIO module, the interrupt through the NVIC module.
- A function checks its parameters and the hardware state before it writes a register; on a failed check no
  register is modified and `USB_REQUEST_ERROR` is returned.
- **Callbacks are called only from the servicing context** (USB interrupt or `Usb_Task()`), the API never calls
  a callback. The module does not disable interrupts: with interrupt servicing, start transfers from the
  callbacks or while the endpoint is idle.

---

## Initialization and deinitialization

`Usb_Init()` executes these steps in this order, any failure releases what was taken and returns an error:

1. A previous initialization of the controller is released (`Usb_Deinit()`).
2. The kernel clock is activated (RCC), its frequency is read and must be 48 MHz.
3. The VDDUSB supply is validated (where the device has it).
4. With HSI48: the automatic trimming by the USB start of frame is activated (RCC / CRS).
5. The peripheral is reset (RCC) and the DM / DP pins are configured (GPIO, AF10).
6. The controller is powered up (`CNTR.PDWN = 0` in the reset state), after the start-up time (64 register
   reads) the reset is released, pending flags are cleared, the pull-up is off, the address is 0 and the buffer
   descriptor table is cleared.
7. The interrupts of the transfers, bus events and errors are enabled (the start of frame interrupt only if
   `SofCallback` is set).
8. With interrupt servicing: handler, priority and enable of the interrupt in the NVIC.

The device is **not connected** to the bus after `Usb_Init()` - `Usb_Set_DeviceConnect()` is called when the
callbacks are ready.

Connect **only when a host is attached** (VBUS present): the module does not sense VBUS (the application does, e.g. with a
sense pin of the board). With the pull-up on and no host (open bus) the idle bus is detected as a suspend and the module then
reports bus resets again and again (measured on the NUCLEO-H503RB: about 12 `ResetCallback` calls per second) - they are not
a sign of a host, the first SETUP packet is.

In the **host mode** (`usb_BusConfig_t::Mode = USB_MODE_HOST`) steps 1 - 5 and 8 are the same; step 6 selects the host
mode while the controller is powered down, powers it up in the reset state, connects the pull-down resistors of the DP / DM
lines (`USB_BCDR.DPPD`) and releases the reset (the bus stays idle); step 7 enables the interrupts of the transfers, the
attach / detach and the packet memory errors (no suspend / resume / bus error interrupt). The 5 V VBUS supply of the port
is **not controlled by the module** - switch it on before or after `Usb_Init()` by the board (see [Host mode](#host-mode)).

`Usb_Deinit()` executes: interrupt off, running transfers cancelled (no callbacks), pull-up off, controller
stopped and powered down, peripheral reset, trimming stopped, clock and supply released. All steps are executed
even if one of them fails; the failure is reported.

---

## Configuration

`Usb_Get_DefaultConfig()` fills `usb_BusConfig_t` with: `USB_PERIPH_FS`, device mode, HSI48 kernel clock, interrupt
servicing with priority 0, endpoint 0 of 64 bytes, no callbacks, pins PA11 and PA12.

| Field | Meaning | Values |
|---|---|---|
| `PeriphId` | Controller | `USB_PERIPH_FS` |
| `Mode` | Operation mode | `USB_MODE_DEVICE`, `USB_MODE_HOST` |
| `ClkSource` | Source of the 48 MHz kernel clock | `USB_CLK_SOURCE_HSI48`, `USB_CLK_SOURCE_PLL1Q`, `USB_CLK_SOURCE_PLL2Q` / `USB_CLK_SOURCE_PLL3Q` (the PLL must be configured to 48 MHz by the RCC module before) |
| `ServiceMode` | Servicing | `USB_SERVICE_MODE_ISR` (interrupt), `USB_SERVICE_MODE_POLL` (`Usb_Task()`) |
| `IrqPriority` | NVIC priority of the USB interrupt (interrupt servicing) | device specific range |
| `Ep0MaxPacketSize` | Maximal packet size of the control endpoint 0 | 8, 16, 32, 64 |
| `Callbacks` | Event callbacks (copied by the module) | pointer, `USB_NULL_PTR` - no callbacks |
| `DmPin`, `DpPin` | Pins | `USB_DM_PIN_PA11` / `USB_DP_PIN_PA12`, `*_UNUSED` - not configured |

---

## Callbacks and servicing

Unused callbacks are set to `USB_NULL_PTR`. All callbacks are called from the servicing context.

| Callback | When |
|---|---|
| `ResetCallback` | Bus reset. Endpoints other than endpoint 0 were closed (the module reopens endpoint 0), the address is 0. |
| `SuspendCallback` | No bus activity for 3 ms (ignored while the device is disconnected - the controller flags it without a host). |
| `ResumeCallback` | Bus activity after suspend (also the remote wakeup). |
| `SofCallback` | Start of frame, every 1 ms. The interrupt is enabled only if the callback is set. |
| `SetupCallback` | SETUP packet on endpoint 0. Running endpoint 0 transfers were aborted. |
| `XferCallback` | A transfer started by `Usb_Set_EpTxStart()` / `Usb_Set_EpRxStart()` finished. The endpoint is free in the callback. |
| `ErrorCallback` | Controller error: bus error, packet memory overrun, SETUP packet with an invalid length. |
| `ConnectCallback` | Host: a device was attached to the port. The bus is not reset yet. |
| `DisconnectCallback` | Host: the device was detached. Running transfers were reported as aborted, all channels were closed. |
| `ChannelCallback` | Host: a transfer started by `Usb_Set_ChannelTxStart()` / `Usb_Set_ChannelRxStart()` finished (count, result). The channel is free in the callback. |

Servicing:

- **Interrupt** (`USB_SERVICE_MODE_ISR`): the module connects its handler to the USB interrupt in the NVIC.
- **Polling** (`USB_SERVICE_MODE_POLL`): the application calls `Usb_Task()` from the main loop. One call
  serves all pending transfer events (at most 64) and the bus events. The call rate has to keep up with the host:
  the controller answers NAK until a transfer is started, so a slow loop only slows the transfers down, but the
  SETUP stage of the enumeration is expected to be answered within 50 ms.
- One service call handles one transfer event (the controller shows one at a time); the controller keeps the
  interrupt request active while events are pending.

---

## Device mode

### Device state

```text
  DISCONNECTED --Usb_Set_DeviceConnect--> CONNECTED --bus reset--> DEFAULT --SET_ADDRESS(n != 0)--> ADDRESSED
        ^                                                              |  ^                            |
        +------------Usb_Set_DeviceDisconnect (from any state)---------+  +------- resume ---- SUSPENDED <-- suspend
```

- The address assigned by the host is applied by the module after the **status stage** of SET_ADDRESS is
  acknowledged (`Usb_Set_DeviceAddress()` may be called before or after the status stage is started). A new SETUP
  packet or a disconnect drops a pending address.
- Bus reset: running transfers are reported as aborted, endpoints other than endpoint 0 are closed, address 0,
  endpoint 0 is reopened (`Ep0MaxPacketSize`), `ResetCallback` is called. The host configures the device again
  and the application opens the other endpoints.
- Suspend / resume keep the state, `Usb_Get_DeviceState()` returns `USB_DEVICE_STATE_SUSPENDED` meanwhile.
- Remote wakeup: `Usb_Set_RemoteWakeupActive()` starts the resume signaling of a suspended device (only if the
  host enabled `DEVICE_REMOTE_WAKEUP`), the application waits 1 - 15 ms and calls `Usb_Set_RemoteWakeupInactive()`.

### Control transfers (endpoint 0)

| Stage | Application |
|---|---|
| SETUP packet | `SetupCallback` is called, running endpoint 0 transfers were aborted (reported by `XferCallback` with the aborted result). The endpoint answers NAK until a stage is started. |
| Data stage IN | `Usb_Set_EpTxStart( USB_EP_ADDR_IN( 0u ), data, size )`, `size` limited by the application to `wLength`. A multiple of the packet size shorter than `wLength` needs a zero length packet: another `Usb_Set_EpTxStart( ..., NULL, 0u )`. |
| Data stage OUT | `Usb_Set_EpRxStart( USB_EP_ADDR_OUT( 0u ), buffer, size )`. |
| Status stage IN | `Usb_Set_EpTxStart( USB_EP_ADDR_IN( 0u ), NULL, 0u )`. |
| Status stage OUT | The module makes the endpoint ready after an IN transfer. `Usb_Set_EpRxStart( USB_EP_ADDR_OUT( 0u ), NULL, 0u )` reports the end of the status stage by `XferCallback`. |
| SET_ADDRESS | `Usb_Set_DeviceAddress()` before or after starting the status stage; the address is applied when the status stage is acknowledged. |
| Unsupported request | Stall both directions: `Usb_Set_EpStallActive( USB_EP_ADDR_IN( 0u ) )` and `USB_EP_ADDR_OUT( 0u )` (cleared by the next SETUP packet). |

After a control transfer finishes (IN transfer ends, zero length OUT packet received) the endpoint 0 is ready to
receive the next SETUP packet automatically. A SETUP packet with a length other than 8 bytes stalls endpoint 0
and is reported by `ErrorCallback` (`USB_ERROR_SETUP_LENGTH`).

### Transfers of other endpoints

- IN: the data are sent in packets of the maximal packet size, the last packet is short. A multiple of the
  packet size is **not** followed by a zero length packet - start a zero size transfer for it.
- OUT: the transfer ends by a short packet or by the requested size. Data over the buffer size are dropped
  (result `USB_XFER_RESULT_OVERFLOW`). Size the buffer as a multiple of the packet size.
- Callback of a finished transfer is called when the endpoint is free, so it can start the next transfer.
- `Usb_Set_EpXferStop()` cancels a running transfer; the callback reports it as aborted.
- `Usb_Set_EpStallActive()` makes the endpoint answer STALL, `Usb_Set_EpStallInactive()` clears it and resets the
  data toggle (CLEAR_FEATURE(ENDPOINT_HALT)).

### Endpoints

| Type | Maximal packet size |
|---|---|
| control, bulk | 8, 16, 32, 64 |
| interrupt | 1 .. 64 |
| isochronous, double buffer | rejected (story AB#997) |

Both directions of one endpoint number use the same type. Endpoint states: `USB_EP_STATE_CLOSED`,
`USB_EP_STATE_IDLE` (NAK), `USB_EP_STATE_BUSY` (transfer running), `USB_EP_STATE_STALLED`.

---

## Host mode

### Port

```text
   DISCONNECTED --device attached--> CONNECTED --Usb_Set_HostPortResetActive--> RESET
                                                                                  |
                                              Usb_Set_HostPortResetInactive, first start of frame
                                                                                  v
                                                                               ENABLED

   Any state --device detached--> DISCONNECTED       ENABLED / RESET --Usb_Set_HostPortResetActive--> RESET
```

- `USB_HOST_PORT_CONNECTED`: a device is attached, `ConnectCallback` was called, the speed is readable by
  `Usb_Get_Speed()` (low speed or full speed). A device behind a hub is not seen by the controller - the hub is the device.
- The module **never waits**. The caller (e.g. the USBX host controller driver) keeps the timing of the USB specification:
  debounce of the attach 100 ms, reset signaling at least 10 ms (the stacks use 50 ms), recovery time at least 10 ms after the
  reset - `Usb_Set_HostPortResetActive()` starts the reset, `Usb_Set_HostPortResetInactive()` releases it.
- After the release the controller generates the start of frame; the first start of frame changes the state to
  `USB_HOST_PORT_ENABLED` (the start of frame interrupt is enabled for this meanwhile) - transfers are possible only then.
- The port reset closes all channels (the controller resets their registers): the channels are initialized again after
  the reset. A detach closes the channels and reports the running transfers as `USB_CHANNEL_RESULT_ABORTED`.
- The detach / attach events caused by the reset signaling of the module are ignored.

### Channels

A channel is the channel register of the controller (8 channels) with its packet buffers: the transmit buffer for the SETUP and OUT
transactions, the receive buffer for the IN transactions. `Usb_Init_Channel()` configures the target:

| Field of `usb_ChannelConfig_t` | Meaning |
|---|---|
| `ChannelId` | Channel 0 .. 7 |
| `Address`, `Type` | Endpoint of the device (`USB_EP_ADDR_IN( n )` / `USB_EP_ADDR_OUT( n )`) and its type: control (both directions), bulk, interrupt |
| `DevAddress`, `DevSpeed` | Address 0 .. 127 and speed (low, full) of the device. A low speed device behind a full speed hub is flagged (low speed preamble of the hub) |
| `MaxPacketSize` | Control / bulk 8, 16, 32, 64; interrupt 1 .. 64 |

- A bulk or interrupt channel has the buffer of its direction only, a control channel both. Packet buffers are allocated
  automatically (first fit, see [Packet memory](#packet-memory)).
- All control transfers of all devices can share **one control channel** (channel 0): an idle channel is initialized again
  for another device (address, packet size) - this is what the USB stacks do. Every other endpoint has its own channel.
- Data toggles: the controller toggles them after every acknowledged packet; a channel starts with DATA0. A control channel
  sets both toggles when a SETUP transaction is acknowledged (data and status stage start with DATA1). After the halt of a
  bulk / interrupt endpoint was cleared (CLEAR_FEATURE) the caller sets DATA0 by `Usb_Set_ChannelDataToggle()`.

### Transfers

| Stage | Call |
|---|---|
| SETUP | `Usb_Set_ChannelTxStart( usbId, ch, USB_TOKEN_SETUP, setupBytes, 8u )` |
| Data stage IN | `Usb_Set_ChannelRxStart( usbId, ch, buffer, size )` |
| Data stage OUT | `Usb_Set_ChannelTxStart( usbId, ch, USB_TOKEN_OUT, data, size )` |
| Status stage of a read | `Usb_Set_ChannelTxStart( usbId, ch, USB_TOKEN_OUT, NULL, 0u )` |
| Status stage of a write / no data request | `Usb_Set_ChannelRxStart( usbId, ch, NULL, 0u )` |

- A transfer is split to packets of the maximal packet size; OUT ends after the last acknowledged packet, IN ends by a
  short packet (also a zero length packet) or when `size` bytes were received. No zero length packet is added.
- The end is reported by **one** `ChannelCallback( usbId, ch, count, result )`; the channel is idle in the callback so the
  next stage can be started from there. `Usb_Set_ChannelXferStop()` cancels a transfer without a callback.

| Result | Meaning | What the caller does |
|---|---|---|
| `USB_CHANNEL_RESULT_OK` | Transfer finished | next stage |
| `USB_CHANNEL_RESULT_NAK` | Interrupt channel only: the device has no data | poll again after the interval of the endpoint |
| `USB_CHANNEL_RESULT_STALL` | The device answered STALL | control: stage failed; bulk: CLEAR_FEATURE(ENDPOINT_HALT) and `Usb_Set_ChannelDataToggle( DATA0 )` |
| `USB_CHANNEL_RESULT_ERROR` | Three consecutive transaction errors (no answer, CRC, bit stuffing) | retry or reset the port |
| `USB_CHANNEL_RESULT_ABORTED` | The device was detached | stop |

- **NAK of a bulk / control transaction** is retried by the controller at the start of the next frame without a callback
  (the transfer stays running); the caller's time-out aborts a transfer which never finishes.
- Errors: a failed transaction is started again, the third error in a row fails the transfer (`USB_CHANNEL_RESULT_ERROR`).

### VBUS and the board

The host supplies 5 V on VBUS to the device (and the hub). The module does not control it. For a device with its own power
(self powered hub) or a flash drive the host has to switch the supply on **before** the attach is expected:

- **NUCLEO-H503RB** (UM3121): the user USB-C connector CN3 (PA11 / PA12) has a VBUS switch controlled by `USB_FS_PWR_EN` (PC10),
  the VBUS sense is PA9, LD5 (red) is the overcurrent indicator and LD6 (green) the VBUS presence indicator. The board is
  delivered as a **device** (5.1 kOhm pull-downs on CC1 / CC2); the host configuration needs SB1, SB7, SB14 ON and SB4, SB8
  OFF (Table 14). **ST states that the host mode on CN3 must not be used - the hardware is not USB-C compliant (Table 19).**
  Use a USB-C to USB-A (host) adapter and check LD6, or wire an external USB-A connector to PA11 (D-), PA12 (D+) and GND with 5 V
  from the board.
- A hub downstream port is handled by the hub driver of the USB stack, not by this module.

### USBX host controller driver

The host interface maps to the commands of the USBX host controller driver (HCD, standalone mode):

| USBX HCD command | Usb module |
|---|---|
| `UX_HCD_GET_PORT_STATUS` | `Usb_Get_HostPortState()`, `Usb_Get_Speed()` |
| `UX_HCD_RESET_PORT` (state machine) | `Usb_Set_HostPortResetActive()` / `Usb_Set_HostPortResetInactive()` + the time of the caller |
| `UX_HCD_CREATE_ENDPOINT` / `UX_HCD_DESTROY_ENDPOINT` | `Usb_Init_Channel()` / `Usb_Deinit_Channel()` |
| `UX_HCD_RESET_ENDPOINT` | `Usb_Set_ChannelDataToggle( DATA0 )` |
| `UX_HCD_TRANSFER_RUN` | `Usb_Set_ChannelTxStart()` / `Usb_Set_ChannelRxStart()` + `ChannelCallback` |
| `UX_HCD_TRANSFER_ABORT` | `Usb_Set_ChannelXferStop()` |
| `UX_HCD_GET_FRAME_NUMBER` | `Usb_Get_FrameNumber()` |
| `UX_HCD_TASKS_RUN` | `Usb_Task()` |

---

## Packet memory

The 2 KB packet memory holds the buffer descriptor table (8 bytes per endpoint, 64 bytes in total) followed by the
packet buffers:

```text
 0x000  +----------------------+
        | buffer descriptors   |  64 bytes: TX address / count and RX address / count + size of every endpoint
 0x040  +----------------------+
        | endpoint 0 OUT (SETUP)|  64 bytes (Ep0MaxPacketSize)
        | endpoint 0 IN         |  64 bytes
        | buffers of the other  |  first fit, size rounded to 4 bytes (up to 62 bytes)
        | endpoints             |  or to 32 bytes (larger buffers)
 0x800  +----------------------+
```

A buffer is freed by `Usb_Deinit_Endpoint()` (or by the bus reset) and can be reused by the next endpoint. An
endpoint which does not fit returns `USB_REQUEST_ERROR` from `Usb_Init_Endpoint()` - the controller supports only
as many endpoints as fit into 2 KB (for example eight bulk endpoints with 64-byte packets use 512 bytes).

---

## Public API

### Module Management
- `usb_ModuleVersion_t     Usb_Get_ModuleVersion(void);`
- `usb_RequestState_t      Usb_Init(usb_BusConfig_t * const usbConfig);`
- `usb_RequestState_t      Usb_Deinit(usb_PeriphId_t usbId);`
- `void                    Usb_Task(void);`
- `usb_RequestState_t      Usb_Get_DefaultConfig(usb_BusConfig_t * const usbConfig);`

### Device Control
- `usb_RequestState_t      Usb_Set_DeviceConnect(usb_PeriphId_t usbId);`
- `usb_RequestState_t      Usb_Set_DeviceDisconnect(usb_PeriphId_t usbId);`
- `usb_RequestState_t      Usb_Set_DeviceAddress(usb_PeriphId_t usbId, usb_DevAddr_t devAddr);`
- `usb_RequestState_t      Usb_Get_DeviceAddress(usb_PeriphId_t usbId, usb_DevAddr_t * const devAddr);`
- `usb_RequestState_t      Usb_Get_DeviceState(usb_PeriphId_t usbId, usb_DeviceState_t * const devState);`
- `usb_RequestState_t      Usb_Get_FrameNumber(usb_PeriphId_t usbId, usb_FrameNumber_t * const frameNumber);`
- `usb_RequestState_t      Usb_Get_Speed(usb_PeriphId_t usbId, usb_Speed_t * const speed);`
- `usb_RequestState_t      Usb_Set_RemoteWakeupActive(usb_PeriphId_t usbId);`
- `usb_RequestState_t      Usb_Set_RemoteWakeupInactive(usb_PeriphId_t usbId);`
- `usb_RequestState_t      Usb_Get_SetupPacket(usb_PeriphId_t usbId, usb_SetupPacket_t * const setupPacket);`

### Endpoints
- `usb_RequestState_t      Usb_Init_Endpoint(usb_PeriphId_t usbId, const usb_EpConfig_t * const epConfig);`
- `usb_RequestState_t      Usb_Deinit_Endpoint(usb_PeriphId_t usbId, usb_EpAddr_t epAddr);`
- `usb_RequestState_t      Usb_Get_EpState(usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_EpState_t * const epState);`
- `usb_RequestState_t      Usb_Set_EpStallActive(usb_PeriphId_t usbId, usb_EpAddr_t epAddr);`
- `usb_RequestState_t      Usb_Set_EpStallInactive(usb_PeriphId_t usbId, usb_EpAddr_t epAddr);`

### Transfers
- `usb_RequestState_t      Usb_Set_EpTxStart(usb_PeriphId_t usbId, usb_EpAddr_t epAddr, const usb_Data_t * const txData, usb_DataCnt_t txSize);`
- `usb_RequestState_t      Usb_Set_EpRxStart(usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize);`
- `usb_RequestState_t      Usb_Set_EpXferStop(usb_PeriphId_t usbId, usb_EpAddr_t epAddr);`
- `usb_RequestState_t      Usb_Get_EpXferCount(usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t * const xferCnt);`

### Host Port
- `usb_RequestState_t      Usb_Get_HostPortState(usb_PeriphId_t usbId, usb_HostPortState_t * const portState);`
- `usb_RequestState_t      Usb_Set_HostPortResetActive(usb_PeriphId_t usbId);`
- `usb_RequestState_t      Usb_Set_HostPortResetInactive(usb_PeriphId_t usbId);`

`Usb_Get_Speed()` and `Usb_Get_FrameNumber()` work in both modes.

### Host Channels
- `usb_RequestState_t      Usb_Init_Channel(usb_PeriphId_t usbId, const usb_ChannelConfig_t * const chConfig);`
- `usb_RequestState_t      Usb_Deinit_Channel(usb_PeriphId_t usbId, usb_ChannelId_t chId);`
- `usb_RequestState_t      Usb_Get_ChannelState(usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_ChannelState_t * const chState);`
- `usb_RequestState_t      Usb_Set_ChannelDataToggle(usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataToggle_t dataToggle);`
- `usb_RequestState_t      Usb_Set_ChannelTxStart(usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Token_t token, const usb_Data_t * const txData, usb_DataCnt_t txSize);`
- `usb_RequestState_t      Usb_Set_ChannelRxStart(usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_Data_t * const rxBuffer, usb_DataCnt_t rxSize);`
- `usb_RequestState_t      Usb_Set_ChannelXferStop(usb_PeriphId_t usbId, usb_ChannelId_t chId);`
- `usb_RequestState_t      Usb_Get_ChannelXferCount(usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t * const xferCnt);`

The device mode functions return an error in the host mode and the host functions in the device mode.

### GPIO Configuration
- `usb_RequestState_t      Usb_InitDmGpio(usb_DmPin_t pinId);`
- `usb_RequestState_t      Usb_InitDpGpio(usb_DpPin_t pinId);`

---

## Usage

### Start of a device

```c
static void Device_ResetCallback ( usb_PeriphId_t usbId );
static void Device_SetupCallback ( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket );
static void Device_XferCallback  ( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult );

static const usb_Callbacks_t deviceCallbacks =
{
    .ResetCallback = Device_ResetCallback,
    .SetupCallback = Device_SetupCallback,
    .XferCallback  = Device_XferCallback,
};

usb_BusConfig_t usbConfig;

(void)Usb_Get_DefaultConfig( &usbConfig );     /* USB_PERIPH_FS, HSI48, interrupt, PA11 / PA12 */
usbConfig.Callbacks = &deviceCallbacks;

(void)Usb_Init( &usbConfig );                  /* controller powered, not visible for the host */
(void)Usb_Set_DeviceConnect( USB_PERIPH_FS );  /* host detects the device and resets the bus   */
```

The module opens the endpoint 0 after every bus reset (`Ep0MaxPacketSize`). The application initializes the
other endpoints after the host configures the device (`Usb_Init_Endpoint()`), they are closed by the next bus reset.

### Answering a request on endpoint 0

```c
static const uint8_t deviceDescriptor[ 18 ] = { /* ... */ };

static void Device_SetupCallback( usb_PeriphId_t usbId, const usb_SetupPacket_t * const setupPacket )
{
    if( ( 0x80u == setupPacket->RequestType ) && ( 0x06u == setupPacket->Request ) )   /* GET_DESCRIPTOR(device) */
    {
        usb_DataCnt_t size = sizeof( deviceDescriptor );

        if( size > setupPacket->Length )
        {
            size = setupPacket->Length;
        }

        (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 0u ), deviceDescriptor, size );
        /* the status stage (OUT, zero length) is accepted by the module */
    }
    else
    {
        (void)Usb_Set_EpStallActive( usbId, USB_EP_ADDR_IN( 0u ) );
        (void)Usb_Set_EpStallActive( usbId, USB_EP_ADDR_OUT( 0u ) );
    }
}
```

### Echo on a bulk endpoint pair

```c
static usb_Data_t echoBuffer[ 64u ];

static void Device_XferCallback( usb_PeriphId_t usbId, usb_EpAddr_t epAddr, usb_DataCnt_t xferCnt, usb_XferResult_t xferResult )
{
    if( ( USB_EP_ADDR_OUT( 1u ) == epAddr ) && ( USB_XFER_RESULT_OK == xferResult ) )
    {
        (void)Usb_Set_EpTxStart( usbId, USB_EP_ADDR_IN( 1u ), echoBuffer, xferCnt );     /* send the data back */
    }
    else if( USB_EP_ADDR_IN( 1u ) == epAddr )
    {
        (void)Usb_Set_EpRxStart( usbId, USB_EP_ADDR_OUT( 1u ), echoBuffer, sizeof( echoBuffer ) ); /* next packet */
    }
}
```

The endpoints are opened after SET_CONFIGURATION:

```c
const usb_EpConfig_t epOut = { USB_EP_ADDR_OUT( 1u ), USB_EP_TYPE_BULK, 64u, USB_EP_BUFFER_SINGLE };
const usb_EpConfig_t epIn  = { USB_EP_ADDR_IN ( 1u ), USB_EP_TYPE_BULK, 64u, USB_EP_BUFFER_SINGLE };

(void)Usb_Init_Endpoint( USB_PERIPH_FS, &epOut );
(void)Usb_Init_Endpoint( USB_PERIPH_FS, &epIn );
(void)Usb_Set_EpRxStart( USB_PERIPH_FS, USB_EP_ADDR_OUT( 1u ), echoBuffer, sizeof( echoBuffer ) );
```

### Start of a host and the first request to a device

```c
static void Host_ConnectCallback ( usb_PeriphId_t usbId );
static void Host_ChannelCallback ( usb_PeriphId_t usbId, usb_ChannelId_t chId, usb_DataCnt_t xferCnt, usb_ChannelResult_t chResult );

static const usb_Callbacks_t hostCallbacks =
{
    .ConnectCallback = Host_ConnectCallback,     /* device attached: wait 100 ms, then reset the port */
    .ChannelCallback = Host_ChannelCallback,     /* stage of a transfer finished                      */
};

usb_BusConfig_t usbConfig;

(void)Usb_Get_DefaultConfig( &usbConfig );
usbConfig.Mode      = USB_MODE_HOST;
usbConfig.Callbacks = &hostCallbacks;

(void)Usb_Init( &usbConfig );                    /* VBUS (5 V) of the port is switched on by the board */

/* ... after the debounce time: reset of the port (the timing is up to the caller) */
(void)Usb_Set_HostPortResetActive( USB_PERIPH_FS );
/* ... 50 ms */
(void)Usb_Set_HostPortResetInactive( USB_PERIPH_FS );
/* ... port state USB_HOST_PORT_ENABLED (first start of frame), wait 20 ms */

const usb_ChannelConfig_t control0 = { 0u, USB_EP_ADDR_OUT( 0u ), USB_EP_TYPE_CONTROL, 0u, USB_SPEED_FULL, 8u };
const uint8_t getDeviceDescriptor[ 8 ] = { 0x80u, 0x06u, 0x00u, 0x01u, 0x00u, 0x00u, 0x08u, 0x00u };

(void)Usb_Init_Channel( USB_PERIPH_FS, &control0 );
(void)Usb_Set_ChannelTxStart( USB_PERIPH_FS, 0u, USB_TOKEN_SETUP, getDeviceDescriptor, 8u );
/* Host_ChannelCallback: SETUP done -> Usb_Set_ChannelRxStart( ..., 0u, descriptor, 8u )
                         data done  -> Usb_Set_ChannelTxStart( ..., 0u, USB_TOKEN_OUT, NULL, 0u )  (status stage)
                         status done-> the descriptor is in the buffer */
```

### Polling instead of the interrupt

```c
usbConfig.ServiceMode = USB_SERVICE_MODE_POLL;
(void)Usb_Init( &usbConfig );

while( 1 )
{
    Usb_Task();       /* serves the controller, calls the callbacks */
    /* other work */
}
```

---

## USBX device controller driver

The interface maps to the functions of the USBX device controller (DCD) one to one:

| USBX DCD function | Usb module |
|---|---|
| `UX_DCD_CREATE_ENDPOINT` | `Usb_Init_Endpoint()` |
| `UX_DCD_DESTROY_ENDPOINT` | `Usb_Deinit_Endpoint()` |
| `UX_DCD_RESET_ENDPOINT` | `Usb_Set_EpStallInactive()` |
| `UX_DCD_STALL_ENDPOINT` | `Usb_Set_EpStallActive()` |
| `UX_DCD_ENDPOINT_STATUS` | `Usb_Get_EpState()` |
| `UX_DCD_TRANSFER_REQUEST` | `Usb_Set_EpTxStart()` / `Usb_Set_EpRxStart()` |
| `UX_DCD_TRANSFER_ABORT` | `Usb_Set_EpXferStop()` |
| `UX_DCD_SET_DEVICE_ADDRESS` | `Usb_Set_DeviceAddress()` |
| `UX_DCD_GET_FRAME_NUMBER` | `Usb_Get_FrameNumber()` |
| `UX_DCD_CHANGE_STATE` | `Usb_Set_DeviceConnect()` / `Usb_Set_DeviceDisconnect()` |
| `UX_DCD_TASKS_RUN` (standalone USBX) | `Usb_Task()` |

---

## Error handling

- Every function returns `USB_REQUEST_ERROR` for an invalid parameter (NULL pointer, enumeration out of range,
  packet size not supported), an uninitialized controller or a hardware state that does not allow the request,
  and does not modify any register in these cases.
- A register write which is not confirmed by the read-back within the timeout returns `USB_REQUEST_ERROR`.
- Run-time problems of the bus are reported by `ErrorCallback`: `USB_ERROR_BUS` (no answer, CRC, bit stuffing),
  `USB_ERROR_PMA_OVERRUN` (packet memory not served in time), `USB_ERROR_SETUP_LENGTH`.
- A transfer interrupted by a SETUP packet, a bus reset or a stop is reported with `USB_XFER_RESULT_ABORTED`;
  after `Usb_Set_DeviceDisconnect()` / `Usb_Deinit()` running transfers are cancelled silently.

---

## Testing

- **Unit tests** (`Tests/UnitTests`): all module sources are compiled unchanged with the real LL driver, the
  `USB_DRD_FS` registers and the packet memory are emulated by RegMem (plain RAM), RCC, NVIC and GPIO are mocked
  by CMock. The tests emulate the behaviour of the controller (flags of the interrupt status register, toggle
  fields of the endpoint registers, received and sent packets) and call the interrupt handler registered in
  the NVIC. 104 tests on the `USB_DRD_FS` devices (65 device mode, 39 host mode), 4 tests on the devices without it
  (`Test_UsbUnsupported.c`).
- **Integration tests** (`Tests/IntegrationTests`, hardware): power-up and release of the controller, pull-up,
  endpoint status toggling on the real controller, packet memory, enumeration of a test device by a PC host
  (vendor specific device 1209:0001, ignored when no host resets the bus within 3 seconds). Connect the user USB
  connector (PA11 / PA12) of the board to a PC for the enumeration test.
- **Integration tests of the host mode** (`ItTest_UsbHost.c`, hardware): power-up and release of the host mode (no device
  needed) and the whole chain on the real controller without the USB stack - attach, debounce, port reset, first start of frame,
  GET_DESCRIPTOR(device) on the default address, SET_ADDRESS and GET_DESCRIPTOR(device) on the new address. A full speed device
  with 5 V on VBUS is needed (ignored without it), see [VBUS and the board](#vbus-and-the-board).

```text
cmake --preset STM32H503xB_UnitTest -B B/STM32H503xB_UT
cmake --build B/STM32H503xB_UT --target UT_Usb
ctest --test-dir B/STM32H503xB_UT -R Usb          # one process per test
```

---

## Errata and known limitations

| Item | Handling |
|---|---|
| ES0561 2.15.1 (STM32H503): the buffer descriptor table update completes after the CTR interrupt, so a quick read of a received packet can be incomplete | `Usb_Pma_Wait_Settle()` waits at least 800 ns (a fixed count of register reads) before a received packet is read - applied to all `USB_DRD_FS` devices (bug AB#1030) |
| Isochronous endpoints, double buffering (device and host) | rejected, story AB#997 |
| Host: suspend / resume of the port, remote wakeup | not implemented |
| Host: low speed devices | supported directly and behind a full speed hub (low speed flag of the channel); the wait before the packet memory is read is 8 times longer (ES0561: 6.4 us) |
| LPM, battery charging detection | not implemented, story AB#1000 |
| OTG controllers (H5E4 / E5 / F4 / F5) | functions return error, story AB#999 |
| Concurrency | the module does not disable interrupts - use the API from the callbacks or while the endpoint is idle |

---

## Troubleshooting

| Symptom | Check |
|---|---|
| `Usb_Init()` fails | The kernel clock must be 48 MHz: HSI48 is 48 MHz after reset; for a PLL clock configure the PLL output in the RCC module first. A failed init releases everything it took. |
| The host does not see the device | `Usb_Set_DeviceConnect()` was called, VBUS is present on the connector, PA11 / PA12 are configured, the USB-C connector of the board is wired as a device (NUCLEO boards: solder bridges, see the board manual). |
| Enumeration starts but stops after the first descriptor | The SETUP callback must start a data stage (`Usb_Set_EpTxStart`) or stall the endpoint; the data stage must not be longer than `wLength`. |
| Short IN transfer never ends on the host | A transfer which is a multiple of the packet size needs a zero length packet (`Usb_Set_EpTxStart( ..., NULL, 0u )`). |
| Endpoint cannot be opened | Packet memory is full (2 KB), packet size is not supported, or the endpoint is already open. |
| Host: no `ConnectCallback` | VBUS is not supplied to the device (NUCLEO-H503RB: `USB_FS_PWR_EN` PC10, LD6 shows VBUS, solder bridges of CN3 for the host), DP / DM pins not configured, the module is in the device mode. |
| Host: the device does not answer after the reset | The recovery time after the reset release (at least 10 ms) and the first start of frame (port state `ENABLED`) must pass; the first request goes to the default address 0 with the packet size 8. |
| Host: `Usb_Init_Channel()` fails | The port is disconnected, the channel is busy, the packet size is not valid for the type, no packet memory. |

---

## CMake Integration

1. Link `Usb_Lib` to your library / application: `target_link_libraries( MyApp PRIVATE Usb_Lib )`.
2. Include `Usb_Port.h` in your sources.
3. The module needs the `Rcc_Lib`, `Gpio_Lib` and `Nvic_Lib` modules and the Ral (`Stm32_usb.h`), they are linked
   by the module itself.
4. The module is added to the project by the BSP handler of the EmBi platform (`Bsp/Mcal/CMakeLists.txt`).

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
