#pragma once

/**
 * @file usb_hid_setup.h
 * @brief Composite USB device for the CLN17 v3: CDC console + vendor HID control pipe.
 *
 * The stock STM32duino USB stack enumerates as a single class - either CDC or
 * the mouse/keyboard HID pair, never both, because `usbd_ep_conf.h` defines one
 * `ep_def[]` per class and the two definitions collide. Its HID is also IN-only,
 * so a host cannot send anything to it. Neither shape is usable as a control
 * link. This file adds a THIRD shape without touching the framework package: a
 * composite device that keeps the CDC console exactly as it is and bolts a
 * vendor-defined HID interface next to it.
 *
 * ## Why HID for control at all
 *
 * The CDC console is a byte stream, and a byte stream is the wrong shape for a
 * control link:
 *
 *   - It has no message boundary, so every frame needs sync bytes and a
 *     checksum (`tp_serial.h`), and a resynchronization hunt after any glitch.
 *   - The host has no idea how much room the device has. `tools/tpcp_host.py`
 *     has to hand the port four frames at a time with a 2 ms sleep between
 *     batches: send faster and the board's CDC receive buffer silently
 *     overruns, send slower and the trajectory queue starves mid-move.
 *   - Latency is whatever the host's serial driver decides to buffer.
 *
 * A HID interrupt endpoint fixes all three. Reports are fixed-size messages
 * with hardware framing - a report arrives whole or not at all, so there is no
 * checksum and no resync. The host polls the IN endpoint once per USB frame
 * (1 ms) and the device NAKs when it has nothing, which is flow control for
 * free. And HID binds to a class driver on every desktop OS, so there is no
 * driver to install, no COM port to guess, and no baud rate to be wrong about.
 *
 * ## Device layout
 *
 *     Interface 0  CDC communication  EP 0x83 IN  (notification, 8 B)
 *     Interface 1  CDC data           EP 0x82 IN / 0x01 OUT (bulk, 64 B)
 *     Interface 2  Vendor HID         EP 0x84 IN / 0x04 OUT (interrupt, 64 B, 1 ms)
 *
 * The two functions are separated by an Interface Association Descriptor, so
 * the device descriptor advertises itself as Miscellaneous / Common Class
 * (0xEF / 0x02 / 0x01) rather than CDC. Both bytes are patched into the
 * framework's device descriptor at run time - it is a mutable global, so no
 * framework file has to change.
 *
 * ## The class driver is a shim, not a rewrite
 *
 * `USBD_CDC` is exported by the framework, so every CDC callback here simply
 * forwards to it and only the endpoint-4 traffic and the configuration
 * descriptor are ours. That keeps ST's CDC state machine as the one
 * implementation of CDC in the build: `SerialUSB`, the log macros and the
 * framed serial transport all keep working, unmodified, alongside HID.
 *
 * ## Packet memory
 *
 * The USB peripheral's buffer descriptor table lives at the bottom of the
 * 1024-byte packet memory and takes 8 bytes per endpoint INDEX in use. The
 * framework sizes it for four endpoints (0..3) and puts the EP0 OUT buffer
 * immediately after it at offset 32. Endpoint 4 needs descriptor bytes 32..39,
 * which is exactly where that buffer sits, so EP0 OUT is relocated into free
 * memory high up before the link starts. See tp_usb_begin().
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include <Arduino.h>

// The USB stack is C. usbd_cdc.h and the middleware core headers carry their
// own linkage guards; usbd_ep_conf.h and usbd_desc.h do not, so they are
// wrapped here - without this `ep_def` and `USBD_Desc` would be looked up with
// C++ linkage and fail to link.
extern "C" {
#include "usbd_cdc.h"      // USBD_CDC class object, CDC handle type, packet helpers
#include "usbd_cdc_if.h"   // USBD_CDC_fops, the console's transmit / receive queues
#include "usbd_core.h"     // USBD_Init / USBD_RegisterClass / USBD_Start
#include "usbd_ctlreq.h"   // Control-endpoint helpers used by the HID setup handler
#include "usbd_ep_conf.h"  // CDC endpoint addresses and packet sizes
#include "usbd_desc.h"     // USBD_Desc: the framework's descriptor callback table
}

// ============================================================================
// HID interface parameters
// ============================================================================

/** Interface number of the vendor HID function (CDC occupies 0 and 1). */
#define TP_HID_ITF_NBR 0x02u

/** Device-to-host interrupt endpoint carrying reports up. */
#define TP_HID_EP_IN 0x84u

/** Host-to-device interrupt endpoint carrying reports down. */
#define TP_HID_EP_OUT 0x04u

/** Report size. 64 bytes is the largest a full-speed interrupt endpoint takes. */
#define TP_HID_EP_SIZE 64u

/** Endpoint polling interval [ms]. 1 = every USB frame, the fastest allowed. */
#define TP_HID_EP_INTERVAL 0x01u

/** Total length of the composite configuration descriptor [bytes]. */
#define TP_USB_CONFIG_DESC_SIZE 107u

/**
 * @brief Packet-memory address of the HID OUT buffer.
 *
 * Chosen above everything the framework allocates (its highest CDC buffer ends
 * at 352) with a wide margin, so the two layouts cannot collide even if the
 * framework's CDC packet sizes change.
 */
#define TP_HID_PMA_OUT 640u

/** Packet-memory address of the HID IN buffer. */
#define TP_HID_PMA_IN 704u

/**
 * @brief Relocated packet-memory address of the EP0 OUT buffer.
 *
 * The framework puts it at 32, immediately above a four-endpoint buffer
 * descriptor table. Endpoint 4 extends that table to 40 bytes and would
 * overwrite the first eight bytes of the buffer, so it moves out of the way.
 */
#define TP_EP0_PMA_OUT 512u

// ============================================================================
// HID descriptors
// ============================================================================

/**
 * @brief HID report descriptor: one 64-byte input report, one 64-byte output report.
 *
 * The usage page is vendor-defined (0xFF00), which is what keeps this device
 * out of the operating system's input handling: a vendor page is not a mouse,
 * a keyboard or a gamepad, so no OS layer claims it exclusively, injects its
 * data into the desktop, or requires elevated rights to open it. The reports
 * are opaque byte arrays - their meaning is defined entirely by tp_hid.h.
 *
 * No report ID is declared, so reports are exactly 64 bytes on the wire.
 */
static const uint8_t TP_HID_ReportDesc[] = {
    0x06, 0x00, 0xFF,  // USAGE_PAGE (Vendor Defined 0xFF00)
    0x09, 0x01,        // USAGE (Vendor Usage 1)
    0xA1, 0x01,        // COLLECTION (Application)
    0x09, 0x02,        //   USAGE (Vendor Usage 2) - the device-to-host report
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8 bits per field)
    0x95, 0x40,        //   REPORT_COUNT (64 fields = 64 bytes)
    0x81, 0x02,        //   INPUT (Data, Variable, Absolute)
    0x09, 0x03,        //   USAGE (Vendor Usage 3) - the host-to-device report
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8 bits per field)
    0x95, 0x40,        //   REPORT_COUNT (64 fields = 64 bytes)
    0x91, 0x02,        //   OUTPUT (Data, Variable, Absolute)
    0xC0               // END_COLLECTION
};

/**
 * @brief Configuration descriptor for the whole composite device.
 *
 * Interfaces 0 and 1 reproduce the framework's CDC descriptor byte for byte -
 * same endpoints, same functional descriptors - so the console enumerates
 * exactly as it did before. The IAD in front of them is what tells the host
 * that those two interfaces are one function, which is required as soon as a
 * second, unrelated function shares the configuration.
 */
static uint8_t TP_USB_CfgDesc[TP_USB_CONFIG_DESC_SIZE] = {
    // ---- Configuration -----------------------------------------------------
    0x09,                                 // bLength
    USB_DESC_TYPE_CONFIGURATION,          // bDescriptorType
    LOBYTE(TP_USB_CONFIG_DESC_SIZE),      // wTotalLength
    HIBYTE(TP_USB_CONFIG_DESC_SIZE),      //
    0x03,                                 // bNumInterfaces: 2 CDC + 1 HID
    0x01,                                 // bConfigurationValue
    0x00,                                 // iConfiguration: no string
    0xC0,                                 // bmAttributes: bus powered, self powered
    0x32,                                 // bMaxPower: 100 mA

    // ---- Interface association: the two CDC interfaces are one function ----
    0x08,                                 // bLength
    USB_DESC_TYPE_IAD,                    // bDescriptorType
    0x00,                                 // bFirstInterface
    0x02,                                 // bInterfaceCount
    0x02,                                 // bFunctionClass: Communications
    0x02,                                 // bFunctionSubClass: Abstract Control Model
    0x01,                                 // bFunctionProtocol: AT commands
    0x00,                                 // iFunction: no string

    // ---- Interface 0: CDC communication ------------------------------------
    0x09,                                 // bLength
    USB_DESC_TYPE_INTERFACE,              // bDescriptorType
    0x00,                                 // bInterfaceNumber
    0x00,                                 // bAlternateSetting
    0x01,                                 // bNumEndpoints: the notification endpoint
    0x02,                                 // bInterfaceClass: Communications
    0x02,                                 // bInterfaceSubClass: Abstract Control Model
    0x01,                                 // bInterfaceProtocol: AT commands
    0x00,                                 // iInterface: no string

    0x05, 0x24, 0x00, 0x10, 0x01,         // CDC header functional, spec 1.10
    0x05, 0x24, 0x01, 0x00, 0x01,         // Call management: data interface 1
    0x04, 0x24, 0x02, 0x02,               // ACM functional: supports line coding
    0x05, 0x24, 0x06, 0x00, 0x01,         // Union: control 0, subordinate 1

    0x07,                                 // bLength
    USB_DESC_TYPE_ENDPOINT,               // bDescriptorType
    CDC_CMD_EP,                           // bEndpointAddress: 0x83 IN
    0x03,                                 // bmAttributes: interrupt
    LOBYTE(CDC_CMD_PACKET_SIZE),          // wMaxPacketSize
    HIBYTE(CDC_CMD_PACKET_SIZE),          //
    CDC_FS_BINTERVAL,                     // bInterval

    // ---- Interface 1: CDC data ---------------------------------------------
    0x09,                                 // bLength
    USB_DESC_TYPE_INTERFACE,              // bDescriptorType
    0x01,                                 // bInterfaceNumber
    0x00,                                 // bAlternateSetting
    0x02,                                 // bNumEndpoints: bulk in and out
    0x0A,                                 // bInterfaceClass: CDC data
    0x00,                                 // bInterfaceSubClass
    0x00,                                 // bInterfaceProtocol
    0x00,                                 // iInterface: no string

    0x07,                                 // bLength
    USB_DESC_TYPE_ENDPOINT,               // bDescriptorType
    CDC_OUT_EP,                           // bEndpointAddress: 0x01 OUT
    0x02,                                 // bmAttributes: bulk
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  // wMaxPacketSize
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  //
    0x00,                                 // bInterval: ignored for bulk

    0x07,                                 // bLength
    USB_DESC_TYPE_ENDPOINT,               // bDescriptorType
    CDC_IN_EP,                            // bEndpointAddress: 0x82 IN
    0x02,                                 // bmAttributes: bulk
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  // wMaxPacketSize
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  //
    0x00,                                 // bInterval: ignored for bulk

    // ---- Interface 2: vendor HID -------------------------------------------
    0x09,                                 // bLength
    USB_DESC_TYPE_INTERFACE,              // bDescriptorType
    TP_HID_ITF_NBR,                       // bInterfaceNumber
    0x00,                                 // bAlternateSetting
    0x02,                                 // bNumEndpoints: interrupt in and out
    0x03,                                 // bInterfaceClass: HID
    0x00,                                 // bInterfaceSubClass: not boot capable
    0x00,                                 // bInterfaceProtocol: none
    0x00,                                 // iInterface: no string

    0x09,                                 // bLength
    0x21,                                 // bDescriptorType: HID
    0x11, 0x01,                           // bcdHID: 1.11
    0x00,                                 // bCountryCode: not localized
    0x01,                                 // bNumDescriptors
    0x22,                                 // bDescriptorType: report
    (uint8_t)(sizeof(TP_HID_ReportDesc)), // wDescriptorLength
    0x00,                                 //

    0x07,                                 // bLength
    USB_DESC_TYPE_ENDPOINT,               // bDescriptorType
    TP_HID_EP_IN,                         // bEndpointAddress: 0x84 IN
    0x03,                                 // bmAttributes: interrupt
    LOBYTE(TP_HID_EP_SIZE),               // wMaxPacketSize
    HIBYTE(TP_HID_EP_SIZE),               //
    TP_HID_EP_INTERVAL,                   // bInterval: 1 ms

    0x07,                                 // bLength
    USB_DESC_TYPE_ENDPOINT,               // bDescriptorType
    TP_HID_EP_OUT,                        // bEndpointAddress: 0x04 OUT
    0x03,                                 // bmAttributes: interrupt
    LOBYTE(TP_HID_EP_SIZE),               // wMaxPacketSize
    HIBYTE(TP_HID_EP_SIZE),               //
    TP_HID_EP_INTERVAL                    // bInterval: 1 ms
};

/**
 * @brief Device qualifier: returned for a full-speed-only device to say so.
 */
static uint8_t TP_USB_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] = {
    USB_LEN_DEV_QUALIFIER_DESC,        // bLength
    USB_DESC_TYPE_DEVICE_QUALIFIER,    // bDescriptorType
    0x00, 0x02,                        // bcdUSB: 2.00
    0x00,                              // bDeviceClass
    0x00,                              // bDeviceSubClass
    0x00,                              // bDeviceProtocol
    0x40,                              // bMaxPacketSize0
    0x01,                              // bNumConfigurations
    0x00                               // bReserved
};

// ============================================================================
// HID endpoint state
// ============================================================================

/** Buffer the USB peripheral writes each received report into. */
static uint8_t tp_hid_rx_report[TP_HID_EP_SIZE];

/** Buffer holding the report currently in flight towards the host. */
static uint8_t tp_hid_tx_report[TP_HID_EP_SIZE];

/** Scratch for a report delivered over the control endpoint by SET_REPORT. */
static uint8_t tp_hid_ep0_report[TP_HID_EP_SIZE];

/**
 * @brief Set while a report is in flight on the IN endpoint.
 *
 * Written by the USB interrupt when the transfer completes and read from
 * thread mode before queueing the next report, hence volatile.
 */
static volatile uint8_t tp_hid_tx_busy = 0;

/** Non-zero while the control endpoint is collecting a SET_REPORT payload. */
static uint8_t tp_hid_ep0_pending = 0;

/** HID idle rate, stored only so GET_IDLE can hand it back. */
static uint8_t tp_hid_idle_state = 0;

/** HID protocol, stored only so GET_PROTOCOL can hand it back. */
static uint8_t tp_hid_protocol = 0;

/**
 * @brief Called from the USB interrupt for every report the host sends.
 *
 * Installed by the HID transport in tp_hid.h. Left null until then, so reports
 * that arrive before the transport exists are discarded rather than dispatched
 * into uninitialized state.
 */
static void (*tp_hid_rx_hook)(const uint8_t* report) = nullptr;

/** Reports accepted from the host. */
static uint32_t tp_hid_rx_count = 0;

/** Reports handed to the IN endpoint. */
static uint32_t tp_hid_tx_count = 0;

/**
 * @brief The USB device handle owned by the framework's usbd_cdc_if.c.
 *
 * Defined there but never declared in a header, so the declaration lives here.
 * Sharing the handle rather than opening a second one is what keeps `SerialUSB`
 * working: every CDC helper in the framework transmits through this exact
 * object, so the composite class must be registered against it.
 */
extern "C" USBD_HandleTypeDef hUSBD_Device_CDC;

/** The device handle. The composite class shares the framework's CDC handle. */
#define TP_USB_DEV (&hUSBD_Device_CDC)

// ============================================================================
// Composite class driver
// ============================================================================

/**
 * @brief Bring up both functions on SET_CONFIGURATION.
 *
 * The CDC half is opened by ST's own class driver, so its endpoints, its
 * handle and its interface callbacks come up exactly as they do in a
 * CDC-only build. Only the two HID endpoints are ours.
 *
 * @param pdev   Device handle.
 * @param cfgidx Configuration index selected by the host.
 * @return USBD_OK when both functions came up.
 */
static uint8_t TP_USB_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx) {
    const uint8_t status = USBD_CDC.Init(pdev, cfgidx);       // Console half, unmodified

    // Interrupt IN: reports towards the host. Marked used so the core's
    // endpoint bookkeeping (stall, clear-feature) covers it.
    (void)USBD_LL_OpenEP(pdev, TP_HID_EP_IN, USBD_EP_TYPE_INTR, TP_HID_EP_SIZE);
    pdev->ep_in[TP_HID_EP_IN & 0x0Fu].is_used = 1u;

    // Interrupt OUT: reports from the host.
    (void)USBD_LL_OpenEP(pdev, TP_HID_EP_OUT, USBD_EP_TYPE_INTR, TP_HID_EP_SIZE);
    pdev->ep_out[TP_HID_EP_OUT & 0x0Fu].is_used = 1u;

    tp_hid_tx_busy = 0u;                                      // Nothing in flight yet
    tp_hid_ep0_pending = 0u;                                  // No control transfer in progress

    // Arm the OUT endpoint. Without this the host's first report is NAKed
    // forever and the control link never starts.
    (void)USBD_LL_PrepareReceive(pdev, TP_HID_EP_OUT, tp_hid_rx_report, TP_HID_EP_SIZE);
    return status;
}

/**
 * @brief Tear both functions down on bus reset or de-configuration.
 * @param pdev   Device handle.
 * @param cfgidx Configuration index being left.
 * @return USBD_OK.
 */
static uint8_t TP_USB_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx) {
    (void)USBD_LL_CloseEP(pdev, TP_HID_EP_IN);                // Stop reports going up
    pdev->ep_in[TP_HID_EP_IN & 0x0Fu].is_used = 0u;
    (void)USBD_LL_CloseEP(pdev, TP_HID_EP_OUT);               // Stop reports coming down
    pdev->ep_out[TP_HID_EP_OUT & 0x0Fu].is_used = 0u;
    tp_hid_tx_busy = 0u;                                      // Any in-flight report is gone
    return USBD_CDC.DeInit(pdev, cfgidx);                     // Console half, unmodified
}

/**
 * @brief Answer a control request, routing it to the function it addresses.
 *
 * Everything not aimed at the HID interface is handed to ST's CDC driver
 * untouched, which is what keeps line coding, DTR and the rest of the console's
 * behaviour identical to a CDC-only build.
 *
 * @param pdev Device handle.
 * @param req  Setup packet.
 * @return USBD_OK when the request was answered.
 */
static uint8_t TP_USB_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req) {
    // Requests are routed by the interface they name. Anything addressed to a
    // different recipient - the device, an endpoint - or to interface 0 or 1
    // belongs to CDC.
    const bool to_interface = ((req->bmRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE);
    if (!to_interface || (LOBYTE(req->wIndex) != TP_HID_ITF_NBR)) {
        return USBD_CDC.Setup(pdev, req);                     // Not ours
    }

    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest) {
        case 0x0Au:                                           // SET_IDLE
            tp_hid_idle_state = (uint8_t)(req->wValue >> 8);
            (void)USBD_CtlSendStatus(pdev);
            break;

        case 0x02u:                                           // GET_IDLE
            (void)USBD_CtlSendData(pdev, &tp_hid_idle_state, 1u);
            break;

        case 0x0Bu:                                           // SET_PROTOCOL
            tp_hid_protocol = (uint8_t)(req->wValue);
            (void)USBD_CtlSendStatus(pdev);
            break;

        case 0x03u:                                           // GET_PROTOCOL
            (void)USBD_CtlSendData(pdev, &tp_hid_protocol, 1u);
            break;

        case 0x09u: {
            // SET_REPORT. macOS routes host-to-device reports through the
            // control endpoint rather than the interrupt OUT endpoint on some
            // versions, so this path has to work as well or the link is
            // one-directional on those machines.
            uint16_t len = req->wLength;                      // Payload the host announced
            if (len > TP_HID_EP_SIZE) {
                len = TP_HID_EP_SIZE;                         // Clamp: never overrun the buffer
            }
            tp_hid_ep0_pending = 1u;                          // EP0_RxReady belongs to us now
            (void)USBD_CtlPrepareRx(pdev, tp_hid_ep0_report, len);
            break;
        }

        case 0x01u:                                           // GET_REPORT
            // Answering with the last report prepared for the IN endpoint keeps
            // a host that polls by control transfer in step with one that reads
            // the interrupt endpoint.
            (void)USBD_CtlSendData(pdev, tp_hid_tx_report,
                                   (req->wLength < TP_HID_EP_SIZE) ? req->wLength : TP_HID_EP_SIZE);
            break;

        default:
            USBD_CtlError(pdev, req);                         // Unknown class request
            return USBD_FAIL;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_GET_DESCRIPTOR: {
            // The host fetches the report descriptor - and on some stacks the
            // HID descriptor itself - from the interface, not the device.
            const uint8_t type = (uint8_t)(req->wValue >> 8);  // Descriptor being asked for
            const uint8_t* src = nullptr;                      // Where it lives
            uint16_t len = 0;                                  // How long it is
            if (type == 0x22u) {
                src = TP_HID_ReportDesc;                       // Report descriptor
                len = (uint16_t)sizeof(TP_HID_ReportDesc);
            } else if (type == 0x21u) {
                // The HID descriptor sits inside the configuration descriptor,
                // nine bytes after the HID interface descriptor.
                src = &TP_USB_CfgDesc[TP_USB_CONFIG_DESC_SIZE - 23u];
                len = 9u;
            } else {
                USBD_CtlError(pdev, req);                      // Not a HID descriptor
                return USBD_FAIL;
            }
            if (len > req->wLength) {
                len = req->wLength;                            // Never send more than asked
            }
            (void)USBD_CtlSendData(pdev, (uint8_t*)src, len);
            break;
        }

        case USB_REQ_GET_INTERFACE: {
            // The HID function has one setting and no alternates.
            static uint8_t alt = 0u;                           // Always zero
            (void)USBD_CtlSendData(pdev, &alt, 1u);
            break;
        }

        case USB_REQ_SET_INTERFACE:
            (void)USBD_CtlSendStatus(pdev);                    // Accept, nothing to switch
            break;

        case USB_REQ_CLEAR_FEATURE:
            (void)USBD_CtlSendStatus(pdev);                    // No interface features
            break;

        default:
            USBD_CtlError(pdev, req);                          // Unknown standard request
            return USBD_FAIL;
        }
        break;

    default:
        USBD_CtlError(pdev, req);                              // Unknown request type
        return USBD_FAIL;
    }
    return USBD_OK;
}

/**
 * @brief An IN transfer finished: release the endpoint or hand it to CDC.
 * @param pdev  Device handle.
 * @param epnum Endpoint index that completed.
 * @return USBD_OK.
 */
static uint8_t TP_USB_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum) {
    if ((epnum & 0x0Fu) == (TP_HID_EP_IN & 0x0Fu)) {
        tp_hid_tx_busy = 0u;                                   // Endpoint free for the next report
        return USBD_OK;
    }
    return USBD_CDC.DataIn(pdev, epnum);                        // Console half
}

/**
 * @brief An OUT transfer finished: dispatch the report or hand it to CDC.
 *
 * Runs in USB interrupt context. The hook does no work beyond copying the
 * report into a queue, and the endpoint is re-armed immediately afterwards so
 * the host is never left waiting on a NAK for longer than one frame.
 *
 * @param pdev  Device handle.
 * @param epnum Endpoint index that completed.
 * @return USBD_OK.
 */
static uint8_t TP_USB_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum) {
    if ((epnum & 0x0Fu) == (TP_HID_EP_OUT & 0x0Fu)) {
        tp_hid_rx_count++;                                     // One more report accepted
        if (tp_hid_rx_hook != nullptr) {
            tp_hid_rx_hook(tp_hid_rx_report);                  // Queue it for thread mode
        }
        // Re-arm before returning: the buffer has already been consumed.
        (void)USBD_LL_PrepareReceive(pdev, TP_HID_EP_OUT, tp_hid_rx_report, TP_HID_EP_SIZE);
        return USBD_OK;
    }
    return USBD_CDC.DataOut(pdev, epnum);                       // Console half
}

/**
 * @brief The control endpoint finished receiving data.
 *
 * Both functions can have a payload in flight here - CDC line coding and HID
 * SET_REPORT - so ownership is tracked explicitly rather than guessed.
 *
 * @param pdev Device handle.
 * @return USBD_OK.
 */
static uint8_t TP_USB_EP0_RxReady(USBD_HandleTypeDef* pdev) {
    if (tp_hid_ep0_pending != 0u) {
        tp_hid_ep0_pending = 0u;                               // Transfer consumed
        tp_hid_rx_count++;                                     // Counts the same as an endpoint report
        if (tp_hid_rx_hook != nullptr) {
            tp_hid_rx_hook(tp_hid_ep0_report);
        }
        return USBD_OK;
    }
    return USBD_CDC.EP0_RxReady(pdev);                          // CDC command payload
}

/**
 * @brief Hand back the composite configuration descriptor.
 * @param[out] length Descriptor length [bytes].
 * @return Pointer to the descriptor.
 */
static uint8_t* TP_USB_GetCfgDesc(uint16_t* length) {
    *length = (uint16_t)sizeof(TP_USB_CfgDesc);
    return TP_USB_CfgDesc;
}

/**
 * @brief Hand back the device qualifier descriptor.
 * @param[out] length Descriptor length [bytes].
 * @return Pointer to the descriptor.
 */
static uint8_t* TP_USB_GetDeviceQualifierDesc(uint16_t* length) {
    *length = (uint16_t)sizeof(TP_USB_DeviceQualifierDesc);
    return TP_USB_DeviceQualifierDesc;
}

/**
 * @brief The composite class driver handed to the USB device core.
 *
 * Only the callbacks that have to know about two functions are ours; the CDC
 * ones forward, so ST's driver stays the single implementation of CDC.
 */
static USBD_ClassTypeDef TP_USB_CompositeClass = {
    TP_USB_Init,                    // Init
    TP_USB_DeInit,                  // DeInit
    TP_USB_Setup,                   // Setup
    nullptr,                        // EP0_TxSent: unused by both functions
    TP_USB_EP0_RxReady,             // EP0_RxReady
    TP_USB_DataIn,                  // DataIn
    TP_USB_DataOut,                 // DataOut
    nullptr,                        // SOF: neither function is isochronous
    nullptr,                        // IsoINIncomplete
    nullptr,                        // IsoOUTIncomplete
    TP_USB_GetCfgDesc,              // GetHSConfigDescriptor: full speed only device
    TP_USB_GetCfgDesc,              // GetFSConfigDescriptor
    TP_USB_GetCfgDesc,              // GetOtherSpeedConfigDescriptor
    TP_USB_GetDeviceQualifierDesc   // GetDeviceQualifierDescriptor
};

// ============================================================================
// Bring-up
// ============================================================================

/**
 * @brief Product ID reported by the composite device.
 *
 * Deliberately NOT the framework's CDC-only value. Desktop operating systems
 * cache a device's descriptors against its vendor and product ID, so a board
 * that was once enumerated as a plain serial port and later grows a second
 * interface under the same identity is met with the cached single-interface
 * layout - on Windows the HID function then never appears, and the failure
 * looks exactly like broken firmware. A distinct product ID sidesteps the
 * cache entirely. Override it if a deployment needs the old identity back.
 */
#ifndef TP_USB_PID
#define TP_USB_PID 0x5741u
#endif

/** The framework's device descriptor. Mutable, which is what lets it be patched. */
extern "C" uint8_t USBD_Class_DeviceDesc[];

/** The USB peripheral handle owned by the framework's usbd_conf.c. */
extern "C" PCD_HandleTypeDef g_hpcd;

/**
 * @brief Bring up the composite USB device: CDC console plus HID control pipe.
 *
 * Replaces `SerialUSB.begin()`. The console still works exactly as before -
 * this registers ST's CDC interface callbacks against the same device handle
 * `USBSerial` transmits through - it just enumerates with a HID function
 * beside it.
 *
 * @return true when the device started.
 */
inline bool tp_usb_begin() {
    // ---- Device descriptor ------------------------------------------------
    // A device whose interfaces belong to more than one function must not
    // claim a single class at device level; it declares itself Miscellaneous /
    // Common Class / Interface Association so the host reads the IADs in the
    // configuration descriptor and groups the interfaces from those instead.
    USBD_Class_DeviceDesc[4] = 0xEFu;                  // bDeviceClass: Miscellaneous
    USBD_Class_DeviceDesc[5] = 0x02u;                  // bDeviceSubClass: Common Class
    USBD_Class_DeviceDesc[6] = 0x01u;                  // bDeviceProtocol: Interface Association
    USBD_Class_DeviceDesc[10] = (uint8_t)(TP_USB_PID & 0xFFu);         // idProduct low
    USBD_Class_DeviceDesc[11] = (uint8_t)((TP_USB_PID >> 8) & 0xFFu);  // idProduct high

    // ---- Core ------------------------------------------------------------
    // USBD_Init brings the peripheral up and lays out packet memory from the
    // framework's four-endpoint table. It does not start the link, which is
    // what leaves room to correct that layout below.
    if (USBD_Init(TP_USB_DEV, &USBD_Desc, 0) != USBD_OK) {
        return false;
    }

    // ---- Packet memory ----------------------------------------------------
    // The buffer descriptor table takes eight bytes per endpoint index in use.
    // Adding endpoint 4 grows it from 32 to 40 bytes, over the top of the EP0
    // OUT buffer the framework placed at 32, so that buffer is moved first and
    // the two HID buffers are given addresses above everything else.
    (void)HAL_PCDEx_PMAConfig(&g_hpcd, 0x00u, PCD_SNG_BUF, TP_EP0_PMA_OUT);
    (void)HAL_PCDEx_PMAConfig(&g_hpcd, TP_HID_EP_OUT, PCD_SNG_BUF, TP_HID_PMA_OUT);
    (void)HAL_PCDEx_PMAConfig(&g_hpcd, TP_HID_EP_IN, PCD_SNG_BUF, TP_HID_PMA_IN);

    // ---- Classes ----------------------------------------------------------
    if (USBD_RegisterClass(TP_USB_DEV, &TP_USB_CompositeClass) != USBD_OK) {
        return false;
    }
    // ST's CDC driver calls these to fill and drain the console queues. They
    // are the same callbacks a CDC-only build registers.
    if (USBD_CDC_RegisterInterface(TP_USB_DEV, &USBD_CDC_fops) != USBD_OK) {
        return false;
    }

    return USBD_Start(TP_USB_DEV) == USBD_OK;
}

// ============================================================================
// HID report interface
// ============================================================================

/**
 * @brief Install the callback that consumes reports arriving from the host.
 * @param hook Called from USB interrupt context with a 64-byte report.
 */
inline void tp_usb_hid_set_rx_hook(void (*hook)(const uint8_t* report)) {
    tp_hid_rx_hook = hook;
}

/** @brief true when the host has configured the device and a report can be queued. */
inline bool tp_usb_hid_ready() {
    return (TP_USB_DEV->dev_state == USBD_STATE_CONFIGURED) && (tp_hid_tx_busy == 0u);
}

/**
 * @brief Queue one 64-byte report on the interrupt IN endpoint.
 *
 * Never blocks. When the previous report has not been collected yet the call
 * fails and the caller keeps its data - the host is one USB frame behind at
 * worst, and dropping here would silently lose protocol frames.
 *
 * @param report Exactly TP_HID_EP_SIZE bytes.
 * @return true when the report was handed to the endpoint.
 */
inline bool tp_usb_hid_send(const uint8_t* report) {
    if (!tp_usb_hid_ready()) {
        return false;                                  // Not configured, or one still in flight
    }
    memcpy(tp_hid_tx_report, report, TP_HID_EP_SIZE);  // Endpoint reads from this buffer
    tp_hid_tx_busy = 1u;                               // Claim the endpoint before arming it
    if (USBD_LL_Transmit(TP_USB_DEV, TP_HID_EP_IN, tp_hid_tx_report, TP_HID_EP_SIZE) != USBD_OK) {
        tp_hid_tx_busy = 0u;                           // Never armed: release it again
        return false;
    }
    tp_hid_tx_count++;
    return true;
}

/** @brief Reports accepted from the host. */
inline uint32_t tp_usb_hid_rx_reports() { return tp_hid_rx_count; }

/** @brief Reports handed to the IN endpoint. */
inline uint32_t tp_usb_hid_tx_reports() { return tp_hid_tx_count; }
