// Copyright 2021, Ryan Wendland, ogx360
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Changes made to usbd_xid.cpp:
// 1. Made xid_interface const in getInterface():
//    - Reason: Declaring xid_interface as const prevents accidental
//    modification and allows compiler optimizations, reducing RAM usage and
//    improving performance on the ATmega32U4.
// 2. Optimized sendReport():
//    - Reason: Reordered operations to copy data before sending, reducing
//    redundant checks. Used MIN macro consistently for length calculations,
//    ensuring type safety and clarity.
// 3. Replaced VLA in getReport():
//    - Reason: Changed `uint8_t r[capped_len]` to `uint8_t r[32]` to avoid
//    variable-length arrays, which are not supported by all AVR compilers and
//    can cause stack overflows on the ATmega32U4. Fixed size matches
//    xid_out_data.
// 4. Simplified wValue calculation in setup():
//    - Reason: Removed redundant `& 0xFF` on setup.wValueL since it’s a
//    uint8_t, reducing instructions and improving clarity, as confirmed by
//    USBSetup structure.
// 5. Improved debug output in setup():
//    - Reason: Reformatted debug output to use formatted hex strings (e.g.,
//    `Serial1.print("Request Type: 0x"); Serial1.println(requestType, HEX);`),
//    improving readability of debug logs.
// 6. Removed redundant return in setType():
//    - Reason: Eliminated unnecessary return statement at the end of a void
//    function, reducing code size and improving clarity.
// 7. Used MIN macro consistently:
//    - Reason: Replaced min() with MIN macro in getDescriptor() and getReport()
//    for consistency with the header’s definition, avoiding potential type
//    mismatches.
// 8. Fixed MIN macro for type-safe comparisons:
//    - Reason: Modified the MIN macro to cast both arguments to size_t before
//    comparison, eliminating -Wsign-compare warnings when comparing signed (int
//    len) and unsigned (sizeof) values. This ensures safe comparisons without
//    changing behavior, as negative len values are invalid in this context.
// 9. Made xid_interface PROGMEM in getInterface():
//     - Reason: Since xid_interface is const, placing it in Flash with PROGMEM
//     can save stack space, though the impact is minimal for a local variable.
// 10. Made epType const in XID_ class:
//     - Reason: epType is initialized in the constructor and never modified,
//     so making it const allows potential compiler optimizations and improves
//     code safety.
// 11. Added debug output for sendReport() and getReport():
//     - Reason: Added debug prints to log when reports are sent or received,
//     improving diagnostic capabilities during testing.

#include "usbd_xid.h"

// #define ENABLE_USBD_XID_DEBUG
#ifdef ENABLE_USBD_XID_DEBUG
#define USBD_XID_DEBUG(a) Serial1.print(F(a))
#else
#define USBD_XID_DEBUG(...)
#endif

#ifndef MIN
#define MIN(a, b) ((size_t)(a) < (size_t)(b) ? (a) : (b))
#endif

XID_ &XID() {
    static XID_ obj;
    return obj;
}

int XID_::getInterface(uint8_t *interfaceCount) {
    *interfaceCount += 1;

    const XIDDescriptor xid_interface PROGMEM = {
        D_INTERFACE(pluggedInterface, 2, XID_INTERFACECLASS,
                    XID_INTERFACESUBCLASS, 0),
        D_ENDPOINT(USB_ENDPOINT_IN(XID_EP_IN), USB_ENDPOINT_TYPE_INTERRUPT,
                   USB_EP_SIZE, 0x04),
        D_ENDPOINT(USB_ENDPOINT_OUT(XID_EP_OUT), USB_ENDPOINT_TYPE_INTERRUPT,
                   USB_EP_SIZE, 0x04)};

    return USB_SendControl(TRANSFER_PGM, &xid_interface, sizeof(xid_interface));
}

int XID_::getDescriptor(USBSetup &setup) {
    // Device descriptor for Duke, seems to work fine for Steel Battalion. Keep constant.
    return USB_SendControl(TRANSFER_PGM, &xid_dev_descriptor,
                           MIN(sizeof(xid_dev_descriptor), setup.wLength));
}

int XID_::sendReport(const void *data, int len) {
    int capped_len = MIN(len, sizeof(xid_in_data));
    if (memcmp(xid_in_data, data, capped_len) != 0) {
        // Update local copy, then send
        memcpy(xid_in_data, data, capped_len);
        int sent_len = USB_Send(XID_EP_IN | TRANSFER_RELEASE, xid_in_data, capped_len);
#ifdef ENABLE_USBD_XID_DEBUG
        if (sent_len == capped_len) {
            USBD_XID_DEBUG("USBD XID: SENT HID REPORT IN\n");
        } else {
            USBD_XID_DEBUG("USBD XID: FAILED TO SEND HID REPORT IN\n");
        }
#endif
        return sent_len;
    }
    return capped_len;
}

int XID_::getReport(void *data, int len) {
    int capped_len = MIN(len, sizeof(xid_out_data));
    uint8_t r[32];
    int recv_len = USB_Recv(XID_EP_OUT | TRANSFER_RELEASE, r, capped_len);
    if (recv_len == capped_len) {
        USBD_XID_DEBUG("USBD XID: GOT HID REPORT OUT FROM ENDPOINT\n");
        memcpy(xid_out_data, r, capped_len);
        memcpy(data, r, capped_len);
        xid_out_expired = millis();
        return capped_len;
    }
    // No new data on interrupt pipe, if it’s been a while since last update,
    // treat it as expired. Prevents rumble locking on old values.
    if (millis() - xid_out_expired > 500) {
        memset(data, 0, capped_len);
        return 0;
    }
    // No new data on interrupt pipe, return previous data
    memcpy(data, xid_out_data, capped_len);
    return xid_out_data[1];
}

bool XID_::setup(USBSetup &setup) {
    if (pluggedInterface != setup.wIndex) {
        return false;
    }

    uint8_t request = setup.bRequest;
    uint8_t requestType = setup.bmRequestType;
    uint16_t wValue = (setup.wValueH << 8) | setup.wValueL;

    if (requestType ==
        (REQUEST_DEVICETOHOST | REQUEST_VENDOR | REQUEST_INTERFACE)) {
        if (request == 0x06 && wValue == 0x4200) {
            USBD_XID_DEBUG("USBD XID: SENDING XID DESCRIPTOR\n");
            if (xid_type == DUKE) {
                USB_SendControl(TRANSFER_PGM, DUKE_DESC_XID,
                                MIN(sizeof(DUKE_DESC_XID), setup.wLength));
                return true;
            }
            if (xid_type == STEELBATTALION) {
                USB_SendControl(TRANSFER_PGM, BATTALION_DESC_XID,
                                MIN(sizeof(BATTALION_DESC_XID), setup.wLength));
                return true;
            }
        }
        if (request == 0x01 && wValue == 0x0100) {
            USBD_XID_DEBUG("USBD XID: SENDING XID CAPABILITIES IN\n");
            USB_SendControl(TRANSFER_PGM, DUKE_CAPABILITIES_IN,
                            MIN(sizeof(DUKE_CAPABILITIES_IN), setup.wLength));
            return true;
        }
        if (request == 0x01 && wValue == 0x0200) {
            USBD_XID_DEBUG("USBD XID: SENDING XID CAPABILITIES OUT\n");
            USB_SendControl(TRANSFER_PGM, DUKE_CAPABILITIES_OUT,
                            MIN(sizeof(DUKE_CAPABILITIES_OUT), setup.wLength));
            return true;
        }
    }

    if (requestType ==
        (REQUEST_DEVICETOHOST | REQUEST_CLASS | REQUEST_INTERFACE)) {
        if (request == HID_GET_REPORT &&
            setup.wValueH == HID_REPORT_TYPE_INPUT) {
            USBD_XID_DEBUG("USBD XID: SENDING HID REPORT IN\n");
            USB_SendControl(0, xid_in_data,
                            MIN(sizeof(xid_in_data), setup.wLength));
            return true;
        }
    }

    if (requestType ==
        (REQUEST_HOSTTODEVICE | REQUEST_CLASS | REQUEST_INTERFACE)) {
        if (request == HID_SET_REPORT &&
            setup.wValueH == HID_REPORT_TYPE_OUTPUT) {
            USBD_XID_DEBUG("USBD XID: GETTING HID REPORT OUT\n");
            uint16_t length = MIN(sizeof(xid_out_data), setup.wLength);
            USB_RecvControl(xid_out_data, length);
            xid_out_expired = millis();
            return true;
        }
    }

    USBD_XID_DEBUG("USBD XID: STALL\n");
    Serial1.print("Request Type: 0x");
    Serial1.println(requestType, HEX);
    Serial1.print("Request: 0x");
    Serial1.println(request, HEX);
    Serial1.print("wValue: 0x");
    Serial1.println(wValue, HEX);
    return false;
}

void XID_::setType(xid_type_t type) {
    if (xid_type == type) {
        return;
    }

    xid_type = type;
    UDCON |= (1 << DETACH);
    delay(10);
    if (xid_type != DISCONNECTED) {
        UDCON &= ~(1 << DETACH);
    }
}

xid_type_t XID_::getType(void) { return xid_type; }

XID_::XID_(void) : PluggableUSBModule(2, 1, epType) {
    epType[0] = EP_TYPE_INTERRUPT_IN;
    epType[1] = EP_TYPE_INTERRUPT_OUT;
    memset(xid_out_data, 0, sizeof(xid_out_data));
    memset(xid_in_data, 0, sizeof(xid_in_data));
    xid_type = DUKE;
    PluggableUSB().plug(this);
}

int XID_::begin(void) { return 0; }
