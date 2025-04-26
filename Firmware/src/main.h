// Copyright 2021, Ryan Wendland, ogx360
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Changes made to main.h:
// 1. Added include guards with unique name:
//    - Reason: Changed `#ifndef _MAIN_H_` to `#ifndef OGX360_MAIN_H_` to avoid potential naming conflicts with other libraries, ensuring the header is included only once during compilation.
// 2. Added explicit include for Arduino.h:
//    - Reason: The header relies on Arduino types (e.g., uint8_t). Explicitly including <Arduino.h> ensures portability and prevents compilation errors if the header is included in a context without Arduino.h.

#ifndef OGX360_MAIN_H_
#define OGX360_MAIN_H_

#include <Arduino.h>
#include "usbd/usbd_xid.h"
#include "usbh/usbh_xinput.h"

#ifndef MAX_GAMEPADS
#define MAX_GAMEPADS 4
#endif

#define USB_HOST_RESET_PIN 9
#define ARDUINO_LED_PIN 17
#define PLAYER_ID1_PIN 19
#define PLAYER_ID2_PIN 20

#ifndef SB_DEFAULT_SENSITIVITY
#define SB_DEFAULT_SENSITIVITY 400
#endif

typedef struct
{
    xid_type_t type;
    usbd_duke_t duke;
    usbd_steelbattalion_t sb;
} usbd_controller_t;

void master_init();
void master_task();
void slave_init();
void slave_task();

#endif
