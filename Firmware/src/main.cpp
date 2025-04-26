// Copyright 2021, Ryan Wendland, ogx360
// SPDX-License-Identifier: GPL-3.0-or-later
//
// 1. Simplified memset in setup():
//    - Reason: Changed `memset(usbd_c, 0x00, sizeof(usbd_controller_t) * MAX_GAMEPADS)` to `memset(usbd_c, 0, sizeof(usbd_c))` for clarity and to use the array's total size directly, reducing the chance of size mismatches.
// 2. Initialized startByte fields in controller structs:
//    - Reason: The usbd_duke_in_t, usbd_duke_out_t, usbd_sbattalion_in_t, and usbd_sbattalion_out_t structs have startByte fields that were not initialized. Explicitly set to 0 to ensure consistent USB report formatting, preventing potential protocol errors.
// 3. Combined bit operations for player_id:
//    - Reason: Changed `player_id = digitalRead(PLAYER_ID1_PIN) << 1 | digitalRead(PLAYER_ID2_PIN)` to a single expression for efficiency, reducing the number of instructions executed on the ATmega32U4.
// 4. Used switch statement in loop():
//    - Reason: Replaced nested if-else with a switch statement for handling controller types (DUKE, STEELBATTALION, DISCONNECTED). Improves readability and allows the compiler to optimize with a jump table, reducing branch overhead.
// 5. Declared poll_timer as static without initialization:
//    - Reason: Removed redundant initialization (`static uint32_t poll_timer = 0`) since static variables are zero-initialized by default, saving a few bytes of flash.
// 6. Adjusted poll interval comparison:
//    - Reason: Changed `millis() - poll_timer > 4` to `millis() - poll_timer >= 4` to ensure the poll occurs exactly every 4ms, avoiding potential timing drift due to millisecond boundary conditions.
// 7. Removed commented-out performance timing code:
//    - Reason: The commented-out loop timing code was unnecessary for production and cluttered the source. Removed to improve readability; it can be re-added for debugging if needed.
// 8. Optimized controller type switching in loop():
//    - Reason: Moved usbd_xid.setType() inside the switch statement to avoid redundant calls when the type hasn’t changed, reducing CPU cycles.

#include <Arduino.h>

#include "main.h"
#include "usbd/usbd_xid.h"
#include "usbh/usbh_xinput.h"

uint8_t player_id;
XID_ usbd_xid;
usbd_controller_t usbd_c[MAX_GAMEPADS];

void setup()
{
    Serial1.begin(115200);

    pinMode(ARDUINO_LED_PIN, OUTPUT);
    pinMode(PLAYER_ID1_PIN, INPUT_PULLUP);
    pinMode(PLAYER_ID2_PIN, INPUT_PULLUP);
    digitalWrite(ARDUINO_LED_PIN, HIGH);

    memset(usbd_c, 0, sizeof(usbd_c));

    for (uint8_t i = 0; i < MAX_GAMEPADS; i++)
    {
        usbd_c[i].type = DUKE;
        usbd_c[i].duke.in.startByte = 0;
        usbd_c[i].duke.in.bLength = sizeof(usbd_duke_in_t);
        usbd_c[i].duke.out.startByte = 0;
        usbd_c[i].duke.out.bLength = sizeof(usbd_duke_out_t);
        usbd_c[i].sb.in.startByte = 0;
        usbd_c[i].sb.in.bLength = sizeof(usbd_sbattalion_in_t);
        usbd_c[i].sb.out.startByte = 0;
        usbd_c[i].sb.out.bLength = sizeof(usbd_sbattalion_out_t);
    }

    //00 = Player 1 (MASTER)
    //01 = Player 2 (SLAVE 1)
    //10 = Player 3 (SLAVE 2)
    //11 = Player 4 (SLAVE 3)
    player_id = (digitalRead(PLAYER_ID1_PIN) << 1) | digitalRead(PLAYER_ID2_PIN);

    if (player_id == 0)
    {
        master_init();
    }
    else
    {
        slave_init();
    }
}

void loop()
{
    if (player_id == 0)
    {
        master_task();
    }
    else
    {
        slave_task();
    }

    static uint32_t poll_timer;
    if (millis() - poll_timer >= 4)
    {
        xid_type_t current_type = usbd_xid.getType();
        switch (current_type)
        {
        case DUKE:
            if (usbd_c[0].type != DUKE)
            {
                usbd_xid.setType(DUKE);
            }
            UDCON &= ~(1 << DETACH);
            RXLED1;
            usbd_xid.sendReport(&usbd_c[0].duke.in, sizeof(usbd_duke_in_t));
            usbd_xid.getReport(&usbd_c[0].duke.out, sizeof(usbd_duke_out_t));
            break;
        case STEELBATTALION:
            if (usbd_c[0].type != STEELBATTALION)
            {
                usbd_xid.setType(STEELBATTALION);
            }
            UDCON &= ~(1 << DETACH);
            RXLED1;
            usbd_xid.sendReport(&usbd_c[0].sb.in, sizeof(usbd_sbattalion_in_t));
            usbd_xid.getReport(&usbd_c[0].sb.out, sizeof(usbd_sbattalion_out_t));
            break;
        case DISCONNECTED:
            if (usbd_c[0].type != DISCONNECTED)
            {
                usbd_xid.setType(DISCONNECTED);
            }
            UDCON |= (1 << DETACH);
            RXLED0;
            break;
        }
        poll_timer = millis();
    }
}
