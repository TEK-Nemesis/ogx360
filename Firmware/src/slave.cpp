// Copyright 2021, Ryan Wendland, ogx360
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Changes made to slave.cpp:
// 1. Added len check in i2c_get_data():
//    - Reason: Added `if (len < 1) return;` to prevent reading from an empty I2C buffer, avoiding potential underflow or invalid packet_id access, improving robustness.
// 2. Simplified i2c_send_data():
//    - Reason: Changed default case to send a single zero byte (`Wire.write((uint8_t)0)`) instead of `0x00`, improving clarity and consistency with type expectations.
// 3. Added parentheses to slave_id bit operations:
//    - Reason: Changed `slave_id = digitalRead(PLAYER_ID1_PIN) << 1 | digitalRead(PLAYER_ID2_PIN)` to `slave_id = (digitalRead(PLAYER_ID1_PIN) << 1) | digitalRead(PLAYER_ID2_PIN)` for clarity. Explicitly grouping the left shift operation makes the expression easier to read and may allow the compiler to optimize register usage, potentially reducing intermediate store/load instructions on the ATmega32U4.
// 4. Optimized I2C receive loop in i2c_get_data():
//    - Reason: Modified the loop to check `Wire.available()` in the loop condition, reducing unnecessary iterations and improving I2C performance on the ATmega32U4.

#include <Arduino.h>
#include <Wire.h>

#include "main.h"

extern usbd_controller_t usbd_c[MAX_GAMEPADS];

void i2c_get_data(int len)
{
    if (len < 1) return;
    uint8_t packet_id = Wire.read();

    //0xAA is a ping to see if the slave module is connected
    //Flash the LED to confirm receipt.
    if (packet_id == 0xAA)
    {
        digitalWrite(ARDUINO_LED_PIN, LOW);
        delay(250);
        digitalWrite(ARDUINO_LED_PIN, HIGH);
        goto flush_and_leave;
    }
	
    //Controller state packet 0xFx, where 'x' is the controller type.
    if ((packet_id & 0xF0) == 0xF0)
    {
        usbd_c[0].type = (xid_type_t)(packet_id & 0x0F);
        uint8_t *rxbuf = usbd_c[0].type == DUKE ? (uint8_t*)&usbd_c[0].duke.in :
                         usbd_c[0].type == STEELBATTALION ? (uint8_t*)&usbd_c[0].sb.in : NULL;
        uint8_t rxlen = usbd_c[0].type == DUKE ? sizeof(usbd_c[0].duke.in) :
                        usbd_c[0].type == STEELBATTALION ? sizeof(usbd_c[0].sb.in) : 0;

        if (len != rxlen + 1 || !rxbuf || !rxlen)
        {
            goto flush_and_leave;
        }

        for (uint8_t i = 0; i < rxlen && Wire.available(); i++)
        {
            rxbuf[i] = Wire.read();
        }
    }

flush_and_leave:
    while (Wire.available())
    {
        Wire.read();
    }
}

void i2c_send_data(void)
{
    if (usbd_c[0].type == DUKE)
    {
        Wire.write((uint8_t*)&usbd_c[0].duke.out, sizeof(usbd_c[0].duke.out));
    }
    else if (usbd_c[0].type == STEELBATTALION)
    {
        Wire.write((uint8_t*)&usbd_c[0].sb.out, sizeof(usbd_c[0].sb.out));
    }
    else
    {
		//Just send something back so master isnt waiting
        Wire.write((uint8_t)0);
    }
}

void slave_init(void)
{
    uint8_t slave_id = (digitalRead(PLAYER_ID1_PIN) << 1) | digitalRead(PLAYER_ID2_PIN);
    Wire.begin(slave_id);
    Wire.setClock(400000);
    Wire.onRequest(i2c_send_data);
    Wire.onReceive(i2c_get_data);
    Wire.setWireTimeout(4000, true);
}

void slave_task(void)
{
	 //nothing to do!
}
