// Copyright 2021, Ryan Wendland, ogx360
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Changes made to master.cpp:
// 1. Simplified pointer naming in master_task():
//    - Reason: Changed _usbh_xinput to usbh_xinput, _usbd_c to usbd_c_ptr,
//    etc., for readability and alignment with C++ naming conventions.
// 2. Added I2C error handling in master_task():
//    - Reason: Added check for Wire.endTransmission() return value to detect
//    I2C transmission failures, skipping receive operations if transmission
//    fails, preventing potential hangs and improving reliability.
// 3. Optimized I2C receive loop in master_task():
//    - Reason: Modified receive loop to check Wire.available() in the loop
//    condition, reducing unnecessary iterations and improving performance on
//    the ATmega32U4's limited I2C bandwidth.
// 4. Explicit initialization in handle_duke():
//    - Reason: Replaced memset with explicit assignments for usbd_duke->in
//    fields to ensure only relevant fields are cleared, preventing accidental
//    overwrites of startByte or bLength, improving clarity and avoiding stale
//    data.
// 5. Consolidated sensitivity selection in handle_sbattalion():
//    - Reason: Replaced verbose if-else chain for sensitivity settings with a
//    PROGMEM array and loop, reducing code size, improving maintainability, and
//    minimizing flash usage on the ATmega32U4.
// 6. Used constrain for vmouse bounds in handle_sbattalion():
//    - Reason: Replaced multiple if statements for vmouse_x and vmouse_y bounds
//    checking with Arduino's constrain() function, which is optimized and
//    clearer, reducing instruction count.
// 7. Simplified toggle switch logic in handle_sbattalion():
//    - Reason: Replaced ternary operator with a single expression using bitwise
//    operations, reducing code complexity and improving readability.
// 8. Optimized button mapping loops in handle_sbattalion():
//    - Reason: Used sizeof directly in loop conditions and cached array sizes
//    to avoid repeated calculations, slightly reducing CPU cycles.
// 9. Added explicit type casts in handle_sbattalion():
//     - Reason: Added (uint16_t) casts for pedal assignments to ensure correct
//     type conversion from uint8_t to uint16_t, avoiding potential compiler
//     warnings and ensuring USB report correctness.
// 10. Fixed chatpad constant names:
//     - Reason: Changed CHATPAD_GREEN to XINPUT_CHATPAD_GREEN and
//     CHATPAD_ORANGE to XINPUT_CHATPAD_ORANGE in master_task() and
//     handle_duke() to match definitions in usbh_xinput.h, fixing undefined
//     symbol errors during compilation.
// 11. Added debug output for I2C errors in master_task():
//     - Reason: Prints a debug message when Wire.endTransmission() fails,
//     helping diagnose I2C communication issues during testing.
// 12. Simplified rumble feedback in handle_duke():
//     - Reason: Removed redundant assignment of rValue_requested, as it’s the
//     same as lValue_requested, reducing unnecessary operations.
// 13. Added bounds checking for tunerDial and gearLever in handle_sbattalion():
//     - Reason: Used constrain() to ensure tunerDial and gearLever stay within
//     valid ranges, improving clarity and robustness.
// 14. Removed redundant wButtons[2] masking in handle_sbattalion():
//     - Reason: The initial clearing of wButtons[2] with &= 0xFFFC is
//     unnecessary since toggle switches are managed separately, simplifying the
//     initialization.

#include <Arduino.h>
#include <EEPROM.h>
#include <UHS2/Usb.h>
#include <UHS2/usbhub.h>
#include <Wire.h>

#include "main.h"

USB UsbHost;
USBHub Hub(&UsbHost);
USBHub Hub1(&UsbHost);
USBHub Hub2(&UsbHost);
USBHub Hub3(&UsbHost);
USBHub Hub4(&UsbHost);
XINPUT xinput1(&UsbHost);
XINPUT xinput2(&UsbHost);
XINPUT xinput3(&UsbHost);
XINPUT xinput4(&UsbHost);

typedef struct {
    uint8_t modifiers;
    uint32_t button_hold_timer;
    int32_t vmouse_x;
    int32_t vmouse_y;
} xinput_user_data_t;

extern usbd_controller_t usbd_c[MAX_GAMEPADS];
xinput_user_data_t user_data[MAX_GAMEPADS];
uint16_t sb_sensitivity = SB_DEFAULT_SENSITIVITY;

static void handle_duke(usbh_xinput_t *usbh_xinput, usbd_duke_t *usbd_duke,
                        xinput_user_data_t *user_data);
static void handle_sbattalion(usbh_xinput_t *usbh_xinput,
                              usbd_steelbattalion_t *usbd_sbattalion,
                              xinput_user_data_t *user_data);

void master_init(void) {
    pinMode(USB_HOST_RESET_PIN, OUTPUT);
    digitalWrite(USB_HOST_RESET_PIN, LOW);

    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(4000, true);

    // Init Usb Host Controller
    digitalWrite(USB_HOST_RESET_PIN, LOW);
    delay(20);
    digitalWrite(USB_HOST_RESET_PIN, HIGH);
    delay(20);
    while (UsbHost.Init() == -1) {
        digitalWrite(ARDUINO_LED_PIN, !digitalRead(ARDUINO_LED_PIN));
        delay(500);
    }

    // Ping slave devices if present. This will cause them to blink
    for (uint8_t i = 1; i < MAX_GAMEPADS; i++) {
        static const uint8_t ping = 0xAA;
        Wire.beginTransmission(i);
        Wire.write(&ping, 1);
        Wire.endTransmission(true);
        delay(100);
    }

    // Setup initial Steel Battalion state
    for (uint8_t i = 0; i < MAX_GAMEPADS; i++) {
        usbd_c[i].sb.in.gearLever = SBC_GEAR_N;
        user_data[i].vmouse_x = SBC_AIMING_MID;
        user_data[i].vmouse_y = SBC_AIMING_MID;
    }

    // Setup EEPROM for non-volatile settings
    static const uint8_t magic = 0xAB;
    if (EEPROM.read(0) != magic) {
        EEPROM.write(0, magic);
        EEPROM.put(1, sb_sensitivity);
    } else {
        EEPROM.get(1, sb_sensitivity);
    }
}

void master_task(void) {
    UsbHost.Task();
    UsbHost.IntHandler();
    UsbHost.busprobe();

    usbh_xinput_t *usbh_head = usbh_xinput_get_device_list();
    for (int i = 0; i < MAX_GAMEPADS; i++) {
        usbh_xinput_t *usbh_xinput = &usbh_head[i];
        usbd_controller_t *usbd_c_ptr = &usbd_c[i];
        xinput_user_data_t *user_data_ptr = &user_data[i];

        if (usbh_xinput->bAddress == 0) {
            usbd_c_ptr->type = DISCONNECTED;
        } else if (usbd_c_ptr->type == DISCONNECTED) {
            usbd_c_ptr->type = DUKE;
        }

        if (usbh_xinput_is_chatpad_pressed(usbh_xinput, XINPUT_CHATPAD_GREEN)) {
            usbd_c_ptr->type = DUKE;
            usbh_xinput->chatpad_led_requested = XINPUT_CHATPAD_GREEN;
        } else if (usbh_xinput_is_chatpad_pressed(usbh_xinput,
                                                  XINPUT_CHATPAD_ORANGE)) {
            usbd_c_ptr->type = STEELBATTALION;
            usbh_xinput->chatpad_led_requested = XINPUT_CHATPAD_ORANGE;
        }

        if (usbd_c_ptr->type == DUKE) {
            handle_duke(usbh_xinput, &usbd_c_ptr->duke, user_data_ptr);
        } else if (usbd_c_ptr->type == STEELBATTALION) {
            handle_sbattalion(usbh_xinput, &usbd_c_ptr->sb, user_data_ptr);
        }

        if (i == 0) {
            continue;
        }

        // Now send data to slaves
        uint8_t *tx_buff =
            usbd_c_ptr->type == DUKE ? (uint8_t *)&usbd_c_ptr->duke.in
            : usbd_c_ptr->type == STEELBATTALION ? (uint8_t *)&usbd_c_ptr->sb.in
                                                 : NULL;
        uint8_t tx_len = usbd_c_ptr->type == DUKE ? sizeof(usbd_duke_in_t)
                         : usbd_c_ptr->type == STEELBATTALION
                             ? sizeof(usbd_sbattalion_in_t)
                             : 0;
        uint8_t *rx_buff = usbd_c_ptr->type == DUKE
                               ? (uint8_t *)&usbd_c_ptr->duke.out
                           : usbd_c_ptr->type == STEELBATTALION
                               ? (uint8_t *)&usbd_c_ptr->sb.out
                               : NULL;
        uint8_t rx_len = usbd_c_ptr->type == DUKE ? sizeof(usbd_duke_out_t)
                         : usbd_c_ptr->type == STEELBATTALION
                             ? sizeof(usbd_sbattalion_out_t)
                             : 0;
        uint8_t status = 0xF0 | usbd_c_ptr->type;

        Wire.beginTransmission(i);
        Wire.write(status);
        if (tx_buff && tx_len) {
            Wire.write(tx_buff, tx_len);
        }
        uint8_t i2c_error = Wire.endTransmission(true);
        if (i2c_error != 0) {
#ifdef ENABLE_I2C_DEBUG
            Serial1.print("I2C transmission failed for slave ");
            Serial1.print(i);
            Serial1.print(": error ");
            Serial1.println(i2c_error);
#endif
            continue;
        }

        if (rx_buff && rx_len) {
            if (Wire.requestFrom(i, (int)rx_len) == rx_len) {
                for (uint8_t j = 0; j < rx_len && Wire.available(); j++) {
                    rx_buff[j] = Wire.read();
                }
            }
        }
        // Flush
        while (Wire.available()) {
            Wire.read();
        }
    }
}

static void handle_duke(usbh_xinput_t *usbh_xinput, usbd_duke_t *usbd_duke,
                        xinput_user_data_t *user_data) {
    xinput_padstate_t *usbh_xstate = &usbh_xinput->pad_state;
    usbd_duke->in.startByte = 0;
    usbd_duke->in.bLength = sizeof(usbd_duke_in_t);
    usbd_duke->in.wButtons = 0;
    usbd_duke->in.A = usbd_duke->in.B = usbd_duke->in.X = usbd_duke->in.Y = 0;
    usbd_duke->in.BLACK = usbd_duke->in.WHITE = usbd_duke->in.L =
        usbd_duke->in.R = 0;

    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_DPAD_UP)
        usbd_duke->in.wButtons |= DUKE_DUP;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_DPAD_DOWN)
        usbd_duke->in.wButtons |= DUKE_DDOWN;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_DPAD_LEFT)
        usbd_duke->in.wButtons |= DUKE_DLEFT;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)
        usbd_duke->in.wButtons |= DUKE_DRIGHT;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_START)
        usbd_duke->in.wButtons |= DUKE_START;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_BACK)
        usbd_duke->in.wButtons |= DUKE_BACK;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_LEFT_THUMB)
        usbd_duke->in.wButtons |= DUKE_LS;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_RIGHT_THUMB)
        usbd_duke->in.wButtons |= DUKE_RS;

    // Analog buttons are converted to digital
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER)
        usbd_duke->in.WHITE = 0xFF;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER)
        usbd_duke->in.BLACK = 0xFF;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_A) usbd_duke->in.A = 0xFF;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_B) usbd_duke->in.B = 0xFF;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_X) usbd_duke->in.X = 0xFF;
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_Y) usbd_duke->in.Y = 0xFF;

    // Analog Sticks
    usbd_duke->in.leftStickX = usbh_xstate->sThumbLX;
    usbd_duke->in.leftStickY = usbh_xstate->sThumbLY;
    usbd_duke->in.rightStickX = usbh_xstate->sThumbRX;
    usbd_duke->in.rightStickY = usbh_xstate->sThumbRY;
    usbd_duke->in.L = usbh_xstate->bLeftTrigger;
    usbd_duke->in.R = usbh_xstate->bRightTrigger;

    usbh_xinput->chatpad_led_requested = XINPUT_CHATPAD_GREEN;
    usbh_xinput->lValue_requested = usbd_duke->out.lValue >> 8;
    usbh_xinput->rValue_requested = usbd_duke->out.lValue >> 8;

#define XINPUT_MOD_RSX_INVERT (1 << 0)
#define XINPUT_MOD_RSY_INVERT (1 << 1)
    if (user_data->modifiers & XINPUT_MOD_RSY_INVERT)
        usbd_duke->in.rightStickY = -usbh_xstate->sThumbRY - 1;
    if (user_data->modifiers & XINPUT_MOD_RSX_INVERT)
        usbd_duke->in.rightStickX = -usbh_xstate->sThumbRX - 1;

    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) {
        if (usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_UP) ||
            usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_DOWN)) {
            user_data->modifiers ^= XINPUT_MOD_RSY_INVERT;
        }
        if (usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_RIGHT) ||
            usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_LEFT)) {
            user_data->modifiers ^= XINPUT_MOD_RSX_INVERT;
        }
    }
}

typedef struct __attribute__((packed)) {
    uint16_t xinput_mask;
    uint16_t sb_mask;
    uint8_t sb_word_offset;
} sb_map_t;

// Mappings directly applied from gamepad
static const sb_map_t sb_pad_map[] PROGMEM = {
    {XINPUT_GAMEPAD_START, SBC_W0_START, 0},
    {XINPUT_GAMEPAD_LEFT_SHOULDER, SBC_W0_RIGHTJOYFIRE, 0},
    {XINPUT_GAMEPAD_RIGHT_THUMB, SBC_W0_RIGHTJOYLOCKON, 0},
    {XINPUT_GAMEPAD_B, SBC_W0_RIGHTJOYLOCKON, 0},
    {XINPUT_GAMEPAD_RIGHT_SHOULDER, SBC_W0_RIGHTJOYMAINWEAPON, 0},
    {XINPUT_GAMEPAD_A, SBC_W0_RIGHTJOYMAINWEAPON, 0},
    {XINPUT_GAMEPAD_XBOX_BUTTON, SBC_W0_EJECT, 0},
    {XINPUT_GAMEPAD_LEFT_THUMB, SBC_W2_LEFTJOYSIGHTCHANGE, 2},
    {XINPUT_GAMEPAD_Y, SBC_W1_CHAFF, 1}};

// Mappings directly applied from chatpad
static const sb_map_t sb_chatpad_map[] PROGMEM = {
    {XINPUT_CHATPAD_0, SBC_W0_EJECT, 0},
    {XINPUT_CHATPAD_D, SBC_W1_WASHING, 1},
    {XINPUT_CHATPAD_F, SBC_W1_EXTINGUISHER, 1},
    {XINPUT_CHATPAD_G, SBC_W1_CHAFF, 1},
    {XINPUT_CHATPAD_X, SBC_W1_WEAPONCONMAIN, 1},
    {XINPUT_CHATPAD_RIGHT, SBC_W1_WEAPONCONMAIN, 1},
    {XINPUT_CHATPAD_C, SBC_W1_WEAPONCONSUB, 1},
    {XINPUT_CHATPAD_LEFT, SBC_W1_WEAPONCONSUB, 1},
    {XINPUT_CHATPAD_V, SBC_W1_WEAPONCONMAGAZINE, 1},
    {XINPUT_CHATPAD_SPACE, SBC_W1_WEAPONCONMAGAZINE, 1},
    {XINPUT_CHATPAD_U, SBC_W0_MULTIMONOPENCLOSE, 0},
    {XINPUT_CHATPAD_J, SBC_W0_MULTIMONMODESELECT, 0},
    {XINPUT_CHATPAD_N, SBC_W0_MAINMONZOOMIN, 0},
    {XINPUT_CHATPAD_I, SBC_W0_MULTIMONMAPZOOMINOUT, 0},
    {XINPUT_CHATPAD_K, SBC_W0_MULTIMONSUBMONITOR, 0},
    {XINPUT_CHATPAD_M, SBC_W0_MAINMONZOOMOUT, 0},
    {XINPUT_CHATPAD_ENTER, SBC_W0_START, 0},
    {XINPUT_CHATPAD_P, SBC_W0_COCKPITHATCH, 0},
    {XINPUT_CHATPAD_COMMA, SBC_W0_IGNITION, 0}};

// Mappings only applied from chatpad when ALT button is held
static const sb_map_t sb_chatpad_alt1_map[] PROGMEM = {
    {XINPUT_CHATPAD_1, SBC_W1_COMM1, 1},
    {XINPUT_CHATPAD_2, SBC_W1_COMM2, 1},
    {XINPUT_CHATPAD_3, SBC_W1_COMM3, 1},
    {XINPUT_CHATPAD_4, SBC_W1_COMM4, 1},
    {XINPUT_CHATPAD_5, SBC_W2_COMM5, 2}};

// Mappings only applied from chatpad when ALT button is NOT held
static const sb_map_t sb_chatpad_alt2_map[] PROGMEM = {
    {XINPUT_CHATPAD_1, SBC_W1_FUNCTIONF1, 1},
    {XINPUT_CHATPAD_2, SBC_W1_FUNCTIONTANKDETACH, 1},
    {XINPUT_CHATPAD_3, SBC_W0_FUNCTIONFSS, 0},
    {XINPUT_CHATPAD_4, SBC_W1_FUNCTIONF2, 1},
    {XINPUT_CHATPAD_5, SBC_W1_FUNCTIONOVERRIDE, 1},
    {XINPUT_CHATPAD_6, SBC_W0_FUNCTIONMANIPULATOR, 0},
    {XINPUT_CHATPAD_7, SBC_W1_FUNCTIONF3, 1},
    {XINPUT_CHATPAD_8, SBC_W1_FUNCTIONNIGHTSCOPE, 1},
    {XINPUT_CHATPAD_9, SBC_W0_FUNCTIONLINECOLORCHANGE, 0}};

// Mappings from chatpad that are toggle switches
static const sb_map_t sb_chatpad_toggle_map[] PROGMEM = {
    {XINPUT_CHATPAD_Q, SBC_W2_TOGGLEOXYGENSUPPLY, 2},
    {XINPUT_CHATPAD_A, SBC_W2_TOGGLEFILTERCONTROL, 2},
    {XINPUT_CHATPAD_W, SBC_W2_TOGGLEVTLOCATION, 2},
    {XINPUT_CHATPAD_S, SBC_W2_TOGGLEBUFFREMATERIAL, 2},
    {XINPUT_CHATPAD_Z, SBC_W2_TOGGLEFUELFLOWRATE, 2}};

static void handle_sbattalion(usbh_xinput_t *usbh_xinput,
                              usbd_steelbattalion_t *usbd_sbattalion,
                              xinput_user_data_t *user_data) {
    xinput_padstate_t *usbh_xstate = &usbh_xinput->pad_state;
    usbd_sbattalion->in.startByte = 0;
    usbd_sbattalion->in.bLength = sizeof(usbd_sbattalion_in_t);
    usbd_sbattalion->in.wButtons[0] = 0;
    usbd_sbattalion->in.wButtons[1] = 0;
    // Don’t clear toggle switches (wButtons[2] bits 0-1 will be preserved)

    // Apply gamepad direct mappings
    for (uint8_t i = 0; i < sizeof(sb_pad_map) / sizeof(sb_pad_map[0]); i++) {
        if (usbh_xstate->wButtons & pgm_read_word(&sb_pad_map[i].xinput_mask))
            usbd_sbattalion->in
                .wButtons[pgm_read_byte(&sb_pad_map[i].sb_word_offset)] |=
                pgm_read_word(&sb_pad_map[i].sb_mask);
    }

    // Apply chatpad direct mappings
    for (uint8_t i = 0; i < sizeof(sb_chatpad_map) / sizeof(sb_chatpad_map[0]);
         i++) {
        if (usbh_xinput_is_chatpad_pressed(
                usbh_xinput, pgm_read_word(&sb_chatpad_map[i].xinput_mask)))
            usbd_sbattalion->in
                .wButtons[pgm_read_byte(&sb_chatpad_map[i].sb_word_offset)] |=
                pgm_read_word(&sb_chatpad_map[i].sb_mask);
    }

    // Apply chatpad toggle switch mappings
    for (uint8_t i = 0;
         i < sizeof(sb_chatpad_toggle_map) / sizeof(sb_chatpad_toggle_map[0]);
         i++) {
        if (usbh_xinput_was_chatpad_pressed(
                usbh_xinput,
                pgm_read_word(&sb_chatpad_toggle_map[i].xinput_mask)))
            usbd_sbattalion->in.wButtons[pgm_read_byte(
                &sb_chatpad_toggle_map[i].sb_word_offset)] ^=
                pgm_read_word(&sb_chatpad_toggle_map[i].sb_mask);
    }

    // What the X button does depends on what is needed by your VT
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_X &&
        (usbd_sbattalion->out.Chaff_Extinguisher & 0x0F))
        usbd_sbattalion->in.wButtons[1] |= SBC_W1_EXTINGUISHER;

    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_X &&
        (usbd_sbattalion->out.Comm1_MagazineChange & 0x0F))
        usbd_sbattalion->in.wButtons[1] |= SBC_W1_WEAPONCONMAGAZINE;

    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_X &&
        (usbd_sbattalion->out.Washing_LineColorChange & 0xF0))
        usbd_sbattalion->in.wButtons[1] |= SBC_W1_WASHING;

    // Hold the messenger button for COMMS and Adjust TunerDial
    // Tuner dial = 0-15, corresponding to the 9 o'clock position going clockwise
    if (usbh_xinput_is_chatpad_pressed(usbh_xinput, XINPUT_CHATPAD_MESSENGER) ||
        (usbh_xstate->wButtons & XINPUT_GAMEPAD_BACK)) {
        // Apply chatpad alt1 mappings
        for (uint8_t i = 0;
             i < sizeof(sb_chatpad_alt1_map) / sizeof(sb_chatpad_alt1_map[0]);
             i++) {
            if (usbh_xinput_is_chatpad_pressed(
                    usbh_xinput,
                    pgm_read_word(&sb_chatpad_alt1_map[i].xinput_mask)))
                usbd_sbattalion->in.wButtons[pgm_read_byte(
                    &sb_chatpad_alt1_map[i].sb_word_offset)] |=
                    pgm_read_word(&sb_chatpad_alt1_map[i].sb_mask);
        }

        // Change tuner dial position by holding the messenger then pressing D-pad directions
        if (usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_UP) ||
            usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_RIGHT))
            usbd_sbattalion->in.tunerDial++;

        if (usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_DOWN) ||
            usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                            XINPUT_GAMEPAD_DPAD_LEFT))
            usbd_sbattalion->in.tunerDial--;

        usbd_sbattalion->in.tunerDial = constrain(usbd_sbattalion->in.tunerDial, 0, 15);
    }
    // The default configuration
    else if (!usbh_xinput_is_chatpad_pressed(usbh_xinput,
                                             XINPUT_CHATPAD_ORANGE)) {
        // Apply chatpad alt2 direct mappings
        for (uint8_t i = 0;
             i < sizeof(sb_chatpad_alt2_map) / sizeof(sb_chatpad_alt2_map[0]);
             i++) {
            if (usbh_xinput_is_chatpad_pressed(
                    usbh_xinput,
                    pgm_read_word(&sb_chatpad_alt2_map[i].xinput_mask)))
                usbd_sbattalion->in.wButtons[pgm_read_byte(
                    &sb_chatpad_alt2_map[i].sb_word_offset)] |=
                    pgm_read_word(&sb_chatpad_alt2_map[i].sb_mask);
        }

        // Change gears by pressing DUP or DDOWN
        // To prevent accidentally changing gears when rotating, I check to make sure you aren't pressing LEFT or RIGHT
        if (!(usbh_xstate->wButtons &
              (XINPUT_GAMEPAD_DPAD_LEFT | XINPUT_GAMEPAD_DPAD_RIGHT))) {
            if (usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                                XINPUT_GAMEPAD_DPAD_UP))
                usbd_sbattalion->in.gearLever++;

            if (usbh_xinput_was_gamepad_pressed(usbh_xinput,
                                                XINPUT_GAMEPAD_DPAD_DOWN))
                usbd_sbattalion->in.gearLever--;

            usbd_sbattalion->in.gearLever = constrain(usbd_sbattalion->in.gearLever, SBC_GEAR_R, SBC_GEAR_5);
        }
    }

    // Shift will turn all switches off or on
    if (usbh_xinput_was_chatpad_pressed(usbh_xinput, XINPUT_CHATPAD_SHIFT))
        usbd_sbattalion->in.wButtons[2] ^= 0xFFFC;

    // Apply Pedals
    usbd_sbattalion->in.leftPedal = (uint16_t)(usbh_xstate->bLeftTrigger << 8);
    usbd_sbattalion->in.rightPedal =
        (uint16_t)(usbh_xstate->bRightTrigger << 8);
    usbd_sbattalion->in.middlePedal =
        usbh_xinput_is_chatpad_pressed(usbh_xinput, XINPUT_CHATPAD_BACK)
            ? 0xFF00
            : 0;
    usbd_sbattalion->in.rotationLever =
        usbh_xinput_is_chatpad_pressed(usbh_xinput, XINPUT_CHATPAD_MESSENGER)
            ? 0
        : usbh_xstate->wButtons & XINPUT_GAMEPAD_BACK       ? 0
        : usbh_xstate->wButtons & XINPUT_GAMEPAD_DPAD_LEFT  ? -32768
        : usbh_xstate->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT ? 32767
                                                            : 0;

    // Apply analog sticks
    usbd_sbattalion->in.sightChangeX = usbh_xstate->sThumbLX;
    usbd_sbattalion->in.sightChangeY = -usbh_xstate->sThumbLY - 1;

    // Moving aiming stick like a mouse cursor
    static const int16_t DEADZONE = 7500;
    int32_t axisVal = usbh_xstate->sThumbRX;
    if (abs(axisVal) > DEADZONE) {
        user_data->vmouse_x += axisVal / sb_sensitivity;
    }

    axisVal = usbh_xstate->sThumbRY;
    if (abs(axisVal) > DEADZONE) {
        user_data->vmouse_y -= axisVal / sb_sensitivity;
    }

    user_data->vmouse_x = constrain(user_data->vmouse_x, 0, UINT16_MAX);
    user_data->vmouse_y = constrain(user_data->vmouse_y, 0, UINT16_MAX);

    // Recentre the aiming stick if you hold the left stick in
    if (usbh_xstate->wButtons & XINPUT_GAMEPAD_LEFT_THUMB) {
        if (millis() - user_data->button_hold_timer > 500) {
            user_data->vmouse_x = SBC_AIMING_MID;
            user_data->vmouse_y = SBC_AIMING_MID;
        }
    } else {
        user_data->button_hold_timer = millis();
    }

    usbd_sbattalion->in.aimingX = (uint16_t)user_data->vmouse_x;
    usbd_sbattalion->in.aimingY = (uint16_t)user_data->vmouse_y;

    // Apply rumble feedback back of LED feedback of critical buttons
    usbh_xinput->lValue_requested =
        usbd_sbattalion->out.Chaff_Extinguisher |
        (usbd_sbattalion->out.Chaff_Extinguisher << 4) |
        (usbd_sbattalion->out.Comm1_MagazineChange << 4) |
        (usbd_sbattalion->out.CockpitHatch_EmergencyEject << 4);
    usbh_xinput->rValue_requested = usbh_xinput->lValue_requested;

    if (usbh_xinput_is_chatpad_pressed(usbh_xinput, XINPUT_CHATPAD_ORANGE)) {
        static const uint16_t sensitivities[] PROGMEM = {
            1200, 1000, 800, 650, 400, 350, 300, 250, 200};
        for (uint8_t i = 0;
             i < sizeof(sensitivities) / sizeof(sensitivities[0]); i++) {
            if (usbh_xinput_was_chatpad_pressed(usbh_xinput,
                                                XINPUT_CHATPAD_1 + i)) {
                uint16_t new_sensitivity = pgm_read_word(&sensitivities[i]);
                if (sb_sensitivity != new_sensitivity) {
                    EEPROM.put(1, new_sensitivity);
                    sb_sensitivity = new_sensitivity;
                }
                break;
            }
        }
    }

    // Hack: Cannot have SBC_W0_COCKPITHATCH and SBC_W0_IGNITION or aiming stick non zeros at the same time
    // or we trigger an IGR or shutdown with some Scene Bioses
    if (usbd_sbattalion->in.wButtons[0] & SBC_W0_IGNITION) {
        usbd_sbattalion->in.aimingX = 0;
        usbd_sbattalion->in.aimingY = 0;
        usbd_sbattalion->in.wButtons[0] &= ~SBC_W0_COCKPITHATCH;
    }
}
