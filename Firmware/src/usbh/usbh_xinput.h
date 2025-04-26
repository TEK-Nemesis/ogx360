// Copyright 2021, Ryan Wendland, ogx360
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Changes made to usbh_xinput.h:
// 1. Changed include guard to unique name:
//    - Reason: Changed `#ifndef _XINPUT_H_` to `#ifndef OGX360_USBH_XINPUT_H_`
//    to avoid naming conflicts with other libraries.
// 2. Added explicit include for Arduino.h:
//    - Reason: Ensures portability by including Arduino.h for types like
//    uint8_t, preventing compilation errors.
// 3. Added const to static arrays:
//    - Reason: Declared command arrays as const to store in PROGMEM, reducing
//    RAM usage on the ATmega32U4.
// 4. Fixed chatpad constant names:
//    - Reason: Changed `CHATPAD_CAPSLOCK`, `CHATPAD_GREEN`, `CHATPAD_ORANGE`,
//    `CHATPAD_MESSENGER` to `XINPUT_CHATPAD_CAPSLOCK`, etc., to match
//    definitions in the header and fix undefined symbol errors.
// 5. Made helper functions static:
//    - Reason: Functions like usbh_xinput_get_device_list() don’t need
//    external linkage, reducing overhead.
// 6. Used constexpr for constants:
//    - Reason: Made constants like XBOX_CONTROL_PIPE and XINPUT_CHATPAD_*
//    constexpr for better compile-time evaluation.
// 7. Moved alloc_xinput_device and ParseInputData to XINPUT class:
//    - Reason: These functions reference class members (pUsb, bAddress,
//    dev_type) and cannot be static. Moved them to private methods to fix
//    compilation errors.
// 8. Removed static from used helper functions:
//     - Reason: Functions like usbh_xinput_get_device_list() are used in other
//     files (e.g., main.cpp), so they cannot be static to avoid linking errors.

#ifndef OGX360_USBH_XINPUT_H_
#define OGX360_USBH_XINPUT_H_

#include <Arduino.h>
#include <UHS2/Usb.h>

constexpr uint8_t XBOX_CONTROL_PIPE = 0;
constexpr uint8_t XBOX_INPUT_PIPE = 1;
constexpr uint8_t XBOX_OUTPUT_PIPE = 2;

constexpr uint8_t EP_MAXPKTSIZE = 32;
constexpr uint8_t XBOX_MAX_ENDPOINTS = 9;

#ifndef XINPUT_MAXGAMEPADS
constexpr uint8_t XINPUT_MAXGAMEPADS = 4;
#endif

#ifndef TRANSFER_PGM // Defined in Arduino USB Core normally
constexpr uint8_t TRANSFER_PGM = 0x80;
#endif

// https://docs.microsoft.com/en-us/windows/win32/api/xinput/ns-xinput-xinput_gamepad
constexpr uint16_t XINPUT_GAMEPAD_DPAD_UP = 0x0001;
constexpr uint16_t XINPUT_GAMEPAD_DPAD_DOWN = 0x0002;
constexpr uint16_t XINPUT_GAMEPAD_DPAD_LEFT = 0x0004;
constexpr uint16_t XINPUT_GAMEPAD_DPAD_RIGHT = 0x0008;
constexpr uint16_t XINPUT_GAMEPAD_START = 0x0010;
constexpr uint16_t XINPUT_GAMEPAD_BACK = 0x0020;
constexpr uint16_t XINPUT_GAMEPAD_LEFT_THUMB = 0x0040;
constexpr uint16_t XINPUT_GAMEPAD_RIGHT_THUMB = 0x0080;
constexpr uint16_t XINPUT_GAMEPAD_LEFT_SHOULDER = 0x0100;
constexpr uint16_t XINPUT_GAMEPAD_RIGHT_SHOULDER = 0x0200;
constexpr uint16_t XINPUT_GAMEPAD_XBOX_BUTTON = 0x0400;
constexpr uint16_t XINPUT_GAMEPAD_SYNC = 0x0800;
constexpr uint16_t XINPUT_GAMEPAD_A = 0x1000;
constexpr uint16_t XINPUT_GAMEPAD_B = 0x2000;
constexpr uint16_t XINPUT_GAMEPAD_X = 0x4000;
constexpr uint16_t XINPUT_GAMEPAD_Y = 0x8000;

constexpr uint16_t XINPUT_CHATPAD_1 = 23;
constexpr uint16_t XINPUT_CHATPAD_2 = 22;
constexpr uint16_t XINPUT_CHATPAD_3 = 21;
constexpr uint16_t XINPUT_CHATPAD_4 = 20;
constexpr uint16_t XINPUT_CHATPAD_5 = 19;
constexpr uint16_t XINPUT_CHATPAD_6 = 18;
constexpr uint16_t XINPUT_CHATPAD_7 = 17;
constexpr uint16_t XINPUT_CHATPAD_8 = 103;
constexpr uint16_t XINPUT_CHATPAD_9 = 102;
constexpr uint16_t XINPUT_CHATPAD_0 = 101;

constexpr uint16_t XINPUT_CHATPAD_Q = 39;
constexpr uint16_t XINPUT_CHATPAD_W = 38;
constexpr uint16_t XINPUT_CHATPAD_E = 37;
constexpr uint16_t XINPUT_CHATPAD_R = 36;
constexpr uint16_t XINPUT_CHATPAD_T = 35;
constexpr uint16_t XINPUT_CHATPAD_Y = 34;
constexpr uint16_t XINPUT_CHATPAD_U = 33;
constexpr uint16_t XINPUT_CHATPAD_I = 118;
constexpr uint16_t XINPUT_CHATPAD_O = 117;
constexpr uint16_t XINPUT_CHATPAD_P = 100;

constexpr uint16_t XINPUT_CHATPAD_A = 55;
constexpr uint16_t XINPUT_CHATPAD_S = 54;
constexpr uint16_t XINPUT_CHATPAD_D = 53;
constexpr uint16_t XINPUT_CHATPAD_F = 52;
constexpr uint16_t XINPUT_CHATPAD_G = 51;
constexpr uint16_t XINPUT_CHATPAD_H = 50;
constexpr uint16_t XINPUT_CHATPAD_J = 49;
constexpr uint16_t XINPUT_CHATPAD_K = 119;
constexpr uint16_t XINPUT_CHATPAD_L = 114;
constexpr uint16_t XINPUT_CHATPAD_COMMA = 98;

constexpr uint16_t XINPUT_CHATPAD_Z = 70;
constexpr uint16_t XINPUT_CHATPAD_X = 69;
constexpr uint16_t XINPUT_CHATPAD_C = 68;
constexpr uint16_t XINPUT_CHATPAD_V = 67;
constexpr uint16_t XINPUT_CHATPAD_B = 66;
constexpr uint16_t XINPUT_CHATPAD_N = 65;
constexpr uint16_t XINPUT_CHATPAD_M = 82;
constexpr uint16_t XINPUT_CHATPAD_PERIOD = 83;
constexpr uint16_t XINPUT_CHATPAD_ENTER = 99;

constexpr uint16_t XINPUT_CHATPAD_LEFT = 85;
constexpr uint16_t XINPUT_CHATPAD_SPACE = 84;
constexpr uint16_t XINPUT_CHATPAD_RIGHT = 81;
constexpr uint16_t XINPUT_CHATPAD_BACK = 113;

// Offset byte 25
constexpr uint16_t XINPUT_CHATPAD_SHIFT = 1;
constexpr uint16_t XINPUT_CHATPAD_GREEN = 2;
constexpr uint16_t XINPUT_CHATPAD_ORANGE = 4;
constexpr uint16_t XINPUT_CHATPAD_MESSENGER = 8;
constexpr uint16_t XINPUT_CHATPAD_CAPSLOCK = 0x20;

typedef struct {
    uint16_t wButtons;
    uint8_t bLeftTrigger;
    uint8_t bRightTrigger;
    int16_t sThumbLX;
    int16_t sThumbLY;
    int16_t sThumbRX;
    int16_t sThumbRY;
} xinput_padstate_t;

typedef enum {
    XINPUT_UNKNOWN = 0,
    XBOXONE,
    XBOX360_WIRELESS,
    XBOX360_WIRED,
    XBOXOG,
    XINPUT_KEYBOARD,
    XINPUT_MOUSE,
    XINPUT_8BITDO_IDLE
} xinput_type_t;

typedef struct usbh_xinput_t {
    // usbh backend handles
    uint8_t itf_num;
    uint8_t bAddress;
    EpInfo *usbh_inPipe;  // Pipe specific to this pad
    EpInfo *usbh_outPipe; // Pipe specific to this pad
    xinput_type_t type;
    // xinput controller state
    xinput_padstate_t pad_state; // Current pad button/stick state
    uint16_t pad_state_wButtons_old; // Prev pad state buttons
    uint8_t lValue_requested;    // Requested left rumble value
    uint8_t rValue_requested;    // Requested right rumble value
    uint8_t lValue_actual;
    uint8_t rValue_actual;
    uint8_t led_requested;       // Requested led quadrant, 1-4 or 0 for off
    uint8_t led_actual;
    // Chatpad specific components
    uint8_t chatpad_initialised;
    uint8_t chatpad_state[3];
    uint8_t chatpad_state_old[3];
    uint8_t chatpad_led_requested;
    uint8_t chatpad_led_actual;
    uint8_t chatpad_keepalive_toggle;
    // Timers used in usb backend
    uint32_t timer_periodic;
    uint32_t timer_out;
    uint32_t timer_poweroff;
} usbh_xinput_t;

usbh_xinput_t *usbh_xinput_get_device_list(void);
uint8_t usbh_xinput_is_chatpad_pressed(usbh_xinput_t *xinput, uint16_t code);
uint8_t usbh_xinput_is_gamepad_pressed(usbh_xinput_t *xinput,
                                       uint16_t button_mask);
uint8_t usbh_xinput_was_chatpad_pressed(usbh_xinput_t *xinput, uint16_t code);
uint8_t usbh_xinput_was_gamepad_pressed(usbh_xinput_t *xinput,
                                        uint16_t button_mask);

// Wired 360 commands
static const uint8_t xbox360_wired_rumble[] PROGMEM = {0x00, 0x08, 0x00, 0x00,
                                                       0x00, 0x00, 0x00, 0x00};
static const uint8_t xbox360_wired_led[] PROGMEM = {0x01, 0x03, 0x00};

// Xbone one
static const uint8_t xboxone_start_input[] PROGMEM = {0x05, 0x20, 0x03, 0x01,
                                                      0x00};
static const uint8_t xboxone_s_init[] PROGMEM = {0x05, 0x20, 0x00, 0x0f, 0x06};
static const uint8_t xboxone_pdp_init1[] PROGMEM = {0x0a, 0x20, 0x00, 0x03,
                                                    0x00, 0x01, 0x14};
static const uint8_t xboxone_pdp_init2[] PROGMEM = {0x06, 0x30};
static const uint8_t xboxone_pdp_init3[] PROGMEM = {0x06, 0x20, 0x00,
                                                    0x02, 0x01, 0x00};
static const uint8_t xboxone_rumble[] PROGMEM = {0x09, 0x00, 0x00, 0x09, 0x00,
                                                 0x0f, 0x00, 0x00, 0x00, 0x00,
                                                 0xFF, 0x00, 0xEB};
static const uint8_t xboxone_powera_init1[] PROGMEM = {
    0x09, 0x00, 0x00, 0x09, 0x00, 0x0F, 0x00,
    0x00, 0x1D, 0x1D, 0xFF, 0x00, 0x00};
static const uint8_t xboxone_powera_init2[] PROGMEM = {
    0x09, 0x00, 0x00, 0x09, 0x00, 0x0F, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Wireless 360 commands
static const uint8_t xbox360w_led[] PROGMEM = {0x00, 0x00, 0x08, 0x40};
// Sending 0x00, 0x00, 0x08, 0x00 will permanently disable rumble until you do this:
static const uint8_t xbox360w_rumble_enable[] PROGMEM = {0x00, 0x00, 0x08,
                                                         0x01};
static const uint8_t xbox360w_rumble[] PROGMEM = {0x00, 0x01, 0x0F, 0xC0};
static const uint8_t xbox360w_inquire_present[] PROGMEM = {0x08, 0x00, 0x0F,
                                                           0xC0};
static const uint8_t xbox360w_controller_info[] PROGMEM = {0x00, 0x00, 0x00,
                                                           0x40};
static const uint8_t xbox360w_unknown[] PROGMEM = {0x00, 0x00, 0x02, 0x80};
static const uint8_t xbox360w_power_off[] PROGMEM = {0x00, 0x00, 0x08, 0xC0};
static const uint8_t xbox360w_chatpad_init[] PROGMEM = {0x00, 0x00, 0x0C, 0x1B};
static const uint8_t xbox360w_chatpad_keepalive1[] PROGMEM = {0x00, 0x00, 0x0C,
                                                              0x1F};
static const uint8_t xbox360w_chatpad_keepalive2[] PROGMEM = {0x00, 0x00, 0x0C,
                                                              0x1E};

// Original Xbox
static const uint8_t xboxog_rumble[] PROGMEM = {0x00, 0x06, 0x00,
                                                0x00, 0x00, 0x00};

// The controller feedbacks the currently set leds. The bitmask for these are in chatpad_mod.
// xbox360w_chatpad_led_ctrl is used to turn on/off a chatpad led. Byte 3 is set to chatpad_led_on[x]
// or chatpad_led_off[x] to turn that respective led on or off.
static const uint8_t xbox360w_chatpad_led_ctrl[] PROGMEM = {0x00, 0x00, 0x0C,
                                                            0x00};
static const uint8_t chatpad_mod[] PROGMEM = {
    XINPUT_CHATPAD_CAPSLOCK, XINPUT_CHATPAD_GREEN, XINPUT_CHATPAD_ORANGE,
    XINPUT_CHATPAD_MESSENGER};
static const uint8_t chatpad_led_on[] PROGMEM = {0x08, 0x09, 0x0A, 0x0B};
static const uint8_t chatpad_led_off[] PROGMEM = {0x00, 0x01, 0x02, 0x03};

class XINPUT : public USBDeviceConfig {
   public:
    XINPUT(USB *pUsb);
    uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed,
                 USB_DEVICE_DESCRIPTOR *udd);
    uint8_t Release();
    uint8_t Poll();
    virtual uint8_t GetAddress() { return bAddress; }
    virtual bool isReady() { return bIsReady; }
    uint8_t SetRumble(usbh_xinput_t *xpad, uint8_t lValue, uint8_t rValue);
    uint8_t SetLed(usbh_xinput_t *xpad, uint8_t quadrant);
    uint8_t WritePacket(usbh_xinput_t *xpad, const uint8_t *data, uint8_t len,
                        uint8_t flags);

   protected:
    USB *pUsb;
    uint8_t bAddress;
    EpInfo epInfo[XBOX_MAX_ENDPOINTS];

   private:
    bool bIsReady;
    uint16_t PID, VID;
    uint8_t iProduct, iManuf, iSerial;
    uint8_t dev_num_eps;
    xinput_type_t dev_type;
    usbh_xinput_t *alloc_xinput_device(uint8_t bAddress, uint8_t itf_num,
                                       EpInfo *in, EpInfo *out,
                                       xinput_type_t type);
    uint8_t free_xinput_device(usbh_xinput_t *xinput_dev);
    bool ParseInputData(usbh_xinput_t **xpad, EpInfo *ep_in);
};

#endif
