#include <stdint.h>
#include <string.h>
#include <math.h>

/* Wii Core Bitmask Definitions (WPAD definitions) */
#define WPAD_BUTTON_2        0x0001
#define WPAD_BUTTON_1        0x0002
#define WPAD_BUTTON_B        0x0004
#define WPAD_BUTTON_A        0x0008
#define WPAD_BUTTON_PLUS     0x0010
#define WPAD_BUTTON_HOME     0x0080
#define WPAD_BUTTON_MINUS    0x1000

/* Wii D-Pad Definitions */
#define WPAD_BUTTON_LEFT     0x0100
#define WPAD_BUTTON_RIGHT    0x0200
#define WPAD_BUTTON_DOWN     0x0400
#define WPAD_BUTTON_UP       0x0800

/* Wii Virtual Extension Bitmasks */
#define NUNCHUK_BUTTON_Z     0x0001
#define NUNCHUK_BUTTON_C     0x0002

#define XBOX_STICK_MAX         32300 // Calibrated threshold slightly lower than absolute max for safety
#define XBOX_DEADZONE_RADIUS   4500  // Optimal radius threshold to combat physical Xbox thumbstick drift

/* Incoming Native Byte Structure of an Xbox Series S Controller */
typedef struct {
    uint8_t  padding[14];   // Skip header/axes offsets for initial button parses
    uint8_t  buttons_1;     // Byte 14: A, B, X, Y, LB, RB
    uint8_t  buttons_2;     // Byte 15: View, Menu, L3, R3
    uint8_t  dpad;          // Byte 16: D-pad rotational vector state
    int16_t  left_stick_x;  // Bytes 17-18
    int16_t  left_stick_y;  // Bytes 19-20
    int16_t  right_stick_x; // Bytes 21-22
    int16_t  right_stick_y; // Bytes 23-24
    uint16_t left_trigger;  // Bytes 25-26
    uint16_t right_trigger; // Bytes 27-28
} __attribute__((packed)) XboxReport;

/* Structural targets defined within Fakemote output states */
typedef struct {
    uint16_t core_buttons;
    uint8_t  ext_type;       // 0 = None, 1 = Nunchuk, 2 = Classic Controller
    uint8_t  nunchuk_stick_x;
    uint8_t  nunchuk_stick_y;
    uint8_t  nunchuk_buttons;
    uint8_t  classic_left_x;
    uint8_t  classic_left_y;
    uint8_t  classic_right_x;
    uint8_t  classic_right_y;
} FakemoteOutputState;

typedef struct {
    float x;
    float y;
} NormalizedVector;

/* Radial Filter to compensate for stick drift */
static NormalizedVector filter_radial_deadzone(int16_t raw_x, int16_t raw_y) {
    NormalizedVector out = {0.0f, 0.0f};
    float fx = (float)raw_x;
    float fy = (float)raw_y;
    float magnitude = sqrtf(fx * fx + fy * fy);

    if (magnitude > XBOX_DEADZONE_RADIUS) {
        if (magnitude > XBOX_STICK_MAX) magnitude = XBOX_STICK_MAX;
        
        // Compute precise angle direction vectors
        float dir_x = fx / magnitude;
        float dir_y = fy / magnitude;
        
        // Re-scale vector accurately across active boundary
        float rescaled_mag = (magnitude - XBOX_DEADZONE_RADIUS) / (XBOX_STICK_MAX - XBOX_DEADZONE_RADIUS);
        out.x = dir_x * rescaled_mag;
        out.y = dir_y * rescaled_mag;
    }
    return out;
}

/**
 * Fakemote Main Injection Routine
 * Evaluates raw incoming Xbox payload bytes and translates them 
 * directly onto the active structural virtual Wiimote framework.
 */
void fakemote_process_xbox_input(const uint8_t *usb_packet_buffer, FakemoteOutputState *wii_out) {
    if (!usb_packet_buffer || !wii_out) return;

    const XboxReport *xbox = (const XboxReport *)usb_packet_buffer;
    
    // Clear old button registers
    wii_out->core_buttons = 0;
    wii_out->nunchuk_buttons = 0;

    // --- Core Wiimote Face Buttons Mappings ---
    if (xbox->buttons_1 & 0x01) wii_out->core_buttons |= WPAD_BUTTON_A;     // Xbox A -> Wii A
    if (xbox->buttons_1 & 0x02) wii_out->core_buttons |= WPAD_BUTTON_B;     // Xbox B -> Wii B
    if (xbox->buttons_1 & 0x08) wii_out->core_buttons |= WPAD_BUTTON_1;     // Xbox X -> Wii 1
    if (xbox->buttons_1 & 0x10) wii_out->core_buttons |= WPAD_BUTTON_2;     // Xbox Y -> Wii 2
    
    // --- Center Navigation Mappings ---
    if (xbox->buttons_2 & 0x04) wii_out->core_buttons |= WPAD_BUTTON_MINUS; // Xbox View -> Wii Minus
    if (xbox->buttons_2 & 0x08) wii_out->core_buttons |= WPAD_BUTTON_PLUS;  // Xbox Menu -> Wii Plus

    // --- Rotational D-Pad Processing ---
    switch (xbox->dpad) {
        case 1: wii_out->core_buttons |= WPAD_BUTTON_UP; break;
        case 2: wii_out->core_buttons |= (WPAD_BUTTON_UP | WPAD_BUTTON_RIGHT); break;
        case 3: wii_out->core_buttons |= WPAD_BUTTON_RIGHT; break;
        case 4: wii_out->core_buttons |= (WPAD_BUTTON_DOWN | WPAD_BUTTON_RIGHT); break;
        case 5: wii_out->core_buttons |= WPAD_BUTTON_DOWN; break;
        case 6: wii_out->core_buttons |= (WPAD_BUTTON_DOWN | WPAD_BUTTON_LEFT); break;
        case 7: wii_out->core_buttons |= WPAD_BUTTON_LEFT; break;
        case 8: wii_out->core_buttons |= (WPAD_BUTTON_UP | WPAD_BUTTON_LEFT); break;
        default: break; // Centered
    }

    // --- Dynamic Extension Logic Handling ---
    if (wii_out->ext_type == 1) { 
        // Mode A: Nunchuk Mode Emulation
        if (xbox->buttons_1 & 0x40) wii_out->nunchuk_buttons |= NUNCHUK_BUTTON_C; // Xbox LB -> Nunchuk C
        if (xbox->buttons_1 & 0x80) wii_out->nunchuk_buttons |= NUNCHUK_BUTTON_Z; // Xbox RB -> Nunchuk Z

        // Apply Deadzone Filter for Left Stick
        NormalizedVector stick = filter_radial_deadzone(xbox->left_stick_x, xbox->left_stick_y);
        
        // Transform floating vector points down into standard 0-255 Wii constraints
        // Note: The Wii environment registers inverted standard Y axis logic natively
        int32_t nx = (int32_t)((stick.x * 127.0f) + 128.0f);
        int32_t ny = (int32_t)((-stick.y * 127.0f) + 128.0f);

        wii_out->nunchuk_stick_x = (uint8_t)(nx > 255 ? 255 : (nx < 0 ? 0 : nx));
        wii_out->nunchuk_stick_y = (uint8_t)(ny > 255 ? 255 : (ny < 0 ? 0 : ny));

    } else if (wii_out->ext_type == 2) {
        // Mode B: Classic Controller Mode Emulation
        NormalizedVector l_stick = filter_radial_deadzone(xbox->left_stick_x, xbox->left_stick_y);
        NormalizedVector r_stick = filter_radial_deadzone(xbox->right_stick_x, xbox->right_stick_y);

        // Map Left stick to 6-bit constraints (0..63)
        int32_t clx = (int32_t)((l_stick.x * 31.0f) + 32.0f);
        int32_t cly = (int32_t)((-l_stick.y * 31.0f) + 32.0f);
        wii_out->classic_left_x = (uint8_t)(clx > 63 ? 63 : (clx < 0 ? 0 : clx));
        wii_out->classic_left_y = (uint8_t)(cly > 63 ? 63 : (cly < 0 ? 0 : cly));

        // Map Right stick to 5-bit constraints (0..31)
        int32_t crx = (int32_t)((r_stick.x * 15.0f) + 16.0f);
        int32_t cry = (int32_t)((-r_stick.y * 15.0f) + 16.0f);
        wii_out->classic_right_x = (uint8_t)(crx > 31 ? 31 : (crx < 0 ? 0 : crx));
        wii_out->classic_right_y = (uint8_t)(cry > 31 ? 31 : (cry < 0 ? 0 : cry));
    }
}
