#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Fakemote structural types assumed from egc wrapper layers */
typedef struct {
    uint16_t core_buttons;
    uint8_t  ext_type;
    uint8_t  nunchuk_stick_x;
    uint8_t  nunchuk_stick_y;
    uint8_t  nunchuk_buttons;
    uint8_t  classic_left_x;
    uint8_t  classic_left_y;
    uint8_t  classic_right_x;
    uint8_t  classic_right_y;
} fakemote_pad_input_matrix_t;

typedef struct {
    int port;
    fakemote_pad_input_matrix_t pad_state;
} egc_device_t;

typedef struct {
    const char *name;
    bool (*probe)(uint16_t vid, uint16_t pid);
    int (*init)(egc_device_t *dev);
    void (*process)(egc_device_t *dev, const uint8_t *packet, uint16_t len);
    void (*close)(egc_device_t *dev);
} egc_device_driver_t;

/* Wii Button Definitions */
#define WPAD_BUTTON_2        0x0001
#define WPAD_BUTTON_1        0x0002
#define WPAD_BUTTON_B        0x0004
#define WPAD_BUTTON_A        0x0008
#define WPAD_BUTTON_PLUS     0x0010
#define WPAD_BUTTON_HOME     0x0080
#define WPAD_BUTTON_MINUS    0x1000
#define WPAD_BUTTON_LEFT     0x0100
#define WPAD_BUTTON_RIGHT    0x0200
#define WPAD_BUTTON_DOWN     0x0400
#define WPAD_BUTTON_UP       0x0800

#define NUNCHUK_BUTTON_Z     0x0001
#define NUNCHUK_BUTTON_C     0x0002

#define XBOX_STICK_MAX         32300
#define XBOX_DEADZONE_RADIUS   4500

typedef struct {
    uint8_t  padding[14];
    uint8_t  buttons_1;
    uint8_t  buttons_2;
    uint8_t  dpad;
    int16_t  left_stick_x;
    int16_t  left_stick_y;
    int16_t  right_stick_x;
    int16_t  right_stick_y;
    uint16_t left_trigger;
    uint16_t right_trigger;
} __attribute__((packed)) XboxReport;

typedef struct {
    float x;
    float y;
} NormalizedVector;

/* Forward declarations */
static bool xbox_probe(uint16_t vid, uint16_t pid);
static int xbox_init(egc_device_t *dev);
static void xbox_process(egc_device_t *dev, const uint8_t *packet, uint16_t len);
static void xbox_close(egc_device_t *dev);

const egc_device_driver_t xbox_usb_device_driver = {
    .name = "Xbox Series S/X",
    .probe = xbox_probe,
    .init = xbox_init,
    .process = xbox_process,
    .close = xbox_close,
};

static bool xbox_probe(uint16_t vid, uint16_t pid) {
    return (vid == 0x045E && pid == 0x0B12);
}

static int xbox_init(egc_device_t *dev) {
    if (!dev) return -1;
    dev->pad_state.ext_type = 1; /* Default to Nunchuk emulation */
    return 0;
}

static void xbox_close(egc_device_t *dev) {
    (void)dev;
}

static NormalizedVector filter_radial_deadzone(int16_t raw_x, int16_t raw_y) {
    NormalizedVector out = {0.0f, 0.0f};
    float fx = (float)raw_x;
    float fy = (float)raw_y;
    float magnitude = sqrtf(fx * fx + fy * fy);

    if (magnitude > XBOX_DEADZONE_RADIUS) {
        if (magnitude > XBOX_STICK_MAX) magnitude = XBOX_STICK_MAX;
        float dir_x = fx / magnitude;
        float dir_y = fy / magnitude;
        float rescaled_mag = (magnitude - XBOX_DEADZONE_RADIUS) / (XBOX_STICK_MAX - XBOX_DEADZONE_RADIUS);
        out.x = dir_x * rescaled_mag;
        out.y = dir_y * rescaled_mag;
    }
    return out;
}

static void xbox_process(egc_device_t *dev, const uint8_t *packet, uint16_t len) {
    if (!dev || !packet || len < sizeof(XboxReport)) return;

    const XboxReport *xbox = (const XboxReport *)packet;
    fakemote_pad_input_matrix_t *wii_out = &dev->pad_state;
    
    wii_out->core_buttons = 0;
    wii_out->nunchuk_buttons = 0;

    if (xbox->buttons_1 & 0x01) wii_out->core_buttons |= WPAD_BUTTON_A;
    if (xbox->buttons_1 & 0x02) wii_out->core_buttons |= WPAD_BUTTON_B;
    if (xbox->buttons_1 & 0x08) wii_out->core_buttons |= WPAD_BUTTON_1;
    if (xbox->buttons_1 & 0x10) wii_out->core_buttons |= WPAD_BUTTON_2;
    if (xbox->buttons_2 & 0x04) wii_out->core_buttons |= WPAD_BUTTON_MINUS;
    if (xbox->buttons_2 & 0x08) wii_out->core_buttons |= WPAD_BUTTON_PLUS;

    switch (xbox->dpad) {
        case 1: wii_out->core_buttons |= WPAD_BUTTON_UP; break;
        case 2: wii_out->core_buttons |= (WPAD_BUTTON_UP | WPAD_BUTTON_RIGHT); break;
        case 3: wii_out->core_buttons |= WPAD_BUTTON_RIGHT; break;
        case 4: wii_out->core_buttons |= (WPAD_BUTTON_DOWN | WPAD_BUTTON_RIGHT); break;
        case 5: wii_out->core_buttons |= WPAD_BUTTON_DOWN; break;
        case 6: wii_out->core_buttons |= (WPAD_BUTTON_DOWN | WPAD_BUTTON_LEFT); break;
        case 7: wii_out->core_buttons |= WPAD_BUTTON_LEFT; break;
        case 8: wii_out->core_buttons |= (WPAD_BUTTON_UP | WPAD_BUTTON_LEFT); break;
        default: break;
    }

    if (wii_out->ext_type == 1) { 
        if (xbox->buttons_1 & 0x40) wii_out->nunchuk_buttons |= NUNCHUK_BUTTON_C;
        if (xbox->buttons_1 & 0x80) wii_out->nunchuk_buttons |= NUNCHUK_BUTTON_Z;

        NormalizedVector stick = filter_radial_deadzone(xbox->left_stick_x, xbox->left_stick_y);
        int32_t nx = (int32_t)((stick.x * 127.0f) + 128.0f);
        int32_t ny = (int32_t)((-stick.y * 127.0f) + 128.0f);

        wii_out->nunchuk_stick_x = (uint8_t)(nx > 255 ? 255 : (nx < 0 ? 0 : nx));
        wii_out->nunchuk_stick_y = (uint8_t)(ny > 255 ? 255 : (ny < 0 ? 0 : ny));
    } else if (wii_out->ext_type == 2) {
        NormalizedVector l_stick = filter_radial_deadzone(xbox->left_stick_x, xbox->left_stick_y);
        NormalizedVector r_stick = filter_radial_deadzone(xbox->right_stick_x, xbox->right_stick_y);

        int32_t clx = (int32_t)((l_stick.x * 31.0f) + 32.0f);
        int32_t cly = (int32_t)((-l_stick.y * 31.0f) + 32.0f);
        wii_out->classic_left_x = (uint8_t)(clx > 63 ? 63 : (clx < 0 ? 0 : clx));
        wii_out->classic_left_y = (uint8_t)(cly > 63 ? 63 : (cly < 0 ? 0 : cly));

        int32_t crx = (int32_t)((r_stick.x * 15.0f) + 16.0f);
        int32_t cry = (int32_t)((-r_stick.y * 15.0f) + 16.0f);
        wii_out->classic_right_x = (uint8_t)(crx > 31 ? 31 : (crx < 0 ? 0 : crx));
        wii_out->classic_right_y = (uint8_t)(cry > 31 ? 31 : (cry < 0 ? 0 : cry));
    }
}
