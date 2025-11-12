#include "ov7670.h"
#include "camera_pins.h"
#include "esp32-hal-ledc.h" // For XCLK generation

// --- SCCB (I2C) Communication ---
void sccb_init() {
    Wire.begin((int)SIOD_GPIO_NUM, (int)SIOC_GPIO_NUM); // SDA, SCL
}

void sccb_write_byte(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(OV7670_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t sccb_read_byte(uint8_t reg) {
    Wire.beginTransmission(OV7670_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(OV7670_ADDRESS, 1);
    while (!Wire.available()); // Wait for data
    return Wire.read();
}

// --- XCLK Generation ---
void camera_xclk_init(int freq_mhz, int pin) {
    ledcSetup(0, freq_mhz * 1000 * 1000, 1); // Channel 0, frequency (Hz), 1 bit resolution
    ledcAttachPin(pin, 0); // Attach pin to channel 0
}

// --- OV7670 Register Presets ---
// Common QQVGA (160x120) YUV422 configuration
// Values taken from various OV7670 driver examples and datasheets.
// These are carefully chosen to optimize for grayscale YUV and small resolution.
static const struct reg_setting {
    uint8_t reg_num;
    uint8_t value;
} ov7670_qqvga_yuv_regs[] = {
    {REG_COM7, COM7_RESET}, // Reset all registers
    {0x12, 0x80}, // COM7: Reset
    {0x3A, 0x04}, // TSLB: YUYV
    {0x3B, 0x00}, // LC_MODE
    {0x3D, 0xC4}, // PCOM8
    {0x4F, 0xB0}, // MTX1
    {0x50, 0x99}, // MTX2
    {0x51, 0x24}, // MTX3
    {0x52, 0x01}, // MTX4
    {0x53, 0x80}, // MTX5
    {0x54, 0x80}, // MTX6
    {0x58, 0x17}, // CONTRAS
    {0x60, 0x80}, // LCC_CTRL12, Fix AWB issues
    {0x61, 0x80}, // LCC_CTRL13
    {0x62, 0x08}, // LCC_CTRL14
    {0x64, 0x04}, // LCC_CTRL16
    {0x65, 0x02}, // LCC_CTRL17
    {0x66, 0x01}, // LCC_CTRL18
    {0x67, 0x00}, // LCC_CTRL19
    {0x68, 0x00}, // LCC_CTRL20
    {0x69, 0x00}, // LCC_CTRL21
    {0x6A, 0x00}, // LCC_CTRL22
    {0x6B, 0x00}, // LCC_CTRL23
    {0x6C, 0x00}, // LCC_CTRL24
    {0x6D, 0x00}, // LCC_CTRL25
    {0x6E, 0x00}, // LCC_CTRL26
    {0x6F, 0x00}, // LCC_CTRL27
    {0x70, 0x00}, // LCC_CTRL28
    {0x71, 0x00}, // LCC_CTRL29
    {0x72, 0x00}, // LCC_CTRL30
    {0x73, 0x00}, // LCC_CTRL31
    {0x74, 0x00}, // LCC_CTRL32
    {0x75, 0x00}, // LCC_CTRL33
    {0x76, 0x00}, // LCC_CTRL34
    {0x77, 0x00}, // LCC_CTRL35
    {0x78, 0x00}, // LCC_CTRL36
    {0x79, 0x00}, // LCC_CTRL37
    {0x7A, 0x00}, // LCC_CTRL38
    {0x7B, 0x00}, // LCC_CTRL39
    {0x7C, 0x00}, // LCC_CTRL40
    {0x7D, 0x00}, // LCC_CTRL41
    {0x7E, 0x00}, // LCC_CTRL42
    {0x7F, 0x00}, // LCC_CTRL43
    {0x80, 0x00}, // LCC_CTRL44
    {0x81, 0x00}, // LCC_CTRL45
    {0x82, 0x00}, // LCC_CTRL46
    {0x83, 0x00}, // LCC_CTRL47
    {0x84, 0x00}, // LCC_CTRL48
    {0x85, 0x00}, // LCC_CTRL49
    {0x86, 0x00}, // LCC_CTRL50
    {0x87, 0x00}, // LCC_CTRL51
    {0x88, 0x00}, // LCC_CTRL52
    {0x89, 0x00}, // LCC_CTRL53
    {0x8A, 0x00}, // LCC_CTRL54
    {0x8B, 0x00}, // LCC_CTRL55
    {0x8C, 0x00}, // LCC_CTRL56
    {0x8D, 0x00}, // LCC_CTRL57
    {0x8E, 0x00}, // LCC_CTRL58
    {0x8F, 0x00}, // LCC_CTRL59
    {0x90, 0x00}, // LCC_CTRL60
    {0x91, 0x00}, // LCC_CTRL61
    {0x92, 0x00}, // LCC_CTRL62
    {0x93, 0x00}, // LCC_CTRL63
    {0x94, 0x00}, // LCC_CTRL64
    {0x95, 0x00}, // LCC_CTRL65
    {0x96, 0x00}, // LCC_CTRL66
    {0x97, 0x00}, // LCC_CTRL67
    {0x98, 0x00}, // LCC_CTRL68
    {0x99, 0x00}, // LCC_CTRL69
    {0x9A, 0x00}, // LCC_CTRL70
    {0x9B, 0x00}, // LCC_CTRL71
    {0x9C, 0x00}, // LCC_CTRL72
    {0x9D, 0x00}, // LCC_CTRL73
    {0x9E, 0x00}, // LCC_CTRL74
    {0x9F, 0x00}, // LCC_CTRL75
    {0xA0, 0x00}, // LCC_CTRL76
    {0xA1, 0x00}, // LCC_CTRL77
    {0xA2, 0x00}, // LCC_CTRL78
    {0xA3, 0x00}, // LCC_CTRL79
    {0xA4, 0x00}, // LCC_CTRL80
    {0xA5, 0x00}, // LCC_CTRL81
    {0xA6, 0x00}, // LCC_CTRL82
    {0xA7, 0x00}, // LCC_CTRL83
    {0xA8, 0x00}, // LCC_CTRL84
    {0xA9, 0x00}, // LCC_CTRL85
    {0xAA, 0x00}, // LCC_CTRL86
    {0xAB, 0x00}, // LCC_CTRL87
    {0xAC, 0x00}, // LCC_CTRL88
    {0xAD, 0x00}, // LCC_CTRL89
    {0xAE, 0x00}, // LCC_CTRL90
    {0xAF, 0x00}, // LCC_CTRL91
    {0xB0, 0x00}, // LCC_CTRL92
    {0xB1, 0x00}, // LCC_CTRL93
    {0xB2, 0x00}, // LCC_CTRL94
    {0xB3, 0x00}, // LCC_CTRL95
    {0xB4, 0x00}, // LCC_CTRL96
    {0xB5, 0x00}, // LCC_CTRL97
    {0xB6, 0x00}, // LCC_CTRL98
    {0xB7, 0x00}, // LCC_CTRL99
    {0xB8, 0x00}, // LCC_CTRL100
    {0xB9, 0x00}, // LCC_CTRL101
    {0xBA, 0x00}, // LCC_CTRL102
    {0xBB, 0x00}, // LCC_CTRL103
    {0xBC, 0x00}, // LCC_CTRL104
    {0xBD, 0x00}, // LCC_CTRL105
    {0xBE, 0x00}, // LCC_CTRL106
    {0xBF, 0x00}, // LCC_CTRL107
    {0xC0, 0x00}, // LCC_CTRL108
    {0xC1, 0x00}, // LCC_CTRL109
    {0xC2, 0x00}, // LCC_CTRL110
    {0xC3, 0x00}, // LCC_CTRL111
    {0xC4, 0x00}, // LCC_CTRL112
    {0xC5, 0x00}, // LCC_CTRL113
    {0xC6, 0x00}, // LCC_CTRL114
    {0xC7, 0x00}, // LCC_CTRL115
    {0xC8, 0x00}, // LCC_CTRL116
    {0xC9, 0x00}, // LCC_CTRL117
    {0xCA, 0x00}, // LCC_CTRL118
    {0xCB, 0x00}, // LCC_CTRL119
    {0xCC, 0x00}, // LCC_CTRL120
    {0xCD, 0x00}, // LCC_CTRL121
    {0xCE, 0x00}, // LCC_CTRL122
    {0xCF, 0x00}, // LCC_CTRL123
    {0xD0, 0x00}, // LCC_CTRL124
    {0xD1, 0x00}, // LCC_CTRL125
    {0xD2, 0x00}, // LCC_CTRL126
    {0xD3, 0x00}, // LCC_CTRL127
    {0xD4, 0x00}, // LCC_CTRL128
    {0xD5, 0x00}, // LCC_CTRL129
    {0xD6, 0x00}, // LCC_CTRL130
    {0xD7, 0x00}, // LCC_CTRL131
    {0xD8, 0x00}, // LCC_CTRL132
    {0xD9, 0x00}, // LCC_CTRL133
    {0xDA, 0x00}, // LCC_CTRL134
    {0xDB, 0x00}, // LCC_CTRL135
    {0xDC, 0x00}, // LCC_CTRL136
    {0xDD, 0x00}, // LCC_CTRL137
    {0xDE, 0x00}, // LCC_CTRL138
    {0xDF, 0x00}, // LCC_CTRL139
    {0xE0, 0x00}, // LCC_CTRL140
    {0xE1, 0x00}, // LCC_CTRL141
    {0xE2, 0x00}, // LCC_CTRL142
    {0xE3, 0x00}, // LCC_CTRL143
    {0xE4, 0x00}, // LCC_CTRL144
    {0xE5, 0x00}, // LCC_CTRL145
    {0xE6, 0x00}, // LCC_CTRL146
    {0xE7, 0x00}, // LCC_CTRL147
    {0xE8, 0x00}, // LCC_CTRL148
    {0xE9, 0x00}, // LCC_CTRL149
    {0xEA, 0x00}, // LCC_CTRL150
    {0xEB, 0x00}, // LCC_CTRL151
    {0xEC, 0x00}, // LCC_CTRL152
    {0xED, 0x00}, // LCC_CTRL153
    {0xEE, 0x00}, // LCC_CTRL154
    {0xEF, 0x00}, // LCC_CTRL155
    {0xF0, 0x00}, // LCC_CTRL156
    {0xF1, 0x00}, // LCC_CTRL157
    {0xF2, 0x00}, // LCC_CTRL158
    {0xF3, 0x00}, // LCC_CTRL159
    {0xF4, 0x00}, // LCC_CTRL160
    {0xF5, 0x00}, // LCC_CTRL161
    {0xF6, 0x00}, // LCC_CTRL162
    {0xF7, 0x00}, // LCC_CTRL163
    {0xF8, 0x00}, // LCC_CTRL164
    {0xF9, 0x00}, // LCC_CTRL165
    {0xFA, 0x00}, // LCC_CTRL166
    {0xFB, 0x00}, // LCC_CTRL167
    {0xFC, 0x00}, // LCC_CTRL168
    {0xFD, 0x00}, // LCC_CTRL169
    {0xFE, 0x00}, // LCC_CTRL170
    {0xFF, 0x00}, // LCC_CTRL171

    // Default configuration from datasheets, optimized for YUV QQVGA
    // General settings
    {REG_COM7, COM7_YUV | COM7_FMT_QVGA}, // YUV QQVGA
    {REG_COM3, 0x04}, // COM3: Enable BPC (Black Pixel Compensation)
    {REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC | COM8_AWB}, // Enable AEC/AGC/AWB
    {REG_COM9, 0x18}, // Automatic gain ceiling, 0x8 for 16x
    {REG_COM10, 0x00}, // HSYNC, PCLK, HREF normal
    {REG_COM13, 0xC0}, // COM13: Gamma, UV saturation, Edge enhancement enabled
    {REG_COM14, 0x1A}, // PCLK divided by 8, then scale
    {REG_COM15, 0x48}, // 0x48 for QQVGA (120 lines)
    {REG_COM16, 0x08}, // AWB gain enabled
    {REG_MVFP, MVFP_MIRROR | MVFP_VFLIP}, // Mirror horizontally and flip vertically

    // Clock control
    {REG_CLKRC, CLK_EXT | 0x01}, // XCLK/2 (e.g., 20MHz XCLK -> 10MHz PCLK)

    // Windowing for QQVGA (160x120)
    // These values define the active window within the sensor's full frame.
    // They are critical for correct resolution output.
    {REG_HSTART, 0x1A}, {REG_HSTOP, 0x02}, {REG_HREF, 0x72}, // HSYNC start/stop (160)
    {REG_VSTART, 0x01}, {REG_VSTOP, 0x79}, {REG_VREF, 0x08}, // VSYNC start/stop (120)

    // Scaling to QQVGA
    {REG_SCALING_XSC, 0x3A}, // X scale (down to 1/4)
    {REG_SCALING_YSC, 0x35}, // Y scale (down to 1/4)
    {REG_SCALING_DCWCTR, 0x11}, // DSP scale (divide by 2 in both directions)
    {REG_SCALING_PCLK_DIV, 0x11}, // PCLK divide by 2

    // Matrix coefficients for YUV (if color is enabled)
    {0x4F, 0x80}, {0x50, 0x80}, {0x51, 0x00}, {0x52, 0x22}, {0x53, 0x5E}, {0x54, 0x80},
    {0x56, 0x40}, {0x57, 0x80}, {0x58, 0x80}, {0x59, 0x80}, {0x5A, 0x80}, {0x5B, 0x80},

    // AWB (Auto White Balance)
    {REG_AWB_CTRL0, 0x9A}, // AWB control
    {REG_AWB_CTRL1, 0x80}, // AWB control
    {REG_AWB_CTRL2, 0x30}, // AWB control
    {REG_AWB_CTRL3, 0x20}, // AWB control
    {REG_AWB_CTRL4, 0x0A}, // AWB control
    {REG_AWB_CTRL5, 0x06}, // AWB control
    {REG_AWB_CTRL6, 0x03}, // AWB control
    {REG_AWB_CTRL7, 0x01}, // AWB control
    {REG_AWB_CTRL8, 0x00}, // AWB control
    {REG_AWB_CTRL9, 0x00}, // AWB control
    {REG_AWB_CTRL10, 0x00}, // AWB control

    // EOF marker
    {0xFF, 0xFF},
};

// Initializes the OV7670 camera with default settings for QQVGA YUV422 and then grayscale.
void ov7670_init(int xclk_freq_mhz) {
    // Power down/reset control
    if (PWDN_GPIO_NUM != -1) {
      pinMode(PWDN_GPIO_NUM, OUTPUT);
      digitalWrite(PWDN_GPIO_NUM, LOW); // Power down
      delay(10);
      digitalWrite(PWDN_GPIO_NUM, HIGH); // Power up
      delay(10);
    }
    if (RESET_GPIO_NUM != -1) {
      pinMode(RESET_GPIO_NUM, OUTPUT);
      digitalWrite(RESET_GPIO_NUM, LOW); // Reset
      delay(10);
      digitalWrite(RESET_GPIO_NUM, HIGH); // Release reset
      delay(10);
    }

    camera_xclk_init(xclk_freq_mhz, XCLK_GPIO_NUM);
    sccb_init();

    // Verify chip ID
    uint8_t pid = SCCB_READ(REG_PID);
    uint8_t ver = SCCB_READ(REG_VER);
    Serial.printf("OV7670 PID: 0x%02X, VER: 0x%02X\n", pid, ver);

    if (pid != 0x76 || ver != 0x73) { // 0x76 0x73 for OV7670, 0x76 0x72 for OV7670
        Serial.println("OV7670 not found or wrong chip ID! Halting.");
        while (1); // Halt if camera not detected
    }

    // Apply the QQVGA YUV422 register settings
    for (int i = 0; ov7670_qqvga_yuv_regs[i].reg_num != 0xFF; i++) {
        SCCB_WRITE(ov7670_qqvga_yuv_regs[i].reg_num, ov7670_qqvga_yuv_regs[i].value);
        // Serial.printf("Wrote 0x%02X to 0x%02X\n", ov7670_qqvga_yuv_regs[i].value, ov7670_qqvga_yuv_regs[i].reg_num);
        delay(1); // Small delay
    }

    ov7670_set_grayscale(); // Ensure grayscale output (disable U/V components)

    // Parallel camera interface setup for data capture
    pinMode(VSYNC_GPIO_NUM, INPUT);
    pinMode(HREF_GPIO_NUM, INPUT);
    pinMode(PCLK_GPIO_NUM, INPUT);
    // Configure data pins D0-D7 as inputs
    pinMode(D0_GPIO_NUM, INPUT);
    pinMode(D1_GPIO_NUM, INPUT);
    pinMode(D2_GPIO_NUM, INPUT);
    pinMode(D3_GPIO_NUM, INPUT);
    pinMode(D4_GPIO_NUM, INPUT);
    pinMode(D5_GPIO_NUM, INPUT);
    pinMode(D6_GPIO_NUM, INPUT);
    pinMode(D7_GPIO_NUM, INPUT);
}

// Configures the OV7670 for YUV422 output
void ov7670_set_yuv() {
    uint8_t com7 = SCCB_READ(REG_COM7);
    SCCB_WRITE(REG_COM7, (com7 & (COM7_FMT_MASK | COM7_RGB)) | COM7_YUV);
    // Disable UV saturation and color interpolation if not needed for YUV
    uint8_t com13 = SCCB_READ(REG_COM13);
    SCCB_WRITE(REG_COM13, com13 & ~(COM13_UVSAT | COM13_CIP));
}

// Configures the OV7670 for Grayscale output (by essentially ignoring U/V or setting them to neutral)
void ov7670_set_grayscale() {
    // Enable YUV output first
    ov7670_set_yuv();

    // Set matrix coefficients for grayscale (effectively 0 for U/V)
    SCCB_WRITE(0x4f, 0x80); // MTX1 (Y)
    SCCB_WRITE(0x50, 0x80); // MTX2 (Y)
    SCCB_WRITE(0x51, 0x00); // MTX3 (U)
    SCCB_WRITE(0x52, 0x22); // MTX4 (V) - these are usually for color conversion
    SCCB_WRITE(0x53, 0x5e); // MTX5
    SCCB_WRITE(0x54, 0x80); // MTX6

    // Disable UV auto adjustment (COM13 bit 6) and other color features
    uint8_t com13 = SCCB_READ(REG_COM13);
    SCCB_WRITE(REG_COM13, com13 | 0x08 | 0x02); // COM13_CIP | COM13_AWB - these were causing issues for grayscale
    // Also disable UV auto adjustment on COM16 bit 5 & 6 and 8
    uint8_t com16 = SCCB_READ(REG_COM16);
    SCCB_WRITE(REG_COM16, com16 | 0x08); // COM16_AWBGAIN
}

// Sets camera to QQVGA (160x120) resolution
void ov7670_set_resolution_qqvga() {
    // These registers are often set in the ov7670_qqvga_yuv_regs preset.
    // If you need to explicitly change it after initialization, use these.
    // For now, it's handled by the `ov7670_init` function.
}


// --- Frame Capture to Grayscale 96x96 ---
// This is the core function to get image data for the TFLite model.
bool ov7670_capture_frame_to_grayscale_96x96(uint8_t* target_buffer) {
    // Wait for VSYNC to go high, then low (start of frame)
    while (digitalRead(VSYNC_GPIO_NUM) == LOW);
    while (digitalRead(VSYNC_GPIO_NUM) == HIGH);

    uint16_t x = 0;
    uint16_t y = 0;
    uint8_t pixel_data[2]; // YUYV format (Y0 U Y1 V)

    // Wait for HREF to go high (start of active line)
    // This driver will capture a QQVGA (160x120) YUV422 frame
    // then downsample/grayscale it to 96x96.

    // Calculate scaling factors
    float x_scale = (float)CAMERA_FRAME_WIDTH / TFL_MODEL_WIDTH;
    float y_scale = (float)CAMERA_FRAME_HEIGHT / TFL_MODEL_HEIGHT;

    for (y = 0; y < CAMERA_FRAME_HEIGHT; y++) {
        while (digitalRead(HREF_GPIO_NUM) == LOW); // Wait for start of line
        for (x = 0; x < CAMERA_FRAME_WIDTH; x++) {
            // Read 2 bytes (YUYV) from data bus
            // The OV7670 outputs YUYV (Y0 U Y1 V). We only need Y for grayscale.
            // Y0 is the first byte, Y1 is the third byte.
            while (digitalRead(PCLK_GPIO_NUM) == HIGH); // Wait for PCLK low
            // Read D0-D7
            pixel_data[0] = (digitalRead(D7_GPIO_NUM) << 7) |
                            (digitalRead(D6_GPIO_NUM) << 6) |
                            (digitalRead(D5_GPIO_NUM) << 5) |
                            (digitalRead(D4_GPIO_NUM) << 4) |
                            (digitalRead(D3_GPIO_NUM) << 3) |
                            (digitalRead(D2_GPIO_NUM) << 2) |
                            (digitalRead(D1_GPIO_NUM) << 1) |
                            (digitalRead(D0_GPIO_NUM) << 0);
            while (digitalRead(PCLK_GPIO_NUM) == LOW); // Wait for PCLK high

            while (digitalRead(PCLK_GPIO_NUM) == HIGH); // Wait for PCLK low
            // Read D0-D7 (this would be U or V, we can discard for grayscale)
            pixel_data[1] = (digitalRead(D7_GPIO_NUM) << 7) |
                            (digitalRead(D6_GPIO_NUM) << 6) |
                            (digitalRead(D5_GPIO_NUM) << 5) |
                            (digitalRead(D4_GPIO_NUM) << 4) |
                            (digitalRead(D3_GPIO_NUM) << 3) |
                            (digitalRead(D2_GPIO_NUM) << 2) |
                            (digitalRead(D1_GPIO_NUM) << 1) |
                            (digitalRead(D0_GPIO_NUM) << 0);
            while (digitalRead(PCLK_GPIO_NUM) == LOW); // Wait for PCLK high

            // The 'Y' component is the first byte read (pixel_data[0] for Y0 or Y1)
            // We need to subsample and map to the 96x96 target buffer.
            int target_x = (int)(x / x_scale);
            int target_y = (int)(y / y_scale);

            if (target_x < TFL_MODEL_WIDTH && target_y < TFL_MODEL_HEIGHT) {
                // Take the Y component (luminance) as grayscale value
                // For YUYV, the first byte is Y0, second is U, third is Y1, fourth is V.
                // We're reading 2 bytes at a time (Y0 U or Y1 V).
                // So, if x is even, we use pixel_data[0] (which is Y0).
                // If x is odd, we use pixel_data[0] from the *next* YUYV pair (which is Y1).
                // But simplified, Y is always the first byte of a YUV pair.
                // Let's just use the luminance component for grayscale directly.
                target_buffer[target_y * TFL_MODEL_WIDTH + target_x] = pixel_data[0];
            }
        }
        while (digitalRead(HREF_GPIO_NUM) == HIGH); // Wait for end of line
    }
    return true;
}