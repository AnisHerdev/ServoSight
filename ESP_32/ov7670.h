#ifndef OV7670_H
#define OV7670_H

#include <Arduino.h>
#include <Wire.h> // For I2C (SCCB) communication

// OV7670 I2C Address
#define OV7670_ADDRESS 0x21

// Frame Buffer / Target Resolution
// We will capture at QQVGA (160x120) from the camera
#define CAMERA_FRAME_WIDTH  160
#define CAMERA_FRAME_HEIGHT 120

// Target resolution for TFLite model (96x96 grayscale)
#define TFL_MODEL_WIDTH     96
#define TFL_MODEL_HEIGHT    96

// --- SCCB Register Read/Write Macros ---
#define SCCB_READ(reg) sccb_read_byte(reg)
#define SCCB_WRITE(reg, data) sccb_write_byte(reg, data)

// --- OV7670 Register Definitions (Selected common registers) ---
#define REG_GAIN        0x00    /* Gain control register */
#define REG_BLUE        0x01    /* Blue gain control */
#define REG_RED         0x02    /* Red gain control */
#define REG_VREF        0x03    /* VREF Control */
#define REG_COM1        0x04    /* Common control 1 */
#define COM1_CCIR656    0x40    /* CCIR656 enable */
#define REG_COM2        0x09    /* Common control 2 */
#define COM2_SSLEEP     0x10    /* Soft sleep mode */
#define REG_PID         0x0A    /* Product ID MSB */
#define REG_VER         0x0B    /* Product ID LSB */
#define REG_COM3        0x0C    /* Common control 3 */
#define COM3_SWAP       0x01    /* Bit swap */
#define COM3_BPC        0x04    /* Black pixel compensation */
#define COM3_WCE        0x08    /* White pixel compensation */
#define COM3_HRF        0x80    /* HREF reverse */
#define REG_COM4        0x0D    /* Common control 4 */
#define REG_COM5        0x0E    /* Common control 5 */
#define REG_COM6        0x0F    /* Common control 6 */
#define REG_AECH        0x10    /* Exposure value */
#define REG_CLKRC       0x11    /* Clock control */
#define CLK_EXT         0x40    /* External clock */
#define CLK_SCALE       0x3F    /* RS output clock prescaler (low 6 bits) */
#define REG_COM7        0x12    /* Common control 7 (SCCB configuration, color, etc.) */
#define COM7_RESET      0x80    /* Register reset */
#define COM7_FMT_MASK   0x38
#define COM7_FMT_VGA    0x00
#define COM7_FMT_CIF    0x20
#define COM7_FMT_QVGA   0x10
#define COM7_FMT_QCIF   0x08
#define COM7_RGB        0x04    /* RGB output */
#define COM7_YUV        0x00    /* YUV output (default) */
#define COM7_BAYER      0x01    /* Bayer output */
#define COM7_PBAYER     0x05    /* Processed Bayer */
#define REG_COM8        0x13    /* Common control 8 */
#define COM8_FASTAEC    0x80    /* Enable fast AGC/AEC */
#define COM8_AECSTEP    0x40    /* AEC step size */
#define COM8_BFILT      0x20    /* Band filter enable */
#define COM8_AGC        0x04    /* Auto gain control */
#define COM8_AEC        0x02    /* Auto exposure control */
#define COM8_AWB        0x01    /* Auto white balance */
#define REG_COM9        0x14    /* Common control 9 - AGC/AEC gain */
#define REG_COM10       0x15    /* Common control 10 */
#define COM10_HSYNC     0x40    /* HSYNC reverse */
#define COM10_PCLK_HB   0x20    /* PCLK no toggle in HBLANK */
#define COM10_HREF_HB   0x08    /* HBLANK pulls HREF high */
#define REG_HSTART      0x17    /* Horizontal window start */
#define REG_HSTOP       0x18    /* Horizontal window stop */
#define REG_VSTART      0x19    /* Vertical window start */
#define REG_VSTOP       0x1A    /* Vertical window stop */
#define REG_PSHFT       0x1B    /* Pixel delay */
#define REG_MIDH        0x1C    /* Manufacturer ID high */
#define REG_MIDL        0x1D    /* Manufacturer ID low */
#define REG_MVFP        0x30    /* Mirror and VFLIP */
#define MVFP_MIRROR     0x20
#define MVFP_VFLIP      0x10
#define REG_AEW         0x24    /* AGC/AEC window */
#define REG_AEB         0x25    /* AGC/AEC bound */
#define REG_VPT         0x26    /* AGC/AEC banding start */
#define REG_HV          0x32    /* GSC parameter */
#define REG_COM13       0x3E    /* Common control 13 */
#define COM13_GAMMA     0x80    /* Gamma enable */
#define COM13_UVSAT     0x40    /* UV saturation enable */
#define COM13_EXFILT    0x10    /* Edge enhancement filter enable */
#define COM13_CIP       0x08    /* Color interpolation enable */
#define COM13_AEW       0x04    /* AGC/AEC window */
#define COM13_AWB       0x02    /* Auto white balance enable */
#define REG_COM14       0x3F    /* Common control 14 */
#define COM14_DCWEN     0x08    /* Data clock output enable */
#define REG_COM16       0x41    /* Common control 16 */
#define COM16_AWBGAIN   0x08    /* AWB gain enable */
#define REG_SLOP        0x48    /* Gamma curve highest segment slope */
#define REG_GAMMA       0x4A    /* Gamma curve set */
#define REG_CHAS        0x4B    /* Colour bar adjustment */
#define REG_ADDVC       0x4F    /* Add Vertical */
#define REG_ADDVS       0x50    /* Add Vertical Size */
#define REG_COM17       0x51    /* Common control 17 */
#define REG_BLUE_FIX    0x6E
#define REG_RED_FIX     0x6F
#define REG_BRIGHTNESS  0x71
#define REG_CONTRAST    0x72
#define REG_GFIX        0x73
#define REG_MVFF        0x74
#define REG_EDGE        0x75
#define REG_UVPM        0x77
#define REG_SATURATION  0x78    /* Saturation control */
#define REG_HUE         0x79    /* Hue control */
#define REG_COM19       0x80
#define REG_GAIN_THRESHOLD 0x85
#define REG_AWB_CTRL0   0x87
#define REG_AWB_CTRL1   0x88
#define REG_AWB_CTRL2   0x89
#define REG_AWB_CTRL3   0x8A
#define REG_AWB_CTRL4   0x8B
#define REG_AWB_CTRL5   0x8C
#define REG_AWB_CTRL6   0x8D
#define REG_AWB_CTRL7   0x8E
#define REG_AWB_CTRL8   0x8F
#define REG_AWB_CTRL9   0x90
#define REG_AWB_CTRL10  0x91
#define REG_COM21       0x92
#define REG_NDDPM       0x93
#define REG_COM26       0x96
#define REG_CONTRAS_CTRL 0x97
#define REG_LCC_CTRL0   0x98
#define REG_LCC_CTRL1   0x99
#define REG_LCC_CTRL2   0x9A
#define REG_LCC_CTRL3   0x9B
#define REG_LCC_CTRL4   0x9C
#define REG_LCC_CTRL5   0x9D
#define REG_LCC_CTRL6   0x9E
#define REG_LCC_CTRL7   0x9F
#define REG_LCC_CTRL8   0xA0
#define REG_LCC_CTRL9   0xA1
#define REG_LCC_CTRL10  0xA2
#define REG_LCC_CTRL11  0xA3
#define REG_LCC_CTRL12  0xA4
#define REG_LCC_CTRL13  0xA5
#define REG_LCC_CTRL14  0xA6
#define REG_LCC_CTRL15  0xA7
#define REG_LCC_CTRL16  0xA8
#define REG_LCC_CTRL17  0xA9
#define REG_LCC_CTRL18  0xAA
#define REG_LCC_CTRL19  0xAB
#define REG_LCC_CTRL20  0xAC
#define REG_LCC_CTRL21  0xAD
#define REG_LCC_CTRL22  0xAE
#define REG_LCC_CTRL23  0xAF
#define REG_LCC_CTRL24  0xB0
#define REG_LCC_CTRL25  0xB1
#define REG_LCC_CTRL26  0xB2
#define REG_LCC_CTRL27  0xB3
#define REG_LCC_CTRL28  0xB4
#define REG_LCC_CTRL29  0xB5
#define REG_LCC_CTRL30  0xB6
#define REG_LCC_CTRL31  0xB7
#define REG_LCC_CTRL32  0xB8
#define REG_LCC_CTRL33  0xB9
#define REG_LCC_CTRL34  0xBA
#define REG_LCC_CTRL35  0xBB
#define REG_LCC_CTRL36  0xBC
#define REG_LCC_CTRL37  0xBD
#define REG_LCC_CTRL38  0xBE
#define REG_LCC_CTRL39  0xBF
#define REG_LCC_CTRL40  0xC0
#define REG_LCC_CTRL41  0xC1
#define REG_LCC_CTRL42  0xC2
#define REG_LCC_CTRL43  0xC3
#define REG_LCC_CTRL44  0xC4
#define REG_LCC_CTRL45  0xC5
#define REG_LCC_CTRL46  0xC6
#define REG_LCC_CTRL47  0xC7
#define REG_LCC_CTRL48  0xC8
#define REG_LCC_CTRL49  0xC9
#define REG_LCC_CTRL50  0xCA
#define REG_LCC_CTRL51  0xCB
#define REG_LCC_CTRL52  0xCC
#define REG_LCC_CTRL53  0xCD
#define REG_LCC_CTRL54  0xCE
#define REG_LCC_CTRL55  0xCF
#define REG_LCC_CTRL56  0xD0
#define REG_LCC_CTRL57  0xD1
#define REG_LCC_CTRL58  0xD2
#define REG_LCC_CTRL59  0xD3
#define REG_LCC_CTRL60  0xD4
#define REG_LCC_CTRL61  0xD5
#define REG_LCC_CTRL62  0xD6
#define REG_LCC_CTRL63  0xD7
#define REG_LCC_CTRL64  0xD8
#define REG_LCC_CTRL65  0xD9
#define REG_LCC_CTRL66  0xDA
#define REG_LCC_CTRL67  0xDB
#define REG_LCC_CTRL68  0xDC
#define REG_LCC_CTRL69  0xDD
#define REG_LCC_CTRL70  0xDE
#define REG_LCC_CTRL71  0xDF
#define REG_LCC_CTRL72  0xE0
#define REG_LCC_CTRL73  0xE1
#define REG_LCC_CTRL74  0xE2
#define REG_LCC_CTRL75  0xE3
#define REG_LCC_CTRL76  0xE4
#define REG_LCC_CTRL77  0xE5
#define REG_LCC_CTRL78  0xE6
#define REG_LCC_CTRL79  0xE7
#define REG_LCC_CTRL80  0xE8
#define REG_LCC_CTRL81  0xE9
#define REG_LCC_CTRL82  0xEA
#define REG_LCC_CTRL83  0xEB
#define REG_LCC_CTRL84  0xEC
#define REG_LCC_CTRL85  0xED
#define REG_LCC_CTRL86  0xEE
#define REG_LCC_CTRL87  0xEF
#define REG_LCC_CTRL88  0xF0
#define REG_LCC_CTRL89  0xF1
#define REG_LCC_CTRL90  0xF2
#define REG_LCC_CTRL91  0xF3
#define REG_LCC_CTRL92  0xF4
#define REG_LCC_CTRL93  0xF5
#define REG_LCC_CTRL94  0xF6
#define REG_LCC_CTRL95  0xF7
#define REG_LCC_CTRL96  0xF8
#define REG_LCC_CTRL97  0xF9
#define REG_LCC_CTRL98  0xFA
#define REG_LCC_CTRL99  0xFB
#define REG_LCC_CTRL100 0xFC
#define REG_LCC_CTRL101 0xFD
#define REG_LCC_CTRL102 0xFE
#define REG_LCC_CTRL103 0xFF


// --- Function Prototypes for OV7670 Driver ---
void sccb_init();
void sccb_write_byte(uint8_t reg, uint8_t data);
uint8_t sccb_read_byte(uint8_t reg);
void camera_xclk_init(int freq_mhz, int pin);
void ov7670_init(int xclk_freq_mhz);
void ov7670_set_yuv(); // Sets camera to YUV output (YUV422)
void ov7670_set_grayscale(); // Configures camera for grayscale output
void ov7670_set_resolution_qqvga(); // Sets camera to QQVGA (160x120)
// Captures a frame from OV7670 (QQVGA YUV422) and converts it to 96x96 grayscale in target_buffer
bool ov7670_capture_frame_to_grayscale_96x96(uint8_t* target_buffer);

#endif // OV7670_H