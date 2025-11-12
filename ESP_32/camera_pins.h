#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// --- OV7670 Camera Pin Definitions for ESP32 ---
// YOU MUST ADJUST THESE TO YOUR ACTUAL WIRING!
// These are common assignments, but can vary.
// If using an ESP32-CAM, this driver is NOT suitable; ESP32-CAM uses a different driver.
// This is for generic ESP32 boards with a separate OV7670 module.

// SDA/SCL (I2C for SCCB control)
#define SIOD_GPIO_NUM       GPIO_NUM_21  // SDA (Serial Camera Data)
#define SIOC_GPIO_NUM       GPIO_NUM_22  // SCL (Serial Camera Clock)

// VSYNC (Vertical Sync)
#define VSYNC_GPIO_NUM      GPIO_NUM_25

// HREF (Horizontal Reference)
#define HREF_GPIO_NUM       GPIO_NUM_23

// PCLK (Pixel Clock)
#define PCLK_GPIO_NUM       GPIO_NUM_14

// XCLK (External Clock) - Often driven by a PWM on an ESP32 GPIO
#define XCLK_GPIO_NUM       GPIO_NUM_26

// Data Bus D0-D7 (ensure these are available and correctly connected)
#define D0_GPIO_NUM         GPIO_NUM_5
#define D1_GPIO_NUM         GPIO_NUM_18
#define D2_GPIO_NUM         GPIO_NUM_19
#define D3_GPIO_NUM         GPIO_NUM_33
#define D4_GPIO_NUM         GPIO_NUM_32
#define D5_GPIO_NUM         GPIO_NUM_27
#define D6_GPIO_NUM         GPIO_NUM_17
#define D7_GPIO_NUM         GPIO_NUM_16

// PWDN (Power Down) - High to enable, Low to power down
// Set to -1 if not connected and tied high externally
#define PWDN_GPIO_NUM       GPIO_NUM_4 // Example, can be -1 if not used

// RESET (Reset) - Low to reset, High for normal operation
// Set to -1 if not connected and tied high externally
#define RESET_GPIO_NUM      GPIO_NUM_15 // Example, can be -1 if not used


// --- Servo Pin Definitions ---
// YOU MUST ADJUST THESE TO YOUR ACTUAL WIRING!
// Ensure these pins are not conflicting with camera pins if you change them.
#define PAN_SERVO_GPIO_NUM  GPIO_NUM_2   // Example: ESP32 GPIO2
#define TILT_SERVO_GPIO_NUM GPIO_NUM_0   // Example: ESP32 GPIO0

#endif // CAMERA_PINS_H