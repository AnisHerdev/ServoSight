#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h> // For ESP32 specific servo control

// Project-specific headers
#include "model.h"     // Our generated TFLite Micro model byte array
#include "camera_pins.h" // Your camera and servo pin definitions
#include "ov7670.h"    // OV7670 camera driver

// --- TensorFlow Lite Micro Includes ---
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/schema/schema_generated.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/micro/compatibility.h>

// --- Servo Control Configuration ---
Servo panServo;
Servo tiltServo;

// Servo initial positions (adjust for your physical setup)
int currentPanAngle = 90;
int currentTiltAngle = 90;

// Servo limits (adjust based on your servo's range and mounting)
const int MIN_PAN_ANGLE = 0;
const int MAX_PAN_ANGLE = 180;
const int MIN_TILT_ANGLE = 0;
const int MAX_TILT_ANGLE = 180;

// PID-like constants for servo adjustment (TUNE THESE FOR SMOOTH TRACKING!)
const float Kp_pan = 0.5;  // Proportional constant for pan
const float Kp_tilt = 0.5; // Proportional constant for tilt
const int SERVO_MOVE_DEADZONE = 5; // Pixels from center before moving servo

// --- TensorFlow Lite Micro Variables ---
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

// An area of memory to use for input, output, and intermediate arrays.
// The interpreter will use this. For person_detect, 100KB is usually safe.
const int kTensorArenaSize = 100 * 1024; // 100 KB
static uint8_t tensor_arena[kTensorArenaSize]; // 'static' to place in BSS/DRAM

// --- Global variables for inference results ---
// The person_detect model outputs two values:
// output->data.uint8[0] for 'no person'
// output->data.uint8[1] for 'person'
float person_detection_score = 0.0;
float no_person_detection_score = 0.0;

// --- Detection threshold (TUNE THIS!) ---
// If person_detection_score exceeds this, we consider a person detected.
const float DETECTION_THRESHOLD = 0.7; // Example: 70% confidence

// --- Function Prototypes ---
void setupServos();
void adjustServos(int face_center_x, int face_center_y);

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Starting ESP32 Face Tracker with OV7670...");

  // Setup error reporter for TensorFlow Lite Micro
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  Serial.println("Initializing TensorFlow Lite Micro...");

  // Map the model into a usable data structure.
  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report("Model provided is schema version %d, but only version %d is supported by this library.",
                           model->version(), TFLITE_SCHEMA_VERSION);
    while (1); // Halt
  }

  // Pull in the standard set of TensorFlow Lite Micro operators
  static tflite::AllOpsResolver resolver;

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    error_reporter->Report("AllocateTensors() failed! Consider increasing kTensorArenaSize.");
    while (1); // Halt
  }

  // Get information about the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  // Ensure input tensor is configured correctly (96x96 grayscale)
  if (input->type != kTfLiteUInt8) {
      error_reporter->Report("Input tensor type is not UINT8");
      while (1);
  }
  if (input->dims->size != 4 || input->dims->data[1] != TFL_MODEL_HEIGHT ||
      input->dims->data[2] != TFL_MODEL_WIDTH || input->dims->data[3] != 1) { // 1 channel for grayscale
      error_reporter->Report("Input tensor dimensions mismatch. Expected 1x%dx%dx1, got %dx%dx%dx%d",
                              TFL_MODEL_HEIGHT, TFL_MODEL_WIDTH,
                              input->dims->data[0], input->dims->data[1],
                              input->dims->data[2], input->dims->data[3]);
      while (1);
  }
  Serial.printf("TFLite Micro Initialized. Input dimensions: %dx%dx%dx%d, Output dimensions: %dx%d\n",
                input->dims->data[0], input->dims->data[1], input->dims->data[2], input->dims->data[3],
                output->dims->data[0], output->dims->data[1]);


  // Initialize OV7670 camera (XCLK at 20MHz)
  Serial.println("Initializing OV7670 Camera...");
  ov7670_init(20); // 20MHz XCLK

  setupServos();
  Serial.println("Setup complete. Starting tracking loop...");
}

void loop() {
  unsigned long start_loop = millis();

  // 1. Capture Frame from OV7670 directly into the TFLite input tensor
  bool capture_success = ov7670_capture_frame_to_grayscale_96x96(input->data.uint8);
  unsigned long end_capture = millis();

  if (!capture_success) {
    Serial.println("Failed to capture frame from OV7670!");
    delay(100);
    return;
  }

  // 2. Run inference
  unsigned long start_inference = millis();
  TfLiteStatus invoke_status = interpreter->Invoke();
  unsigned long end_inference = millis();

  if (invoke_status != kTfLiteOk) {
    error_reporter->Report("Invoke failed");
    return;
  }

  // 3. Process Inference Results
  // Convert quantized scores back to float for a more intuitive understanding
  // (Assuming output tensor has scale and zero_point for UINT8 quantization)
  person_detection_score = (static_cast<float>(output->data.uint8[1]) - output->params.zero_point) * output->params.scale;
  no_person_detection_score = (static_cast<float>(output->data.uint8[0]) - output->params.zero_point) * output->params.scale;

  Serial.printf("Capture: %lu ms, Inference: %lu ms. Total Frame: %lu ms\n",
                end_capture - start_loop, end_inference - start_inference, millis() - start_loop);
  Serial.printf("No Person Score: %.2f, Person Score: %.2f\n",
                no_person_detection_score, person_detection_score);

  // 4. Decision Logic and Servo Adjustment
  if (person_detection_score > DETECTION_THRESHOLD && person_detection_score > no_person_detection_score) {
    Serial.println("!!! PERSON (FACE) DETECTED !!!");
    // Since person_detect is a classification model, it doesn't give bounding box coordinates directly.
    // For a simple pan-tilt, we can assume the detected "person" is roughly in the center
    // and try to keep it there.
    // Future enhancement: Integrate a model that provides actual bounding boxes.
    
    // For now, we assume the "center" of the detected person is the center of our 96x96 input frame.
    // This will instruct the servos to constantly try to center the area where a person is detected.
    int target_center_x = TFL_MODEL_WIDTH / 2;
    int target_center_y = TFL_MODEL_HEIGHT / 2;
    adjustServos(target_center_x, target_center_y);
  } else {
    Serial.println("No person/face detected. Holding position.");
    // Optionally, return servos to home or sweep slowly
    // adjustServos(TFL_MODEL_WIDTH / 2, TFL_MODEL_HEIGHT / 2); // Keep centered even if no detection
  }

  // Small delay to prevent watchdog timer from barking and stabilize serial output
  delay(50);
}

void setupServos() {
  Serial.println("Setting up Servos...");
  // Allow allocation of all timers to `micros()` for servo timing.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  panServo.setPeriodHertz(50); // Standard 50Hz servo
  tiltServo.setPeriodHertz(50);

  // Attach servos to pins defined in camera_pins.h
  panServo.attach((int)PAN_SERVO_GPIO_NUM, 500, 2400); // Typical min/max pulse width for 0-180 deg
  tiltServo.attach((int)TILT_SERVO_GPIO_NUM, 500, 2400);

  panServo.write(currentPanAngle);
  tiltServo.write(currentTiltAngle);

  Serial.printf("Servos attached to Pan: GPIO%d, Tilt: GPIO%d\n", PAN_SERVO_GPIO_NUM, TILT_SERVO_GPIO_NUM);
  delay(1000); // Give servos time to move to initial position
}

void adjustServos(int target_center_x, int target_center_y) {
  // We're assuming the detected "face" is at target_center_x, target_center_y
  // For the person_detect model, it's a binary classification. We assume the "person"
  // is centered in the frame.
  // The 'target_center_x' and 'target_center_y' arguments will be TFL_MODEL_WIDTH/2 and TFL_MODEL_HEIGHT/2
  // if a person is detected. This means we're trying to keep the detected object
  // centered in the view.

  int ideal_center_x = TFL_MODEL_WIDTH / 2;
  int ideal_center_y = TFL_MODEL_HEIGHT / 2;

  // Calculate error (deviation from ideal center)
  int error_x = target_center_x - ideal_center_x; // Positive if target is right of center
  int error_y = target_center_y - ideal_center_y; // Positive if target is below center

  Serial.printf("Target Center: (%d, %d), Ideal Center: (%d, %d), Error: (%d, %d)\n",
                target_center_x, target_center_y, ideal_center_x, ideal_center_y, error_x, error_y);

  // Only adjust if error is outside a deadzone to prevent jitter
  if (abs(error_x) > SERVO_MOVE_DEADZONE) {
    // Pan adjustment:
    // If error_x is positive (object right), decrease pan angle to move camera right.
    // If error_x is negative (object left), increase pan angle to move camera left.
    int pan_adjustment = static_cast<int>(error_x * Kp_pan);
    currentPanAngle -= pan_adjustment;
    currentPanAngle = constrain(currentPanAngle, MIN_PAN_ANGLE, MAX_PAN_ANGLE);
    panServo.write(currentPanAngle);
    Serial.printf("Adjusting Pan by %d, New Pan Angle: %d\n", pan_adjustment, currentPanAngle);
  }

  if (abs(error_y) > SERVO_MOVE_DEADZONE) {
    // Tilt adjustment:
    // If error_y is positive (object below), increase tilt angle to move camera down.
    // If error_y is negative (object above), decrease tilt angle to move camera up.
    int tilt_adjustment = static_cast<int>(error_y * Kp_tilt);
    currentTiltAngle += tilt_adjustment;
    currentTiltAngle = constrain(currentTiltAngle, MIN_TILT_ANGLE, MAX_TILT_ANGLE);
    tiltServo.write(currentTiltAngle);
    Serial.printf("Adjusting Tilt by %d, New Tilt Angle: %d\n", tilt_adjustment, currentTiltAngle);
  }
}