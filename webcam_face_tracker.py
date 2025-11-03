import serial
import time
import os
import sys
import cv2

# --- Configuration ---
# NOTE: YOU MUST CHANGE THE SERIAL PORT!
# On Windows, this is usually 'COM3' or similar.
# On Linux/macOS, this is usually '/dev/ttyACM0' or '/dev/tty.usbmodemXXXX'.
# Check your Arduino IDE to find the correct port.
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 9600 # Must match the Arduino sketch
CAMERA_INDEX = 0 # Usually 0 for the built-in webcam

# --- Tracking Parameters ---
# The target frame dimensions will be automatically detected, 
# but these constants are used for the control loop's mapping.
KP_PAN = 0.04  # Proportional gain for horizontal movement (Pan)
KP_TILT = 0.04 # Proportional gain for vertical movement (Tilt)

# Initial angles (must match the Arduino's initial setting)
current_pan_angle = 90
current_tilt_angle = 90

# Load the pre-trained Haar Cascade classifier for face detection
# This file is usually included in your OpenCV installation
CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(CASCADE_PATH)

def setup_serial():
    """Initializes the serial connection to the Arduino."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) # Wait for the Arduino to reset and initialize
        print(f"Serial connection established on {SERIAL_PORT} at {BAUD_RATE} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        print("Please check the port name and ensure the Arduino is connected and not busy.")
        sys.exit(1)

def send_angles(ser, pan, tilt):
    """Formats and sends the pan and tilt angles to the Arduino."""
    # Constrain angles to the 0-180 degree range
    pan = max(0, min(180, int(pan)))
    tilt = max(0, min(180, int(tilt)))
    
    # Format the message as "PAN_ANGLE,TILT_ANGLE\n"
    message = f"{pan},{tilt}\n"
    
    try:
        ser.write(message.encode('utf-8'))
        # print(f"Sent: {message.strip()}") # Uncomment for debugging
    except SerialException:
        print("Warning: Could not send data. Serial connection lost?")
        # Attempt to re-establish connection if needed, or simply skip

def main():
    """Main function for camera capture and angle calculation."""
    global current_pan_angle, current_tilt_angle
    
    # 1. Setup Serial Communication
    ser = setup_serial()

    # 2. Setup Camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Cannot open camera with index {CAMERA_INDEX}.")
        sys.exit(1)
        
    # Get frame dimensions for centering calculations
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = frame_width // 2
    center_y = frame_height // 2
    
    print(f"Camera feed initialized: {frame_width}x{frame_height}. Center: ({center_x}, {center_y})")

    try:
        while True:
            # 3. Capture Frame
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # Flip the frame horizontally for a mirror effect (more intuitive for control)
            frame = cv2.flip(frame, 1)

            # Convert to grayscale for faster processing
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 4. Detect Faces
            faces = face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.1, 
                minNeighbors=5, 
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )

            if len(faces) > 0:
                # Target the largest face (assumed to be the nearest)
                (x, y, w, h) = max(faces, key=lambda f: f[2] * f[3])
                
                # Draw the bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Find the center of the detected object
                face_center_x = x + w // 2
                face_center_y = y + h // 2
                
                # Draw a dot at the face center
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

                # 5. Calculate Error
                error_x = face_center_x - center_x
                error_y = face_center_y - center_y
                
                # 6. Calculate New Angles (Proportional Control)
                # Note: Pan is controlled by horizontal error (X)
                # The relationship is usually inverse: if the target is to the right (positive error_x), 
                # the servo needs to turn LEFT (decrease angle) to center it.
                current_pan_angle -= error_x * KP_PAN
                
                # Note: Tilt is controlled by vertical error (Y)
                # If the target is down (positive error_y), the servo needs to turn UP (decrease angle) 
                # to center it, assuming the camera is mounted "upright" on the tilt servo.
                current_tilt_angle += error_y * KP_TILT 

                # 7. Send Angles
                send_angles(ser, current_pan_angle, current_tilt_angle)

            # Draw crosshairs at the frame center
            cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (255, 0, 0), 1)
            cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (255, 0, 0), 1)
            cv2.putText(frame, f"Pan: {int(current_pan_angle)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Tilt: {int(current_tilt_angle)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 8. Display Frame
            cv2.imshow('OpenCV Face Tracker', frame)

            # Exit if 'q' is pressed
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        # 9. Cleanup
        cap.release()
        cv2.destroyAllWindows()
        ser.close()
        print("Camera and Serial connection closed.")

def main_test():
    """Main function for camera capture and angle calculation (no serial)."""
    global current_pan_angle, current_tilt_angle

    # 1. Skip Serial Setup for Testing
    # ser = setup_serial()  # Commented out

    # 2. Setup Camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Cannot open camera with index {CAMERA_INDEX}.")
        sys.exit(1)
        
    # Get frame dimensions for centering calculations
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = frame_width // 2
    center_y = frame_height // 2
    
    print(f"Camera feed initialized: {frame_width}x{frame_height}. Center: ({center_x}, {center_y})")

    try:
        while True:
            # 3. Capture Frame
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 4. Detect Faces
            faces = face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.1, 
                minNeighbors=5, 
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )

            if len(faces) > 0:
                (x, y, w, h) = max(faces, key=lambda f: f[2] * f[3])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                face_center_x = x + w // 2
                face_center_y = y + h // 2
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

                # 5. Calculate Error
                error_x = face_center_x - center_x
                error_y = face_center_y - center_y

                # 6. Calculate New Angles
                current_pan_angle -= error_x * KP_PAN
                current_tilt_angle += error_y * KP_TILT

                # Constrain to 0–180
                current_pan_angle = max(0, min(180, int(current_pan_angle)))
                current_tilt_angle = max(0, min(180, int(current_tilt_angle)))

                # 7. Print angles instead of sending
                print(f"Pan: {current_pan_angle}, Tilt: {current_tilt_angle}")

            # Draw crosshairs and angle display
            cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (255, 0, 0), 1)
            cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (255, 0, 0), 1)
            cv2.putText(frame, f"Pan: {int(current_pan_angle)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Tilt: {int(current_tilt_angle)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Show the result
            cv2.imshow('OpenCV Face Tracker (Test Mode)', frame)

            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera closed. (No serial connection used.)")


if __name__ == "__main__":
    main_test()
    # main()
