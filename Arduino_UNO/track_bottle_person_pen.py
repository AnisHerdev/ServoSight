import cv2
import serial
import time
import sys
from ultralytics import YOLO # pip install ultralytics

# --- Configuration ---
SERIAL_PORT = 'COM8'     # Update to your Arduino port.
BAUD_RATE = 9600
CAMERA_INDEX = 1
KP_PAN = 0.04
KP_TILT = 0.04
current_pan_angle = 90
current_tilt_angle = 90

# Load a pre-trained YOLOv5 model
# You might need to install ultralytics: pip install ultralytics
# The first time you run this, it will download the yolov5s.pt weights.
model = YOLO('yolov5s.pt')

def send_angles(ser, pan, tilt):
    """Send the pan and tilt angles to Arduino over serial."""
    pan = max(0, min(180, int(pan)))
    tilt = max(0, min(180, int(tilt)))
    try:
        ser.write(f"{pan},{tilt}\n".encode('utf-8'))
    except serial.SerialException:
        print("Serial communication lost")

def main():
    global current_pan_angle, current_tilt_angle

    # Get target object from user
    target_object_name = input("Enter the name of the object to track (e.g., 'person', 'bottle', 'pen'): ").lower()

    # Serial setup for Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"Serial connection established on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        sys.exit(1)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_EXPOSURE, -8)
    if not cap.isOpened():
        print("Cannot open camera")
        sys.exit(1)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = frame_width // 2
    center_y = frame_height // 2

    print(f"Camera initialized: {frame_width}x{frame_height}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera frame not received")
                break

            # Perform object detection
            results = model(frame, verbose=False) # verbose=False to suppress extensive output per frame

            object_found = False
            for r in results:
                for *xyxy, conf, cls in r.boxes.data:
                    class_name = model.names[int(cls)].lower()

                    if class_name == target_object_name:
                        # Convert tensor to numpy array for drawing
                        x1, y1, x2, y2 = map(int, xyxy)

                        # Draw bounding box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

                        # Calculate center of the detected object
                        object_center_x = (x1 + x2) // 2
                        object_center_y = (y1 + y2) // 2
                        cv2.circle(frame, (object_center_x, object_center_y), 5, (0, 0, 255), -1)

                        # Calculate error and update pan/tilt angles
                        error_x = object_center_x - center_x
                        error_y = object_center_y - center_y

                        current_pan_angle -= error_x * KP_PAN
                        current_tilt_angle += error_y * KP_TILT

                        current_pan_angle = max(0, min(180, current_pan_angle))
                        current_tilt_angle = max(0, min(180, current_tilt_angle))

                        send_angles(ser, current_pan_angle, current_tilt_angle)
                        object_found = True
                        break # Track the first instance found
                if object_found:
                    break

            # Display on frame
            cv2.putText(frame, f"Tracking: {target_object_name.capitalize()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Pan: {int(current_pan_angle)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Tilt: {int(current_tilt_angle)}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Pan-Tilt Object Tracker", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()

if __name__ == "__main__":
    main()
