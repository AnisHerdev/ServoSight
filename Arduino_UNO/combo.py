import cv2
import mediapipe as mp
import serial
import time
import sys

# --- Configuration ---
SERIAL_PORT = 'COM8'  # Update this to your Arduino port
BAUD_RATE = 9600
CAMERA_INDEX = 0

# Control gains for proportional controller
KP_PAN = 0.04
KP_TILT = 0.04

# Initial servo angles
current_pan_angle = 90
current_tilt_angle = 90

# Setup MediaPipe Hands for gesture detection
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands_detector = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

# Load OpenCV face cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def is_palm(landmarks):
    tips_ids = [8, 12, 16, 20]
    for tip_id in tips_ids:
        if landmarks.landmark[tip_id].y > landmarks.landmark[tip_id - 2].y:
            return False
    return True

def is_peace(landmarks):
    tips_ids = [8, 12]
    folded_ids = [16, 20]
    for tip_id in tips_ids:
        if landmarks.landmark[tip_id].y > landmarks.landmark[tip_id - 2].y:
            return False
    for tip_id in folded_ids:
        if landmarks.landmark[tip_id].y < landmarks.landmark[tip_id - 2].y:
            return False
    return True

def setup_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"Serial connection established on {SERIAL_PORT}.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        print("Please check the port and Arduino connection.")
        sys.exit(1)

def send_angles(ser, pan, tilt):
    pan = max(0, min(180, int(pan)))
    tilt = max(0, min(180, int(tilt)))
    message = f"{pan},{tilt}\n"
    try:
        ser.write(message.encode('utf-8'))
    except serial.SerialException:
        print("Warning: Serial connection lost?")

def main_test():
    global current_pan_angle, current_tilt_angle

    # Skip serial setup in test mode
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Cannot open camera {CAMERA_INDEX}. Exiting...")
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
                print("No frame from camera. Exiting...")
                break

            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Face detection
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            if len(faces) > 0:
                x, y, w, h = max(faces, key=lambda f: f[2]*f[3])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                face_center_x = x + w // 2
                face_center_y = y + h // 2
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

                error_x = face_center_x - center_x
                error_y = face_center_y - center_y

                # Pan angle decreases when face is right, tilt increases when face is down
                current_pan_angle -= error_x * KP_PAN
                current_tilt_angle += error_y * KP_TILT

                current_pan_angle = max(0, min(180, current_pan_angle))
                current_tilt_angle = max(0, min(180, current_tilt_angle))

                # Print angles instead of sending to Arduino in test mode
                print(f"Pan: {int(current_pan_angle)}, Tilt: {int(current_tilt_angle)}")

            # Gesture detection
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands_detector.process(img_rgb)
            gesture = "None"
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    if is_palm(hand_landmarks):
                        gesture = "Palm"
                    elif is_peace(hand_landmarks):
                        gesture = "Peace"
                    else:
                        gesture = "Unknown"

            cv2.putText(frame, f'Gesture: {gesture}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Center crosshair
            cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (255, 0, 0), 1)
            cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (255, 0, 0), 1)
            cv2.putText(frame, f"Pan: {int(current_pan_angle)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Tilt: {int(current_tilt_angle)}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow('Pan-Tilt Face Tracker with Gesture Control', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Cleaned up resources.")

if __name__ == "__main__":
    main_test()