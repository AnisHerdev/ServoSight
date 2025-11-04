import cv2
import mediapipe as mp
import serial
import time
import datetime
import sys

# --- Configuration ---
SERIAL_PORT = 'COM8'     # Update to your Arduino port.
BAUD_RATE = 9600
CAMERA_INDEX = 0
KP_PAN = 0.04
KP_TILT = 0.04
current_pan_angle = 90
current_tilt_angle = 90

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands_detector = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

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

    # Serial setup for Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"Serial connection established on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        sys.exit(1)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Cannot open camera")
        sys.exit(1)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = frame_width // 2
    center_y = frame_height // 2

    print(f"Camera initialized: {frame_width}x{frame_height}")

    video_recording = False
    video_writer = None
    video_filename = ""
    last_peace_time = 0
    last_palm_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera frame not received")
                break
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # --- Face detection and pan/tilt tracking ---
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            if len(faces) > 0:
                x, y, w, h = max(faces, key=lambda f: f[2]*f[3])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0,255,0), 2)
                face_center_x = x + w // 2
                face_center_y = y + h // 2
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0,0,255), -1)

                error_x = face_center_x - center_x
                error_y = face_center_y - center_y

                current_pan_angle -= error_x * KP_PAN
                current_tilt_angle += error_y * KP_TILT

                current_pan_angle = max(0, min(180, current_pan_angle))
                current_tilt_angle = max(0, min(180, current_tilt_angle))

                send_angles(ser, current_pan_angle, current_tilt_angle)

            # --- Gesture detection ---
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands_detector.process(img_rgb)
            gesture = "None"
            peace_found = False
            palm_found = False
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    if is_peace(hand_landmarks):
                        gesture = "Peace"
                        peace_found = True
                    elif is_palm(hand_landmarks):
                        gesture = "Palm"
                        palm_found = True
                    else:
                        gesture = "Unknown"

            # --- Photo capture (Peace) ---
            if peace_found and (time.time() - last_peace_time > 2):  # debounce 2s
                now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                photo_filename = f"photo_{now}.jpg"
                cv2.imwrite(photo_filename, frame)
                print(f"Photo saved: {photo_filename}")
                last_peace_time = time.time()

            # --- Video recording toggle (Palm) ---
            if palm_found and (time.time() - last_palm_time > 2):  # debounce 2s
                if not video_recording:
                    now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    video_filename = f"video_{now}.avi"
                    video_writer = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*'XVID'), 20, (frame_width, frame_height))
                    video_recording = True
                    print(f"Started recording video: {video_filename}")
                else:
                    video_recording = False
                    if video_writer is not None:
                        video_writer.release()
                        print(f"Stopped recording video: {video_filename}")
                    video_writer = None
                last_palm_time = time.time()

            # --- Save video frames ---
            if video_recording and video_writer is not None:
                video_writer.write(frame)

            # --- Display on frame ---
            cv2.putText(frame, f"Gesture: {gesture}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(frame, f"Pan: {int(current_pan_angle)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"Tilt: {int(current_tilt_angle)}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.imshow("Pan-Tilt Face Tracker with Gesture Control", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        cap.release()
        if video_writer is not None:
            video_writer.release()
        cv2.destroyAllWindows()
        ser.close()

if __name__ == "__main__":
    main()
