import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize MediaPipe Hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

def is_palm(landmarks):
    # Palm: all fingers extended (approximate by checking fingertips above PIP joints)
    tips_ids = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky fingertips
    for tip_id in tips_ids:
        if landmarks.landmark[tip_id].y > landmarks.landmark[tip_id - 2].y:
            return False
    return True

def is_peace(landmarks):
    # Peace: Index and Middle fingers extended, others folded
    tips_ids = [8, 12]
    folded_ids = [16, 20]
    for tip_id in tips_ids:
        if landmarks.landmark[tip_id].y > landmarks.landmark[tip_id - 2].y:
            return False
    for tip_id in folded_ids:
        if landmarks.landmark[tip_id].y < landmarks.landmark[tip_id - 2].y:
            return False
    return True

cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, img = cap.read()
    if not success:
        break
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    gesture = "None"
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            if is_palm(hand_landmarks):
                gesture = "Palm"
            elif is_peace(hand_landmarks):
                gesture = "Peace"
            else:
                gesture = "Unknown"

    cv2.putText(img, f'Gesture: {gesture}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow("Gesture Recognition", img)

    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
        break

cap.release()
cv2.destroyAllWindows()