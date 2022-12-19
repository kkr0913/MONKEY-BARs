from cvzone.HandTrackingModule import HandDetector
import cv2
import math
import requests

# Get video from ESP32-CAM web stream
URL = "http://10.247.137.35"
cap = cv2.VideoCapture(URL + ":81/stream")
serverPostSignal = URL + ":83/postSignal"
detector = HandDetector(detectionCon=0.5, maxHands=1)
cap.set(3, 1280)
cap.set(4, 720)

# General variables
gesture = "Stop"
prev_gesture = "Stop"
command = "Stop"
cnt = 0


# Function to detect hand gestures
def get_gesture():
    if totalFingers == 5:
            return "Stop"
    elif totalFingers == 0 and thumb_joint_x < wrist_x:
            return "Move Base Forward"
    elif totalFingers == 0 and thumb_joint_x > wrist_x:
        return "Move Base Backward"
    elif totalFingers == 2 and fingers[0] * fingers[4] == 1:
        if thumb_joint_x < wrist_x:
            return "Move Track Forward"
        else:
            return "Move Track Backward"
    elif totalFingers == 2 and fingers[1] * fingers[2] == 1:
        length = math.sqrt(pow(index_x - index_joint_x, 2) + pow(index_y - index_joint_y, 2))
        distance = math.sqrt(pow(middle_x - index_x, 2) + pow(middle_y - index_y, 2))
        if distance > length:
            return "Open Hook"
        else:
            return "Close Hook"
    elif totalFingers == 1 and fingers[0] == 1:
        if thumb_y < thumb_joint_y:
            return "Increase Sequence"
        else:
            return "Decrease Sequence"
    else:
        return "Stop"


while True:
    success, img = cap.read()
    img = cv2.flip(img, 0)
    img = detector.findHands(img)
    lmList, bbox = detector.findPosition(img)

    if lmList:  # Hand is detected
        fingers = detector.fingersUp()
        totalFingers = fingers.count(1)
        wrist_x, wrist_y = lmList[0][0], lmList[0][1]
        thumb_joint_x, thumb_joint_y = lmList[3][0], lmList[3][1]
        thumb_x, thumb_y = lmList[4][0], lmList[4][1]
        index_joint_x, index_joint_y = lmList[6][0], lmList[6][1]
        index_x, index_y = lmList[8][0], lmList[8][1]
        middle_x, middle_y = lmList[12][0], lmList[12][1]
        center_x, center_y = lmList[9][0], lmList[9][1]
        gesture = get_gesture()
        if prev_gesture != gesture:
            cnt = 0
        else:
            cnt += 1
        prev_gesture = gesture
        if cnt > 5:
            command = gesture
        requests.post(serverPostSignal, data=command)
        cv2.putText(img, f'Fingers: {totalFingers}', (20, 120), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 3)

    # Display
    cv2.putText(img, command, (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
