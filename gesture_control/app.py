# Description: This script is used to control the robotic arm using hand gestures.
# Author: Burak Keçeci

import math

import cv2
import mediapipe as mp
import serial

debug = False

if not debug:
    ser = serial.Serial('COM7', 115200) # change your serial port here

# Configurations
x_min = 13
x_mid = 93
x_max = 173

# use angle between wrist and index finger to control x axis
palm_angle_min = -50
palm_angle_mid = 20

y_min = 70
y_mid = 110
y_max = 150

# use wrist y to control y axis
wrist_y_min = 0.3
wrist_y_max = 0.9

z_min = 113
z_mid = 146
z_max = 180

# use palm size to control z axis
plam_size_min = 0.1
plam_size_max = 0.3

claw_open_angle = 93
claw_close_angle = 160

servo_angle = [x_mid, y_mid, z_mid, claw_open_angle]  # [x, y, z, claw]
prev_servo_angle = servo_angle
fist_threshold = 7

cap_width = 1400
cap_height = 900

# Model Load
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5,
)
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Camera preparation
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)

clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
map_range = lambda x, in_min, in_max, out_min, out_max: abs(
    (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)


def is_fist(hand_landmarks, palm_size):
    # calculate the distance between the wrist and the each finger tip
    distance_sum = 0
    WRIST = hand_landmarks.landmark[0]
    for i in [7, 8, 11, 12, 15, 16, 19, 20]:
        distance_sum += ((WRIST.x - hand_landmarks.landmark[i].x) ** 2 + \
                         (WRIST.y - hand_landmarks.landmark[i].y) ** 2 + \
                         (WRIST.z - hand_landmarks.landmark[i].z) ** 2) ** 0.5

    return distance_sum / palm_size < fist_threshold


def landmark_to_servo_angle(hand_landmarks):
    servo_angle = [x_mid, y_mid, z_mid, claw_open_angle]
    WRIST = hand_landmarks.landmark[0]
    INDEX_FINGER_MCP = hand_landmarks.landmark[5]
    # calculate the distance between the wrist and the index finger
    palm_size = ((WRIST.x - INDEX_FINGER_MCP.x) ** 2 + (WRIST.y - INDEX_FINGER_MCP.y) ** 2 + (
            WRIST.z - INDEX_FINGER_MCP.z) ** 2) ** 0.5

    # cv2.putText(img, "PALM SIZE "+str(palm_size), (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 8)

    if is_fist(hand_landmarks, palm_size):
        servo_angle[3] = claw_close_angle
        cv2.putText(image, "CLOSE " + str(palm_size), (10, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    else:
        servo_angle[3] = claw_open_angle
        cv2.putText(image, "OPEN " + str(palm_size), (10, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # calculate x angle
    distance = palm_size
    angle = (WRIST.x - INDEX_FINGER_MCP.x) / distance  # calculate the radian between the wrist and the index finger
    angle = int(angle * 180 / 3.1415926)  # convert radian to degree
    angle = clamp(angle, palm_angle_min, palm_angle_mid)
    servo_angle[0] = map_range(angle, palm_angle_min, palm_angle_mid, x_max, x_min)
    cv2.putText(image, " X  " + str(servo_angle[0]), (10, 180), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # calculate y angle
    wrist_y = clamp(WRIST.y, wrist_y_min, wrist_y_max)
    servo_angle[1] = map_range(wrist_y, wrist_y_min, wrist_y_max, y_max, y_min)
    cv2.putText(image, " Y  " + str(servo_angle[1]), (10, 210), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # calculate z angle
    palm_size = clamp(palm_size, plam_size_min, plam_size_max)
    servo_angle[2] = map_range(palm_size, plam_size_min, plam_size_max, z_max, z_min)
    cv2.putText(image, " Z  " + str(servo_angle[2]), (10, 240), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # float to int
    servo_angle = [int(i) for i in servo_angle]

    return servo_angle


def log_scale(value):
    return (math.log(value)) * 100


while True:
    ret, image = cap.read()
    if not ret:
        print(ret)
        continue
    image = cv2.flip(image, 1)  # Mirror display

    # Detection implementation
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    image.flags.writeable = False
    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        if len(results.multi_hand_landmarks) == 1:
            hand_landmarks = results.multi_hand_landmarks[0]
            servo_angle = landmark_to_servo_angle(hand_landmarks)

            if servo_angle != prev_servo_angle:
                # print("Servo angle: ", servo_angle)
                prev_servo_angle = servo_angle
                if not debug:
                    ser.write(bytearray(servo_angle))

        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())

    cv2.putText(image, "Servo Angle: " + str(servo_angle), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
