# Libraries
import cv2
import numpy as np

# Initialize video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Create two windows: one for output and one for sliders
cv2.namedWindow('Output')
cv2.namedWindow('Trackbars')

def nothing(x):
    pass

# Trackbars for general HSV filtering (Trackbars window)
cv2.createTrackbar('H_min', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('S_min', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('V_min', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('H_max', 'Trackbars', 180, 180, nothing)
cv2.createTrackbar('S_max', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('V_max', 'Trackbars', 255, 255, nothing)

# Extra trackbars for Red Filter Mode
cv2.createTrackbar('H_Red_Min_1', 'Trackbars', 0, 10, nothing)     # First red range
cv2.createTrackbar('H_Red_Max_1', 'Trackbars', 10, 10, nothing)
cv2.createTrackbar('H_Red_Min_2', 'Trackbars', 170, 180, nothing)  # Second red range
cv2.createTrackbar('H_Red_Max_2', 'Trackbars', 180, 180, nothing)

# Mode switch: 0 = normal filter, 1 = red filter
red_filter_mode = False

while True:
    k = cv2.waitKey(5) & 0xFF
    if k == 27:  # Exit on ESC key
        break
    elif k == ord('r'):  # Toggle red filter mode
        red_filter_mode = not red_filter_mode
        print("Red filter mode:", red_filter_mode)

    ret, frame = cap.read()
    if not ret:
        continue

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if red_filter_mode:
        # Get red HSV values from trackbars
        h1_min = cv2.getTrackbarPos('H_Red_Min_1', 'Trackbars')
        h1_max = cv2.getTrackbarPos('H_Red_Max_1', 'Trackbars')
        h2_min = cv2.getTrackbarPos('H_Red_Min_2', 'Trackbars')
        h2_max = cv2.getTrackbarPos('H_Red_Max_2', 'Trackbars')

        s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
        v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
        s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
        v_max = cv2.getTrackbarPos('V_max', 'Trackbars')

        # Red color range (two sections)
        lower_red1 = np.array([h1_min, s_min, v_min])
        upper_red1 = np.array([h1_max, s_max, v_max])
        lower_red2 = np.array([h2_min, s_min, v_min])
        upper_red2 = np.array([h2_max, s_max, v_max])

        # Apply masks
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

    else:
        # Get HSV values from trackbars for general filtering
        h_min = cv2.getTrackbarPos('H_min', 'Trackbars')
        s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
        v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H_max', 'Trackbars')
        s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
        v_max = cv2.getTrackbarPos('V_max', 'Trackbars')

        # Apply normal masking
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

    # Apply mask and show output
    Output = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('Output', Output)

cap.release()
cv2.destroyAllWindows()
