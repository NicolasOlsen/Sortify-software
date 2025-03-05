import cv2
import numpy as np

# Store previous bounding box positions for smoothing
prev_boxes = {"Red": None, "Green": None, "Blue": None}
alpha = 0.6  # Smoothing factor (0 = no smoothing, 1 = instant update)

def smooth_bbox(new_bbox, prev_bbox, alpha=0.6):
    """Smooth bounding box movement using exponential moving average (EMA)."""
    if prev_bbox is None:
        return new_bbox  # First frame, no smoothing
    return tuple(int(alpha * new + (1 - alpha) * prev) for new, prev in zip(new_bbox, prev_bbox))

def nothing(x):
    pass

cap = cv2.VideoCapture(0)  # Use webcam

cv2.namedWindow("Trackbars")

# Red
cv2.createTrackbar("Low-H Red", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("Low-S Red", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("Low-V Red", "Trackbars", 70, 255, nothing)
cv2.createTrackbar("High-H Red", "Trackbars", 10, 179, nothing)
cv2.createTrackbar("High-S Red", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High-V Red", "Trackbars", 255, 255, nothing)

# Green
cv2.createTrackbar("Low-H Green", "Trackbars", 36, 179, nothing)
cv2.createTrackbar("Low-S Green", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("Low-V Green", "Trackbars", 70, 255, nothing)
cv2.createTrackbar("High-H Green", "Trackbars", 89, 179, nothing)
cv2.createTrackbar("High-S Green", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High-V Green", "Trackbars", 255, 255, nothing)

# Blue
cv2.createTrackbar("Low-H Blue", "Trackbars", 94, 179, nothing)
cv2.createTrackbar("Low-S Blue", "Trackbars", 80, 255, nothing)
cv2.createTrackbar("Low-V Blue", "Trackbars", 2, 255, nothing)
cv2.createTrackbar("High-H Blue", "Trackbars", 126, 179, nothing)
cv2.createTrackbar("High-S Blue", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High-V Blue", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Get HSV values from trackbars
    l_h_r, l_s_r, l_v_r = [cv2.getTrackbarPos("Low-H Red", "Trackbars"), cv2.getTrackbarPos("Low-S Red", "Trackbars"), cv2.getTrackbarPos("Low-V Red", "Trackbars")]
    h_h_r, h_s_r, h_v_r = [cv2.getTrackbarPos("High-H Red", "Trackbars"), cv2.getTrackbarPos("High-S Red", "Trackbars"), cv2.getTrackbarPos("High-V Red", "Trackbars")]

    l_h_g, l_s_g, l_v_g = [cv2.getTrackbarPos("Low-H Green", "Trackbars"), cv2.getTrackbarPos("Low-S Green", "Trackbars"), cv2.getTrackbarPos("Low-V Green", "Trackbars")]
    h_h_g, h_s_g, h_v_g = [cv2.getTrackbarPos("High-H Green", "Trackbars"), cv2.getTrackbarPos("High-S Green", "Trackbars"), cv2.getTrackbarPos("High-V Green", "Trackbars")]

    l_h_b, l_s_b, l_v_b = [cv2.getTrackbarPos("Low-H Blue", "Trackbars"), cv2.getTrackbarPos("Low-S Blue", "Trackbars"), cv2.getTrackbarPos("Low-V Blue", "Trackbars")]
    h_h_b, h_s_b, h_v_b = [cv2.getTrackbarPos("High-H Blue", "Trackbars"), cv2.getTrackbarPos("High-S Blue", "Trackbars"), cv2.getTrackbarPos("High-V Blue", "Trackbars")]

    # Define HSV ranges
    lower_red1, upper_red1 = np.array([l_h_r, l_s_r, l_v_r]), np.array([h_h_r, h_s_r, h_v_r])
    lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])

    lower_green, upper_green = np.array([l_h_g, l_s_g, l_v_g]), np.array([h_h_g, h_s_g, h_v_g])
    lower_blue, upper_blue = np.array([l_h_b, l_s_b, l_v_b]), np.array([h_h_b, h_s_b, h_v_b])

    # Create masks
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Remove noise with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

    # Function to find and label objects with bounding box smoothing
    def detect_and_label(mask, color_name, frame, color):
        global prev_boxes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bounding_boxes = []

        for contour in contours:
            if cv2.contourArea(contour) > 3000:  # Ignore small detections
                x, y, w, h = cv2.boundingRect(contour)
                bounding_boxes.append((x, y, x + w, y + h))  # Store box coordinates

        if bounding_boxes:
            x_min = min(box[0] for box in bounding_boxes)
            y_min = min(box[1] for box in bounding_boxes)
            x_max = max(box[2] for box in bounding_boxes)
            y_max = max(box[3] for box in bounding_boxes)

            # Smooth the bounding box position
            smoothed_box = smooth_bbox((x_min, y_min, x_max, y_max), prev_boxes[color_name], alpha=0.6)
            prev_boxes[color_name] = smoothed_box  # Store last position

            cv2.rectangle(frame, (smoothed_box[0], smoothed_box[1]), (smoothed_box[2], smoothed_box[3]), color, 2)
            cv2.putText(frame, color_name, (smoothed_box[0], smoothed_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # Apply detection
    detect_and_label(mask_red, "Red", frame, (0, 0, 255))
    detect_and_label(mask_green, "Green", frame, (0, 255, 0))
    detect_and_label(mask_blue, "Blue", frame, (255, 0, 0))

    cv2.imshow("Original Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
