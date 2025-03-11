import cv2
import numpy as np

# Store previous positions for smoothing
prev_circles = {"Red": None, "Green": None, "Blue": None}
alpha = 0.6  # Smoothing factor

def smooth_circle(new_circle, prev_circle, alpha=0.6):
    """Smooth circle movement using exponential moving average (EMA)."""
    if prev_circle is None:
        return new_circle
    return tuple(int(alpha * new + (1 - alpha) * prev) for new, prev in zip(new_circle, prev_circle))

def nothing(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow("Trackbars")

# Red
cv2.createTrackbar("Low-H Red", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("Low-S Red", "Trackbars", 160, 255, nothing)
cv2.createTrackbar("Low-V Red", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("High-H Red", "Trackbars", 10, 179, nothing)
cv2.createTrackbar("High-S Red", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High-V Red", "Trackbars", 255, 255, nothing)

# Green
cv2.createTrackbar("Low-H Green", "Trackbars", 36, 179, nothing)
cv2.createTrackbar("Low-S Green", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("Low-V Green", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("High-H Green", "Trackbars", 89, 179, nothing)
cv2.createTrackbar("High-S Green", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High-V Green", "Trackbars", 255, 255, nothing)

# Blue
cv2.createTrackbar("Low-H Blue", "Trackbars", 94, 179, nothing)
cv2.createTrackbar("Low-S Blue", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("Low-V Blue", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("High-H Blue", "Trackbars", 126, 179, nothing)
cv2.createTrackbar("High-S Blue", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("High-V Blue", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocessing - Reduce noise
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # Convert BGR to LAB color space to enhance red detection
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    a = clahe.apply(a)
    lab = cv2.merge([l, a, b])
    enhanced_frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # Convert to HSV
    hsv = cv2.cvtColor(enhanced_frame, cv2.COLOR_BGR2HSV)

    # Get HSV values from trackbars
    def get_hsv_values(color):
        return [
            cv2.getTrackbarPos(f"Low-H {color}", "Trackbars"),
            cv2.getTrackbarPos(f"Low-S {color}", "Trackbars"),
            cv2.getTrackbarPos(f"Low-V {color}", "Trackbars")
        ], [
            cv2.getTrackbarPos(f"High-H {color}", "Trackbars"),
            cv2.getTrackbarPos(f"High-S {color}", "Trackbars"),
            cv2.getTrackbarPos(f"High-V {color}", "Trackbars")
        ]

    lower_red1, upper_red1 = np.array([0, *get_hsv_values("Red")[0][1:]]), np.array([10, *get_hsv_values("Red")[1][1:]])
    lower_red2, upper_red2 = np.array([170, *get_hsv_values("Red")[0][1:]]), np.array([180, *get_hsv_values("Red")[1][1:]])

    lower_green, upper_green = np.array(get_hsv_values("Green")[0]), np.array(get_hsv_values("Green")[1])
    lower_blue, upper_blue = np.array(get_hsv_values("Blue")[0]), np.array(get_hsv_values("Blue")[1])

    # Create masks
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Remove noise with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

    # Prevent overlapping detections
    mask_green[mask_red > 0] = 0
    mask_blue[mask_red > 0] = 0
    mask_blue[mask_green > 0] = 0  # Green takes priority over blue

    # Function to detect and label circles
    def detect_and_label(mask, color_name, frame, color):
        global prev_circles
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_circles = [(int(x), int(y), int(radius)) for contour in contours
                            if 4000 < cv2.contourArea(contour) < 30000
                            for (x, y), radius in [cv2.minEnclosingCircle(contour)]]

        if detected_circles:
            x, y, radius = max(detected_circles, key=lambda c: c[2])
            prev_circle = prev_circles[color_name]
            if prev_circle:
                x = int(alpha * x + (1 - alpha) * prev_circle[0])
                y = int(alpha * y + (1 - alpha) * prev_circle[1])
                radius = int(alpha * radius + (1 - alpha) * prev_circle[2])

            prev_circles[color_name] = (x, y, radius)
            cv2.circle(frame, (x, y), radius, color, 2)
            cv2.putText(frame, color_name, (x - radius, y - radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    detect_and_label(mask_red, "Red", frame, (0, 0, 255))
    detect_and_label(mask_green, "Green", frame, (0, 255, 0))
    detect_and_label(mask_blue, "Blue", frame, (255, 0, 0))

    cv2.imshow("Original Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
