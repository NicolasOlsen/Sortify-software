import cv2
import numpy as np

def detect_balls_webcam():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Open webcam

    # Define refined color ranges (HSV)
    color_ranges = {
        'red1': [(0, 90, 140), (10, 255, 255)],   # First red range
        'red2': [(170, 90, 140), (180, 255, 255)],  # Second red range
        'blue': [(100, 70, 80), (120, 255, 255)],
        'green': [(24, 70, 85), (60, 255, 255)]
    }

    # Define display colors for circles
    display_colors = {
        'red': (0, 0, 255),   # Unified Red Color
        'blue': (255, 0, 0),
        'green': (0, 255, 0)
    }

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        blurred = cv2.GaussianBlur(frame, (5, 5), 2)  # Reduce noise
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # **Merge red1 and red2 into a single mask**
        mask_red1 = cv2.inRange(hsv, np.array(color_ranges['red1'][0]), np.array(color_ranges['red1'][1]))
        mask_red2 = cv2.inRange(hsv, np.array(color_ranges['red2'][0]), np.array(color_ranges['red2'][1]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)  # Merge both red masks

        # Initialize masks dictionary
        masks = {
            'red': mask_red,
            'blue': cv2.inRange(hsv, np.array(color_ranges['blue'][0]), np.array(color_ranges['blue'][1])),
            'green': cv2.inRange(hsv, np.array(color_ranges['green'][0]), np.array(color_ranges['green'][1]))
        }

        for color, mask in masks.items():
            # Morphological transformations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Removes small white spots
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # Closes small gaps

            # Apply Gaussian Blur for smoother Hough Circles detection
            blurred_mask = cv2.GaussianBlur(mask, (15, 15), 0)

            # Show each mask in a separate window
            cv2.imshow(f"Mask - {color}", mask)

            # Detect circles using Hough Circles
            circles = cv2.HoughCircles(
                blurred_mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                param1=70, param2=40, minRadius=10, maxRadius=100
            )

            if circles is not None:
                circles = np.uint16(np.around(circles))  # Convert to integer values
                for i in circles[0, :]:
                    center = (i[0], i[1])
                    radius = i[2]

                    if radius > 10:  # Ignore very small noise
                        cv2.circle(frame, center, radius, display_colors[color], 2)
                        cv2.putText(frame, color, (center[0] - 10, center[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, display_colors[color], 2)

        # Show the detected circles on the original frame
        cv2.imshow("Detected Balls", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the webcam-based ball detection
detect_balls_webcam()
