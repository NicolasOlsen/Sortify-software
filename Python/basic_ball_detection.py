import cv2
import numpy as np

def detect_balls_webcam():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Open webcam

    # Define refined color ranges (HSV)
    color_ranges = {
        'red': [(0, 120, 70), (10, 255, 255)],
        'blue': [(100, 150, 50), (130, 255, 255)],  # More precise blue
        'green': [(36, 50, 50), (86, 255, 255)]
    }

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        blurred = cv2.GaussianBlur(frame, (9, 9), 2)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Morphological transformations to reduce noise
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Removes small white spots
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) # Closes small gaps

            # Detect contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                area = cv2.contourArea(contour)

                # Check if the contour is circular
                if len(approx) > 8 and area > 500:
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)

                    if radius > 10:  # Ignore small noise detections
                        cv2.circle(frame, center, radius, (0, 255, 0), 2)
                        cv2.putText(frame, color, (center[0] - 10, center[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Show results
        cv2.imshow("Detected Balls", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Run the webcam-based ball detection
detect_balls_webcam()
