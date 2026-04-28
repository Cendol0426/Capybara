import cv2
import numpy as np

def nothing(x):
    pass

def detect_yellow_square():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not access webcam")
        return

    cv2.namedWindow("Trackbars")

    # Yellow HSV initial values
    cv2.createTrackbar("LH", "Trackbars", 20, 180, nothing)
    cv2.createTrackbar("UH", "Trackbars", 35, 180, nothing)
    cv2.createTrackbar("LS", "Trackbars", 100, 255, nothing)
    cv2.createTrackbar("US", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("LV", "Trackbars", 100, 255, nothing)
    cv2.createTrackbar("UV", "Trackbars", 255, 255, nothing)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Trackbar values
        l_h = cv2.getTrackbarPos("LH", "Trackbars")
        u_h = cv2.getTrackbarPos("UH", "Trackbars")
        l_s = cv2.getTrackbarPos("LS", "Trackbars")
        u_s = cv2.getTrackbarPos("US", "Trackbars")
        l_v = cv2.getTrackbarPos("LV", "Trackbars")
        u_v = cv2.getTrackbarPos("UV", "Trackbars")

        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower, upper)

        # Clean noiseq
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000: #ignore small noise area
                continue

            # Approximate shape
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            # Check for quadrilateral (square/rectangle)
            if len(approx) == 4:
                # Draw contour
                cv2.drawContours(frame, [approx], 0, (0, 255, 255), 3)

                # Compute centroid
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"]) #algorithm
                cy = int(M["m01"] / M["m00"])

                # Draw midpoint
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

                # Label
                cv2.putText(frame, f"Center: ({cx},{cy})",
                            (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)

                print(f"Midpoint -> X: {cx}, Y: {cy}")

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_yellow_square()