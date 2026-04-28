"""
Steps to Recognise Letter from Image:
1. Capture / Load Image
2. Preprocess Image 
3. Find Letter Blocks (Using Contours)
4. Fix Orientation
5. Crop the Region
6. Clean Image
7. Recognise Letter (Using OCR)
"""

import cv2
import numpy as np
import pytesseract

# 1. Capture / Load Image
def load_image(image_path):
    image = cv2.imread(image_path)
    return image

# 2. Preprocess Image
def preprocess(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Convert to grayscale
    blur = cv2.GaussianBlur(gray, (5, 5), 0) #Blur Image to reduce noise

    # Adaptive threshold (better for lighting changes)
    thresh = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11, 2
    ) #Adaptive thresholding to handle varying lighting conditions
    #Will do threshold by comparing local area to get black and white image
    return thresh

# 3. Find Letter Blocks (Using Contours)
def find_blocks(thresh):
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#Find contours in the thresholded image

    blocks = []
    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > 500:  # filter noise
            x, y, w, h = cv2.boundingRect(cnt)

            aspect_ratio = w / float(h)

            # roughly square (letter block assumption)
            if 0.5 < aspect_ratio < 2.0:
                blocks.append((cnt, x, y, w, h)) # Store contour and bounding box info for valid blocks

    return blocks

# 4. Fix Orientation
def straighten(image, cnt):
    rect = cv2.minAreaRect(cnt) #Get the minimum area rectangle for the contour
    angle = rect[-1]

    if angle < -45:
        angle = 90 + angle #Adjust angle to rotate correctly

    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)

    M = cv2.getRotationMatrix2D(center, angle, 1.0) #Get the rotation matrix for the calculated angle
    rotated = cv2.warpAffine(image, M, (w, h)) #Rotate the image using the rotation matrix

    return rotated


# 5. Crop the Region
def crop(image, x, y, w, h):
    return image[y:y+h, x:x+w]

# 6. Clean Image
def clean(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

    # Morphological cleanup
    kernel = np.ones((3, 3), np.uint8)
    clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel) #Close small gaps in the letters to improve OCR accuracy

    return clean

# 7. Recognise Letter (Using OCR)
def recognize(image):
    config = r'--psm 10 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    text = pytesseract.image_to_string(image, config=config)
    return text.strip()

# Main Function
def main(): 
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not access webcam")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break  

        thresh = preprocess(frame)
        blocks = find_blocks(thresh)

        for cnt, x, y, w, h in blocks:
            straightened = straighten(frame, cnt)
            cropped = crop(straightened, x, y, w, h)
            cleaned = clean(cropped)
            letter = recognize(cleaned)

            print(f"Recognised Letter: {letter}")

            cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":    main()
