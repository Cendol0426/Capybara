import cv2
import json
import os
from google import genai
from PIL import Image

try:
    api_key = os.environ.get("GOOGLE_API_KEY", "")
    ai_client = genai.Client(api_key=api_key)
    print(">>> Gemini API configured.")
except Exception as e:
    print(f"CRITICAL ERROR: {e}")
    ai_client = None

def build_prompt():
    return """
    Analyze this image from a robot's webcam. Look closely if there is a cube right below the robot.
    Look for a red or blue cube. If there are multiple, take the nearest one.
    Check if the cube has the correct target alphabet: B, C, E, M, R, U.
    Wrong alphabets are: H, N, O, P.

    Return ONLY a valid JSON object.
    {
        "is_a_cube": true,
        "is_correct_cube": true,
        "confidence_score": 0.95
    }
    """
def main():
    # Open camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("ERROR: Cannot open camera.")
        return

    print("\n" + "="*40)
    print(" VISION TEST MODE")
    print("="*40)

    while True:
        # Press Enter to capture image
        user_input = input("\nHold a cube in front of the camera, then press ENTER to test (or type 'q' to quit): ")
        
        if user_input.lower() == 'q':
            break

        print("Snapping photo...")
        
        # Refresh camera buffer to current 5 frames
        for _ in range(5):
            cap.read()
            
        # Take picture
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            continue

        # Save the image
        image_filename = "vision_test.jpg"
        cv2.imwrite(image_filename, frame)
        print(f"Saved picture as '{image_filename}'. Sending to Gemini...")

        # Convert to PIL
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb_frame)
        
        try:
            response = ai_client.models.generate_content(
                model="gemini-2.5-flash",
                contents=[build_prompt(), pil_image]
            )

            # Clean JSON
            raw_text = response.text.strip()
            if raw_text.startswith("```"):
                raw_text = raw_text.split("```")[1]
                if raw_text.lower().startswith("json"):
                    raw_text = raw_text[4:]
            
            result = json.loads(raw_text.strip())
            is_cube = result.get("is_a_cube", False)
            is_correct = result.get("is_correct_cube", False)
            confidence = result.get("confidence_score", 0.0)

            # Print results
            print("-" * 30)
            if is_correct:
                print(f"AI RESULT: CORRECT CUBE! (Confidence: {confidence:.2f})")
            else:
                print(f"AI RESULT: WRONG CUBE. (Confidence: {confidence:.2f})")
            print("-" * 30)

            return is_cube, is_correct

        except Exception as e:
            print(f"AI Error: {e}")

    # Cleanup
    cap.release()
    print("Done!")

