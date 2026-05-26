import cv2
from google import genai
from PIL import Image
import os
import json
import serial
import time

MEGA_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

try:
    api_key = os.environ.get("GOOGLE_API_KEY", "AIzaSyADPa8cFlTQ-hduit4LTY2kQllWZ0ZDn60")
    ai_client = genai.Client(api_key=api_key)
    print(">>> Gemini API configured.")
except Exception as e:
    print(f"CRITICAL ERROR: {e}")
    ai_client = None

try:
    mega = serial.Serial(MEGA_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Give Arduino time to reboot on connection
    print(f">>> Connected to Arduino on {MEGA_PORT}")
except Exception as e:
    print(f"Serial Error: Could not connect to Arduino. {e}")
    mega = None

# Open the system's default camera (usually index 0)
cap = cv2.VideoCapture(0)

def build_prompt():
    return """
    Analyze this image from a robot's webcam. Look closely if there is a cube right below the robot.
    Look for a red or blue cube.
    Check if the cube has the correct target alphabet: B, C, E, M, R, U.
    Cube with wrong target alphabets: H, N, O, P.

    Return ONLY a valid JSON object.
    {
        "is_a_cube" : true,
        "is_correct_cube": true,
        "confidence_score": 0.95
    }
    """

def detect_cube(frame):
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb_frame)
    
    try:
        response = ai_client.models.generate_content(
            model="gemini-2.5-flash",
            contents=[build_prompt(), pil_image]
        )

        raw_text = response.text.strip()
        if raw_text.startswith("```"):
            raw_text = raw_text.split("```")[1]
            if raw_text.lower().startswith("json"):
                raw_text = raw_text[4:]
        
        result = json.loads(raw_text.strip())
        is_cube = result.get("is_a_cube", False)
        is_correct = result.get("is_correct_cube", False)
        confidence = result.get("confidence_score", 0.0)

        if is_cube:
            if is_correct:
                print(f"Correct Cube. (Confidence: {confidence:.2f})")
                if mega:
                    mega.write(b"M:GRAB\n")
                    print("-> Sent <M:GRAB> to Mega")
            else:
                print(f"Wrong cube. (Confidence: {confidence:.2f})")
                if mega:
                    mega.write(b"M:IGNORE\n")
                    print("-> Sent <M:IGNORE> to Mega")
        else:
            print(f"Not a cube. (Confidence: {confidence:.2f})")
            if mega:
                mega.write(b"M:IGNORE\n")
                print("-> Sent <M:IGNORE> to Mega")

        return is_cube, is_correct

    except Exception as e:
        print(f"AI Error: {e}")
        # Default to ignore if AI fails, so the robot doesn't get stuck
        if mega:
            mega.write(b"M:IGNORE\n")

def main():
    print(">>> Autonomous Brain running. Waiting for Arduino 'ARRIVED' signal...")
    
    while True:
        last_capture_time = time.time()
        test_interval_seconds = 5.0

        # 1. Constantly read the camera to keep the hardware buffer fresh.
        # If we don't do this, the camera will give us a 10-second-old picture when we ask for one!
        ret, frame = cap.read()
        if not ret:
            continue

        # Optional: display what the robot sees (can be commented out to save CPU)
        cv2.imshow("Robot Vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        current_time = time.time()
        if current_time - last_capture_time >= test_interval_seconds:
            print(f"\n--- Auto-Test Triggered ({test_interval_seconds}s interval) ---")
            is_cube, is_correct = detect_cube(frame)
            
            # Reset the timer
            last_capture_time = current_time

        # 2. Check if the Arduino has sent us a message
        if mega and mega.in_waiting > 0:
            try:
                # Read the incoming message
                msg = mega.readline().decode('utf-8').strip()
                
                if msg:
                    print(f"Mega says: {msg}")

                # 3. THE TRIGGER
                if "STATUS:ARRIVED" in msg:
                    # We arrived at a cube! Analyze the CURRENT frame.
                    detect_cube(frame)
                    
            except UnicodeDecodeError:
                pass # Ignore weird serial noise

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if ai_client:
        main()

        
