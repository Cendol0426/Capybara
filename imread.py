# letter_detector.py
# ============================================================
# Capture an image using Raspberry Pi camera,
# detect the main letter in the image,
# even if the image is rotated or upside down.
#
# REQUIREMENTS:
# pip install pillow google-genai
#
# ENVIRONMENT VARIABLE:
# export GOOGLE_API_KEY="your_api_key"
# ============================================================

import os
import subprocess
from PIL import Image
from google import genai

# CONFIG
CAPTURE_DIR = "captures"
os.makedirs(CAPTURE_DIR, exist_ok=True)

# GOOGLE AI SETUP
try:
    api_key = os.environ["GOOGLE_API_KEY"]
    ai_client = genai.Client(api_key=api_key)
    print(">>> Gemini API configured successfully.")
except KeyError:
    print("CRITICAL ERROR: GOOGLE_API_KEY not set.")
    exit()

# CAMERA FUNCTION
def capture_image(filename="letter.jpg"):
    filepath = os.path.join(CAPTURE_DIR, filename)

    cmd = [
        "fswebcam",
        "-r", "1280x720",
        "--no-banner",
        filepath
    ]

    try:
        print(f"[Camera] Capturing image...")
        subprocess.run(cmd, check=True, text=True, capture_output=True)
        print(f"[Camera] Saved → {filepath}")
        return filepath

    except subprocess.CalledProcessError as e:
        print(f"[Camera] Capture failed: {e.stderr}")
        return None

    except FileNotFoundError:
        print("[Camera] fswebcam not installed.")
        return None

# AI LETTER DETECTION
def detect_letter(image_path):

    try:
        img = Image.open(image_path)

    except Exception as e:
        print(f"[AI] Failed to open image: {e}")
        return None

    prompt = """
    Analyze this image.

    A single alphabet letter is shown in the image.
    The image may be:
    - rotated
    - upside down
    - sideways
    - tilted

    Identify the letter correctly regardless of orientation.

    Return ONLY:
    - one uppercase letter A-Z

    If no clear letter exists, return:
    UNKNOWN
    """

    try:
        print("[AI] Detecting letter...")

        response = ai_client.models.generate_content(
            model="gemini-2.5-flash",
            contents=[prompt, img]
        )

        result = response.text.strip()

        print(f"[AI] Result: {result}")

        return result

    except Exception as e:
        print(f"[AI] Error: {e}")
        return None

# MAIN
def main():

    image_path = capture_image()

    if not image_path:
        return

    detected_letter = detect_letter(image_path)

    print("\n==============================")
    print(f"Detected Letter: {detected_letter}")
    print("==============================")

# ============================================================

if __name__ == "__main__":
    main()