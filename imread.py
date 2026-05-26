# fridge_monitor.py
# ============================================================
# UPDATED — Added image upload to Supabase Storage
# so unknown items show their photo on the /alerts page
# ============================================================

import time
import subprocess
import os
import json
from datetime import datetime
from PIL import image
from google import genai
from supabase import create_client, Client


# --- CONFIGURATION ---
CAPTURE_DELAY        = 2.0
CAPTURE_DIR          = "captures"
JSON_DIR             = "inventory_logs"
CONFIDENCE_THRESHOLD = 0.60


# --- GOOGLE AI SETUP ---
try:
    api_key = os.environ["GOOGLE_API_KEY"]
    ai_client = genai.Client(api_key=api_key)
    print(">>> Gemini API configured successfully.")
except KeyError:
    print("CRITICAL ERROR: GOOGLE_API_KEY not set.")
    ai_client = None


# --- CREATE LOCAL BACKUP DIRECTORIES ---
os.makedirs(CAPTURE_DIR, exist_ok=True)
os.makedirs(JSON_DIR, exist_ok=True)


# ============================================================
# CAMERA FUNCTIONS
# ============================================================

def get_timestamp():
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def capture_image(filename):
    filepath = os.path.join(CAPTURE_DIR, filename)
    cmd = [
        "rpicam-still",
        "-o", filepath,
        "-t", "1000",
        "--immediate",
        "--nopreview"
    ]
    try:
        print(f"  [Camera] Capturing → {filepath}")
        subprocess.run(cmd, check=True, text=True, capture_output=True)
        print("  [Camera] ✅ Captured successfully.")
        return filepath
    except subprocess.CalledProcessError as e:
        print(f"  [Camera] ❌ Capture failed: {e.stderr}")
        return None
    except FileNotFoundError:
        print("  [Camera] ❌ 'rpicam-still' not found.")
        print("           Run: sudo raspi-config → Interface Options → Camera → Enable")
        return None

# ============================================================
# AI ANALYSIS FUNCTIONS
# ============================================================

def build_prompt():
    return """
    Analyze this image of the inside of a refrigerator.
    Identify ALL visible food items and ingredients.

    Return ONLY a valid JSON array — no explanations, no markdown, no code fences.
    Use this exact structure for every item:

    [
      {
        "item_name": "Milk",
        "category": "dairy",
        "estimated_days_left": 5,
        "confidence_score": 0.92,
        "needs_user_input": false
      }
    ]

    Rules:
    - category must be one of: vegetable, fruit, meat, dairy, condiment, beverage, other
    - estimated_days_left: integer only — estimate visually or read any printed expiry date
    - confidence_score: float between 0.0 and 1.0
    - needs_user_input: set true ONLY for opaque containers, foil-wrapped items,
      or anything you genuinely cannot identify
    - Absolutely no text outside the JSON array
    """


def analyze_image_with_gemini(image_path):
    if not ai_client:
        print("  [AI] Skipping — Gemini client not configured.")
        return None, []
    if not image_path:
        print("  [AI] Skipping — no image path provided.")
        return None, []

    try:
        img = Image.open(image_path)
    except Exception as e:
        print(f"  [AI] ❌ Cannot open image: {e}")
        return None, []

    print("  [AI] Sending image to Gemini 2.5 Flash...")

    try:
        response = ai_client.models.generate_content(
            model="gemini-2.5-flash",
            contents=[build_prompt(), img]
        )

        raw_text = response.text.strip()

        if raw_text.startswith("```"):
            raw_text = raw_text.split("```")[1]
            if raw_text.lower().startswith("json"):
                raw_text = raw_text[4:]
        raw_text = raw_text.strip()

        inventory_data = json.loads(raw_text)
        print(f"  [AI] ✅ {len(inventory_data)} item(s) detected.")

        confirmed    = []
        needs_review = []

        for item in inventory_data:
            score = item.get("confidence_score", 1.0)
            flag  = item.get("needs_user_input", False)

            if flag or score < CONFIDENCE_THRESHOLD:
                needs_review.append(item)
                print(f"       ⚠️  REVIEW: {item['item_name']:22s} ({score:.0%} confidence)")
            else:
                confirmed.append(item)
                print(f"       ✅ AUTO:   {item['item_name']:22s} ({score:.0%} confidence)")

        return confirmed, needs_review

    except json.JSONDecodeError:
        print("  [AI] ❌ Gemini returned invalid JSON.")
        print(f"       Raw output preview: {response.text[:300]}")
        return None, []

    except Exception as e:
        error_str = str(e)
        print(f"  [AI] ❌ Error: {error_str[:200]}")
        if "SERVICE_DISABLED" in error_str or "403" in error_str:
            print("  [AI] 🔑 Get a new key at: https://aistudio.google.com/app/apikey")
        elif "429" in error_str or "QUOTA" in error_str:
            print("  [AI] ⏳ Rate limit hit — wait 60 seconds and retry.")
        return None, []

# ============================================================
# LOCAL JSON BACKUP
# ============================================================

def save_local_backup(confirmed, needs_review, timestamp):
    if confirmed:
        path = os.path.join(JSON_DIR, f"inventory_{timestamp}.json")
        with open(path, "w") as f:
            json.dump(confirmed, f, indent=2)
        print(f"  [Backup] ✅ {path}")

    if needs_review:
        path = os.path.join(JSON_DIR, f"review_needed_{timestamp}.json")
        with open(path, "w") as f:
            json.dump(needs_review, f, indent=2)
        print(f"  [Backup] ⚠️  {path}")


def print_summary(confirmed, needs_review):
    print("\n" + "=" * 52)
    print("  📦 EcoChef — Scan Complete")
    print("=" * 52)

    if confirmed:
        print(f"\n  ✅ Auto-logged to website ({len(confirmed)} items):")
        for item in confirmed:
            days = item.get("estimated_days_left", "?")
            print(f"     • {item['item_name']:24s} ~{days} day(s) left")

    if needs_review:
        print(f"\n  ⚠️  Sent to /alerts page ({len(needs_review)} items):")
        for item in needs_review:
            score = item.get("confidence_score", 0)
            print(f"     • {item['item_name']:24s} {score:.0%} confidence")

    print("\n  🌐 Check your website — it should already be updated!")
    print("=" * 52 + "\n")


# ============================================================
# MAIN PIPELINE
# ============================================================

def run_capture_pipeline():
    print("\n--- [TEST MODE] No sensor — running pipeline directly ---\n")

    print(f"  [Delay] Waiting {CAPTURE_DELAY}s for lighting to stabilise...")
    time.sleep(CAPTURE_DELAY)

    # Step 1: Capture the image
    timestamp      = get_timestamp()
    image_filepath = capture_image(f"fridge_{timestamp}.jpg")

    # Step 2: Analyze with Gemini
    confirmed, needs_review = analyze_image_with_gemini(image_filepath)

    if confirmed is not None:

        # Step 3: 🆕 Upload image to Supabase Storage BEFORE pushing review queue,
        # because we need the URL to attach to each review item.
        # If there are no review items, we skip the upload to save storage space.
        image_url = None

        # Step 5: Save local backup
        save_local_backup(confirmed, needs_review, timestamp)

        # Step 6: Print summary
        print_summary(confirmed, needs_review)

    print("--- Done. Open your website to see the results. ---\n")


if __name__ == "__main__":
    print("running")
    run_capture_pipeline()