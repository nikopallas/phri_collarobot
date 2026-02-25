"""Quick debug script: opens camera, waits for autofocus, then loops capture + detect.

Usage:
    Online (camera):   python -m collarobot_perception.debug_capture
    Offline (image):   python -m collarobot_perception.debug_capture --offline
"""
import sys
import time
import cv2
from pathlib import Path

from collarobot_perception.detect_zones import get_state, MarkerDetectionError

IMAGE_DIR = Path(__file__).parent.parent / "images"
SAVED_FRAME_PATH = IMAGE_DIR / "debug_last_capture.png"

OFFLINE = "--offline" in sys.argv
CAM_ID = 1


def run_offline():
    print(f"Offline mode: loading {SAVED_FRAME_PATH}")
    frame = cv2.imread(str(SAVED_FRAME_PATH))
    if frame is None:
        print(f"ERROR: Could not load {SAVED_FRAME_PATH}")
        print("Run in online mode first to capture and save a frame.")
        return
    process_frame(frame)


def run_online():
    print(f"Opening camera {CAM_ID}...")
    cap = cv2.VideoCapture(CAM_ID, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    print("Waiting 5s for autofocus...")
    time.sleep(5)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Image capture failed, retrying...")
            time.sleep(0.5)
            continue

        # Save frame for offline use
        cv2.imwrite(str(SAVED_FRAME_PATH), frame)
        print(f"Captured frame: {frame.shape} (saved to {SAVED_FRAME_PATH.name})")

        if not process_frame(frame):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Done.")


def process_frame(frame):
    """Process a single frame. Returns False if user wants to quit."""
    from collarobot_perception.detect_zones import LAST_KNOWN_MARKER_COORDS
    
    # Reset internal state to ensure determinism for this specific capture
    for k in LAST_KNOWN_MARKER_COORDS:
        LAST_KNOWN_MARKER_COORDS[k] = None

    try:
        state = get_state(frame, debug=True)
        print("=== STATE ===")
        print(f"  Storage:  {state['storage']}")
        print(f"  Proposed: {state['proposed']}")
        print(f"  Accepted: {state['accepted']}")
        print(f"  Positions: {state['relative_positions']}")
    except MarkerDetectionError as e:
        print(f"Detection failed: {e}")
        cv2.imshow("Failed frame", frame)
        print("Press any key to retry, or 'q' to quit...")
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            return False
        cv2.destroyAllWindows()
        return True
    except Exception as e:
        print(f"Unexpected error: {e}")

    print("\nPress any key for next capture, 'q' to quit...")
    cv2.imshow("Captured", frame)
    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        return False
    cv2.destroyAllWindows()
    return True


if OFFLINE:
    run_offline()
else:
    run_online()
