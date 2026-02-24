from typing import Tuple, List
from .take_image import capture_frame
from .detect_zones import get_state as analyze_zones_state


def get_state(debug=False) -> Tuple[List, List, List]:
    """
    Captures a frame from the webcam and returns the current state of markers in zones.
    Returns:
        Tuple[storage, proposed, accepted]: Lists of marker IDs in each zone.
    """
    frame = capture_frame()
    if frame is None:
        print("Error: Could not capture frame from webcam.")
        return [], [], []

    return analyze_zones_state(frame, debug=debug)


if __name__ == "__main__":
    # Test the real-time detection
    storage, proposed, accepted = get_state(debug=True)
    print("--- Real-time State ---")
    print(f"Storage:  {storage}")
    print(f"Proposed: {proposed}")
    print(f"Accepted: {accepted}")
