
import cv2
from pathlib import Path
from collarobot_perception.detect_zones import (
    assign_markers_to_zones, analyze_zones_state as get_state
)

# Path to the example image
IMAGE_DIR = Path(__file__).parent.parent / "images"
TEST_IMAGE_PATH = IMAGE_DIR / "three_objects.png"

# Expected marker assignments for the example image
EXPECTED_ASSIGNMENTS = {
    12: 'Zone1',
    13: 'Zone3',
    17: 'Zone2',
    9: 'Zone1',
}


def test_marker_zone_assignment():
    # Load the test image
    img = cv2.imread(str(TEST_IMAGE_PATH))
    assert img is not None, "Test image could not be loaded"

    # Run the assignment
    assignments, _ = assign_markers_to_zones(img, debug=False)

    # Check that all expected markers are detected and assigned correctly
    for marker_id, expected_zone in EXPECTED_ASSIGNMENTS.items():
        assert marker_id in assignments, f"Marker {marker_id} not detected"
        assert assignments[marker_id] == expected_zone, (
            f"Marker {marker_id} assigned to {assignments[marker_id]}, "
            f"expected {expected_zone}"
        )

    # Optional: check no extra markers are included
    extra_markers = set(assignments.keys()) - set(EXPECTED_ASSIGNMENTS.keys())
    assert not extra_markers, f"Unexpected markers detected: {extra_markers}"


def test_get_state_sorting():
    # Load the test image
    img = cv2.imread(str(TEST_IMAGE_PATH))
    assert img is not None, "Test image could not be loaded"

    # Run get_state (analyze_zones_state)
    storage, proposed, accepted, relative_positions = get_state(img, debug=False)

    # Based on EXPECTED_ASSIGNMENTS:
    # 12 -> Zone1 (storage)
    # 9 -> Zone1 (storage)
    # 17 -> Zone2 (proposed)
    # 13 -> Zone3 (accepted)
    assert set(storage) == {12, 9}
    assert set(proposed) == {17}
    assert set(accepted) == {13}
