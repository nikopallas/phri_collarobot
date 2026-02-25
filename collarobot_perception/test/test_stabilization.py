import pytest
import numpy as np
from collarobot_perception.detect_zones import (
    get_workspace_corners, compute_zones, MarkerDetectionError, LAST_KNOWN_MARKER_COORDS
)


def test_cache_fallback():
    # Reset cache
    for i in range(6):
        LAST_KNOWN_MARKER_COORDS[i] = None

    # Simulate first detection where all markers are present
    mock_corner = np.array([[[10, 10], [20, 10], [20, 20], [10, 20]]], dtype=np.float32)
    full_marker_dict = {i: mock_corner for i in range(6)}

    # Pre-fill cache (simulating a previous successful detection)
    for i in range(6):
        LAST_KNOWN_MARKER_COORDS[i] = full_marker_dict[i]

    # Now simulate a detection where marker 0 is missing
    partial_marker_dict = {i: mock_corner for i in range(1, 6)}

    # Should not raise error because marker 0 is in cache
    corners = get_workspace_corners(partial_marker_dict)
    assert corners is not None

    # Should not raise error for compute_zones
    zones = compute_zones(partial_marker_dict)
    assert "Zone1" in zones


def test_cache_failure():
    # Reset cache
    for i in range(6):
        LAST_KNOWN_MARKER_COORDS[i] = None

    # Detect with missing marker and empty cache
    with pytest.raises(MarkerDetectionError):
        get_workspace_corners({})
