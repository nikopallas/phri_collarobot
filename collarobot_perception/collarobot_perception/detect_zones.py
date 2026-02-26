import cv2
import cv2.aruco as aruco
import numpy as np
from pathlib import Path
from typing import Tuple, List, Dict

from collarobot_perception.transformation import (
    create_homography_from_corners, transform_points
)

# Use your old image path
IMAGE_DIR = Path(__file__).parent.parent / "images"
BASE_IMAGE_PATH = IMAGE_DIR / "base_template.jpeg"

TEST_IMAGE_PATH = Path("~/collarobot_ws/src/collarobot_perception/images/three_objects.png").expanduser()


class MarkerDetectionError(ValueError):
    """Custom error for when required markers cannot be detected."""
    pass


# Global cache for all marker positions (persists across frames)
LAST_KNOWN_MARKER_COORDS = {i: None for i in range(25)}


def _make_corners(cx, cy, size=15):
    """Create a fake marker corner array from a center point and half-size."""
    s = size
    return np.array([[[cx - s, cy - s], [cx + s, cy - s],
                      [cx + s, cy + s], [cx - s, cy + s]]], dtype=np.float32)


# Hardcoded layout marker positions (camera is fixed at 1920x1080)
# Top edge left→right: 0 (TL), 3 (zone divider), 4 (zone divider), 1 (TR)
# Bottom edge left→right: 2 (BL), 5 (BR)
LAYOUT_POSITIONS = {
    0: (820, 27),
    3: (1050, 280),
    4: (1330, 260),
    1: (1500, 243),
    2: (825, 730),
    5: (1525, 710),
}


def detect_aruco_markers(image, debug=False):
    """Detect ArUco markers: layout markers hardcoded, ingredients detected."""
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker_dict = {}
    MAX_VALID_ID = 24
    SCALE = 4

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

    # --- Phase 1: Hardcoded layout markers (0-5) ---
    for mid, (cx, cy) in LAYOUT_POSITIONS.items():
        marker_dict[mid] = _make_corners(cx, cy)
        if debug:
            print(f"[Layout] ID {mid} at ({cx}, {cy})")

    # --- Phase 2: Detect ingredient markers (6-24) ---
    # Crop to the board region defined by layout markers (with padding)
    all_x = [p[0] for p in LAYOUT_POSITIONS.values()]
    all_y = [p[1] for p in LAYOUT_POSITIONS.values()]
    pad = 50
    x1 = max(0, min(all_x) - pad)
    y1 = max(0, min(all_y) - pad)
    x2 = min(image.shape[1], max(all_x) + pad)
    y2 = min(image.shape[0], max(all_y) + pad)

    board = image[y1:y2, x1:x2] if len(image.shape) == 3 else gray[y1:y2, x1:x2]
    board_gray = gray[y1:y2, x1:x2]

    p = aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 23
    p.adaptiveThreshWinSizeStep = 3
    p.minMarkerPerimeterRate = 0.003
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    detector = aruco.ArucoDetector(aruco_dict, p)

    for name, base in [("color", board), ("clahe", clahe.apply(board_gray))]:
        up = cv2.resize(base, None, fx=SCALE, fy=SCALE,
                        interpolation=cv2.INTER_LINEAR)
        corners, ids, _ = detector.detectMarkers(up)
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                mid = int(mid)
                if mid <= MAX_VALID_ID and mid not in marker_dict:
                    c = corners[i] / SCALE
                    c[0][:, 0] += x1
                    c[0][:, 1] += y1
                    marker_dict[mid] = c
                    if debug:
                        print(f"[Detect-{name}] Found ID {mid}")

    # Update cache with freshly detected ingredient positions
    for mid, corners in marker_dict.items():
        if mid > 5:
            LAST_KNOWN_MARKER_COORDS[mid] = corners

    # Fill in missing ingredient markers from cache
    for mid in range(6, 25):
        if mid not in marker_dict and LAST_KNOWN_MARKER_COORDS[mid] is not None:
            marker_dict[mid] = LAST_KNOWN_MARKER_COORDS[mid]
            if debug:
                print(f"[Cache] ID {mid} from previous frame")

    if debug:
        found = sorted(marker_dict.keys())
        print(f"Detection complete: found={found}")

    return marker_dict, list(marker_dict.keys())


def marker_center(corner):
    corner = corner[0]
    return (int(corner[:, 0].mean()), int(corner[:, 1].mean()))


def get_workspace_corners(marker_dict):
    """Extract the 4 workspace corners in TL, TR, BR, BL order."""
    corners = np.array([
        marker_dict[0][0][0],  # Top-Left of Marker 0
        marker_dict[1][0][1],  # Top-Right of Marker 1
        marker_dict[5][0][2],  # Bottom-Right of Marker 5
        marker_dict[2][0][3]   # Bottom-Left of Marker 2
    ], dtype=np.float32)
    return corners


def compute_zones(marker_dict):
    """Compute three perspective-correct zone quadrilaterals."""
    c0 = np.array(marker_center(marker_dict[0]))
    c3 = np.array(marker_center(marker_dict[3]))
    c4 = np.array(marker_center(marker_dict[4]))
    c1 = np.array(marker_center(marker_dict[1]))

    # Outer edges of the workspace
    corners = get_workspace_corners(marker_dict)
    w_tl, w_tr, w_br, w_bl = corners

    # Proportional positions of markers 3 & 4 along the top edge centers
    mid_top_len = np.linalg.norm(c1 - c0)
    t3 = np.linalg.norm(c3 - c0) / mid_top_len
    t4 = np.linalg.norm(c4 - c0) / mid_top_len

    # Points along the top boundary
    top_v3 = w_tl + t3 * (w_tr - w_tl)
    top_v4 = w_tl + t4 * (w_tr - w_tl)

    # Points along the bottom boundary
    bot_v3 = w_bl + t3 * (w_br - w_bl)
    bot_v4 = w_bl + t4 * (w_br - w_bl)

    def quad(tl, tr, br, bl):
        return np.array([tl, tr, br, bl], dtype=np.int32)

    zones = {
        "Zone1": quad(w_tl, top_v3, bot_v3, w_bl),
        "Zone2": quad(top_v3, top_v4, bot_v4, bot_v3),
        "Zone3": quad(top_v4, w_tr, w_br, bot_v4),
    }

    return zones


def point_in_zone(pt, polygon):
    """Check if a point is inside a quadrilateral zone polygon."""
    return cv2.pointPolygonTest(polygon.reshape(-1, 1, 2).astype(np.float32),
                                (float(pt[0]), float(pt[1])), False) >= 0


def assign_markers_to_zones(image, debug=False):
    """
    Assign markers to zones.
    Args:
        image: BGR or grayscale image
        debug: if True, returns annotated image
    Returns:
        result: dict {marker_id: zone_name} for markers > 4
        marker_dict: all detected markers
        debug_image (if debug=True)
    """
    marker_dict, ids = detect_aruco_markers(image, debug=debug)

    if not marker_dict:
        if debug:
            return {}, {}, image.copy()
        return {}, {}

    zones = compute_zones(marker_dict)
    excluded_ids = [0, 1, 2, 3, 4, 5]

    result = {}
    debug_image = image.copy() if debug else None

    if debug:
        for zone_name, poly in zones.items():
            cv2.polylines(debug_image, [poly], True, (0, 255, 0), 2)
            tx, ty = poly[0]
            cv2.putText(debug_image, zone_name, (tx + 5, ty + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    for id_ in ids:
        if id_ in excluded_ids:
            continue
        center = marker_center(marker_dict[id_])
        assigned_zone = None
        for zone_name, poly in zones.items():
            if point_in_zone(center, poly):
                assigned_zone = zone_name
                break
        if assigned_zone:
            result[int(id_)] = assigned_zone
            if debug:
                cv2.circle(debug_image, center, 6, (0, 0, 255), -1)
                cv2.putText(debug_image, str(id_), (center[0]+5, center[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            if debug:
                cv2.circle(debug_image, center, 6, (255, 0, 0), -1)
                cv2.putText(debug_image, str(id_), (center[0]+5, center[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    if debug:
        return result, marker_dict, debug_image
    else:
        return result, marker_dict


def get_state(image, debug=False) -> dict:
    """Analyzes an image and returns the current state of markers in zones."""
    storage, proposed, accepted, relative_positions = analyze_zones_state(image, debug=debug)

    return {
        "storage": storage,
        "proposed": proposed,
        "accepted": accepted,
        "relative_positions": relative_positions
    }


def analyze_zones_state(image: cv2.Mat, debug=False) -> Tuple[List, List, List, Dict]:
    if debug:
        markers_in_zones, marker_dict, debug_img = assign_markers_to_zones(image, debug=True)
        print("Marker assignments:", markers_in_zones)
        cv2.imwrite("zones_debug_real.jpg", debug_img)
    else:
        markers_in_zones, marker_dict = assign_markers_to_zones(image, debug=False)

    storage = []
    proposed = []
    accepted = []

    for marker, zone in markers_in_zones.items():
        if zone == "Zone1":
            storage.append(marker)
        elif zone == "Zone2":
            proposed.append(marker)
        elif zone == "Zone3":
            accepted.append(marker)

    relative_positions = {}
    try:
        corner_pixels = get_workspace_corners(marker_dict)
        H = create_homography_from_corners(corner_pixels)

        excluded_ids = [0, 1, 2, 3, 4, 5]
        marker_ids_to_transform = [mid for mid in marker_dict.keys() if mid not in excluded_ids]

        if marker_ids_to_transform:
            points = np.array([marker_center(marker_dict[mid]) for mid in marker_ids_to_transform], dtype=np.float32)
            normalized_coords = transform_points(points, H)

            for i, mid in enumerate(marker_ids_to_transform):
                x_norm, y_norm = normalized_coords[i]
                relative_positions[int(mid)] = (round(float(x_norm), 3), round(float(y_norm), 3))
    except (ValueError, KeyError) as e:
        if debug:
            print(f"Could not compute relative positions: {e}")

    return storage, proposed, accepted, relative_positions


# --- Example usage ---
if __name__ == "__main__":
    img = cv2.imread(str(TEST_IMAGE_PATH))
    state = get_state(img)
    print("Storage:", state["storage"])
    print("Proposed:", state["proposed"])
    print("Accepted:", state["accepted"])
    print("Relative Positions:", state["relative_positions"])
