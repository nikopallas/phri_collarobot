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

TEST_IMAGE_PATH = IMAGE_DIR / "real_img.png"


class MarkerDetectionError(ValueError):
    """Custom error for when required markers cannot be detected."""
    pass


# Global cache for the 6 layout markers (0-5)
LAST_KNOWN_MARKER_COORDS = {i: None for i in range(6)}


def detect_aruco_markers(gray_image, debug=False):
    """Detect ArUco markers using an extremely robust multi-pass strategy."""
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker_dict = {}
    all_rejected = []

    # Define various parameter sets
    param_sets = []

    # Pass 1: Standard sensitive
    p1 = aruco.DetectorParameters()
    p1.adaptiveThreshWinSizeMin = 3
    p1.adaptiveThreshWinSizeMax = 23
    p1.adaptiveThreshWinSizeStep = 5
    p1.minMarkerPerimeterRate = 0.005
    param_sets.append(("Sensitive", p1))

    # Pass 2: Blur-tolerant
    p2 = aruco.DetectorParameters()
    p2.polygonalApproxAccuracyRate = 0.05
    p2.errorCorrectionRate = 0.8
    param_sets.append(("Blurry", p2))

    # Pass 3: Low Contrast (Constant Thresh)
    p3 = aruco.DetectorParameters()
    p3.adaptiveThreshConstant = 7
    param_sets.append(("LowContrast", p3))

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

    def run_detection(img_to_check, scale, context="Global"):
        for p_name, p in param_sets:
            detector = aruco.ArucoDetector(aruco_dict, p)
            corners, ids, rejected = detector.detectMarkers(img_to_check)
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    mid = int(mid)
                    if mid not in marker_dict:
                        marker_dict[mid] = corners[i] / scale
                        if debug:
                            print(f"[{context}] Found ID {mid} (Scale:{scale}, Params:{p_name})")
            if rejected:
                all_rejected.extend([r / scale for r in rejected])

    # 1. Global passes
    for img_variant, name in [(gray_image, "raw"), (clahe.apply(gray_image), "clahe")]:
        for scale in [1, 2, 4]:
            if scale == 1:
                test_img = img_variant
            else:
                test_img = cv2.resize(img_variant, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            run_detection(test_img, scale, context=f"Global-{name}")

    # 2. Localized Refinement
    layout_ids = [0, 1, 2, 3, 4, 5]
    for cand in all_rejected:
        if all(i in marker_dict for i in layout_ids) and len(marker_dict) > 6:
            break
        pts = cand[0].astype(np.float32)
        x, y, w, h = cv2.boundingRect(pts)
        if w < 5 or h < 5:
            continue

        pad = int(max(w, h))
        y1, y2 = max(0, y - pad), min(gray_image.shape[0], y + h + pad)
        x1, x2 = max(0, x - pad), min(gray_image.shape[1], x + w + pad)

        crop = gray_image[y1:y2, x1:x2]
        crop_en = clahe.apply(crop)
        for l_scale in [4, 8]:
            up_crop = cv2.resize(crop_en, None, fx=l_scale, fy=l_scale, interpolation=cv2.INTER_LINEAR)
            for p_name, lp in [("Sensitive", p1), ("LowContrast", p3)]:
                l_detector = aruco.ArucoDetector(aruco_dict, lp)
                l_corners, l_ids, _ = l_detector.detectMarkers(up_crop)
                if l_ids is not None:
                    for i, mid in enumerate(l_ids.flatten()):
                        mid = int(mid)
                        if mid not in marker_dict:
                            c = l_corners[i] / l_scale
                            c[0][:, 0] += x1
                            c[0][:, 1] += y1
                            marker_dict[mid] = c
                            if debug:
                                print(f"[Local-Refine] Found ID {mid} (L-Scale:{l_scale}, Pass:{p_name})")
    return marker_dict, list(marker_dict.keys())


def marker_center(corner):
    corner = corner[0]
    return (int(corner[:, 0].mean()), int(corner[:, 1].mean()))


def get_workspace_corners(marker_dict):
    """Extract the 4 workspace corners in TL, TR, BR, BL order."""
    required = [0, 1, 5, 2]

    final_dict = {}
    missing = []
    for m in required:
        if m in marker_dict:
            final_dict[m] = marker_dict[m]
        elif LAST_KNOWN_MARKER_COORDS[m] is not None:
            final_dict[m] = LAST_KNOWN_MARKER_COORDS[m]
        else:
            missing.append(m)

    if missing:
        raise MarkerDetectionError(f"Missing layout markers: {missing}")

    # tl: Marker 0 top-left, tr: Marker 1 top-right, br: Marker 5 bottom-right, bl: Marker 2 bottom-left
    corners = np.array([
        final_dict[0][0][0],  # Top-Left of Marker 0
        final_dict[1][0][1],  # Top-Right of Marker 1
        final_dict[5][0][2],  # Bottom-Right of Marker 5
        final_dict[2][0][3]   # Bottom-Left of Marker 2
    ], dtype=np.float32)
    return corners


def compute_zones(marker_dict):
    """Compute three perspective-correct zone quadrilaterals."""
    required = [0, 1, 2, 3, 4, 5]

    final_dict = {}
    missing = []
    for m in required:
        if m in marker_dict:
            final_dict[m] = marker_dict[m]
        elif LAST_KNOWN_MARKER_COORDS[m] is not None:
            final_dict[m] = LAST_KNOWN_MARKER_COORDS[m]
        else:
            missing.append(m)

    if missing:
        raise MarkerDetectionError(f"Missing layout markers: {missing}")

    # Layout centers for relative spacing calculation
    c0 = np.array(marker_center(final_dict[0]))
    c3 = np.array(marker_center(final_dict[3]))
    c4 = np.array(marker_center(final_dict[4]))
    c1 = np.array(marker_center(final_dict[1]))

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
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    marker_dict, ids = detect_aruco_markers(gray, debug=debug)

    # Update global cache for layout markers
    for i in range(6):
        if i in marker_dict:
            LAST_KNOWN_MARKER_COORDS[i] = marker_dict[i]

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
