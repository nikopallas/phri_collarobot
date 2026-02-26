import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Dict, List, Tuple

from collarobot_perception.transformation import (
    create_homography_from_corners,
    transform_points,
)

IMAGE_PATH = "C:/Users/Ringer/PycharmProjects/phri_collarobot/collarobot_perception/images/optimal_image.jpeg"
OUTPUT_PATH = "zones_debug_result.jpg"

# Persistent cache so layout markers survive momentary detection failures
LAST_KNOWN_MARKER_COORDS = {i: None for i in range(6)}


class MarkerDetectionError(Exception):
    """Raised when required layout markers cannot be found."""
    pass


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def center_of(corner):
    """Centre of a single ArUco corner array (shape 1×4×2 or 4×2)."""
    return np.mean(corner.reshape(4, 2), axis=0)


def marker_center(corners):
    """Return the (x, y) integer centre of a marker's corner array."""
    c = center_of(corners)
    return (int(c[0]), int(c[1]))


def detect_aruco_markers(gray, debug=False):
    """
    Detect ArUco markers in a grayscale image.

    Returns:
        marker_dict: {id: corners_array}
        ids: list of detected ids
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejected = detector.detectMarkers(gray)

    marker_dict = {}
    flat_ids = []

    if ids is not None:
        for i, id_ in enumerate(ids.flatten()):
            marker_dict[int(id_)] = corners[i]
            flat_ids.append(int(id_))

    return marker_dict, flat_ids


def get_workspace_corners(marker_dict):
    """
    Compute workspace corner points from layout markers 0, 1, 2, 5.

    Returns (top-left, top-right, bottom-right, bottom-left) as numpy arrays.
    """
    required = [0, 1, 2, 5]
    for m in required:
        if m not in marker_dict:
            raise ValueError(f"Workspace corner marker {m} not detected")

    tl = np.array(marker_center(marker_dict[0]), dtype=np.float64)
    tr = np.array(marker_center(marker_dict[1]), dtype=np.float64)
    bl = np.array(marker_center(marker_dict[2]), dtype=np.float64)
    br = np.array(marker_center(marker_dict[5]), dtype=np.float64)

    return tl, tr, br, bl


# ---------------------------------------------------------------------------
# Zone computation
# ---------------------------------------------------------------------------

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
    corners = get_workspace_corners(final_dict)
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


# ---------------------------------------------------------------------------
# Marker-to-zone assignment
# ---------------------------------------------------------------------------

def point_in_zone(pt, polygon):
    """Check if a point is inside a quadrilateral zone polygon."""
    return cv2.pointPolygonTest(
        polygon.reshape(-1, 1, 2).astype(np.float32),
        (float(pt[0]), float(pt[1])), False
    ) >= 0


def assign_markers_to_zones(image, debug=False):
    """
    Assign markers to zones.

    Args:
        image: BGR or grayscale image
        debug: if True, returns annotated image

    Returns:
        result: dict {marker_id: zone_name} for markers > 5
        marker_dict: all detected markers
        debug_image (if debug=True)
    """
    gray = (cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if len(image.shape) == 3 else image)
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
                cv2.putText(debug_image, str(id_),
                            (center[0] + 5, center[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            if debug:
                cv2.circle(debug_image, center, 6, (255, 0, 0), -1)
                cv2.putText(debug_image, str(id_),
                            (center[0] + 5, center[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    if debug:
        return result, marker_dict, debug_image
    else:
        return result, marker_dict


# ---------------------------------------------------------------------------
# High-level state API
# ---------------------------------------------------------------------------

def get_state(image, debug=False) -> dict:
    """Analyzes an image and returns the current state of markers in zones."""
    storage, proposed, accepted, relative_positions = analyze_zones_state(
        image, debug=debug
    )
    return {
        "storage": storage,
        "proposed": proposed,
        "accepted": accepted,
        "relative_positions": relative_positions,
    }


def analyze_zones_state(
    image: cv2.Mat, debug=False
) -> Tuple[List, List, List, Dict]:
    """
    Full pipeline: detect markers, assign to zones, compute relative positions.
    """
    if debug:
        markers_in_zones, marker_dict, debug_img = assign_markers_to_zones(
            image, debug=True
        )
        print("Marker assignments:", markers_in_zones)
        cv2.imwrite("zones_debug_real.jpg", debug_img)
    else:
        markers_in_zones, marker_dict = assign_markers_to_zones(
            image, debug=False
        )

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
        H = create_homography_from_corners(
            np.array(corner_pixels, dtype=np.float32)
        )

        excluded_ids = [0, 1, 2, 3, 4, 5]
        marker_ids_to_transform = [
            mid for mid in marker_dict.keys() if mid not in excluded_ids
        ]

        if marker_ids_to_transform:
            points = np.array(
                [marker_center(marker_dict[mid])
                 for mid in marker_ids_to_transform],
                dtype=np.float32,
            )
            normalized_coords = transform_points(points, H)

            for i, mid in enumerate(marker_ids_to_transform):
                x_norm, y_norm = normalized_coords[i]
                relative_positions[int(mid)] = (
                    round(float(x_norm), 3),
                    round(float(y_norm), 3),
                )
    except (ValueError, KeyError) as e:
        if debug:
            print(f"Could not compute relative positions: {e}")

    return storage, proposed, accepted, relative_positions


# ---------------------------------------------------------------------------
# Standalone visualisation (original main)
# ---------------------------------------------------------------------------

def main():
    img = cv2.imread(IMAGE_PATH)
    orig = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected = detector.detectMarkers(gray)

    detected = {}

    # Store detected markers
    if ids is not None:
        for i, id_ in enumerate(ids.flatten()):
            detected[id_] = center_of(corners[i])
            cv2.circle(img, tuple(detected[id_].astype(int)), 8,
                       (0, 255, 0), -1)
            cv2.putText(img, f"ID {id_}",
                        tuple(detected[id_].astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    height, width = img.shape[:2]

    # Assign marker 4 if missing
    if 4 not in detected:
        print("Inferring marker 4 from rejected candidates")

        best_candidate = None
        best_score = 0

        x_min = detected[3][0] + 30 if 3 in detected else width * 0.4
        x_max = detected[1][0] - 30 if 1 in detected else width * 0.85

        for r in rejected:
            c = center_of(r)
            if x_min < c[0] < x_max and c[1] < height * 0.35:
                score = c[0]
                if score > best_score:
                    best_score = score
                    best_candidate = c

        if best_candidate is not None:
            detected[4] = best_candidate
            cv2.circle(img, tuple(best_candidate.astype(int)), 10,
                       (255, 0, 0), -1)
            cv2.putText(img, "ID 4 (inferred)",
                        tuple(best_candidate.astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # Infer marker 0 from geometry
    if 0 not in detected and 2 in detected and 3 in detected:
        print("Inferring marker 0 geometrically")
        y = detected[3][1]
        x = detected[2][0]
        detected[0] = np.array([x, y])
        cv2.circle(img, (int(x), int(y)), 10, (0, 0, 255), -1)
        cv2.putText(img, "ID 0 (inferred)",
                    (int(x), int(y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Draw vertical split lines
    if 3 in detected:
        x3 = int(detected[3][0])
        cv2.line(img, (x3, 0), (x3, height), (0, 255, 255), 3)

    if 4 in detected:
        x4 = int(detected[4][0])
        cv2.line(img, (x4, 0), (x4, height), (0, 255, 255), 3)

    # Draw upper line (marker 0 -> marker 1)
    if 0 in detected and 1 in detected:
        p0 = tuple(detected[0].astype(int))
        p1 = tuple(detected[1].astype(int))
        cv2.line(img, p0, p1, (0, 200, 255), 3)

    # Draw lower line (marker 2 -> marker 5)
    if 2 in detected and 5 in detected:
        p2 = tuple(detected[2].astype(int))
        p5 = tuple(detected[5].astype(int))
        cv2.line(img, p2, p5, (0, 200, 255), 3)

    # Draw workspace rectangle
    if all(k in detected for k in [0, 1, 2, 5]):
        pts = np.array([
            detected[0], detected[1],
            detected[5], detected[2]
        ], dtype=int)
        cv2.polylines(img, [pts], True, (255, 255, 0), 4)

    cv2.imwrite(OUTPUT_PATH, img)
    print("Saved to:", OUTPUT_PATH)


if __name__ == "__main__":
    main()