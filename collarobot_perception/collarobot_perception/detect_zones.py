import cv2
import cv2.aruco as aruco
import numpy as np
from pathlib import Path

# Use your old image path
IMAGE_DIR = Path(__file__).parent.parent / "images"
BASE_IMAGE_PATH = IMAGE_DIR / "base_template.jpeg"

TEST_IMAGE_PATH = IMAGE_DIR / "three_objects.png"


def detect_aruco_markers(gray_image, scale=2):
    """Detect ArUco markers in a grayscale image.

    The image is upscaled by *scale* before detection so that
    small or distant markers (like marker 2) are reliably decoded.
    Returned corner coordinates are mapped back to the original
    resolution.
    """
    upscaled = cv2.resize(
        gray_image, None, fx=scale, fy=scale,
        interpolation=cv2.INTER_CUBIC,
    )
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict)
    corners, ids, _ = detector.detectMarkers(upscaled)
    if ids is None:
        return {}, []
    # Scale corners back to original resolution
    corners = [c / scale for c in corners]
    ids = ids.flatten()
    marker_dict = {id_: corner for id_, corner in zip(ids, corners)}
    return marker_dict, ids


def marker_center(corner):
    """Compute the center point of a marker from its corners."""
    corner = corner[0]
    x = int(corner[:, 0].mean())
    y = int(corner[:, 1].mean())
    return (x, y)


def compute_zones(marker_dict):
    """Compute three perspective-correct zone quadrilaterals using all markers.

    Layout (camera view of workspace):
        Marker 0 ---- Marker 3 ---- Marker 4 ---- Marker 1   (top)
          \\   Zone1   \\   Zone2    \\   Zone3    \\
        Marker 2 -------------------------------- Marker 5   (bottom)

    The workspace corners are:
    - Top-Left: Top-left corner of Marker 0
    - Top-Right: Top-right corner of Marker 1
    - Bottom-Left: Bottom-left corner of Marker 2
    - Bottom-Right: Bottom-right corner of Marker 5

    Zone dividers (from 3 and 4) are interpolated along the top and bottom
    edges based on their relative positions in the top row.
    """
    required = [0, 1, 2, 3, 4, 5]
    missing = [m for m in required if m not in marker_dict]
    if missing:
        raise ValueError(f"Missing layout markers: {missing}")

    # Layout centers for relative spacing calculation
    c0 = np.array(marker_center(marker_dict[0]))
    c3 = np.array(marker_center(marker_dict[3]))
    c4 = np.array(marker_center(marker_dict[4]))
    c1 = np.array(marker_center(marker_dict[1]))

    # Outer edges of the workspace
    # marker_dict[id][0] points: 0:tl, 1:tr, 2:br, 3:bl (clockwise)
    w_tl = np.array(marker_dict[0][0][0])  # Top-Left corner of Marker 0
    w_tr = np.array(marker_dict[1][0][1])  # Top-Right corner of Marker 1
    w_bl = np.array(marker_dict[2][0][3])  # Bottom-Left corner of Marker 2
    w_br = np.array(marker_dict[5][0][2])  # Bottom-Right corner of Marker 5

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
        debug_image (if debug=True)
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    marker_dict, ids = detect_aruco_markers(gray)
    if not marker_dict:
        return {} if not debug else ({}, image.copy())

    zones = compute_zones(marker_dict)
    excluded_ids = [0, 1, 2, 3, 4]

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
        return result, debug_image
    else:
        return result


# --- Example usage ---
if __name__ == "__main__":
    img = cv2.imread(str(TEST_IMAGE_PATH))
    markers_in_zones, debug_img = assign_markers_to_zones(img, debug=True)
    print("Marker assignments:", markers_in_zones)

    # Show debug image
    max_height = 800  # maximum height in pixels
    max_width = 1200  # maximum width in pixels

    h, w = debug_img.shape[:2]
    scale = min(max_width / w, max_height / h, 1.0)  # don't upscale if smaller
    display_img = cv2.resize(debug_img, (int(w*scale), int(h*scale)))

    cv2.imshow("Zones Debug", display_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
