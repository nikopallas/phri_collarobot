import numpy as np
import cv2


def create_homography_from_corners(corner_pixels):
    """
    Creates homography matrix from 4 known corner pixels.

    corner_pixels: np.array shape (4,2)
                   Order MUST be:
                   [top-left, top-right, bottom-right, bottom-left]
    """

    # Normalized square target (0–1 coordinate system)
    world_points = np.array([
        [0.0, 0.0],   # top-left
        [1.0, 0.0],   # top-right
        [1.0, 1.0],   # bottom-right
        [0.0, 1.0]    # bottom-left
    ], dtype=np.float32)

    H, _ = cv2.findHomography(corner_pixels, world_points)
    return H


def transform_points(points, H):
    """
    Transforms pixel points into normalized coordinate system.

    points: np.array shape (N,2)
    H: homography matrix

    Returns: transformed points shape (N,2)
    """

    points = np.array(points, dtype=np.float32)
    points = points.reshape(-1, 1, 2)

    transformed = cv2.perspectiveTransform(points, H)

    return transformed.reshape(-1, 2)


if __name__ == "__main__":

    # ------------------------------------------------
    # 1️⃣ Example: 4 known corner pixel coordinates
    # ------------------------------------------------
    # IMPORTANT: Order must be:
    # [top-left, top-right, bottom-right, bottom-left]

    corner_pixels = np.array([
        [120, 80],    # top-left
        [520, 100],   # top-right
        [500, 420],   # bottom-right
        [100, 400]    # bottom-left
    ], dtype=np.float32)

    # ------------------------------------------------
    # 2️⃣ Example: 5 marker pixel coordinates
    # ------------------------------------------------

    marker_pixels = np.array([
        [250, 200],
        [300, 250],
        [400, 300],
        [200, 350],
        [350, 150]
    ], dtype=np.float32)

    # ------------------------------------------------
    # 3️⃣ Compute Homography
    # ------------------------------------------------

    H = create_homography_from_corners(corner_pixels)

    print("Homography Matrix:")
    print(H)
    print()

    # ------------------------------------------------
    # 4️⃣ Transform marker coordinates
    # ------------------------------------------------

    normalized_coords = transform_points(marker_pixels, H)

    print("Normalized Coordinates (0–1 system):")
    for i, coord in enumerate(normalized_coords):
        print(f"Marker {i+1}: x={coord[0]:.3f}, y={coord[1]:.3f}")
