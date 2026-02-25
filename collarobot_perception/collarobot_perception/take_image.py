import cv2
import atexit

# Initialize camera at import time so it can autofocus
_cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


def _release_camera():
    if _cap.isOpened():
        _cap.release()


atexit.register(_release_camera)


def capture_frame():
    """
    Captures a single frame from the already-open camera.
    Returns:
        frame: The captured frame (BGR) or None if capture failed.
    """
    if not _cap.isOpened():
        return None

    ret, frame = _cap.read()
    if ret:
        return frame
    return None


if __name__ == "__main__":
    import time
    # Give camera a moment to autofocus
    time.sleep(5)
    frame = capture_frame()
    if frame is not None:
        cv2.imwrite("captured_test_2.png", frame)
        print("Image captured and saved to captured_test_2.png")
    else:
        print("Failed to capture image")
