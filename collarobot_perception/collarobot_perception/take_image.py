import cv2
import time
import atexit

# Initialize camera at import time so it can autofocus
_cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
_cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)           # Enable autofocus
_cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)        # Auto exposure mode
_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)           # Minimal buffer = freshest frame


def _release_camera():
    if _cap.isOpened():
        _cap.release()


atexit.register(_release_camera)


def capture_frame():
    """
    Captures a single frame from the already-open camera.
    Flushes warmup frames to let auto-exposure/autofocus settle.
    Returns:
        frame: The captured frame (BGR) or None if capture failed.
    """
    if not _cap.isOpened():
        return None

    # Flush warmup frames so auto-exposure and autofocus converge
    for _ in range(10):
        _cap.read()
    time.sleep(0.3)

    ret, frame = _cap.read()
    if ret:
        return frame
    return None


if __name__ == "__main__":
    time.sleep(3)
    frame = capture_frame()
    if frame is not None:
        cv2.imwrite("captured_test_2.png", frame)
        print("Image captured and saved to captured_test_2.png")
    else:
        print("Failed to capture image")
