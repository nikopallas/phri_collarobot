import cv2


def capture_frame(camera_id=1, width=3840, height=2160):
    """
    Captures a single frame from the specified camera.
    Returns:
        frame: The captured frame (BGR) or None if capture failed.
    """
    cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)

    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    ret, frame = cap.read()
    cap.release()

    if ret:
        return frame
    return None


if __name__ == "__main__":
    frame = capture_frame()
    if frame is not None:
        cv2.imwrite("captured_test.png", frame)
        print("Image captured and saved to captured_test.png")
    else:
        print("Failed to capture image")
