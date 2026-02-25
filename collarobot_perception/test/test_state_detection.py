import numpy as np
from unittest.mock import patch
from collarobot_perception.collarobot_perception.state_detection import get_state


@patch('collarobot_perception.collarobot_perception.state_detection.capture_frame')
@patch('collarobot_perception.collarobot_perception.state_detection.analyze_zones_state')
def test_get_state_flow(mock_analyze, mock_capture):
    # Setup mocks
    mock_frame = np.zeros((100, 100, 3), dtype=np.uint8)
    mock_capture.return_value = mock_frame
    mock_analyze.return_value = ([9], [17], [13])

    # Run
    storage, proposed, accepted = get_state(debug=True)

    # Verify
    mock_capture.assert_called_once()
    mock_analyze.assert_called_once_with(mock_frame, debug=True)
    assert storage == [9]
    assert proposed == [17]
    assert accepted == [13]


@patch('collarobot_perception.collarobot_perception.state_detection.capture_frame')
def test_get_state_capture_fail(mock_capture):
    # Setup mock for failure
    mock_capture.return_value = None
    # Run
    storage, proposed, accepted = get_state()

    # Verify
    assert storage == []
    assert proposed == []
    assert accepted == []
