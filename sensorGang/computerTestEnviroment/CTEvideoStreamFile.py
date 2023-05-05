"""Stream video from file."""
import cv2


class VideoStreamFile:
    """Stream video from file."""

    def __init__(self):
        """Init stream from file."""
        self.vid_capture = cv2.VideoCapture('yes.mp4')

    def start(self):
        """Do nothing."""
        return

    def read(self):
        """Read frame from file."""
        ret, frame = self.vid_capture.read()
        return frame

    def stop(self):
        """Stop process."""
        self.vid_capture.release()
