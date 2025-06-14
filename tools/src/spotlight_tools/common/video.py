import cv2
from pathlib import Path


def get_video_info(video_path: Path):
    video = cv2.VideoCapture(str(video_path))
    if not video.isOpened():
        print("Error: Could not open video.")
        return None

    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_count = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    video.release()

    return width, height, frame_count
