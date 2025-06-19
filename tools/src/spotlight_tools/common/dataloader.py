import numpy as np
import cv2
from pathlib import Path


def load_muscle_image(recording_dir: Path, muscle_frame_id: int) -> np.ndarray:
    muscle_image_path = (
        recording_dir
        / "processed/muscle_images"
        / f"muscle_frame_{muscle_frame_id:09d}.tif"
    )
    return cv2.imread(str(muscle_image_path), cv2.IMREAD_UNCHANGED)


def load_behavior_frame(recording_dir: Path, behavior_frame_id: int) -> np.ndarray:
    behavior_video_path = recording_dir / "processed/behavior_video.mkv"
    behavior_video_capture = cv2.VideoCapture(str(behavior_video_path))
    num_frames = int(behavior_video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if behavior_frame_id >= num_frames:
        return None  # Return None if the frame ID is out of bounds
    behavior_video_capture.set(cv2.CAP_PROP_POS_FRAMES, behavior_frame_id)
    ret, frame = behavior_video_capture.read()
    if not ret:
        raise ValueError(f"Could not read frame {behavior_frame_id} from video.")
    behavior_video_capture.release()
    return frame[:, :, 0]
