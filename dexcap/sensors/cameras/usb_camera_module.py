from copy import deepcopy
import cv2
import numpy as np
import os
import random
from collections import defaultdict
import time


def gather_usb_cameras(max_tested=10):
    """Scans system for available USB cameras using OpenCV indices."""
    all_usb_cameras = []
    for device_id in range(max_tested):
        cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                all_usb_cameras.append(USBCamera(device_id))
            cap.release()
    return all_usb_cameras


resize_func_map = {"cv2": cv2.resize, None: None}

standard_params = dict(
    resolution=(1280, 720),
    fps=60,
)

advanced_params = dict(
    resolution=(1920, 1080),
    fps=15,
)


class USBCamera:
    def __init__(self, device_id):
        # Save Parameters #
        self.device_id = device_id
        self.serial_number = str(device_id)
        self.is_wrist_camera = self.serial_number[0] == '0'
        self.current_mode = "disabled"
        self._current_params = None
        self._cam = None

        print(f"Opening USB Camera Index: {self.serial_number}")

    def set_reading_parameters(
        self,
        image=True,
        depth=False,
        pointcloud=False,
        concatenate_images=False,
        resolution=(0, 0),
        resize_func=None,
    ):
        # Non-Permanent Values #
        self.traj_image = image
        self.traj_concatenate_images = concatenate_images
        self.traj_resolution = resolution

        # Permanent Values #
        self.depth = False
        self.pointcloud = False
        self.resize_func = resize_func_map.get(resize_func, None)

    def set_trajectory_mode(self):
        self.image = self.traj_image
        self.skip_reading = not self.image

        if self.resize_func is None:
            self.cam_resolution = self.traj_resolution
            self.resizer_resolution = (0, 0)
        else:
            self.cam_resolution = (0, 0)
            self.resizer_resolution = self.traj_resolution

        change_settings = self._current_params != standard_params
        if change_settings or not self.is_running():
            self._configure_camera(standard_params)
        self.current_mode = "trajectory"

    def _configure_camera(self, init_params):
        self.disable_camera()

        self._cam = cv2.VideoCapture(self.device_id)
        
        self._current_params = init_params
        width, height = init_params.get("resolution", (640, 480))
        fps = init_params.get("fps", 30)

        self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cam.set(cv2.CAP_PROP_FPS, fps)

        if not self._cam.isOpened():
            raise RuntimeError(f"USB Camera {self.device_id} Failed To Open")

        self.latency = int(2.5 * (1e3 / fps))

    def _is_stereo_frame(self, frame: np.ndarray) -> bool:
        """Detects if a frame is side-by-side stereo based on aspect ratio."""
        if frame is None:
            return False
        height, width = frame.shape[:2]
        aspect_ratio = width / height

        return aspect_ratio > 2.4

    def _process_frame(self, frame):
        frame = deepcopy(frame)
        if self.resizer_resolution == (0, 0) or self.resizer_resolution == (0, 0, 0):
            return frame
        return self.resize_func(frame, (self.resizer_resolution[0], self.resizer_resolution[1]))

    def read_camera(self):
        if self.skip_reading or not self.is_running():
            return {}, {}

        timestamp_dict = {self.serial_number + "_read_start": time.time_ns()}
        ret, frame = self._cam.read()
        timestamp_dict[self.serial_number + "_read_end"] = time.time_ns()

        if not ret or frame is None:
            return None, None

        # Benchmark Latency #
        received_time = time.time() * 1000.0
        timestamp_dict[self.serial_number + "_frame_received"] = received_time
        timestamp_dict[self.serial_number + "_estimated_capture"] = received_time - self.latency

        stereo_f = self._is_stereo_frame(frame)

        data_dict = {}
        if self.image:
            data_dict["image"] = {
                self.serial_number: self._process_frame(frame)
            }
            if stereo_f:
                left_frame, right_frame = np.split(frame, 2, axis=1)

                data_dict["image"] = {
                    self.serial_number + "_left": self._process_frame(left_frame),
                    self.serial_number + "_right": self._process_frame(right_frame),
                }

        return data_dict, timestamp_dict

    def disable_camera(self):
        if self.current_mode == "disabled":
            return
        if self._cam is not None and self._cam.isOpened():
            self._cam.release()
            self._cam = None
        self._current_params = None
        self.current_mode = "disabled"

    def is_running(self):
        return self.current_mode != "disabled" and self._cam is not None and self._cam.isOpened()


class MultiCameraWrapper:
    def __init__(self, camera_kwargs={}):
        usb_cameras = gather_usb_cameras()
        self.camera_dict = {cam.serial_number: cam for cam in usb_cameras}

        # Set Correct Parameters #
        for cam_id in self.camera_dict.keys():
            cam_type = "wrist" if self.camera_dict[cam_id].is_wrist_camera else "variable"
            curr_cam_kwargs = camera_kwargs.get(cam_type, {})
            self.camera_dict[cam_id].set_reading_parameters(**curr_cam_kwargs)

        self.set_trajectory_mode()

    def get_camera(self, camera_id):
        return self.camera_dict[str(camera_id)]

    def set_trajectory_mode(self):
        for cam in self.camera_dict.values():
            cam.set_trajectory_mode()

    def read_cameras(self):
        full_obs_dict = defaultdict(dict)
        full_timestamp_dict = {}

        all_cam_ids = list(self.camera_dict.keys())
        random.shuffle(all_cam_ids)

        for cam_id in all_cam_ids:
            if not self.camera_dict[cam_id].is_running():
                continue
            data_dict, timestamp_dict = self.camera_dict[cam_id].read_camera()

            if data_dict is None:
                continue

            for key in data_dict:
                full_obs_dict[key].update(data_dict[key])
            full_timestamp_dict.update(timestamp_dict)

        return full_obs_dict, full_timestamp_dict

    def disable_cameras(self):
        for camera in self.camera_dict.values():
            camera.disable_camera()

