from copy import deepcopy
import cv2
import numpy as np
import os
import random
from collections import defaultdict
import time

try:
    import pyzed.sl as sl
except ModuleNotFoundError:
    print("WARNING: You have not setup the ZED cameras, and currently cannot use them")


def gather_zed_cameras():
    all_zed_cameras = []
    try:
        cameras = sl.Camera.get_device_list()
    except NameError:
        return []

    for cam in cameras:
        cam = ZedCamera(cam)
        all_zed_cameras.append(cam)

    return all_zed_cameras


resize_func_map = {"cv2": cv2.resize, None: None}

standard_params = dict(
    depth_minimum_distance=0.1, camera_resolution=sl.RESOLUTION.HD720, depth_stabilization=False, camera_fps=60, camera_image_flip=sl.FLIP_MODE.OFF
)

advanced_params = dict(
    depth_minimum_distance=0.1, camera_resolution=sl.RESOLUTION.HD2K, depth_stabilization=False, camera_fps=15, camera_image_flip=sl.FLIP_MODE.OFF
)


class ZedCamera:
    def __init__(self, camera):
        # Save Parameters #
        self.serial_number = str(camera.serial_number)
        self.is_wrist_camera = self.serial_number[0] == '1'
        self.current_mode = None
        self._current_params = None
        self._extriniscs = {}

        # Open Camera #
        print("Opening Zed: ", self.serial_number)

    def set_reading_parameters(
        self,
        image=True,
        depth=False,
        pointcloud=False,
        concatenate_images=False,
        resolution=(0, 0),
        resize_func=None,
    ):
        # Non-Permenant Values #
        self.traj_image = image
        self.traj_concatenate_images = concatenate_images
        self.traj_resolution = resolution

        # Permenant Values #
        self.depth = depth
        self.pointcloud = pointcloud
        self.resize_func = resize_func_map[resize_func]

    ### Camera Modes ###
    def set_trajectory_mode(self):
        # Set Parameters #
        self.image = self.traj_image
        self.concatenate_images = self.traj_concatenate_images
        self.skip_reading = not any([self.image, self.depth, self.pointcloud])

        if self.resize_func is None:
            self.zed_resolution = sl.Resolution(*self.traj_resolution)
            self.resizer_resolution = (0, 0)
        else:
            self.zed_resolution = sl.Resolution(0, 0)
            self.resizer_resolution = self.traj_resolution

        # Set Mode #
        change_settings = self._current_params != standard_params
        if change_settings:
            self._configure_camera(standard_params)
        self.current_mode = "trajectory"

    def _configure_camera(self, init_params):
        # Close Existing Camera #
        self.disable_camera()

        # Initialize Readers #
        self._cam = sl.Camera()
        self._sbs_img = sl.Mat()
        self._left_img = sl.Mat()
        self._right_img = sl.Mat()
        self._runtime = sl.RuntimeParameters()

        # Open Camera #
        self._current_params = init_params
        sl_params = sl.InitParameters(**init_params)
        sl_params.set_from_serial_number(int(self.serial_number))
        sl_params.camera_image_flip = sl.FLIP_MODE.OFF
        status = self._cam.open(sl_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError("Camera Failed To Open")

        # Save Intrinsics #
        self.latency = int(2.5 * (1e3 / sl_params.camera_fps))

    ### Basic Camera Utilities ###
    def _process_frame(self, frame):
        frame = deepcopy(frame.get_data())
        if self.resizer_resolution == (0, 0):
            return frame
        return self.resize_func(frame, self.resizer_resolution)

    def read_camera(self):
        # Skip if Read Unnecesary #
        if self.skip_reading:
            return {}, {}

        # Read Camera #
        timestamp_dict = {self.serial_number + "_read_start": time.time_ns()}
        err = self._cam.grab(self._runtime)
        if err != sl.ERROR_CODE.SUCCESS:
            return None
        timestamp_dict[self.serial_number + "_read_end"] = time.time_ns()

        # Benchmark Latency #
        received_time = self._cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
        timestamp_dict[self.serial_number + "_frame_received"] = received_time
        timestamp_dict[self.serial_number + "_estimated_capture"] = received_time - self.latency

        # Return Data #
        data_dict = {}

        if self.image:
            if self.concatenate_images:
                self._cam.retrieve_image(self._sbs_img, sl.VIEW.SIDE_BY_SIDE, resolution=self.zed_resolution)
                data_dict["image"] = {self.serial_number: self._process_frame(self._sbs_img)}
            else:
                self._cam.retrieve_image(self._left_img, sl.VIEW.LEFT, resolution=self.zed_resolution)
                self._cam.retrieve_image(self._right_img, sl.VIEW.RIGHT, resolution=self.zed_resolution)
                data_dict["image"] = {
                    self.serial_number + "_left": self._process_frame(self._left_img),
                    self.serial_number + "_right": self._process_frame(self._right_img),
                }

        return data_dict, timestamp_dict

    def disable_camera(self):
        if self.current_mode == "disabled":
            return
        if hasattr(self, "_cam"):
            self._current_params = None
            self._cam.close()
        self.current_mode = "disabled"

    def is_running(self):
        return self.current_mode != "disabled"


class MultiCameraWrapper:
    def __init__(self, camera_kwargs={}):
        # Open Cameras #
        zed_cameras = gather_zed_cameras()
        self.camera_dict = {cam.serial_number: cam for cam in zed_cameras}

        # Set Correct Parameters #
        for cam_id in self.camera_dict.keys():
            cam_type = "wrist" if self.camera_dict[cam_id].is_wrist_camera else "variable"
            curr_cam_kwargs = camera_kwargs.get(cam_type, {})
            self.camera_dict[cam_id].set_reading_parameters(**curr_cam_kwargs)

        # Launch Camera #
        self.set_trajectory_mode()

    ### Calibration Functions ###
    def get_camera(self, camera_id):
        return self.camera_dict[camera_id]

    def set_trajectory_mode(self):
        # Put All Cameras In Trajectory Mode #
        for cam in self.camera_dict.values():
            cam.set_trajectory_mode()

    ### Basic Camera Functions ###
    def read_cameras(self):
        full_obs_dict = defaultdict(dict)
        full_timestamp_dict = {}

        # Read Cameras In Randomized Order #
        all_cam_ids = list(self.camera_dict.keys())
        random.shuffle(all_cam_ids)

        for cam_id in all_cam_ids:
            if not self.camera_dict[cam_id].is_running():
                continue
            data_dict, timestamp_dict = self.camera_dict[cam_id].read_camera()

            for key in data_dict:
                full_obs_dict[key].update(data_dict[key])
            full_timestamp_dict.update(timestamp_dict)

        return full_obs_dict, full_timestamp_dict

    def disable_cameras(self):
        for camera in self.camera_dict.values():
            camera.disable_camera()