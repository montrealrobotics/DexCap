from copy import deepcopy
import cv2
import numpy as np
import os
import random
import threading
from collections import defaultdict
import time


def truncated_jpeg_frame(img):
    """A frame that lost data on the way is decoded up to the break and its last block repeated to the end,
    so its bottom-right corner is exactly flat; real camera images never are. (Frames missing their end
    marker are caught before decoding.)"""
    corner = img[-16:, -128:]
    return bool((np.ptp(corner.reshape(-1, corner.shape[-1]), axis=0) <= 1).all())


def gather_usb_cameras(max_tested=10, device_ids=None):
    """Scans system for available USB cameras using OpenCV indices (only device_ids, if given)."""
    all_usb_cameras = []
    for device_id in (device_ids if device_ids is not None else range(max_tested)):
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

        # Ask for compressed MJPEG before setting the size: uncompressed video from webcams like the C920 is
        # too much for USB passed through to WSL (usbipd) and arrives striped or drops the device. Cameras
        # without MJPEG (e.g. the ZED) keep their own format.
        self._cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cam.set(cv2.CAP_PROP_FPS, fps)

        if not self._cam.isOpened():
            raise RuntimeError(f"USB Camera {self.device_id} Failed To Open")

        # With MJPEG, take the compressed frames and decode them here: over usbipd some frames arrive
        # incomplete (~9% for a C920), and OpenCV would decode them into half-grey images ("Corrupt JPEG
        # data"). Incomplete frames are dropped instead.
        self.raw_mjpeg = int(self._cam.get(cv2.CAP_PROP_FOURCC)) == cv2.VideoWriter_fourcc(*"MJPG")
        if self.raw_mjpeg:
            self._cam.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.incomplete_frames = 0

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
        if getattr(self, "raw_mjpeg", False):
            data = frame.reshape(-1)
            if not (data[:2].tobytes() == b"\xff\xd8" and data.tobytes().rfind(b"\xff\xd9") > 2):
                self.incomplete_frames += 1
                return None, None
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if frame is None or truncated_jpeg_frame(frame):
                self.incomplete_frames += 1
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
    def __init__(self, camera_kwargs={}, device_ids=None):
        usb_cameras = gather_usb_cameras(device_ids=device_ids)
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

    def start_background(self):
        """Read every camera on its own thread and keep its latest frame.

        Cameras are otherwise read one after another, so a slow one (e.g. a GelSight Mini, whose
        3280x2464 MJPEG frames take ~50 ms to decode) holds up the whole teleop loop. In the background
        they decode in parallel and read_cameras() returns the latest frames without waiting.
        """
        self._latest = {}
        self._lock = threading.Lock()
        self._running = True
        self._recording = None  # (folder, {cam_id: name}) while full-resolution videos are being written
        self._writers_closed = {cam_id: threading.Event() for cam_id in self.camera_dict}
        for event in self._writers_closed.values():
            event.set()

        def loop(cam_id, cam):
            writer = None  # this camera's VideoWriter while a recording is on
            while self._running and cam.is_running():
                data_dict, timestamp_dict = cam.read_camera()
                if data_dict:
                    with self._lock:
                        self._latest[cam_id] = (data_dict, timestamp_dict)
                writer = self._record_frame(cam_id, cam, data_dict, writer)
            self._record_frame(cam_id, cam, None, writer, closing=True)

        self._threads = [threading.Thread(target=loop, args=(cam_id, cam), daemon=True)
                         for cam_id, cam in self.camera_dict.items()]
        for thread in self._threads:
            thread.start()

    # Full-resolution videos, like DROID's per-trajectory recordings: one MP4 per camera (a stereo camera's
    # whole side-by-side frame) plus a CSV of each frame's capture time, next to the trajectory file.
    # Written by each camera's background thread, so encoding never slows the teleop loop.
    VIDEO_SCALE = {"tactile": 0.5}  # name prefix -> scale; tactile sensors (e.g. 3280x2464 GelSight) at half size

    def start_recording(self, folder, names):
        """names: {cam_id: file name}, e.g. {"2": "wrist", "0": "tactile_left"}. Needs start_background()."""
        self._recording = (folder, dict(names))

    def stop_recording(self, timeout=3.0):
        """Stop the videos and wait until every file is closed (so they are complete)."""
        self._recording = None
        for event in getattr(self, "_writers_closed", {}).values():
            event.wait(timeout)

    def _record_frame(self, cam_id, cam, data_dict, writer, closing=False):
        recording = None if closing else self._recording
        name = recording[1].get(cam_id) if recording else None
        if writer is not None and (name is None or writer["folder"] != recording[0]):
            writer["video"].release()
            writer["times"].close()
            self._writers_closed[cam_id].set()
            writer = None
        if name is None or not data_dict:
            return writer
        images = data_dict.get("image", {})
        frame = images.get(cam.serial_number)
        if frame is None and cam.serial_number + "_left" in images:  # stereo: keep the whole pair
            frame = np.hstack([images[cam.serial_number + "_left"], images[cam.serial_number + "_right"]])
        if frame is None:
            return writer
        scale = next((s for prefix, s in self.VIDEO_SCALE.items() if name.startswith(prefix)), 1.0)
        if scale != 1.0:
            frame = cv2.resize(frame, (int(frame.shape[1] * scale) // 2 * 2, int(frame.shape[0] * scale) // 2 * 2))
        if writer is None:
            fps = cam._cam.get(cv2.CAP_PROP_FPS) or 30
            path = os.path.join(recording[0], f"{name}.mp4")
            video = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (frame.shape[1], frame.shape[0]))
            times = open(os.path.join(recording[0], f"{name}_timestamps.csv"), "w")
            times.write("frame,capture_time_ns\n")
            writer = {"folder": recording[0], "video": video, "times": times, "count": 0, "size": frame.shape[1::-1]}
            self._writers_closed[cam_id].clear()
        if frame.shape[1::-1] != writer["size"]:
            frame = cv2.resize(frame, writer["size"])
        writer["video"].write(frame[..., :3])
        writer["times"].write(f"{writer['count']},{time.time_ns()}\n")
        writer["count"] += 1
        return writer

    def read_cameras(self):
        if getattr(self, "_running", False):
            full_obs_dict = defaultdict(dict)
            full_timestamp_dict = {}
            with self._lock:
                latest = list(self._latest.values())
            for data_dict, timestamp_dict in latest:
                for key in data_dict:
                    full_obs_dict[key].update(data_dict[key])
                full_timestamp_dict.update(timestamp_dict)
            return full_obs_dict, full_timestamp_dict

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
        self.stop_recording()
        self._running = False
        for thread in getattr(self, "_threads", []):
            thread.join(timeout=1)
        for camera in self.camera_dict.values():
            camera.disable_camera()

