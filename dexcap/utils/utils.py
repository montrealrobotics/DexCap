import logging
import time
import subprocess
import platform
from enum import IntEnum
from PIL import Image
import numpy as np

class CustomFormatter(logging.Formatter):
    yellow = "\x1b[33;21m"
    red = "\x1b[31;21m"
    reset = "\x1b[0m"
    format_str = "%(asctime)s - %(levelname)s - %(message)s"

    FORMATS = {
        logging.DEBUG: format_str + reset,
        logging.INFO: format_str + reset,
        logging.WARNING: yellow + format_str + reset,
        logging.ERROR: red + format_str + reset,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt, datefmt="%H:%M:%S")
        return formatter.format(record)

class StatusCode(IntEnum):
    SUCCESS = 0
    APP_RESTART = 1
    SOCKET_TIMEOUT = 2

def get_logger(name="dexcap"):
    logger = logging.getLogger(name)
    logger.propagate = False
    if not logger.handlers:
        logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setFormatter(CustomFormatter())
        logger.addHandler(ch)
    return logger

def check_connection(host_name, ip_address, logger=None):
    """Wait for connection to ip address"""

    if logger is None:
        logger = get_logger(__name__)

    param = "-n" if platform.system().lower() == "windows" else "-c"

    logger.warning(f"Waiting for {host_name} ({ip_address}), please check that device is connected...")

    while True:
        result = subprocess.run(
            ["ping", param, "1", "-W", "5", ip_address],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        if result.returncode == 0:
            logger.info(f"{host_name} ip is REACHABLE.")
            break

        logger.warning(f"{host_name} ({ip_address}) not found. Retrying in 5s...")
        time.sleep(2)

def extract_img_observation(obs_dict, left_camera_id, right_camera_id, wrist_camera_id):
    image_observations = obs_dict["image"]
    left_image, right_image, wrist_image = None, None, None
    for key in image_observations:
        # Note the "left" below refers to the left camera in the stereo pair.
        # The model is only trained on left stereo cams, so we only feed those.
        if left_camera_id in key and "left" in key:
            left_image = image_observations[key]
        elif right_camera_id in key and "left" in key:
            right_image = image_observations[key]
        elif wrist_camera_id in key and "left" in key:
            wrist_image = image_observations[key]

    # Drop the alpha dimension and convert to RGB
    if left_image is not None:
        left_image = left_image[..., :3]
        left_image = left_image[..., ::-1]
    if right_image is not None:
        right_image = right_image[..., :3]
        right_image = right_image[..., ::-1]
    if wrist_image is not None:
        wrist_image = wrist_image[..., :3]
        wrist_image = wrist_image[..., ::-1]

    return {
        "left_image": left_image,
        "right_image": right_image,
        "wrist_image": wrist_image,
    }

def resize_with_pad(images, height, width, method=Image.BILINEAR) -> np.ndarray:
    """Replicates tf.image.resize_with_pad for multiple images using PIL. Resizes a batch of images to a target height.

    Args:
        images: A batch of images in [..., height, width, channel] format.
        height: The target height of the image.
        width: The target width of the image.
        method: The interpolation method to use. Default is bilinear.

    Returns:
        The resized images in [..., height, width, channel].
    """
    # If the images are already the correct size, return them as is.
    if images.shape[-3:-1] == (height, width):
        return images

    original_shape = images.shape

    images = images.reshape(-1, *original_shape[-3:])
    resized = np.stack([_resize_with_pad_pil(Image.fromarray(im), height, width, method=method) for im in images])
    return resized.reshape(*original_shape[:-3], *resized.shape[-3:])


def _resize_with_pad_pil(image: Image.Image, height: int, width: int, method: int) -> Image.Image:
    """Replicates tf.image.resize_with_pad for one image using PIL. Resizes an image to a target height and
    width without distortion by padding with zeros.

    Unlike the jax version, note that PIL uses [width, height, channel] ordering instead of [batch, h, w, c].
    """
    cur_width, cur_height = image.size
    if cur_width == width and cur_height == height:
        return image  # No need to resize if the image is already the correct size.

    ratio = max(cur_width / width, cur_height / height)
    resized_height = int(cur_height / ratio)
    resized_width = int(cur_width / ratio)
    resized_image = image.resize((resized_width, resized_height), resample=method)

    zero_image = Image.new(resized_image.mode, (width, height), 0)
    pad_height = max(0, int((height - resized_height) / 2))
    pad_width = max(0, int((width - resized_width) / 2))
    zero_image.paste(resized_image, (pad_width, pad_height))
    assert zero_image.size == (width, height)
    return zero_image
