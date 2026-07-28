import sys
from dexcap.sensors.cameras.zed_module import MultiCameraWrapper
from dexcap.utils.utils import (
    check_connection,
    get_logger,
    StatusCode,
    extract_img_observation,
    resize_with_pad,
)

logger = get_logger("TestCameraReader")


def test_camera_reader():
    # Target camera serials
    left_camera_id: str = "29712701"
    right_camera_id: str = "23697076"
    wrist_camera_id: str = "12715737"
    
    camera_kwargs = {}

    logger.info("Initializing MultiCameraWrapper...")
    camera_reader = MultiCameraWrapper(camera_kwargs)
    detected_keys = set(camera_reader.camera_dict.keys())

    # Check presence of specific cameras
    left_cam = any(left_camera_id in key for key in detected_keys)
    right_cam = any(right_camera_id in key for key in detected_keys)
    wrist_cam = any(wrist_camera_id in key for key in detected_keys)

    logger.info(f"Connected cameras: {list(detected_keys)}")
    logger.info(f"Status -> Left: {left_cam} | Right: {right_cam} | Wrist: {wrist_cam}")

    if not (left_cam or right_cam or wrist_cam):
        logger.warning("No matching cameras were detected! Attempting read anyway...")

    # Capture frames
    logger.info("Reading frames from cameras...")
    camera_obs, camera_timestamp = camera_reader.read_cameras()

    obs_dict = {
        "timestamp": {"cameras": camera_timestamp}
    }
    obs_dict.update(camera_obs)

    logger.info("Extracting image observations...")
    img_obs = extract_img_observation(
        obs_dict, left_camera_id, right_camera_id, wrist_camera_id
    )

    obs = {}
    target_dim = (224, 224)

    if wrist_cam and "wrist_image" in img_obs:
        obs["observation/wrist_image_left"] = resize_with_pad(
            img_obs["wrist_image"], *target_dim
        )
        logger.info(f"Processed wrist_image -> shape: {obs['observation/wrist_image_left'].shape}")

    if left_cam and "left_image" in img_obs:
        obs["observation/exterior_image_1_left"] = resize_with_pad(
            img_obs["left_image"], *target_dim
        )
        logger.info(f"Processed left_image  -> shape: {obs['observation/exterior_image_1_left'].shape}")

    if right_cam and "right_image" in img_obs:
        obs["observation/exterior_image_2_left"] = resize_with_pad(
            img_obs["right_image"], *target_dim
        )
        logger.info(f"Processed right_image -> shape: {obs['observation/exterior_image_2_left'].shape}")

    logger.info(f"Successfully processed {len(obs)} camera feed(s).")
    return obs


if __name__ == "__main__":
    try:
        test_camera_reader()
        logger.info("Camera test completed successfully.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        sys.exit(1)