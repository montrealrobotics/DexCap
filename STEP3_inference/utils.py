import logging
import time
import subprocess
import platform
from enum import IntEnum

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