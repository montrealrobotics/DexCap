from importlib.resources import files
from deoxys.utils import YamlConfig

robot_arm = "xarm"

ip_config_file = files("dexcap").joinpath("configs/" + robot_arm + '.yaml')
ip_config = YamlConfig(str(ip_config_file)).as_easydict()
arm_config_file = files("dexcap").joinpath("configs/robot_config/" + robot_arm + '_arm.yaml')
arm_config = YamlConfig(str(arm_config_file)).as_easydict()

print(arm_config)
print(ip_config)