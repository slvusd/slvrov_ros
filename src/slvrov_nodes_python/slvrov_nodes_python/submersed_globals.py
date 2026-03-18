from pathlib import Path
from slvrov_tools.i2c_tools import *
from slvrov_tools.pca9685 import *

#I2C_BUS1 = I2C_Bus(1)
CWD = Path.cwd()
# Resolve the pin config file relative to the launch working directory so the
# service and hardware nodes share the same runtime JSON file.
PCA9685_PIN_CONFIG_PATH = f"{CWD}/pca9685_pin_configs.json"
JOYSTICK_BINDINGS_PATH = f"{CWD}/joystick_bindings.yaml"