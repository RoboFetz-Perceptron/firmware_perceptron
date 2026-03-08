# Perceptron Firmware

ESP32-C6 firmware for the Perceptron 3-omni-wheel combat robot.

## Build & Flash

```bash
idf.py set-target esp32c6
idf.py build flash monitor -p /dev/ttyACM0
```

Configuration: `idf.py menuconfig` -> Perceptron (node name, namespace, BLE name, roslog toggle).

## BLE Agent

Micro-ROS needs an agent running on the host system that translates from XRCE-DDS to full fledged DDS. So make sure you clone [our micro-ROS BLE agent](https://github.com/RoboFetz-Perceptron/micro_ros_agent_ble.git) into your ROS2 workspace and build it.

After building and sourcing, you can run it using `ros2 launch micro_ros_agent_ble agent_ble.launch.py`. Make sure the BLE device name of the agent and the firmware (configureable through menuconfig) matches.

## Status LED

- **White flash (3x)** - booting
- **Blue (bright)** - waiting for BLE connection
- **Blue (dim)** - BLE connected, waiting for micro-ROS agent
- **Orange** - connected, estop active (motors disabled)
- **Green** - running, motors enabled
- **Red** - error, will restart

## Usage

Once the agent shows a connection, disable estop to enable motors (3s weapon arm delay):
```bash
ros2 topic pub --once /perceptron/estop std_msgs/msg/Bool "data: false"
```

Drive with keyboard teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/perceptron/cmd_vel
```

Spin up weapon (20 is already significant, max is 255):
```bash
ros2 topic pub --once /perceptron/weapon_duty std_msgs/msg/UInt8 "data: 20"
```

Flip mode (inverts drive when robot is upside down):
```bash
ros2 topic pub --once /perceptron/is_flipped std_msgs/msg/Bool "data: true"
```

ESC throttle calibration (only tested with one ESC so far):
```bash
ros2 service call /perceptron/calibrate_esc std_srvs/srv/Trigger
```

## Parameters

All parameters persist in NVS across reboots (as long as the configured NVS_NAMESPACE stays the same). `rqt` does not work for parameters on this node (not super sure why yet), but using the CLI works without issues:

```bash
ros2 param set /perceptron/perceptron motor1_reversed true
ros2 param set /perceptron/perceptron wpn_pulse_min 850
ros2 param set /perceptron/perceptron wpn_pulse_max 2000
```

Battery voltage is published on `/perceptron/battery_voltage` every 5 seconds.

Thats it, goodbye and thank's for the fish!