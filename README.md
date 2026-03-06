# Perceptron Firmware

ESP32-C6 firmware for the Perceptron 3-omni-wheel combat robot. micro-ROS over BLE (NimBLE NUS).

## Build & Flash

```bash
idf.py set-target esp32c6
idf.py build flash monitor -p /dev/ttyACM0
```

Configuration: `idf.py menuconfig` → Perceptron (node name, namespace, BLE name, roslog toggle).

## ROS 2 Commands

All topics are under the `/perceptron` namespace (default).

TODO: Better document startup usage also in combination with BLE agent
```bash
ros2 topic pub /perceptron/weapon_duty std_msgs/msg/UInt8 "{data: 128}"

ros2 topic pub /perceptron/is_flipped std_msgs/msg/Bool "{data: true}"

ros2 topic pub /perceptron/estop std_msgs/msg/Bool "{data: false}"

ros2 service call /perceptron/calibrate_esc std_srvs/srv/Trigger

ros2 param set /perceptron/perceptron motor1_reversed true
ros2 param set /perceptron/perceptron motor2_reversed false
ros2 param set /perceptron/perceptron motor3_reversed true

ros2 param set /perceptron/perceptron wpn_pulse_min 850
ros2 param set /perceptron/perceptron wpn_pulse_max 2000
```
