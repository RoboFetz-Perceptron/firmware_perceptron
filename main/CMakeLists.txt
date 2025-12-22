set(SOURCES
    src/main.c
    src/robot_controller/robot_controller.c
    src/app_utils/indicator_led.c
    src/app_utils/jtag_debug.c
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES
        common_utils
        esp32_servo
        esp32_can
        odrive_can
        ackermann_kinematics
        mros
    PRIV_REQUIRES
        micro_ros_espidf_component
        espressif__led_strip
)
