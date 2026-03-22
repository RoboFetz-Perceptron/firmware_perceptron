#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct {
    QueueHandle_t cmd_vel;
    QueueHandle_t weapon;
    QueueHandle_t flipped;
    QueueHandle_t estop;
    QueueHandle_t calibrate;
} ros_queues_t;

bool ros_node_init(ros_queues_t *queues);
void ros_node_fini(void);
void ros_node_spin(void);
void ros_node_publish_battery(float voltage);
bool ros_node_time_sync(void);
bool ros_node_time_synced(void);
int64_t ros_node_last_sync_time(void);

#endif
