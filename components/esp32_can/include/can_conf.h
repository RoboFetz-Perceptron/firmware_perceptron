#ifndef CAN_CONF_H
#define CAN_CONF_H

#include <sdkconfig.h>

// CAN dispatcher configuration
#define CAN_DISPATCHER_TAG "CAN_DISPATCHER"
#define MAX_CBS 16
#define TASK_NAME "can_dispatcher_task"
#define TASK_STACK_SZ 4096
#define TASK_PRIO 6

// CAN driver configuration
#ifdef CONFIG_CAN_TX_GPIO
#define CAN_TX_GPIO CONFIG_CAN_TX_GPIO
#else
#define CAN_TX_GPIO GPIO_NUM_23
#endif

#ifdef CONFIG_CAN_RX_GPIO
#define CAN_RX_GPIO CONFIG_CAN_RX_GPIO
#else
#define CAN_RX_GPIO GPIO_NUM_22
#endif

#define CAN_TX_QUEUE_LEN 10
#define CAN_RX_QUEUE_LEN 10
#define CAN_LOGGER_TAG "CAN_DRV"
#define CAN_MAX_RECEIVE_TIMEOUT_MS 1000

#endif // CAN_CONF_H