#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifndef COMMON_H
#define COMMON_H

#define CLIENT_ID           CONFIG_ESP_CLIENT_ID
#define POLLING_PERIOD_MS   ( CONFIG_POLLING_PERIOD_MS )

#define REPORTING_BME280_BIT ( 1 << 0 )
#define REPORTING_SCALE_BIT  ( 1 << 1 )

typedef struct TaskArgs_t {
    QueueHandle_t *p_msg_queue;
    QueueHandle_t *p_config_queue;
    EventGroupHandle_t *p_reporting_event_group;
} TaskArgs_t;

typedef struct MqttArgs_t {
    EventGroupHandle_t *p_evt_group_handle;
    //QueueHandle_t *p_led_queue;
} MqttArgs_t;

#endif