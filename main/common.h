#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#ifndef COMMON_H
#define COMMON_H

#define CLIENT_ID           CONFIG_ESP_CLIENT_ID
#define POLLING_PERIOD_MS   ( CONFIG_POLLING_PERIOD_MS )

#define REPORTING_THERMO_BIT ( 1 << 0 )

typedef struct TaskArgs_t {
    QueueHandle_t *p_msg_queue;
    QueueHandle_t *p_config_queue;
    EventGroupHandle_t *p_reporting_event_group;
} TaskArgs_t;

typedef struct MqttArgs_t {
    EventGroupHandle_t *p_evt_group_handle;
} MqttArgs_t;

typedef struct Config_t {
    uint period;
    int min_temp;
    int max_temp;
} Config_t;

int get_sleep_period();

#endif