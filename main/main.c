/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "thermo_task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "wifi_task.h"
#include "mqtt_task.h"
#include "common.h"
#include "nvs_flash.h"
#include "core_json.h"
#include "led_task.h"
#include "driver/ledc.h"


#define LEDC_CH0_GPIO (14)
#define LEDC_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_CH0_BRIGHTNESS 3000

//static xQueueHandle gpio_evt_queue = NULL;

static char *TAG = "MainProg";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* FreeRTOS event group to signal that we have reported the values to cloud */
static EventGroupHandle_t s_reporting_event_group;

/* FreeRTOS queue to send messages to MQTT */
static QueueHandle_t message_queue = NULL;

/* FreeRTOS queue to notify configuration changes to tasks */
static QueueHandle_t config_queue = NULL;

/* FreeRTOS queue to send led display pattern to led task */
// static QueueHandle_t led_queue = NULL;

/* The polling period for the reporting */
static int s_period = POLLING_PERIOD_MS;

/* Cound be set via config */
static int s_min_temp = 2;
static int s_max_temp = 2;

JSONStatus_t get_shadow_value(char *data, char *query, int *out) {
    char* value;
    size_t valueLength;
    JSONStatus_t result = JSON_Search( data, strlen(data), query, strlen(query),
                              &value, &valueLength );

    if( result == JSONSuccess )
    {
        // The pointer "value" will point to a location in the "buffer".
        char save = value[ valueLength ];
        // After saving the character, set it to a null byte for printing.
        value[ valueLength ] = '\0';
        // "Found: bar.foo -> xyz" will be printed.
        ESP_LOGI(TAG, "Found: %s -> %s\n", query, value );
        value[ valueLength ] = save;
        *out = atoi(value);
    } else {
        ESP_LOGI(TAG, "%s not found\n", query);
    }
    return result;
}

static void mqtt_message_handler(char *topic, char *data) 
{
    ESP_LOGI(TAG, "Got message %s on topic %s", data, topic);
    
    JSONStatus_t result;
    int out;
    result = get_shadow_value(data, "state.period", &out);
    if (result == JSONSuccess) {
        s_period = out;
    }
    result = get_shadow_value(data, "state.min_temp", &out);
    if (result == JSONSuccess) {
        s_min_temp = out;
    }
    result = get_shadow_value(data, "state.max_temp", &out);
    if (result == JSONSuccess) {
        s_max_temp = out;
    }

    Config_t conf;
    conf.period = s_period;
    conf.min_temp = s_min_temp;
    conf.max_temp = s_max_temp;
    xQueueSend(config_queue, &conf, 100);
    char reported[128];
    char shadow_topic[64];
    sprintf(shadow_topic, "$aws/things/%s/shadow/name/config/update", CLIENT_ID);
    sprintf(reported, "{\"state\":{\"reported\":{\"period\":%d, \"min_temp\":%d, \"max_temp\":%d}}}", s_period, s_min_temp, s_max_temp);
    mqtt_publish(shadow_topic, reported, 1);
}

int get_sleep_period() {
    return s_period;
}


static void task_publish(void* arg)
{
    TaskArgs_t *queues = (TaskArgs_t *) arg;
    QueueHandle_t *q = queues->p_msg_queue;
    char *msg;
    char topic[64];
    char shadow_msg[128];
    sprintf(topic, "$aws/things/%s/shadow/update", CLIENT_ID);
    for (;;) {
        if (is_mqtt_connected()) {
            if (pdPASS == xQueueReceive(*q, &msg , portMAX_DELAY)) {
                ESP_LOGI(TAG, "%s", msg);
                sprintf(shadow_msg, "{\"state\":{\"reported\": %s}}", msg);
                mqtt_publish(topic, shadow_msg, 0);
            }
        } else {
            vTaskDelay(1 * portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());

    message_queue = xQueueCreate(10, sizeof(char *));
    config_queue = xQueueCreate(10, sizeof(Config_t));
    s_reporting_event_group = xEventGroupCreate();

    TaskArgs_t queues = {
        .p_msg_queue = &message_queue,
        .p_config_queue = &config_queue,
        .p_reporting_event_group = &s_reporting_event_group, 
    };
  
    LedConfig_t led_config = {
        .ch = LEDC_CH0_CHANNEL,
        .pin = LEDC_CH0_GPIO,
        .brightness = LEDC_CH0_BRIGHTNESS,
        .unit_ms = 100,
    };

    s_wifi_event_group = xEventGroupCreate();
    

    mqtt_register_handler(&mqtt_message_handler);
    
    xTaskCreate(task_publish, "task_publish", 2048, &queues, 6, NULL);
    xTaskCreate(led_task, "led_task", 4096, &led_config, 6, NULL);
    // These are not tasks since they use the Event Loop
    
    start_wifi(&s_wifi_event_group);
    MqttArgs_t mqtt_args = {
        .p_evt_group_handle = &s_wifi_event_group,
    };
    start_mqtt(&mqtt_args);
    xTaskCreate(task_thermo, "thermo", 4096, &queues, 6, NULL);
    int cnt = 0;
    
    // Start the main task - should not do anything
    // All the wiring is done in the task

    led_display_pattern(">>>><<<<");
    EventBits_t bits;
    while(1) {
        bits = xEventGroupWaitBits(s_reporting_event_group, 
          REPORTING_THERMO_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(1000));
        char buf[128];

        if ( bits & REPORTING_THERMO_BIT && xQueuePeek(message_queue, &buf, 100) == pdFALSE) {
            ESP_LOGI(TAG, "Ready to sleep");
            mqtt_disconnect();
            stop_wifi();
            break;
        }
        ESP_LOGI(TAG, "Got bits %d", bits);
        printf("cnt: %d\n", cnt++);
        vTaskDelay(pdMS_TO_TICKS(1000));
       
    }
    ESP_LOGI(TAG, "Entering deep sleep for %d ms", s_period);
    vEventGroupDelete(s_reporting_event_group);
    vEventGroupDelete(s_wifi_event_group);
    vQueueDelete(message_queue);
    vQueueDelete(config_queue);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_sleep_enable_timer_wakeup(s_period * 1000);
    esp_deep_sleep_start();
}

