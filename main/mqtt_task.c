/* MQTT Mutual Authentication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "common.h"
#include "wifi_task.h"
#include "mqtt_task.h"
#include "led_task.h"

#define MQTT_BROKER CONFIG_ESP_MQTT_BROKER


static const char *TAG_MQTT = "MQTTS";

static EventGroupHandle_t *sp_wifi_event_group;
//static QueueHandle_t * sp_led_queue;
static esp_mqtt_client_handle_t client;

static message_handler_fptr_t message_handler = NULL;

extern const uint8_t client_cert_pem_start[] asm("_binary_cert_pem_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_cert_pem_end");
extern const uint8_t client_key_pem_start[] asm("_binary_priv_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_priv_key_end");
extern const uint8_t server_cert_pem_start[] asm("_binary_AmazonRootCA1_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_AmazonRootCA1_pem_end");

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_MQTT, "Last error %s: 0x%x", message, error_code);
    }
}

void mqtt_register_handler(message_handler_fptr_t handler) {
    message_handler = handler;
}

void mqtt_publish(const char *topic, char* data, int qos) {
    if (client) {
        int msg_id = esp_mqtt_client_publish(client, topic, data, 0, qos, 0);
        led_display_pattern("._!");
        //xQueueSend(*sp_led_queue, "._!", 100);
        ESP_LOGI(TAG_MQTT, "Published with id: %d", msg_id);
    } else {
        ESP_LOGW(TAG_MQTT, "Client not connected");
    }
}

void mqtt_disconnect() {
    if (client) {
        esp_mqtt_client_disconnect(client);
    }
}

static bool s_status_connected;

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_status_connected = true;
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        char topic[64];
        sprintf(topic, "$aws/things/%s/shadow/name/config/update/delta", CLIENT_ID);
        msg_id = esp_mqtt_client_subscribe(client, topic, 1);
        ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);
        //xQueueSend(*sp_led_queue, "._._._!", 100);
        led_display_pattern("._._._!");
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_status_connected = false;
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        char shadow_topic[64];
        sprintf(shadow_topic, "$aws/things/%s/shadow/name/config/update", CLIENT_ID);
        char config_data[128];
        sprintf(config_data, "{\"state\": {\"reported\": {\"period\": %d }}}", POLLING_PERIOD_MS);
        mqtt_publish(shadow_topic, config_data, 1);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
        //xQueueSend(*sp_led_queue, "..__..__!", 100);
        led_display_pattern("..__..__!");
        // Process message  
        if (message_handler) {
            char topic[event->topic_len+1];
            char data[event->data_len+1];
            sprintf(topic, "%.*s", event->topic_len, event->topic);
            sprintf(data, "%.*s", event->data_len, event->data);
            message_handler(topic, data);
        }
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_BROKER,
        .client_id = CLIENT_ID,
        .client_cert_pem = (const char *)client_cert_pem_start,
        .client_key_pem = (const char *)client_key_pem_start,
        .cert_pem = (const char *)server_cert_pem_start,

    };

    ESP_LOGI(TAG_MQTT, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    ESP_LOGI(TAG_MQTT, "Client started");
}

bool is_mqtt_connected() {
    return s_status_connected;
}

void start_mqtt(void *args)
{
    MqttArgs_t *mqtt_args = (MqttArgs_t *)args;
    sp_wifi_event_group = mqtt_args->p_evt_group_handle;
    //sp_led_queue = mqtt_args->p_led_queue;
    s_status_connected = false;
    esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    // esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    // esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    //ESP_ERROR_CHECK(nvs_flash_init());
    
    EventBits_t bits;
    for (;;) {
        bits = xEventGroupWaitBits(*sp_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG_MQTT, "connected to ap");
            //xQueueSend(*sp_led_queue, "._", 100);
            led_display_pattern("._");
            break;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG_MQTT, "Failed to connect to wifi");
            //xQueueSend(*sp_led_queue, "._._._____________________", 100);
            led_display_pattern("._._._____________________");
        } else {
            ESP_LOGE(TAG_MQTT, "UNEXPECTED EVENT");
        }
    }
    mqtt_app_start();
}