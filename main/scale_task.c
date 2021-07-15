#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "common.h"

#include "hx711.h"

#define TAG_SCALE "Scale"

#define DOUT_PIN    ( 4 )
#define PD_SCK_PIN  ( 5 )

int32_t readAvg(hx711_t *p_dev, uint8_t samples) {
    int32_t avg = 0;
    bool ready;
    if (ESP_OK != hx711_power_down(p_dev, false)) {
        ESP_LOGE(TAG_SCALE, "Error powering up HX711");
    }
    hx711_wait(p_dev, 400);
    hx711_is_ready(p_dev, &ready);
    if (ready) {
        while (samples-- > 0) {
            int32_t data;
            hx711_read_data(p_dev, &data);
            avg += data/100;
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        
    } else {
        ESP_LOGE(TAG_SCALE, "An error occured : not ready");
    }
    hx711_power_down(p_dev, true);
    return avg;
}

void scale_task(void *args) {
    // Get the queues - config and message
    TaskArgs_t *queues = (TaskArgs_t *) args;
    QueueHandle_t *p_message_queue = queues->p_msg_queue;
    QueueHandle_t *p_config_q = queues->p_config_queue;
    EventGroupHandle_t *p_evt_group = queues->p_reporting_event_group;
    char msg_buf[128];


    hx711_t dev = {
        .dout = DOUT_PIN,
        .pd_sck = PD_SCK_PIN,
        .gain = HX711_GAIN_A_128
    };
    esp_err_t res = hx711_init(&dev);
    float scale = 1.0;
    ESP_LOGI(TAG_SCALE, "Init hx711 = %d", res);
    int period = POLLING_PERIOD_MS; // TODO: Add value to menuconfig
    //if (ESP_OK == res) {
        ESP_LOGI(TAG_SCALE, "Running the task loop");
        int32_t tare = readAvg(&dev, 5);
        int new_period;
        for (;;) {
            if (pdPASS == xQueueReceive(*p_config_q, &new_period, 0)) {
                ESP_LOGD(TAG_SCALE, "New period %d", new_period);
                period = new_period;
            };
            int32_t avg = readAvg(&dev, 5);
            ESP_LOGI(TAG_SCALE, "Weight: %d", avg-tare);
            sprintf(msg_buf, "{ \"w\":%.2f }", (avg-tare)*scale);
            char *pBuf = msg_buf;
            xQueueSend(*p_message_queue, &pBuf, 0);
            xEventGroupSetBits(*p_evt_group, REPORTING_SCALE_BIT);
            vTaskDelay(period / portTICK_PERIOD_MS);
        }
    //}
    ESP_LOGE(TAG_SCALE, "Somethng went wrong - quitting task");
    vTaskDelete(NULL);

}
