#include "freertos/queue.h"
#include "stdlib.h"
#include "driver/gpio.h"

#ifndef LED_TASK_H
#define LED_TASK_H

#define LED_MAX_PATTERN_LENGHT 32

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

typedef struct LedConfig_t {
    //QueueHandle_t *p_led_queue;
    uint32_t brightness;
    gpio_num_t pin;
    uint8_t ch;
    uint32_t unit_ms;
} LedConfig_t;


void led_task(void *args);

void led_display_pattern(char *pattern);

#endif