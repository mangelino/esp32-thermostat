/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "string.h"
#include "freertos/queue.h"
#include "led_task.h"

#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE


#define TAG "ledc"

/* FreeRTOS queue to send led display pattern to led task */
static QueueHandle_t led_queue = NULL;

void led_on(int ch, int brightness)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, brightness);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

void led_off(int ch)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

void pulse_led(int ch, int brightness, int period_ms, int duty)
{
    led_on(ch, brightness);
    int on_ms = period_ms * duty / 100;
    ESP_LOGI(TAG, "On for %d", on_ms);
    vTaskDelay(on_ms / portTICK_PERIOD_MS);
    led_off(ch);
    int off_ms = period_ms * (100 - duty) / 100;
    ESP_LOGI(TAG, "Off for %d", off_ms);
    vTaskDelay(off_ms / portTICK_PERIOD_MS);
}

void led_pattern(int ch, int brightness, int unit, char *pattern)
{
    for (int i = 0; i < strlen(pattern); i++)
    {
        if (pattern[i] == '.')
        {
            led_on(ch, brightness);
            vTaskDelay(unit / portTICK_PERIOD_MS);
        }
        else if (pattern[i] == '_')
        {
            led_off(ch);
            vTaskDelay(unit / portTICK_PERIOD_MS);
        }
    }
}

esp_err_t led_init(uint8_t ch, gpio_num_t pin)
{
    ESP_LOGI(TAG, "Led init %d %d", ch, pin);
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    CHECK(ledc_timer_config(&ledc_timer));
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel =
        {
            .channel = ch,
            .duty = 0,
            .gpio_num = pin,
            .speed_mode = LEDC_HS_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_HS_TIMER};

    // Set LED Controller with previously prepared configuration

    CHECK(ledc_channel_config(&ledc_channel));
    // Initialize fade service.
    CHECK(ledc_fade_func_install(0));
    ESP_LOGI(TAG, "Led init done.");
    return ESP_OK;
}

void led_display_pattern(char *pattern) {
    if (led_queue != NULL) {
        xQueueSend(led_queue, pattern, 100);
    }
}

void led_task(void *args)
{
    LedConfig_t *p_led_config = args;

    // QueueHandle_t *p_queue = p_led_config->p_led_queue;
    led_queue = xQueueCreate(10, LED_MAX_PATTERN_LENGHT);

    char pattern[32] = "";
    gpio_num_t pin = p_led_config->pin;
    uint8_t ch = p_led_config->ch;
    uint32_t brightness = p_led_config->brightness;
    uint32_t unit_ms = p_led_config->unit_ms;
    ESP_LOGI(TAG, "Init led task");

    if (ESP_OK == led_init(ch, pin))
    {
        for (;;)
        {
            // Wait for the pattern
            ESP_LOGI(TAG, "Waiting for pattern");

            while (pdPASS != xQueueReceive(led_queue, &pattern, 1))
            {
                vTaskDelay(1);
            }
            ESP_LOGI(TAG, "Got pattern %s of length %d", pattern, strlen(pattern));

            bool repeat = true;
            while (repeat) {
                vTaskDelay(1);
                for (int i = 0; i < strlen(pattern); i++)
                {
                    if (pdPASS == xQueueReceive(led_queue, &pattern, 1))
                    {
                        break;
                    }
                    if (pattern[i] == '.')
                    {
                        led_on(ch, brightness);
                        vTaskDelay(unit_ms / portTICK_PERIOD_MS);
                    }
                    else if (pattern[i] == '_')
                    {
                        led_off(ch);
                        vTaskDelay(unit_ms / portTICK_PERIOD_MS);
                    }
                    else if (pattern[i] == '>')
                    {
                        int period_up = 0;
                        while (i + 1 < strlen(pattern) && pattern[i + 1] == '>')
                        {
                            period_up++;
                            i++;
                        }
                        ledc_set_fade_with_time(LEDC_HS_MODE,
                                                ch, brightness, period_up * unit_ms);
                        ledc_fade_start(LEDC_HS_MODE,
                                        ch, LEDC_FADE_NO_WAIT);
                        vTaskDelay(period_up * unit_ms / portTICK_PERIOD_MS);
                    }
                    else if (pattern[i] == '<')
                    {
                        int period_down = 0;
                        while (i + 1 < strlen(pattern) && pattern[i + 1] == '<')
                        {
                            period_down++;
                            i++;
                        }
                        ledc_set_fade_with_time(LEDC_HS_MODE,
                                                ch, 0, period_down * unit_ms);
                        ledc_fade_start(LEDC_HS_MODE,
                                        ch, LEDC_FADE_NO_WAIT);
                        vTaskDelay(period_down * unit_ms / portTICK_PERIOD_MS);
                    } 
                    else if (pattern[i] == '!')
                    {
                        repeat = false;
                        break;
                    }
                }
            }
        }
    }
    ESP_LOGE(TAG, "Something went wrong - failed to initialize task");
    vTaskDelete(NULL);
}

void app_test(void)
{
    led_init(0, 14);
    QueueHandle_t queue = xQueueCreate(1, 32);
    ESP_LOGI(TAG, "Init task");
    xTaskCreate(led_task, "led_task", 1024, &queue, 6, NULL);
    //vTaskStartScheduler();
    while (1)
    {
        // printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
        //     ledc_fade_start(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        // }
        // vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        // printf("2. LEDC fade down to duty = 0\n");
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
        //     ledc_fade_start(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        // }
        // vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        // printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
        //     ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        // }
        //pulse_led(LEDC_CHANNEL_0, 3000, 500, 50);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, "._", 100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, "..__", 100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, "....____", 100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, "_", 100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, ".", 100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, ">>><<<", 100);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, ">>>>><<<<<", 100);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sending pattern");
        xQueueSend(queue, ">>><<<<<", 100);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        // printf("4. LEDC set duty  0 without fade\n");
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
        //     ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        // }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
