/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
/******************************************************************************/
/*!                         Own header files                                  */
#include "bme280.h"
#include "common.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static const char *TAG_BME = "bme280";

static gpio_num_t i2c_gpio_sda = 22;
static gpio_num_t i2c_gpio_scl = 23;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

/******************************************************************************/
/*!                               Structures                                  */

/* Structure that contains identifier details used in example */
struct identifier
{
    /* Variable to hold device address */
    uint8_t dev_addr;

    /* Variable that contains file descriptor */
    int8_t fd;
};

/****************************************************************************/
/*!                         Functions                                       */

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs.
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *  @return void.
 *
 */
void user_delay_us(uint32_t period, void *intf_ptr);

/*!
 * @brief Function for print the temperature, humidity and pressure data.
 *
 * @param[out] comp_data    :   Structure instance of bme280_data
 *
 * @note Sensor data whose can be read
 *
 * sens_list
 * --------------
 * Pressure
 * Temperature
 * Humidity
 *
 */
void print_sensor_data(struct bme280_data *comp_data);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[out] data          : Pointer to the data buffer to store the read data.
 *  @param[in] len            : No of bytes to read.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval 0 -> Success
 *  @retval > 0 -> Failure Info
 *
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr       : Register address.
 *  @param[in] data           : Pointer to the data buffer whose value is to be written.
 *  @param[in] len            : No of bytes to write.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BME280_OK -> Success
 *  @retval BME280_E_COMM_FAIL -> Communication failure.
 *
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

/*!
 * @brief Function reads temperature, humidity and pressure data in forced mode.
 *
 * @param[in] dev   :   Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval BME280_OK - Success.
 * @retval BME280_E_NULL_PTR - Error: Null pointer error
 * @retval BME280_E_COMM_FAIL - Error: Communication fail error
 * @retval BME280_E_NVM_COPY_FAILED - Error: NVM copy failed
 *
 */
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);

/*!
* @brief Init I2C bus on ESP32
*/

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_port, &conf);
    if (err != 0) {
        ESP_LOGE(TAG_BME, "Error initializing I2C");
        return err;
    }
    err = i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    return err;
}



void task_bme280_normal_mode(void *args)
{
	TaskArgs_t *queues = (TaskArgs_t *) args;
    QueueHandle_t *q = queues->p_msg_queue;
    QueueHandle_t *p_config_q = queues->p_config_queue;
    EventGroupHandle_t *p_evt_group = queues->p_reporting_event_group;
	uint32_t com_rslt;
    struct bme280_dev dev;
    
    struct identifier id;

    /* Variable to define the result */
    int8_t rslt = BME280_OK;

    /* Make sure to select BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC as needed */
    id.dev_addr = BME280_I2C_ADDR_PRIM;

    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    char msg_buf[128];
    /* Update interface pointer with the structure that contains both device address and file descriptor */
    dev.intf_ptr = &id;

    ESP_LOGI(TAG_BME, "Initializing i2c...");
    i2c_master_driver_initialize();
    
    ESP_LOGI(TAG_BME, "done.\n");

    /* Initialize the bme280 */
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
    {
        sprintf(msg_buf, "Failed to initialize the device (code %+d).\n", rslt);
        xQueueSend(*q, msg_buf, 0);
        ESP_LOGE(TAG_BME, "Failed to initialize the device (code %+d).\n", rslt);
        com_rslt=1;
    }
    ESP_LOGI(TAG_BME, "BME280 initialized");
	com_rslt = 0;

    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.standby_time = BME280_STANDBY_TIME_10_MS;
    dev.settings.filter = BME280_FILTER_COEFF_16;
    ESP_LOGD(TAG_BME, "Settings");
    com_rslt += bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &dev);
    ESP_LOGD(TAG_BME, "MODE");
	com_rslt += bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

    struct bme280_data data;
    int period = POLLING_PERIOD_MS;
	if (com_rslt == 0) {
		while(true) {
            int new_period;
            if (pdPASS == xQueueReceive(*p_config_q, &new_period, 0)) {
                ESP_LOGD(TAG_BME, "New period %d", new_period);
                period = new_period;
            };
            // Make this parameter configurable
			vTaskDelay(period / portTICK_PERIOD_MS);
            
			com_rslt = bme280_get_sensor_data(BME280_ALL, &data, &dev);

			if (com_rslt == 0) {
				
                sprintf(msg_buf, "{ \"t\":%.2f, \"p\":%.3f, \"rh\": %.3f }",
					data.temperature,
					data.pressure/100, // Pa -> hPa
					data.humidity);
                ESP_LOGI(TAG_BME, "%.2f degC / %.3f hPa / %.3f %%",
					data.temperature,
					data.pressure/100, // Pa -> hPa
					data.humidity);
                char *pBuf = msg_buf;
                xQueueSend(*q, &pBuf, 0);
                xEventGroupSetBits(*p_evt_group, REPORTING_BME280_BIT);
			} else {
				ESP_LOGE(TAG_BME, "measure error. code: %d", com_rslt);
			}
		}
	} else {
		ESP_LOGE(TAG_BME, "init or setting error. code: %d", com_rslt);
	}

	vTaskDelete(NULL);
}

/*!
 * @brief This function reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct identifier id;

    id = *((struct identifier *)intf_ptr);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (data != 0) {
        i2c_master_write_byte(cmd, id.dev_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
    }
    i2c_master_write_byte(cmd, id.dev_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        for (int i = 0; i < len; i++) {
            ESP_LOGD(TAG_BME, "0x%02x ", data[i]);
            if ((i + 1) % 16 == 0) {
                ESP_LOGD(TAG_BME, "\r\n");
            }
        }
        if (len % 16) {
            ESP_LOGD(TAG_BME, "\r\n");
        }
        return 0;
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_BME, "Bus is busy");
        return 1;
    } else {
        ESP_LOGW(TAG_BME, "Read failed");
        return 2; 
    }

    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs
 */
void user_delay_us(uint32_t period, void *intf_ptr)
{
    vTaskDelay(period / 1000 * portTICK_RATE_MS);
}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct identifier id;

    id = *((struct identifier *)intf_ptr);
     /* Implement the I2C write routine according to the target machine. */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, id.dev_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
   
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_BME, "Write OK");
        return BME280_OK;
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_BME, "Bus is busy");
        return 1;
    } else {
        ESP_LOGW(TAG_BME, "Write Failed");
        return 2;
    }
    return 3;
}

/*!
 * @brief This API used to print the sensor temperature, pressure and humidity data.
 */
void print_sensor_data(struct bme280_data *comp_data)
{
    float temp, press, hum;

#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
}

/*!
 * @brief This API reads the sensor temperature, pressure and humidity data in forced mode.
 */
// int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
// {
//     /* Variable to define the result */
//     int8_t rslt = BME280_OK;

//     /* Variable to define the selecting sensors */
//     uint8_t settings_sel = 0;

//     /* Variable to store minimum wait time between consecutive measurement in force mode */
//     uint32_t req_delay;

//     /* Structure to get the pressure, temperature and humidity values */
//     struct bme280_data comp_data;

//     /* Recommended mode of operation: Indoor navigation */
//     dev->settings.osr_h = BME280_OVERSAMPLING_1X;
//     dev->settings.osr_p = BME280_OVERSAMPLING_16X;
//     dev->settings.osr_t = BME280_OVERSAMPLING_2X;
//     dev->settings.filter = BME280_FILTER_COEFF_16;

//     settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

//     /* Set the sensor settings */
//     rslt = bme280_set_sensor_settings(settings_sel, dev);
//     if (rslt != BME280_OK)
//     {
//         printf("Failed to set sensor settings (code %+d).", rslt);

//         return rslt;
//     }

//     printf("Temperature, Pressure, Humidity\n");

//     /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
//      *  and the oversampling configuration. */
//     req_delay = bme280_cal_meas_delay(&dev->settings);

//     /* Continuously stream sensor data */
//     // while (1)
//     // {
//     //     /* Set the sensor to forced mode */
//     //     rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
//     //     if (rslt != BME280_OK)
//     //     {
//     //         printf("Failed to set sensor mode (code %+d).", rslt);
//     //         break;
//     //     }

//     //     /* Wait for the measurement to complete and print data */
//     //     dev->delay_us(req_delay, dev->intf_ptr);
//     //     rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
//     //     if (rslt != BME280_OK)
//     //     {
//     //         printf("Failed to get sensor data (code %+d).", rslt);
//     //         break;
//     //     }

//     //     print_sensor_data(&comp_data);
//     // }

//     return rslt;
// }