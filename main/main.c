#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "si7021.h"

#define I2C_MASTER_SDA         GPIO_NUM_0
#define I2C_MASTER_SCL         GPIO_NUM_1
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000

void app_main(void) {
    const char *TAG = "SI7021_APP";
    si7021_t reading;

    i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));

    i2c_master_dev_handle_t si7021_dev;
    i2c_device_config_t si7021_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI7021_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 0,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &si7021_cfg, &si7021_dev));    

    while (1) {
        esp_err_t ret = readSensors(si7021_dev, &reading);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Humidity: %.2f %%  Temperature: %.2f C",
                     reading.humidity, reading.temperature);
        } else {
            ESP_LOGE(TAG, "readSensors failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}