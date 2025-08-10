/* ************************************************************************ *
* ESP-IDF driver for the SI7021 sensor
* 
* Datasheet:
* https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
* ************************************************************************ */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "si7021.h"

const si7021_api_t si7021 = {
    .open = si7021_open,
    .close = si7021_close,
    .readHumidity = readHumidity,
    .readTemperature = readTemperature,
    .readSensors = readSensors,
    .reset = reset,
};

esp_err_t si7021_open(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev) {
    if (!bus || !dev) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI7021_I2C_ADDR,
        .scl_speed_hz = I2C_SPEED_HZ,
        .scl_wait_us = I2C_SCL_WAIT_US,
    };
    return i2c_master_bus_add_device(bus, &cfg, dev);
}

esp_err_t si7021_close(i2c_master_dev_handle_t dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return i2c_master_bus_rm_device(dev);
}

esp_err_t readHumidity(i2c_master_dev_handle_t dev, float *humidity) {
    if (!dev || !humidity) return ESP_ERR_INVALID_ARG;

    // command write
    const uint8_t cmd = SI7021_READ_RH;
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return ret;

    // sensor conversion time
    vTaskDelay(pdMS_TO_TICKS(100));

    // read 2 bytes
    uint8_t buf[2] = {0};
    ret = i2c_master_receive(dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return ret;

    uint16_t code = ((uint16_t)buf[0] << 8) | buf[1];
    *humidity = ((125.0f * code) / 65536.0f) - 6.0f;
    return ESP_OK;
}

esp_err_t readTemperature(i2c_master_dev_handle_t dev, float *temperature) {
    if (!dev || !temperature) return ESP_ERR_INVALID_ARG;

    // command write
    const uint8_t cmd = SI7021_READ_TEMP;
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return ret;

    // sensor conversion time
    vTaskDelay(pdMS_TO_TICKS(100));

    // read 2 bytes
    uint8_t buf[2] = {0};
    ret = i2c_master_receive(dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return ret;

    uint16_t code = ((uint16_t)buf[0] << 8) | buf[1];
    *temperature = ((175.72f * code) / 65536.0f) - 46.85f;
    return ESP_OK;
}

esp_err_t readSensors(i2c_master_dev_handle_t dev, si7021_t *readings) {
    if (!dev || !readings) return ESP_ERR_INVALID_ARG;

    // humidity
    esp_err_t ret = readHumidity(dev, &readings->humidity);
    if (ret != ESP_OK) return ret;

    // temperature after humidity (use previous-temp command)
    const uint8_t cmd = SI7021_READ_TEMP_PREV_RH;
    ret = i2c_master_transmit(dev, &cmd, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t buf[2] = {0};
    ret = i2c_master_receive(dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return ret;

    uint16_t code = ((uint16_t)buf[0] << 8) | buf[1];
    readings->temperature = ((175.72f * code) / 65536.0f) - 46.85f;
    return ESP_OK;
}

esp_err_t reset(i2c_master_dev_handle_t dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    const uint8_t cmd = SI7021_RESET;
    return i2c_master_transmit(dev, &cmd, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}