# ESP-Si7021
An ESP-IDF component for the Silicon Labs Si7021-A20 Relative Humidity and Temperature Sensor with I2C Interface.

### Links:
- [Product Page](https://www.silabs.com/sensors/humidity/si7006-13-20-21-34)
- [Datasheet](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf)
- [Adafruit Breakout Board](https://www.adafruit.com/product/3251)
- [SparkFun Breakout Board](https://www.sparkfun.com/sparkfun-humidity-and-temperature-sensor-breakout-si7021.html)
- [ESP IDF I2C Docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/api-reference/peripherals/i2c.html)

### Requirements
- ESP-IDF IoT Development Environment

### API
Data Structures
```
typedef struct si7021 {
    float humidity;
    float temperature;
} si7021_t;
```

### Functions
`esp_err_t si7021_open(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev);`  
`esp_err_t si7021_close(i2c_master_dev_handle_t dev);`  
`esp_err_t readHumidity(const i2c_port_t i2c_num, float *humidity);`  
`esp_err_t readTemperature(const i2c_port_t i2c_num, float *temperature);`  
`esp_err_t readSensors(const i2c_port_t i2c_num, si7021_t *sensor_data);`  
`esp_err_t reset(const i2c_port_t i2c_num);`

### Example Use
```c
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
#define I2C_GLITCH_IGNORE_CNT  7

void app_main(void) {
    const char *TAG = "SI7021_APP";
    si7021_t reading;

    // Create master bus
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_GLITCH_IGNORE_CNT,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));

    // Add SI7021 device to the bus
    i2c_master_dev_handle_t si7021_dev;
    ESP_ERROR_CHECK(si7021.open(i2c_bus, &si7021_dev));

    while (1) {
        esp_err_t ret = si7021.readSensors(si7021_dev, &reading);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Humidity: %.2f %%  Temperature: %.2f C",
                     reading.humidity, reading.temperature);
        } else {
            ESP_LOGE(TAG, "readSensors failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Cleanup if you ever exit the loop:
    ESP_ERROR_CHECK(si7021.close(si7021_dev));
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
}
```

### Relative Humidity & Temperature Read Order
Each time a relative humidity measurement is made a temperature measurement is 
also made for the purposes of temperature compensation of the relative humidity 
measurement. Therefore if you wish to read relative humidity and temperature, 
`readSensors()` will be the most efficient read as it only performs one 
temperature measurement.
