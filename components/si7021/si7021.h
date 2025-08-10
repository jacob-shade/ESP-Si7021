/* ************************************************************************ *
* ESP-IDF header file for the SI7021 sensor
*
* Datasheet:
* https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
* ************************************************************************ */

#ifndef __SI7021_H
#define __SI7021_H

#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "hal/gpio_types.h"
#include "driver/i2c_master.h"

/* I2C RELATED VALUES */
#define I2C_TIMEOUT_MS              1000
#define I2C_SPEED_HZ                100000
#define I2C_SCL_WAIT_US             0

#define SI7021_I2C_ADDR             0x40

/* I2C COMMANDS */
#if CONFIG_USE_CLOCK_STRETCHING
    #define SI7021_READ_RH          0xE5
    #define SI7021_READ_TEMP        0xE3
#else
    #define SI7021_READ_RH          0xF5
    #define SI7021_READ_TEMP        0xF3
#endif

#define SI7021_READ_TEMP_PREV_RH    0xE0
#define SI7021_USER_REG_READ        0xE7
#define SI7021_USER_REG_WRITE       0xE6
#define SI7021_HTRE_REG_READ        0x11
#define SI7021_HTRE_REG_WRITE       0x51
#define SI7021_HEATER_ON            0x3E
#define SI7021_HEATER_OFF           0x3A
#define SI7021_RESET                0xFE

/* STRUCTS */

typedef struct si7021 {
    float humidity;
    float temperature;
} si7021_t;

typedef struct {
    esp_err_t (*open)(i2c_master_bus_handle_t i2c_bus, i2c_master_dev_handle_t *dev);
    esp_err_t (*close)(i2c_master_dev_handle_t dev);
    esp_err_t (*readHumidity)(i2c_master_dev_handle_t dev, float *humidity);
    esp_err_t (*readTemperature)(i2c_master_dev_handle_t dev, float *temperature);
    esp_err_t (*readSensors)(i2c_master_dev_handle_t dev, si7021_t *readings);
    esp_err_t (*reset)(i2c_master_dev_handle_t dev);
  } si7021_api_t;
  
  extern const si7021_api_t si7021;

/* FUNCTION PROTOTYPES */

/**
    Sets up the Si7021 device on the given i2c bus.

    Params:
    i2c_master_bus_handle_t i2c_bus: initialized reference to the bus
    i2c_master_dev_handle_t dev: to be initialized refrence of Si7021 dev

    Returns esp_err_t error code
*/
esp_err_t si7021_open(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev);

/**
    Deletes the Si7021 device on the given i2c bus.

    Params:
    i2c_master_dev_handle_t dev: refrence of Si7021 dev

    Returns esp_err_t error code
*/
esp_err_t si7021_close(i2c_master_dev_handle_t dev);

/**
    Reads the Relative Humidity from the Si7021

    Params:
    i2c_master_dev_handle_t dev: refrence of Si7021 dev
    float *humidity: stores the humidity value (output parameter)

    Returns esp_err_t error code
*/
esp_err_t readHumidity(i2c_master_dev_handle_t dev, float *humidity);

/**
    Reads the Temperature from the Si7021

    Params:
    i2c_master_dev_handle_t dev: refrence of Si7021 dev
    float *temperature: stores the temperature value (output parameter)

    Returns esp_err_t error code
*/
esp_err_t readTemperature(i2c_master_dev_handle_t dev, float *temperature);

/**
    Reads the Relative Humidity and Temperature from the Si7021

    Params:
    i2c_master_dev_handle_t dev: refrence of Si7021 dev
    si7021_t readings: stores humidity and temperature (output parameter)

    Returns esp_err_t error code
*/
esp_err_t readSensors(i2c_master_dev_handle_t dev, si7021_t *readings);

/**
    Resets the Si7021 device to factory default settings

    Params:
    i2c_master_dev_handle_t dev: refrence of Si7021 dev

    Returns esp_err_t error code
*/
esp_err_t reset(i2c_master_dev_handle_t dev);

#endif