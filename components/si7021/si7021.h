/* ************************************************************************ *
* ESP-IDF header file for the SI7021 sensor
*
* Datasheet:
* https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
* ************************************************************************ */

#ifndef __SI7021_H
#define __SI7021_H

// ---------------------------------------------------------------------------
// INCLUDES

#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"

// ---------------------------------------------------------------------------
// DEFINES

/* i2c RELATED VALUES */

#define I2C_TIMEOUT_MS              1000

#define ACK_CHECK_EN                0x1
#define ACK_CHECK_DIS               0x0

#define ACK_VAL                     0x0
#define NACK_VAL                    0x1

#define SI7021_I2C_ADDR             0x40

/* SENSOR COMMANDS */

#if CONFIG_USE_CLOCK_STRETCHING
    #define SI7021_READ_RH          0xE5
#else
    #define SI7021_READ_RH          0xF5
#endif

#if CONFIG_USE_CLOCK_STRETCHING
    #define SI7021_READ_TEMP        0xE3
#else
    #define SI7021_READ_TEMP        0xF3
#endif

#define SI7021_READ_TEMP_PREV_RH    0xE0

/* REGISTER COMMANDS */

#define SI7021_USER_REG_READ        0xE7
#define SI7021_USER_REG_WRITE       0xE6

#define SI7021_HTRE_REG_READ        0x11
#define SI7021_HTRE_REG_WRITE       0x51

/* MISCELLANEOUS COMMANDS */

#define SI7021_HEATER_ON            0x3E
#define SI7021_HEATER_OFF           0x3A

#define SI7021_RESET                0xFE

/* ------------------------------------------------------------------------- */
/* STRUCTURES */

typedef struct si7021 {
    float humidity;
    float temperature;
} si7021_t;

/* ------------------------------------------------------------------------- */
/* CONSTANTS */

static const uint8_t READ_ID_FIRST_ACCESS[]  = { 0xFA, 0x0F };
static const uint8_t READ_ID_SECOND_ACCESS[] = { 0xFC, 0xC9 };

static const uint8_t READ_FW_REVISION[]      = { 0x84, 0xB8 };

/* --------------------------------------------------------------------------- */
/* FUNCTION PROTOTYPES */

esp_err_t readHumidity(i2c_master_dev_handle_t dev, float *humidity);

esp_err_t readTemperature(i2c_master_dev_handle_t dev, float *temperature);

esp_err_t readSensors(i2c_master_dev_handle_t dev, si7021_t *sensor_data);

esp_err_t softwareReset(i2c_master_dev_handle_t dev);

#endif