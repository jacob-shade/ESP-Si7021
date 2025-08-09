# ESP-Si7021
An ESP-IDF component for the Silicon Labs Si7021-A20 Relative Humidity and Temperature Sensor with I2C Interface.

### Links:
- [Product Page](https://www.silabs.com/sensors/humidity/si7006-13-20-21-34)
- [Datasheet](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf)
- [Adafruit Breakout Board](https://www.adafruit.com/product/3251)
- [SparkFun Breakout Board](https://www.sparkfun.com/sparkfun-humidity-and-temperature-sensor-breakout-si7021.html)
- [ESP IDF I2C DOCS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32h2/api-reference/peripherals/i2c.html)

### Requirements
- ESP-IDF IoT Development Environment

### Interface
- Measure Relative Humidity
- Measure Temperature
- Read and Write Registers
- Read the Electronic Serial Number
- Read the Firmware Revision
- Perform a software reset of the sensor

### Relative Humidity & Temperature Read Order
Each time a relative humidity measurement is made a temperature measurement is 
also made for the purposes of temperature compensation of the relative humidity 
measurement. Therefore if you wish to read relative humidity and temperature, 
`readSensors()` will be the most efficient read as it only performs one 
temperature measurement.
