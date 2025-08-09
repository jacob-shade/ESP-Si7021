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
 
 /* ---------------------------------------------------------------------------
  * PRIVATE PROTOTYPES
  */
 
 static esp_err_t _getSensorReading(i2c_master_dev_handle_t dev, const uint8_t command,
                                    float *output, float (*fn)(const uint16_t));
 static esp_err_t _readResponseBytes(i2c_master_dev_handle_t dev, uint8_t *output,
                                     size_t nbytes);
 static esp_err_t _writeCommandBytes(i2c_master_dev_handle_t dev, const uint8_t *i2c_command,
                                     size_t nbytes);
 static float _rh_code_to_pct(const uint16_t rh_code);
 static float _temp_code_to_celsius(const uint16_t temp_code);
 
 /* ---------------------------------------------------------------------------
  * PUBLIC INTERFACE
  */
 
 /* SENSOR READINGS */
 
 esp_err_t readHumidity(i2c_master_dev_handle_t dev, float *humidity) {
     return _getSensorReading(dev, SI7021_READ_RH, humidity, &_rh_code_to_pct);
 }
 
 esp_err_t readTemperature(i2c_master_dev_handle_t dev, float *temperature) {
     return _getSensorReading(dev, SI7021_READ_TEMP, temperature, &_temp_code_to_celsius);
 }
 
 static esp_err_t readTemperatureAfterHumidity(i2c_master_dev_handle_t dev, float *temperature) {
     return _getSensorReading(dev, SI7021_READ_TEMP_PREV_RH, temperature, &_temp_code_to_celsius);
 }
 
 esp_err_t readSensors(i2c_master_dev_handle_t dev, si7021_t *sensor_data) {
     esp_err_t ret = readHumidity(dev, &sensor_data->humidity);
     if (ret != ESP_OK) return ret;
     return readTemperatureAfterHumidity(dev, &sensor_data->temperature);
 }
 
 /* DEVICE IDENTIFICATION AND INFORMATION */
 
 esp_err_t readSerialNumber(i2c_master_dev_handle_t dev, uint8_t *serial) {
     // not performing the first access prior to the second access; works in practice
     esp_err_t ret = _writeCommandBytes(dev, READ_ID_SECOND_ACCESS, 2);
     if (ret != ESP_OK) return ret;
 
     vTaskDelay(pdMS_TO_TICKS(100));
 
     uint8_t buf[4];
     ret = _readResponseBytes(dev, buf, 4);
     if (ret != ESP_OK) return ret;
 
     *serial = buf[0];
     return ESP_OK;
 }
 
 esp_err_t readFirmwareRevision(i2c_master_dev_handle_t dev, uint8_t *revision) {
     esp_err_t ret = _writeCommandBytes(dev, READ_FW_REVISION, 2);
     if (ret != ESP_OK) return ret;
 
     vTaskDelay(pdMS_TO_TICKS(100));
 
     uint8_t buf[2];
     ret = _readResponseBytes(dev, buf, 2);
     if (ret != ESP_OK) return ret;
 
     *revision = buf[0];
     return ESP_OK;
 }
 
 /* REGISTER SETTINGS */
 
 esp_err_t readRegister(i2c_master_dev_handle_t dev, const uint8_t command, uint8_t *settings) {
     esp_err_t ret = _writeCommandBytes(dev, &command, 1);
     if (ret != ESP_OK) return ret;
 
     vTaskDelay(pdMS_TO_TICKS(100));
 
     uint8_t reg = 0;
     ret = _readResponseBytes(dev, &reg, 1);
     if (ret != ESP_OK) return ret;
 
     *settings = reg;
     return ESP_OK;
 }
 
 esp_err_t writeRegister(i2c_master_dev_handle_t dev, const uint8_t command, const uint8_t settings) {
     const uint8_t full_command[] = { command, settings };
     return _writeCommandBytes(dev, full_command, 2);
 }
 
 /* OTHER MISCELLANEOUS FEATURES */
 
 esp_err_t softwareReset(i2c_master_dev_handle_t dev) {
     const uint8_t command = SI7021_RESET;
     return _writeCommandBytes(dev, &command, 1);
 }
 
 /* ---------------------------------------------------------------------------
  * PRIVATE INTERFACE
  */
 
 /* GENERIC TASKS */
 
 static esp_err_t _getSensorReading(i2c_master_dev_handle_t dev, const uint8_t command,
                                    float *output, float (*fn)(const uint16_t)) {
     esp_err_t ret = _writeCommandBytes(dev, &command, 1);
     if (ret != ESP_OK) return ret;
 
     vTaskDelay(pdMS_TO_TICKS(100));
 
     uint8_t buf[2];
     ret = _readResponseBytes(dev, buf, 2);
     if (ret != ESP_OK) return ret;
 
     uint16_t bytes = ((uint16_t)buf[0] << 8) | buf[1];
     *output = fn(bytes);
     return ESP_OK;
 }
 
 /* hardware interaction */
 
 static esp_err_t _readResponseBytes(i2c_master_dev_handle_t dev, uint8_t *output, size_t nbytes) {
     return i2c_master_receive(dev, output, nbytes, I2C_TIMEOUT_MS);
 }
 
 static esp_err_t _writeCommandBytes(i2c_master_dev_handle_t dev, const uint8_t *i2c_command, size_t nbytes) {
     return i2c_master_transmit(dev, i2c_command, nbytes, I2C_TIMEOUT_MS);
 }
 
 /* conversion functions */
 
 static float _rh_code_to_pct(const uint16_t rh_code) {
     return ((125.0f * rh_code) / 65536.0f) - 6.0f;
 }
 
 static float _temp_code_to_celsius(const uint16_t temp_code) {
     return ((175.72f * temp_code) / 65536.0f) - 46.85f;
 }