#include "ens160.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ens160 {

static const char *const TAG = "ens160.sensor";

// registers
static const uint8_t ENS160_PART_ID    = 0x00;
static const uint8_t ENS160_OP_MODE    = 0x10;
static const uint8_t ENS160_CONFIG     = 0x11;

static const uint8_t ENS160_TEMP_IN    = 0x13;
static const uint8_t ENS160_RH_IN      = 0x15;

static const uint8_t ENS160_STATUS     = 0x20;
static const uint8_t ENS160_DATA_AQI   = 0x21;
static const uint8_t ENS160_DATA_TVOC  = 0x22;
static const uint8_t ENS160_DATA_ECO2  = 0x24;

static const uint8_t ENS160_DATA_T     = 0x30;
static const uint8_t ENS160_DATA_RH    = 0x32;

// device id
static const uint16_t ENS160_DEVICE_ID = 0x0160;

// operating modes
static const uint8_t ENS160_DEEP_SLEEP  = 0x00;
static const uint8_t ENS160_IDLE        = 0x01;
static const uint8_t ENS160_STANDARD    = 0x02;
static const uint8_t ENS160_RESET       = 0xF0;

// mask on status register to extract operating state 
static const uint8_t ENS160_VALIDITY_FLAG_MASK = 0x0C;

// mask on status register to extract data ready bit  
static const uint8_t ENS160_DATA_READY_MASK    = 0x02;

// mask on aqi data register to extract aqi
static const uint8_t ENS160_DATA_AQI_MASK      = 0x07;

// ms delay to allow reset to complete 
static const uint8_t ENS160_RESET_DELAY = 10;

void ENS160Component::setup() {
  uint16_t id;
  uint8_t status;  
  uint8_t op_mode;   
   
  ESP_LOGCONFIG(TAG, "Setting up ENS160 sensor");
   
  if (this->read_register(ENS160_PART_ID, reinterpret_cast<uint8_t *>(&id), 2) != i2c::ERROR_OK) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return; 	
  }
  if (id != ENS160_DEVICE_ID ) {
    this->error_code_ = DEVICE_ID_INCORRECT;
    this->mark_failed();
    return; 	 
  }

  if (!this->write_byte(ENS160_OP_MODE, ENS160_RESET)) {
    this->error_code_ = WRITE_OPMODE_FAILED;
    this->mark_failed();
    return; 
  }
   
  delay(ENS160_RESET_DELAY);
   
  if (!this->read_byte(ENS160_STATUS,&status)) {
    this->error_code_ = READ_STATUS_FAILED;
    this->mark_failed();
    return; 
  }
  this->validity_flag_ = static_cast<ValidityFlag>((status & ENS160_VALIDITY_FLAG_MASK) >> 2);
   
  if (this->validity_flag_ == INVALID_OUTPUT) {
    this->error_code_ = VALIDITY_INVALID_OUTPUT;
    this->mark_failed();
    return; 
  }
   
  if (!this->read_byte(ENS160_OP_MODE,&op_mode)) {
    this->error_code_ = READ_OPMODE_FAILED;
    this->mark_failed();
    return;
  }
  
  // should be in idle after RESET but check anyway
  if (op_mode == ENS160_DEEP_SLEEP) {
    if (!this->write_byte(ENS160_OP_MODE, ENS160_IDLE)) {
      this->error_code_ = WRITE_OPMODE_FAILED;
      this->mark_failed();
      return;
    }
  }

  if (!this->read_byte(ENS160_OP_MODE,&op_mode)) {
    this->error_code_ = READ_OPMODE_FAILED;
    this->mark_failed();
    return; 
  }

  if (op_mode == ENS160_IDLE) {
   if (!this->write_byte(ENS160_OP_MODE, ENS160_STANDARD)) {
     this->error_code_ = WRITE_OPMODE_FAILED;
     this->mark_failed();
     return;
   }
  }

  if (!this->read_byte(ENS160_OP_MODE,&op_mode)) {
    this->error_code_ = READ_OPMODE_FAILED;
    this->mark_failed();
    return; 
  }	
	
	if (op_mode != ENS160_STANDARD) {
    this->error_code_ = STANDARD_OPMODE_FAILED;
    this->mark_failed();
    return; 
  }
}

void ENS160Component::dump_config() {
  ESP_LOGCONFIG(TAG, "ENS160:");

  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "  Error reading Device ID");
      break;
    case DEVICE_ID_INCORRECT:
      ESP_LOGE(TAG, "  Incorrect Device ID");
      break;
    case READ_STATUS_FAILED:
      ESP_LOGE(TAG, "  Error reading Device Status");
      break;   
    case VALIDITY_INVALID_OUTPUT:
      ESP_LOGE(TAG, "  Invalid Device Status - No valid output");
      break; 
    case READ_OPMODE_FAILED:
      ESP_LOGE(TAG, "  Error reading Operation Mode");
      break;
    case WRITE_OPMODE_FAILED:
      ESP_LOGE(TAG, "  Error writing Operation Mode");
      break;
    case STANDARD_OPMODE_FAILED:
      ESP_LOGE(TAG, "   Device failed to achieve Standard Operating Mode");
      break;  
    case NONE:
      ESP_LOGD(TAG, "  Setup successful");
      break;
  }
  
  // to do extract and display ENS160 firmware version
  // ESP_LOGCONFIG(TAG, "  Firmware:"); 
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "AQI:", this->aqi_sensor_);
  LOG_SENSOR("  ", "TVOC:", this->tvoc_sensor_);
  LOG_SENSOR("  ", "eCO2:", this->eco2_sensor_);
  
  if (this->temperature_sensor_ != nullptr && this->humidity_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Compensation:");
    ESP_LOGCONFIG(TAG, "    Temperature: '%s'", this->temperature_sensor_->get_name().c_str());
    ESP_LOGCONFIG(TAG, "    Relative Humidity: '%s'", this->humidity_sensor_->get_name().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Compensation: Not configured");
  }
}

void ENS160Component::update() {

  uint8_t op_mode, status, data_ready;

  uint8_t aqi;
  uint16_t tvoc, eco2;

  if (!this->read_byte(ENS160_STATUS,&status)) {
    ESP_LOGW(TAG, "Error reading status register");
    this->status_set_warning();
    return;
  }

  ESP_LOGV(TAG, "Status Register value: %u", status);

  data_ready = status & ENS160_DATA_READY_MASK; 
  this->validity_flag_ = static_cast<ValidityFlag>((status & ENS160_VALIDITY_FLAG_MASK) >> 2);

  switch (validity_flag_) {
    case NORMAL_OPERATION:
      if (data_ready != ENS160_DATA_READY_MASK) {
        ESP_LOGD(TAG, "ENS160 Sensor readings not available yet - Normal Operation but readings not ready"); 
        return;
      }
      break;
    case INITIAL_STARTUP:
      if (!this->initial_startup_) {
        this->initial_startup_ = true;
        ESP_LOGI(TAG, "ENS160 Sensor readings not available yet - Initial Start up requires 1 hour after first power on");
      }
      return;
    case WARMING_UP:
      if (!this->warming_up_) {
        this->warming_up_ = true;
        ESP_LOGI(TAG, "ENS160 Sensor readings not available yet - Warming up requires 3 minutes");
        this->update_compensation();
      }
      return;
    case INVALID_OUTPUT:
      ESP_LOGE(TAG, "ENS160 Invalid Status - No Invalid Output");
      this->status_set_warning();
      return;
  }

  if (!this->read_byte(ENS160_DATA_AQI,&aqi)) {
    ESP_LOGW(TAG, "Error reading AQI data register");
    this->status_set_warning();
    return;
  }
  aqi = aqi & ENS160_DATA_AQI_MASK;
  
  if (this->read_register(ENS160_DATA_TVOC, reinterpret_cast<uint8_t *>(&tvoc), 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error reading TVOC data register");
    this->status_set_warning();
    return;
  }

  if (this->read_register(ENS160_DATA_ECO2, reinterpret_cast<uint8_t *>(&eco2), 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error reading eCO2 data register");
    this->status_set_warning();
    return;
  }

  if (this->aqi_sensor_ != nullptr) {
    this->aqi_sensor_->publish_state(aqi);
    ESP_LOGV(TAG, "Published AQI: %d", aqi);
  }
  if (this->tvoc_sensor_ != nullptr) {
    this->tvoc_sensor_->publish_state(tvoc);
    ESP_LOGV(TAG, "Published TVOC: %d", tvoc);
  }
  if (this->eco2_sensor_ != nullptr) {
    this->eco2_sensor_->publish_state(eco2);
    ESP_LOGV(TAG, "Publish eCO2: %d", eco2);
  }

  this->update_compensation();
  this->status_clear_warning();
  
}

// write temperature and humidity compensation for improved accuracy of readings
void ENS160Component::update_compensation() {
  
  float compensation_temperature,compensation_humidity;

  if (this->temperature_sensor_ == nullptr || this->humidity_sensor_ == nullptr) {
    ESP_LOGD(TAG, "External compensation temperature/humidity values not available");
    return;
  }

  if (!this->get_temperature_conversion(&compensation_temperature)) return;

  float new_temperature = NAN;
  new_temperature = float(this->temperature_sensor_->state);
  if (std::isnan(new_temperature) || new_temperature < 0.0f || new_temperature > 40.0f) {
    ESP_LOGD(TAG, "Invalid external temperature - compensation values not updated");
    return;
  }
  else {
    ESP_LOGV(TAG, "Current external temperature: %0.2f °C", new_temperature);
  }

  if (!this->get_humidity_conversion(&compensation_humidity)) return;

  float new_humidity = NAN;
  new_humidity = float(this->humidity_sensor_->state);
  if (std::isnan(new_humidity) || new_humidity < 10.0f || new_humidity > 90.0f) {
    ESP_LOGD(TAG, "Invalid external humidity - compensation values not updated");
    return;
  }
  else {
    ESP_LOGV(TAG, "Current external humidity: %0.2f %%", new_humidity);
  }
  
  if (std::abs(compensation_temperature - new_temperature) < 0.1f && std::abs(compensation_humidity - new_humidity) < 0.2f) {
    ESP_LOGD(TAG, "Compensation values not updated - insignificant change in external temperature and humidity");
    return;
  }

  uint16_t raw_temperature = (uint16_t)((new_temperature + 273.15f) * 64.0f); 
  uint16_t raw_humidity = (uint16_t)(new_humidity * 512.0f);

  uint32_t raw_compensation = (raw_humidity << 16) + raw_temperature;

  if (this->write_register(ENS160_TEMP_IN, reinterpret_cast<const uint8_t *>(&raw_compensation), 4) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error writing temperature and humidity compensation");
    this->status_set_warning();
    return;
  }
  ESP_LOGD(TAG, "Compensation values updated");
  return; 
}

// read temperature compensation value previous written set 
bool ENS160Component::get_temperature_conversion(float* temperature) {

  int16_t temperature_conversion; 

  if (this->read_register(ENS160_DATA_T, reinterpret_cast<uint8_t *>(&temperature_conversion), 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error reading temperature compensation register");
    this->status_set_warning();
    return false;
  }
  *temperature = ((float)temperature_conversion / 64.0f) - 273.15f;
  ESP_LOGV(TAG, "Set compensation temperature: %.2f °C", *temperature);
  return true;
}


// read humidity compensation value previous written set 
bool ENS160Component::get_humidity_conversion(float* humidity) {

  uint16_t humidity_conversion; 

  if (this->read_register(ENS160_DATA_RH, reinterpret_cast<uint8_t *>(&humidity_conversion), 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error reading humidity compensation register");
    this->status_set_warning();
    return false;
  }
  *humidity = (float)humidity_conversion / 512.0f;
  ESP_LOGV(TAG, "Set compensation humidity: %.2f %%", *humidity);
  return true;
}

float ENS160Component::get_setup_priority() const { return setup_priority::DATA; }

} // namespace ens160
} // namespace esphome