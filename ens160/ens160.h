#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ens160 {


class ENS160Component : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
  void set_aqi_sensor(sensor::Sensor *aqi_sensor) { aqi_sensor_ = aqi_sensor; }
  void set_tvoc_sensor(sensor::Sensor *tvoc_sensor) { tvoc_sensor_ = tvoc_sensor; }
  void set_eco2_sensor(sensor::Sensor *eco2_sensor) { eco2_sensor_ = eco2_sensor; }
  
  void set_humidity_sensor(sensor::Sensor *humidity) { humidity_sensor_ = humidity; }
  void set_temperature_sensor(sensor::Sensor *temperature) { temperature_sensor_ = temperature; }
 
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;
 
 protected:
  void update_compensation();
  bool get_temperature_conversion(float* temperature);
  bool get_humidity_conversion(float* humidity);
  
  bool warming_up_{false};
  bool initial_startup_{false};
  
  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    DEVICE_ID_INCORRECT,
    READ_STATUS_FAILED,
    VALIDITY_INVALID_OUTPUT,
    READ_OPMODE_FAILED,
    WRITE_OPMODE_FAILED,
    STANDARD_OPMODE_FAILED,
  } error_code_{NONE};
	
  enum ValidityFlag {
    NORMAL_OPERATION = 0,
    WARMING_UP,
    INITIAL_STARTUP,
    INVALID_OUTPUT,
  } validity_flag_;
	
  sensor::Sensor *aqi_sensor_{nullptr};
  sensor::Sensor *tvoc_sensor_{nullptr};
  sensor::Sensor *eco2_sensor_{nullptr};
 
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  
};

}  // namespace ens160
}  // namespace esphome