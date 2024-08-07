#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/sensirion_common/i2c_sensirion.h"

#include <cinttypes>

namespace esphome {
namespace sht3xd {

/// This class implements support for the SHT3x-DIS family of temperature+humidity i2c sensors.
class SHT3XDComponent : public PollingComponent, public sensirion_common::SensirionI2CDevice {
 public:
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  void set_heater_enabled(bool heater_enabled) { heater_enabled_ = heater_enabled; }

 protected:
  enum ErrorCode {
    NONE = 0,
    READ_SERIAL_STRETCHED_FAILED,
    READ_SERIAL_FAILED,
    WRITE_HEATER_MODE_FAILED,
  } error_code_{NONE};
  
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  bool heater_enabled_{true};
  uint32_t serial_number_{0};
};

}  // namespace sht3xd
}  // namespace esphome
