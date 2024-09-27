#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l1x {

enum DistanceMode {
  SHORT = 0,
  LONG,
};

class VL53L1XComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_range_status_sensor(sensor::Sensor *range_status_sensor) { range_status_sensor_ = range_status_sensor; }
  void set_timeout_sensor(sensor::Sensor *timeout_sensor) { timeout_sensor_ = timeout_sensor; }
  void config_distance_mode(DistanceMode distance_mode ) { distance_mode_ = distance_mode; }

  bool soft_reset();
  bool initialise();
  void restart();

  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override;

 protected:
  DistanceMode distance_mode_{LONG};

  uint16_t distance_{0};
  
  enum RangeStatus {
    RANGE_VALID  = 0,
    SIGMA_FAIL_WARNING,
    SIGNAL_FAIL_WARNING,
    OUT_OF_BOUNDS_FAIL,
    WRAP_AROUND_FAIL,
    UNDEFINED,
  } range_status_{UNDEFINED};

  enum ErrorCode {
    NONE = 0,
    SOFT_RESET_FAILED,
    BOOT_STATE_FAILED,
    BOOT_TIMEOUT,
    CONFIGURATION_FAILED,
    WRONG_CHIP_ID,
    START_RANGING_FAILED,
    DATA_READY_FAILED,
    DATA_READY_TIMEOUT,
    CLEAR_INTERRUPT_FAILED,
    STOP_RANGING_FAILED,
    COMMUNICATION_FAILED,
    TIMING_BUDGET_FAILED,
    INTERM_PERIOD_FAILED,
    DISTANCE_MODE_FAILED,
  } error_code_{NONE};

  bool clear_interrupt();
  bool start_ranging();
  bool start_oneshot_ranging();
  bool stop_ranging();
  bool check_for_dataready(bool *is_dataready);
  
  bool set_timing_budget(uint16_t timing_budget_ms);
  bool get_timing_budget(uint16_t *timing_budget_ms);

  bool set_distance_mode(DistanceMode mode);
  bool get_distance_mode(DistanceMode *mode);

  bool set_intermeasurement_period(uint16_t intermeasurement_ms);
  bool get_intermeasurement_period(uint16_t *intermeasurement_ms);

  bool get_distance(uint16_t *distance);
  bool get_range_status();

  i2c::ErrorCode vl53l1x_write_register(uint16_t a_register, const uint8_t *data, size_t len);
  i2c::ErrorCode vl53l1x_read_register(uint16_t a_register, uint8_t *data, size_t len);

  bool vl53l1x_write_bytes(uint16_t a_register, const uint8_t *data, uint8_t len);
  bool vl53l1x_write_byte(uint16_t a_register, uint8_t data);
  bool vl53l1x_write_bytes_16(uint8_t a_register, const uint16_t *data, uint8_t len);
  bool vl53l1x_write_byte_16(uint16_t a_register, uint16_t data);

  bool vl53l1x_read_byte(uint16_t a_register, uint8_t *data); 
  bool vl53l1x_read_bytes(uint16_t a_register, uint8_t *data, uint8_t len);

  bool vl53l1x_read_byte_16(uint16_t a_register, uint16_t *data);
  bool vl53l1x_read_bytes_16(uint16_t a_register, uint16_t *data, uint8_t len);
  
  uint32_t last_loop_time_{0};
  uint32_t start_ranging_time_{0};
  uint32_t number_timeouts_ {0};

  bool distance_mode_overriden_{false};
  bool run_loop_ {false};
  bool have_new_distance_ {false};
  bool have_new_range_status_ {false};

  uint16_t sensor_id_{0};

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *range_status_sensor_{nullptr};
  sensor::Sensor *timeout_sensor_{nullptr};
};

}  // namespace vl53l1x
}  // namespace esphome
