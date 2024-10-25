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

// enum TimingBudget : uint16_t {
//   TIMING_BUDGET_100MS = 100,
//   TIMING_BUDGET_200MS = 200,
//   TIMING_BUDGET_500MS = 500,
// };

class VL53L1XComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  void set_range_status_sensor(sensor::Sensor *range_status_sensor) { range_status_sensor_ = range_status_sensor; }
  void config_distance_mode(DistanceMode distance_mode ) { distance_mode_ = distance_mode; }

  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override;

 protected:
  DistanceMode distance_mode_{LONG};
  //TimingBudget timing_budget_{TIMING_BUDGET_500MS};

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
    READ_CHIP_ID_FAILED,
    WRONG_CHIP_ID,
    START_CONTINUOUS_RANGING_FAILED,
    START_ONESHOT_RANGING_FAILED,
    DATA_READY_FAILED,
    DATA_READY_TIMEOUT,
    INTERRUPT_POLARITY_FAILED,
    CLEAR_INTERRUPT_FAILED,
    STOP_RANGING_FAILED,
    TIMING_BUDGET_FAILED,
    INTERM_PERIOD_FAILED,
    DISTANCE_MODE_FAILED,
    TOO_MANY_DATA_READY_ATTEMPTS,
    GET_DISTANCE_FAILED,
    GET_RANGE_STATUS_FAILED,
  } error_code_{NONE};

  
  // Internal state machine to make sure no blocking execution in loop()
  enum State {
    STARTING_UP = 0,
    SETUP_COMPLETE,
    IDLE,
    RANGING_STARTED,
    WAITING_FOR_RANGING,
    CHECK_DATA_READY,
    READ_AND_PUBLISH,
  } state_{STARTING_UP};

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

  bool get_distance();
  bool get_range_status();

  bool vl53l1x_write_bytes(uint16_t a_register, const uint8_t *data, uint8_t len);
  bool vl53l1x_write_byte(uint16_t a_register, uint8_t data);
  bool vl53l1x_write_bytes_16(uint8_t a_register, const uint16_t *data, uint8_t len);
  bool vl53l1x_write_byte_16(uint16_t a_register, uint16_t data);

  bool vl53l1x_read_byte(uint16_t a_register, uint8_t *data); 
  bool vl53l1x_read_bytes(uint16_t a_register, uint8_t *data, uint8_t len);

  bool vl53l1x_read_byte_16(uint16_t a_register, uint16_t *data);
  bool vl53l1x_read_bytes_16(uint16_t a_register, uint16_t *data, uint8_t len);
  
  uint16_t data_ready_retries_{0};
  uint16_t time_to_wait_for_ranging_{0};

  bool distance_mode_overriden_{false};

  uint16_t sensor_id_{0};

  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *range_status_sensor_{nullptr};
};

}  // namespace vl53l1x
}  // namespace esphome
