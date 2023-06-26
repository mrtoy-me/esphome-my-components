
## ESPHome component for VL53L1X and VL53L4CD
STMicroelectronic VL53L1X ultra lite driver (STSW-IMG009), VL53L1 API Core
has been adapted and modified so it functions using the ESPHome component framework.
Calibration and ROI functions have not been implemented.

The STMicroelectronic VL53L1X ultra lite API Core is
Copyright (c) 2017, STMicroelectronics - All Rights Reserved
VL53L1 Core may be distributed under the terms of 'BSD 3-clause "New" or "Revised" License'

**Copyright notices are shown in vl53l1x.cpp and licence.md files**

## Usage: VL53L1X component
Copy components files to a components directory under your homeassistant's esphome directory
The following yaml can then be used so ESPHome accesses the component files:

***external_components:***
  ***- source: components***

The component uses the sensor's default i2C address of 0x29
Calibration functions have not been implemented.
Timing budget (measurement period) is fixed at 500ms, so update interval should be 1 second or greater
YAML Configuration of ***distance_mode:*** can be either ***short*** or ***long***
However, VL53L4CD sensor can only be short and if VL53L4CD is detected, distance mode is set to short

Two sensors must be configured ***distance:*** and ***range_status:***
Distance has units mm while range status gives the status code of the distance measurement
The following range status descriptions are a summary of explanations provided in STMicroelectronic VL53L1X ultra lite driver, UM2510 user manual.
Range status values are as follows:
0 = RANGE VALID 

1 = SIGMA FAIL WARNING  
(poor measurement repeatability or standard deviation)

2 = SIGNAL FAIL WARNING
(return signal is too week to return a good measurement)

3 = OUT OF BOUNDS ERROR
(generally occurs when target is at or more than sensor maximum distance)

4 = WRAP AROUND ERROR
(occurs when the target is very reflective and the
distance to the target is more than sensor maximum distance)

5 = UNDEFINED

# Example YAML

external_components:
  - source: components

#example configure I2C
i2c:
  - id: bus_a 
    sda: 21
    scl: 22
    scan: true

sensor:
  - platform: vl53l1x
    i2c_id: bus_a
    distance_mode: long
    distance:
      name: "Distance"
    range_status:
      name: "Range Status"
    update_interval: 60s