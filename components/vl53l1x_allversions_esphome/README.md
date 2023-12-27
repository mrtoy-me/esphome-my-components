
## ESPHome component for VL53L1X and VL53L4CD
STMicroelectronic VL53L1X ultra lite driver (STSW-IMG009), VL53L1 API Core
has been adapted and modified so it functions using the ESPHome component framework.
Calibration and ROI functions have not been implemented.<BR>

The STMicroelectronic VL53L1X ultra lite API Core is
Copyright (c) 2017, STMicroelectronics - All Rights Reserved.<BR>
VL53L1 Core may be distributed under the terms of 'BSD 3-clause "New" or "Revised" License'

**All Copyright licences are shown in vl53l1x.cpp and licence.md files**

## NOTE: This component should function on all Esphome versions 2023 onwards. 

## Usage: VL53L1X component
Copy files in "vl53l1x_allversion_esphome" directory into a "vl53l1x" subdirectory under a "components" directory<BR>
under the "esphome" addon directory (of your homeassistant install)<BR>
For example:
```
esphome
├── components
   ├── vl53l1x
```
The following yaml can then be used to access the component:
```
external_components:
  - source: components
```
The component supports VL53L1X (up to 4000mm range) and VL53L4CD (up to 1300mm range) with default i2c address of 0x29.<BR>
Timing budget (measurement period) is set internally at 500ms. Ranging occurs continuously every 500ms, but measurements are published at the specified update interval. **Note: update interval should be greater than 1 second.**<BR>

The ***vl53l1x:*** configuration allows defining:<BR>
      ***distance_mode:*** which can be either ***short*** or ***long*** with default ***long***.<BR>
      ***update_interval:*** which defaults to 60s<BR>
**Note: the VL53L4CD sensor can only have distance_mode: short, if VL53L4CD is detected then distance mode is forced to ***short***.**<BR>

Two sensors can be configured ***distance:*** which is required and ***range_status:*** which is optional<BR>
Distance has units mm while range status gives the status code of the distance measurement.<BR>
The following range status descriptions are a summary of explanations provided in STMicroelectronic VL53L1X ultra lite driver, UM2510 user manual.<BR>
Range status values are as follows:<BR>

0 = RANGE VALID<BR>

1 = SIGMA FAIL WARNING<BR> 
(poor measurement repeatability or standard deviation)

2 = SIGNAL FAIL WARNING<BR> 
(return signal is too week to return a good measurement)

3 = OUT OF BOUNDS ERROR<BR> 
(generally occurs when target is at or more than sensor maximum distance)

4 = WRAP AROUND ERROR<BR> 
(occurs when the target is very reflective and the<BR> 
distance to the target is more than sensor maximum distance)<BR> 

5 = UNDEFINED<BR>

Three optional binary sensors can be configured as follows:<BR>
  ***range_valid:*** If configured this binary sensor will turn **ON** if range_status is VALID and **OFF** if Range Status is not VALID<BR>
  ***above_threshold:***  If configured ***above_distance:*** is required to specify the distance in mm **above** which the binary sensor will turn **ON**<BR>
  ***below_threshold:***  If configured ***below_distance:*** is required to specify the distance in mm **below** which the binary sensor will turn **ON**<BR>
  **Note: if Range Status is not valid, ***above_threshold:*** and ***below_threshold:*** binary sensors are set to OFF.**

## Example YAML
```
external_components:
  - source: components

#example configure I2C
i2c:
  - id: bus_a 
    sda: 21
    scl: 22
    scan: true

vl53l1x:
  distance_mode: long
  update_interval: 60s

sensor:
  - platform: vl53l1x
    i2c_id: bus_a
    distance:
      name: Distance
    range_status:
      name: Range Status

binary_sensor:
  - platform: vl53l1x
    range_valid:
      name: Distance Reading Valid
    above_threshold: 
      name: Above Threshold Alert
      above_distance: 100
    below_threshold: 
      name: Below Threshold Alert
      below_distance: 500  
```