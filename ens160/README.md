
## ENS160 - ScioSense Digital Metal-Oxide Multi-Gas Sensor
The ENS160 is a digital multi-gas sensor which allows the detection of a wide range of volatile organic compounds. It's algorithms calculate Total Volatile Organic Carbon, equivalent Carbon Dioxide and air quality index (AQI) and
also perform humidity and temperature compensation.

This ESPHOME component is based on ScioSense ENS160 Arduino library together with the Sparkfun ENS160 Arduino library which was released under the [MIT License](http://opensource.org/licenses/MIT). The [ScioSense ENS160 datasheet](https://www.sciosense.com/products/environmental-sensors/digital-multi-gas-sensor/)
was also used extensively.

### Usage

Example configuration entry
```
	sensor:
	  - platform: ens160
        aqi:
	      name: "ENS160 Air Quality Index"
	      unit_of_measurement: "AQI"
	      accuracy_decimals: 0
	    tvoc:
	      name: "ENS160 Total Volatile Organics"
	      unit_of_measurement: "ppb"
	      accuracy_decimals: 0
	    eco2:
	      name: "ENS160 Equivalent Carbon Dioxide"
	      unit_of_measurement: "ppm"
	      accuracy_decimals: 0
	    compensation:
	      temperature_source: "id_temperature_sensor"
	      humidity_source: "id_humidity_sensor"
```   
      
Configuration variables:
eco2 (Required): The information for the CO₂eq. sensor.

name (Required, string): The name for the CO₂eq sensor.

id (Optional, ID): Set the ID of this sensor for use in lambdas.

All other options from Sensor.

tvoc (Required): The information for the total Volatile Organic Compounds sensor.

name (Required, string): The name for the humidity sensor.

id (Optional, ID): Set the ID of this sensor for use in lambdas.

All other options from Sensor.

store_baseline (Optional, boolean): Store the sensor baselines persistently when calculated or updated. Defaults to yes.

address (Optional, int): Manually specify the I²C address of the sensor. Defaults to 0x58.

update_interval (Optional, Time): The interval to check the sensor. Defaults to 1s.

Advanced:

baseline (Optional): The block containing baselines for calibration purposes. See Calibrating Baseline for more info.

eco2_baseline (Required, int): The eCO2 baseline for calibration purposes. After OTA, this value is used to calibrate the sensor.

tvoc_baseline (Required, int): The TVOC baseline for calibration purposes. After OTA, this value is used to calibrate the sensor.

eco2_baseline (Optional): The information for the CO₂eq. sensor baseline value. Baseline value is published in decimals.

name (Required, string): The name for the CO₂eq baseline value sensor.

id (Optional, ID): Set the ID of this sensor for use in lambdas.

All other options from Sensor.

tvoc_baseline (Optional): The information for the TVOC baseline value sensor. Baseline value is published in in decimals.

name (Required, string): The name for the TVOC baseline value sensor.

id (Optional, ID): Set the ID of this sensor for use in lambdas.

All other options from Sensor.

compensation (Optional): The block containing sensors used for compensation.

temperature_source (Optional, ID): Give an external temperature sensor ID here. This can improve the sensor’s internal calculations.

humidity_source (Optional, ID): Give an external humidity sensor ID here. This can improve the sensor’s internal calculations.



 