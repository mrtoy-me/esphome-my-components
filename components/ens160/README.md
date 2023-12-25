
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
