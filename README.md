# ESP32-C6 &amp; ESP32-H2 Zigbee Sensor (Analog NTC) Custom Cluster Reporting

This project is about reporting temperature values through zigbee network. Temperature is measured by ADC with NTC resistor. Zigbee function is a custom cluster with minimal, maximal, and on-change reporting intervals in order to transmit more frequently on changes. Light sleep is used to reduce energy consumption.

Transmitted data is received by tasmota zigbee coordinator running on esp32c3.

Previous work considered for this project are the examples from Espressif SDK (in particular "light sleep", "sleepy ED", and others from Zigbee-SDK) and the following projects:

https://github.com/lmahmutov/esp32_c6_co2_sensor

https://github.com/prairiesnpr/esp_zha_test_bench

https://github.com/luar123/esphome_zb_sensor


## Getting started

For a fresh install make directory ~/esp and within install esp-idf (legacy installation) and esp-zigbee-sdk according to this instructions:

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup-legacy.html

https://docs.espressif.com/projects/esp-zigbee-sdk/en/latest/esp32/developing.html

Put folder light_sleep into folder ~/esp and compile according esp-idf manual (run export.sh, set-target, menuconfig, and build). For this to work I had to set the following parameters:
- Change paths for SRC_DIRS and INCLUDE_DIRS in main/CMakeLists.txt according to your installation.
- Within menuconfig select a) zigbee end device, b) custom partition table, c) power management, and d) flash size.

Tasmota zigbee coordinator was version 14.2.0 on esp32c3 with cc2530/cc2591 zigbee module:

https://tasmota.github.io/docs/Zigbee/

Within tasmota after enabling joining (ZbPermitJoin) bind the device with ZbBind {"Device":"0x1ae5", "Cluster": "0xff00", "Endpoint": 90}, changing device address to the applicable value. The data (here ADC value 1853) can be retrieved via MQTT in the following format, where "FF00/0001" is the chosen custom cluster and custom attribute ID: tele/tasmota_C126AC/SENSOR = {"ZbReceived":{"0x1AE5":{"Device":"0x1AE5","FF00/0001":1853,"Endpoint":90,"LinkQuality":184}}}

Reporting intervals and other parameters are set in zb_ntc_sleep_xx.h. Data is reported when changed more then DELTA, latest after MAX_INTERVAL, earliest after MIN_INTERVAL. A switch (here GPIO3 in esp32c6 and GPIO4 in esp32h2) is added to transmit more frequently in order to run a calibration sequence. Obtaining correct temperature from the ADC reading is done in a seperate script including the calibration function, e.g. a python script listening to MQTT and writing to a database.

Although in the schematic 3.0 V is drawn as supply, I observed instabilities with 3.0 V and now prefer using 3.3 V or 3.6 V.

Binaries in /bin are compiled for esp32c6.
