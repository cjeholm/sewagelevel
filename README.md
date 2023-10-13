# ESP32 Sewage Level Sensor
A simple program for measuring waste water level using ESP32 and ToF sensor.

This code is written for the M5Stack ATOM Lite ESP32 using the Time-of-Flight Distance Ranging Sensor Unit VL53L0X. Measurements are posted to a MQTT server for home automation integration.

The program runs once and then activates the ESP32's sleep function. Upon waking the program simply restarts and this repeats forever. The ESP32's power consumption is miniscule when sleeping and this allows the setup to run on battery power.
