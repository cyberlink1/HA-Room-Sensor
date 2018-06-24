# HA-Room-Sensor

This uses a NodeMCU with a DHT11 temperature/Humidity sensor, BMP180 Temperature/Borametric Pressure sensor, an LDR as a light sensor, and a PIR as a motion sensor.

Note: There is a bug in the ESP8266 Arduino IDE V2.4.0 and v2.4.1 that causes a 56 byte memory leak. I had to downgrade to V2.3.0 to compile the code. You can do this by selecting Tools-->board-->Board Manager-->ESP8266 and then select the version in the bottom right. Click upgrade and it will work.

If you would like to see the sensor box I have the STL's up on thingiverse as well

https://www.thingiverse.com/thing:2973430
