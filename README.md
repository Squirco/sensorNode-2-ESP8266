# sensorNode-2-ESP8266

Firmware files for Squirco Sensor Network Light Switch's Networking component (ESP8266 variant).
More about the project can be seen on Hackaday here:  
https://hackaday.io/project/3321-squirco-smart-home-system-sensor-network

#Libraries Used
Arduino libraries used in this project may be modified to fit the project. All credits due to the originators.

https://github.com/thijse/Arduino-CmdMessenger

https://github.com/tzapu/WiFiManager

https://github.com/bblanchon/ArduinoJson

https://github.com/Imroy/pubsubclient

https://github.com/mathertel/OneButton

The networking component receives data from the sensorhub, controls the light switch relay, and takes care of networking over wi-fi.
Wi-Fi configuration is done using tzapu's WiFiManager library. MQTT broker information can be configured at the same time (tested with mosquitto on raspberryPi and adafruit.io)

*Note for adafruit.io users: the topics in the code are designed to work for a more general purpose mqtt broker such as mosquitto. If you wish to use adafruit.io, the topics must be modified accordingly.)
