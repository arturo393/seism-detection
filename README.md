# Código para la detecci'on de sismos 
Arduino code for the ESP8266 NodeMcu 1.0 dev board and 3 type of acelerometers.
- SparkFun Triple Axis Accelerometer Breakout - MMA8452Q https://www.sparkfun.com/products/12756
- MinIMU-9 v2 Gyro, Accelerometer, and Compass (L3GD20 and LSM303DLHC Carrier) https://www.pololu.com/product/1268
- SparkFun Triple Axis Accelerometer Breakout - ADXL345 https://www.sparkfun.com/products/9836
- Módulo GY-291 Acelerómetro ADXL345 Digital de 3 Ejes http://www.maxelectronica.cl/sensores/280-modulo-gy-291-acelerometro-adxl345-digital-de-3-ejes.html

First attempt at a library. Lots more changes and fixes to do. Contributions are welcome.

#### This works with the ESP8266 Arduino platform with a recent stable release(2.0.0 or newer) https://github.com/esp8266/Arduino



### Instalación
You can either install through the Arduino Library Manager or checkout the latest changes or a release from github

#### Install through Library Manager
__Currently version 0.8+ works with release 2.0.0 or newer of the [ESP8266 core for Arduino](https://github.com/esp8266/Arduino)__
 - in Arduino IDE got to Sketch/Include Library/Manage Libraries
  ![Manage Libraries](http://i.imgur.com/9BkEBkR.png)

 - search for WiFiManager
  ![WiFiManager package](http://i.imgur.com/18yIai8.png)

 - click Install and start [using it](#using)

####  Checkout from github
__Github version works with release 2.0.0 or newer of the [ESP8266 core for Arduino](https://github.com/esp8266/Arduino)__
- Checkout library to your Arduino libraries folder

####  Libraries Needed
- Arduino Time Library https://github.com/PaulStoffregen/Time
- ArduinoSort https://github.com/emilv/ArduinoSort
- ArduinoJson https://github.com/bblanchon/ArduinoJson
- WiFi Location https://github.com/gmag11/ESPWifiLocation
- WiFiManager https://github.com/tzapu/WiFiManager
- Arduino library for Pololu LSM303 boards https://github.com/pololu/lsm303-arduino
- SparkFun MMA8452Q Arduino Library https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
- Arduino-ADXL345 https://github.com/jarzebski/Arduino-ADXL345



### Using
- Include in your sketch
```cpp
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LSM303.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoSort.h>
#include <ArduinoJson.h>
#include <WifiLocation.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
```

## Documentation


#### Debug
Debug is enabled by default on Serial. 
Sample data output
```cpp
 Date Time(hh:mm:ss:mss) Sample x y x NetAcc xMax yMax zMax NetAccMax t_NetAccMax ZC IQR CAV RSL
 
 22/6/2017 21:22:36.673      68 0 -2 -2 282  3 6 3 6 608 673 26 103 200 111 
 22/6/2017 21:22:36.684      69 0 -1 -2 223  2 6 3 6 608 684 25 103 199 111 
 22/6/2017 21:22:36.694      70 0 0 -1 100  3 6 3 6 608 694 25 103 199 111 
 22/6/2017 21:22:36.704      71 0 0 -2 200  2 6 3 6 608 705 25 103 199 111 
 22/6/2017 21:22:36.715      72 1 0 -2 223  3 6 3 6 608 715 25 103 199 111 


```
##Hardware
EAGLE files can be found on Hardware folder. And a autocad files and BOM can be found here https://circuits.io/circuits/5318012
## Releases
##### v0.1



### Contributions and thanks
The support and help I got from the community has been nothing short of phenomenal. I can't thank you guys enough. This is my first real attept in developing open source stuff and I must say, now I understand why people are so dedicated to it, it is because of all the wonderful people involved.

__THANK YOU__

#### Inspiration
