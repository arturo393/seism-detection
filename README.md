# Código para la detección de sismos 
Código Arduino para el firmware NodeMcu 1.0 y 3 tipos de acelerometros.
- SparkFun Triple Axis Accelerometer Breakout - MMA8452Q https://www.sparkfun.com/products/12756
- MinIMU-9 v2 Gyro, Accelerometer, and Compass (L3GD20 and LSM303DLHC Carrier) https://www.pololu.com/product/1268
- SparkFun Triple Axis Accelerometer Breakout - ADXL345 https://www.sparkfun.com/products/9836
- Módulo GY-291 Acelerómetro ADXL345 Digital de 3 Ejes http://www.maxelectronica.cl/sensores/280-modulo-gy-291-acelerometro-adxl345-digital-de-3-ejes.html

First attempt at a library. Lots more changes and fixes to do. Contributions are welcome.

#### Este código funciona con la plataforma ESP8266 Arduino , mas detalles sobre la instalación para desarrollar en https://github.com/esp8266/Arduino


####  Libreras necesarias
- Arduino Time Library https://github.com/PaulStoffregen/Time
- ArduinoSort https://github.com/emilv/ArduinoSort
- ArduinoJson https://github.com/bblanchon/ArduinoJson
- WiFi Location https://github.com/gmag11/ESPWifiLocation
- WiFiManager https://github.com/tzapu/WiFiManager
- Arduino library for Pololu LSM303 boards https://github.com/pololu/lsm303-arduino
- SparkFun MMA8452Q Arduino Library https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
- Arduino-ADXL345 https://github.com/jarzebski/Arduino-ADXL345


## Documentación
Revisar la sección https://github.com/arturo393/seism-detection/wiki para más detalles del funcionaminamiento.


#### Salida Serial
Para analizar los datos de salida checkear el puerto serial. Un ejemplo de los datos con su hearder es:
```cpp
 Date Time(hh:mm:ss:mss) Sample x y x NetAcc xMax yMax zMax NetAccMax t_NetAccMax ZC IQR CAV RSL
 
 22/6/2017 21:22:36.673      68 0 -2 -2 282  3 6 3 6 608 673 26 103 200 111 
 22/6/2017 21:22:36.684      69 0 -1 -2 223  2 6 3 6 608 684 25 103 199 111 
 22/6/2017 21:22:36.694      70 0 0 -1 100  3 6 3 6 608 694 25 103 199 111 
 22/6/2017 21:22:36.704      71 0 0 -2 200  2 6 3 6 608 705 25 103 199 111 
 22/6/2017 21:22:36.715      72 1 0 -2 223  3 6 3 6 608 715 25 103 199 111 


```
## Hardware
Se pueden encontrar archvios para EAGLE pero es una versión no probada.

