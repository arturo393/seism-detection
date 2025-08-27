# Sistema de Detección de Sismos

Un sistema de detección de terremotos basado en acelerómetros que utiliza algoritmos avanzados de procesamiento de señales para detectar eventos sísmicos en tiempo real.

## 📋 Descripción

Este proyecto implementa un sistema completo de detección sísmica que procesa datos de acelerómetros utilizando parámetros como IQR (Rango Intercuartílico), CAV (Vector Acelerativo Acumulativo), RSL (Relación de Señal) y ZC (Cruces por Cero) para identificar eventos sísmicos.

### Características principales

- **Detección en tiempo real** de eventos sísmicos
- **Múltiples plataformas**: ESP8266 (v1) y Particle Photon (v2)
- **Soporte para múltiples acelerómetros**
- **Conectividad WiFi** para envío de datos
- **Georeferenciación** automática de eventos
- **Análisis de datos** con herramientas Python
- **Configuración remota** de parámetros

## 🏗️ Arquitectura del Sistema

### Versión 1 (ESP8266)
- **Microcontrolador**: NodeMCU 1.0 (ESP8266)
- **Ubicación**: `/prosismic/`
- **Acelerómetros soportados**:
  - SparkFun Triple Axis Accelerometer Breakout - MMA8452Q
  - MinIMU-9 v2 (L3GD20 y LSM303DLHC)
  - SparkFun Triple Axis Accelerometer Breakout - ADXL345
  - Módulo GY-291 ADXL345

### Versión 2 (Particle Photon)
- **Microcontrolador**: Particle Photon
- **Ubicación**: `/q_up/`
- **Acelerómetro**: MinIMU-V5 con LIS3MDL
- **Mejoras**: Configuración remota, mejor conectividad

## 🛠️ Instalación y Configuración

### Prerrequisitos

#### Para Versión 1 (ESP8266)
- Arduino IDE con soporte ESP8266
- Librerías necesarias (ver sección de dependencias)

#### Para Versión 2 (Particle Photon)
- Particle CLI o Particle Web IDE
- Cuenta en Particle Cloud

### Instalación de Dependencias

#### Versión 1 (ESP8266)
```bash
# Instalar soporte ESP8266 en Arduino IDE
# Agregar URL en Preferences: https://github.com/esp8266/Arduino
```

**Librerías requeridas:**
- [Arduino Time Library](https://github.com/PaulStoffregen/Time)
- [ArduinoSort](https://github.com/emilv/ArduinoSort)
- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [WiFi Location](https://github.com/gmag11/ESPWifiLocation)
- [WiFiManager](https://github.com/tzapu/WiFiManager)
- [LSM303 Arduino Library](https://github.com/pololu/lsm303-arduino)
- [SparkFun MMA8452Q Library](https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library)
- [Arduino-ADXL345](https://github.com/jarzebski/Arduino-ADXL345)

#### Versión 2 (Particle Photon)
```bash
# Instalar Particle CLI
npm install -g particle-cli
particle login
```

### Configuración

1. **Configurar WiFi** (Versión 1):
   ```cpp
   #define _SSID "tu_red_wifi"
   #define _NETPASS "tu_contraseña"
   ```

2. **Configurar parámetros de detección**:
   ```cpp
   #define SAMPLES 200           // Número de muestras
   #define Factor_IQR  5.0      // Factor de referencia IQR
   #define Factor_CAV  1.5      // Factor de referencia CAV
   #define Factor_ZC   1        // Factor de referencia ZC
   ```

### Compilación y Carga

#### Versión 1 (ESP8266)
```bash
cd prosismic/prosismic/
# Abrir prosismic.ino en Arduino IDE
# Seleccionar board: NodeMCU 1.0 (ESP-12E Module)
# Compilar y cargar
```

#### Versión 2 (Particle Photon)
```bash
cd q_up/
particle compile photon
particle flash [device_name] firmware.bin
```

## 🚀 Uso

### Parámetros de Detección

El sistema utiliza los siguientes parámetros para detectar eventos sísmicos:

- **IQR (Interquartile Range)**: Mide la variabilidad de la señal
- **CAV (Cumulative Absolute Velocity)**: Acumulación de velocidad absoluta
- **RSL (Response Spectrum Level)**: Nivel del espectro de respuesta
- **ZC (Zero Crossings)**: Cruces por cero de la señal

### Salida de Datos

El sistema genera datos en formato serie con el siguiente formato:
```
Date Time(hh:mm:ss:mss) Sample x y z NetAcc xMax yMax zMax NetAccMax t_NetAccMax ZC IQR CAV RSL

22/6/2017 21:22:36.673    68 0 -2 -2 282  3 6 3 6 608 673 26 103 200 111
22/6/2017 21:22:36.684    69 0 -1 -2 223  2 6 3 6 608 684 25 103 199 111
```

### Herramientas de Análisis

#### Visualización de datos (Python)
```bash
python acc_plot.py
```

#### Servidor serie para captura de datos
```bash
python serial_server.py
```

## 📁 Estructura del Proyecto

```
seism-detection/
├── prosismic/           # Versión 1 (ESP8266)
│   ├── prosismic/
│   │   └── prosismic.ino
│   ├── lib/             # Librerías específicas
│   └── platformio.ini
├── q_up/                # Versión 2 (Particle Photon)
│   ├── src/
│   │   └── q_up.ino
│   ├── lib/             # Librerías específicas
│   └── project.properties
├── Case/                # Diseños de carcasa (DXF)
├── Hardware/            # Esquemas electrónicos
├── acc_plot.py          # Herramienta de visualización
├── serial_server.py     # Servidor de captura de datos
└── README.md
```

## 🔧 Configuración Avanzada

### Parámetros Configurables

- `FrecRPS`: Frecuencia de muestreo (Hz)
- `SAMPLES`: Número de muestras para análisis
- `DPTIME`: Tiempo para determinar desplazamiento (ms)
- `MTIME`: Tiempo para determinar movimiento (ms)

### API de Configuración Remota (Versión 2)

La versión 2 permite configuración remota a través de Particle Cloud:
- `setIQR(value)`: Configurar umbral IQR
- `setCAV(value)`: Configurar umbral CAV
- `setRSL(value)`: Configurar umbral RSL
- `setRestartTime(time)`: Configurar tiempo de reinicio

## 🤝 Contribución

¡Las contribuciones son bienvenidas! Por favor lee [CONTRIBUTING.md](CONTRIBUTING.md) para detalles sobre el proceso de contribución.

## 📄 Licencia

Este proyecto está licenciado bajo la Licencia MIT - ver el archivo [LICENSE](LICENSE) para detalles.

## 👨‍💻 Autor

**Arturo Veras** - *Desarrollo inicial* - [arturo393](https://github.com/arturo393)

## 📖 Documentación Adicional

Para más información detallada sobre el funcionamiento del algoritmo, revisar la [Wiki del proyecto](https://github.com/arturo393/seism-detection/wiki).

## 🏷️ Versiones

Ver [CHANGELOG.md](CHANGELOG.md) para una lista de cambios en cada versión.

---

⚡ **Estado del Proyecto**: Activo y en desarrollo continuo
