# Seismic Detection v2 (Particle Photon)

Sistema de detección de sismos basado en Particle Photon con acelerómetro MinIMU-V5.

## 📋 Descripción

Esta es la **Versión 2** del sistema de detección sísmica, que utiliza la plataforma Particle Photon para mejorar la conectividad y permitir configuración remota a través de Particle Cloud.

### Características principales
- **Microcontrolador**: Particle Photon
- **Acelerómetro**: MinIMU-V5 con sensor LIS3MDL
- **Conectividad**: WiFi con Particle Cloud
- **Configuración remota**: Parámetros ajustables vía cloud
- **Georeferenciación**: Google Maps Device Locator
- **Frecuencia de muestreo**: 100 Hz configurable

## 🛠️ Instalación

### Prerrequisitos
```bash
# Instalar Particle CLI
npm install -g particle-cli

# Iniciar sesión
particle login
```

### Compilación
```bash
# Compilar para Photon
particle compile photon

# Flashear al dispositivo
particle flash [device_name] firmware.bin
```

## ⚙️ Configuración

### Variables Cloud Disponibles
- `ZCref`: Umbral de cruces por cero
- `IQRref`: Umbral del rango intercuartílico  
- `CAVref`: Umbral del vector acelerativo acumulativo
- `RSLref`: Umbral de relación de señal
- `Latitude`: Latitud del dispositivo
- `Longitude`: Longitud del dispositivo

### Funciones Cloud Disponibles
- `setZC(value)`: Configurar umbral ZC
- `setIQR(value)`: Configurar umbral IQR
- `setCAV(value)`: Configurar umbral CAV
- `setRSL(value)`: Configurar umbral RSL
- `setReferencia(mode)`: Activar/desactivar referencias variables
- `setRestartTime(minutes)`: Configurar tiempo de reinicio
- `setOnlineTime(minutes)`: Configurar tiempo online

## 🔧 Estructura del Proyecto

#### `/src` folder
Contiene el firmware principal del proyecto. El archivo `q_up.ino` implementa:
- Algoritmo de detección sísmica
- Comunicación con Particle Cloud
- Gestión de sensores LSM6
- Sistema de georeferenciación

#### `project.properties` file
Especifica las dependencias de librerías del proyecto:
- LSM6: Comunicación con acelerómetro
- HttpClient: Comunicación HTTP
- google-maps-device-locator: Georeferenciación

#### `/lib` folder
Librerías específicas del proyecto:
- **Filters**: Filtros digitales para procesamiento de señales
- **HttpClient**: Cliente HTTP para Particle
- **LSM6**: Driver para acelerómetro MinIMU-V5

## 📊 Parámetros de Detección

El sistema utiliza cuatro parámetros principales:

| Parámetro | Descripción | Unidad |
|-----------|-------------|---------|
| **IQR** | Rango Intercuartílico | mg |
| **CAV** | Vector Acelerativo Acumulativo | mg |
| **RSL** | Relación de Señal | % |
| **ZC** | Cruces por Cero | count |

## 🚀 Uso

### Monitoreo en Tiempo Real
```bash
# Ver logs del dispositivo
particle serial monitor

# Ver variables cloud
particle get [device_name] ZCref
particle get [device_name] IQRref
```

### Configuración Remota
```bash
# Configurar umbrales
particle call [device_name] setIQR "500"
particle call [device_name] setCAV "1000"

# Configurar tiempos
particle call [device_name] setRestartTime "10"
```

## 📈 Salida de Datos

El sistema publica eventos al Particle Cloud cuando detecta actividad sísmica:

**Evento**: `Sismo`  
**Formato**: `tiempo;aceleración_max;trigger;ZC;IQR;CAV;RSL;latitud;longitud`

## 🔄 Diferencias con Versión 1

| Aspecto | Versión 1 (ESP8266) | Versión 2 (Photon) |
|---------|---------------------|---------------------|
| Microcontrolador | NodeMCU ESP8266 | Particle Photon |
| Acelerómetro | Múltiples (MMA8452Q, LSM303, ADXL345) | MinIMU-V5 (LIS3MDL) |
| Configuración | Hardcoded | Remota vía Cloud |
| Conectividad | WiFi directo | Particle Cloud |
| Georeferenciación | WiFi Location | Google Maps API |

## 📝 Desarrollo

Para agregar archivos adicionales al proyecto, colócalos en la carpeta `/src`. Para librerías externas, crea `/lib/<nombre_libreria>/src` y coloca los archivos `.h` y `.cpp` ahí.

Los archivos que se envían al servicio de compilación incluyen:
- Todo en la carpeta `/src`
- El archivo `project.properties`
- Cualquier librería en `lib/<nombre_libreria>/src`

---

Para más información sobre el proyecto completo, consulta el [README principal](../README.md).