# Changelog

Todos los cambios notables en este proyecto serán documentados en este archivo.

El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Documentación completa del proyecto (README.md, CONTRIBUTING.md, LICENSE)
- Guía de contribución con convenciones de código y flujo de trabajo
- Licencia MIT para el proyecto

### Changed
- Reestructuración completa del README.md con mejor organización
- Mejora en la descripción de arquitectura y características del sistema

## [2.2] - 2017-10-31

### Added
- Librería Filter para procesamiento de señales
- Organización mejorada de funciones en el código
- Soporte para acelerómetro LIS3MDL en MinIMU-V5

### Changed
- Migración de ESP8266 a Particle Photon para versión 2
- Actualización de librerías para compatibilidad con Particle

### Technical Details
- Implementación de algoritmo de detección sísmica usando parámetros IQR, CAV, RSL y ZC
- Sistema de configuración remota a través de Particle Cloud
- Georeferenciación automática de eventos detectados

## [2.1] - 2017-08-04

### Added
- Sistema de detección de sismos versión 2 con Particle Photon
- Soporte para configuración remota de parámetros
- Mejoras en la conectividad WiFi y envío de datos

### Changed
- Transición del microcontrolador ESP8266 al Particle Photon
- Actualización del acelerómetro a MinIMU-V5

## [2.0] - 2017-06-22

### Added
- Sistema completo de detección sísmica versión 1
- Soporte para múltiples acelerómetros (MMA8452Q, LSM303DLHC, ADXL345)
- Algoritmo de procesamiento de señales con parámetros sísmicos
- Conectividad WiFi para envío de datos
- Sistema de georeferenciación automática
- Herramientas de análisis en Python (acc_plot.py, serial_server.py)

### Features
- **Parámetros de detección**: IQR, CAV, RSL, ZC
- **Frecuencia de muestreo**: 100 Hz
- **Ventana de análisis**: 200 muestras
- **Conectividad**: WiFiManager para configuración automática
- **Hardware soportado**: NodeMCU 1.0 (ESP8266)

## [1.0] - 2017-05-15

### Added
- Implementación inicial del algoritmo de detección sísmica
- Estructura básica del proyecto
- Primeras pruebas con acelerómetro MMA8452Q

---

## Notas de Versiones

### Versión 2.x (Particle Photon)
- **Microcontrolador**: Particle Photon
- **Acelerómetro**: MinIMU-V5 (LIS3MDL)
- **Características**: Configuración remota, mejor conectividad cloud
- **Ubicación**: `/q_up/`

### Versión 1.x (ESP8266)
- **Microcontrolador**: NodeMCU 1.0 (ESP8266)
- **Acelerómetros**: MMA8452Q, LSM303DLHC, ADXL345
- **Características**: WiFiManager, múltiples sensores
- **Ubicación**: `/prosismic/`

---

## Convenciones

### Tipos de cambios
- `Added` para nuevas funcionalidades
- `Changed` para cambios en funcionalidades existentes
- `Deprecated` para funcionalidades que serán removidas
- `Removed` para funcionalidades removidas
- `Fixed` para correcciones de bugs
- `Security` para vulnerabilidades de seguridad

### Versionado
Este proyecto sigue [Semantic Versioning](https://semver.org/):
- **MAJOR**: Cambios incompatibles de API
- **MINOR**: Nueva funcionalidad compatible
- **PATCH**: Correcciones de bugs compatibles