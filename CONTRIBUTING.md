# Guía de Contribución

¡Gracias por tu interés en contribuir al Sistema de Detección de Sismos! Este documento te guiará a través del proceso de contribución.

## 📋 Tabla de Contenidos

- [Código de Conducta](#código-de-conducta)
- [¿Cómo puedo contribuir?](#cómo-puedo-contribuir)
- [Configuración del Entorno de Desarrollo](#configuración-del-entorno-de-desarrollo)
- [Flujo de Trabajo con Git](#flujo-de-trabajo-con-git)
- [Estándares de Código](#estándares-de-código)
- [Convenciones de Commits](#convenciones-de-commits)
- [Proceso de Pull Request](#proceso-de-pull-request)
- [Reportar Bugs](#reportar-bugs)
- [Sugerir Mejoras](#sugerir-mejoras)

## 📜 Código de Conducta

Este proyecto adhiere a un código de conducta. Al participar, se espera que mantengas este código. Por favor reporta comportamientos inaceptables.

### Nuestros Valores
- **Respeto**: Tratamos a todos con cortesía y respeto
- **Inclusión**: Damos la bienvenida a contribuidores de todos los trasfondos
- **Colaboración**: Trabajamos juntos constructivamente
- **Calidad**: Nos esforzamos por la excelencia técnica

## 🤝 ¿Cómo puedo contribuir?

### Tipos de Contribuciones

1. **Reportar Bugs**: Ayuda a identificar problemas
2. **Sugerir Funcionalidades**: Propón nuevas características
3. **Mejorar Documentación**: Clarifica y expande la documentación
4. **Contribuir Código**: Implementa fixes y nuevas funcionalidades
5. **Revisar Código**: Ayuda revisando Pull Requests
6. **Testear**: Prueba el software en diferentes configuraciones

### Áreas de Contribución

- **Algoritmos de Detección**: Mejoras en los algoritmos sísmicos
- **Soporte de Hardware**: Nuevos acelerómetros y microcontroladores
- **Conectividad**: Mejoras en WiFi y comunicación
- **Herramientas de Análisis**: Scripts de Python para análisis de datos
- **Documentación**: README, guías, comentarios de código
- **Testing**: Pruebas unitarias y de integración

## 🛠️ Configuración del Entorno de Desarrollo

### Prerrequisitos

#### Para desarrollo de firmware
```bash
# Arduino IDE (Versión 1)
# Descargar de: https://www.arduino.cc/en/software

# Particle CLI (Versión 2)
npm install -g particle-cli
```

#### Para herramientas de análisis
```bash
# Python 3.7+
pip install numpy scipy matplotlib pylab
```

### Configuración del Repositorio

```bash
# 1. Fork el repositorio en GitHub
# 2. Clona tu fork
git clone https://github.com/TU_USUARIO/seism-detection.git
cd seism-detection

# 3. Configura el repositorio upstream
git remote add upstream https://github.com/arturo393/seism-detection.git

# 4. Instala dependencias (si aplica)
cd prosismic && # instalar librerías Arduino
cd ../q_up && # configurar Particle
```

## 🔄 Flujo de Trabajo con Git

### Modelo de Branching

Utilizamos un flujo de trabajo basado en **Git Flow**:

- `main`: Rama principal con código estable
- `develop`: Rama de desarrollo con últimas funcionalidades
- `feature/*`: Ramas para nuevas funcionalidades
- `hotfix/*`: Ramas para correcciones urgentes
- `release/*`: Ramas para preparación de releases

### Flujo de Desarrollo

```bash
# 1. Mantén tu fork actualizado
git checkout main
git pull upstream main
git push origin main

# 2. Crea una rama para tu feature
git checkout -b feature/nombre-descriptivo

# 3. Haz tus cambios y commits
git add .
git commit -m "tipo: descripción breve"

# 4. Push a tu fork
git push origin feature/nombre-descriptivo

# 5. Crea un Pull Request en GitHub
```

## 📝 Estándares de Código

### C/C++ (Arduino/Particle)

```cpp
// Naming conventions
int sampleCount;        // camelCase para variables
const int MAX_SAMPLES;  // UPPER_CASE para constantes
void calculateIQR();    // camelCase para funciones

// Comentarios
/**
 * Calcula el rango intercuartílico de las muestras
 * @param samples: Array de muestras de aceleración
 * @param size: Tamaño del array
 * @return: Valor IQR calculado
 */
int calculateIQR(int* samples, int size) {
    // Implementación aquí
}

// Indentación: 2 espacios
if (condition) {
  doSomething();
  if (anotherCondition) {
    doMore();
  }
}
```

### Python

```python
# Seguir PEP 8
import numpy as np
import matplotlib.pyplot as plt

def analyze_seismic_data(data_array):
    """
    Analiza datos sísmicos y genera visualizaciones.
    
    Args:
        data_array (np.array): Array con datos de aceleración
        
    Returns:
        dict: Resultados del análisis
    """
    # Implementación aquí
    pass

# Naming: snake_case
sample_rate = 100
max_acceleration = 1000
```

### Documentación

- **Comentarios**: En español para consistencia
- **Funciones**: Documentar parámetros y valores de retorno
- **Variables**: Nombres descriptivos y autoexplicativos
- **README**: Mantener actualizado con cambios

## 📋 Convenciones de Commits

Utilizamos **Conventional Commits** con mensajes en español:

### Formato
```
tipo(scope): descripción breve

[cuerpo opcional]

[footer opcional]
```

### Tipos de Commit
- `feat`: Nueva funcionalidad
- `fix`: Corrección de bug
- `docs`: Cambios en documentación
- `style`: Cambios de formato (no afectan funcionalidad)
- `refactor`: Refactorización de código
- `test`: Agregar o modificar tests
- `chore`: Tareas de mantenimiento

### Ejemplos
```bash
git commit -m "feat(detection): agregar soporte para acelerómetro ADXL345"
git commit -m "fix(wifi): corregir reconexión automática"
git commit -m "docs(readme): actualizar instrucciones de instalación"
git commit -m "refactor(filters): optimizar cálculo de IQR"
```

## 🔍 Proceso de Pull Request

### Antes de Enviar

1. **Asegúrate de que compila**: Sin errores de compilación
2. **Ejecuta tests**: Si existen, que pasen todos
3. **Revisa el código**: Elimina código comentado y debug
4. **Actualiza documentación**: Si tu cambio lo requiere
5. **Revisa el diff**: Que solo incluya cambios necesarios

### Información del PR

Tu Pull Request debe incluir:

```markdown
## Descripción
Breve descripción de los cambios realizados.

## Tipo de Cambio
- [ ] Bug fix (cambio que corrige un issue)
- [ ] Nueva funcionalidad (cambio que agrega funcionalidad)
- [ ] Breaking change (fix o feature que causaría que funcionalidad existente no funcione como se espera)
- [ ] Cambio de documentación

## ¿Cómo se ha probado?
Describe las pruebas que ejecutaste.

## Checklist
- [ ] Mi código sigue las convenciones del proyecto
- [ ] He realizado una auto-revisión de mi código
- [ ] He comentado mi código, particularmente en áreas difíciles de entender
- [ ] He realizado cambios correspondientes a la documentación
- [ ] Mis cambios no generan nuevas advertencias
```

### Proceso de Revisión

1. **Automated checks**: CI/CD verificará que compila
2. **Code review**: Miembro del equipo revisará el código
3. **Testing**: Pruebas en hardware si es necesario
4. **Merge**: Una vez aprobado, se hará merge a la rama principal

## 🐛 Reportar Bugs

### Antes de Reportar
1. Revisa si el bug ya fue reportado
2. Verifica que estés usando la última versión
3. Revisa la documentación y configuración

### Información a Incluir
```markdown
## Descripción del Bug
Descripción clara y concisa del problema.

## Pasos para Reproducir
1. Ve a '...'
2. Haz click en '....'
3. Desplázate hacia '....'
4. Ver error

## Comportamiento Esperado
Descripción clara de lo que esperabas que pasara.

## Capturas de Pantalla
Si aplica, agrega capturas para explicar el problema.

## Información del Sistema
- Versión del firmware:
- Microcontrolador: [ESP8266/Particle Photon]
- Acelerómetro: [Modelo]
- Versión de librerías:

## Información Adicional
Cualquier otro contexto sobre el problema.
```

## 💡 Sugerir Mejoras

### Template para Funcionalidades
```markdown
## ¿Tu solicitud está relacionada con un problema?
Una descripción clara de cuál es el problema.

## Describe la solución que te gustaría
Una descripción clara y concisa de lo que quieres que pase.

## Describe alternativas que has considerado
Una descripción clara de cualquier solución o funcionalidad alternativa que hayas considerado.

## Contexto adicional
Agrega cualquier otro contexto o capturas sobre la solicitud aquí.
```

## 🚀 Releases y Versionado

Utilizamos **Semantic Versioning** (SemVer):
- **MAJOR**: Cambios incompatibles de API
- **MINOR**: Funcionalidad agregada de forma compatible
- **PATCH**: Bug fixes compatibles

## 📞 Contacto

- **Issues**: Para bugs y feature requests
- **Discussions**: Para preguntas generales
- **Email**: Para contacto directo con mantenedores

## 🙏 Reconocimientos

¡Gracias a todos los contribuidores que han ayudado a hacer este proyecto mejor!

---

Recuerda: Cada contribución, sin importar qué tan pequeña, es valiosa. ¡Esperamos tus aportes!