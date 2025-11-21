# Medidor de Fuerza H:Q (Isquiotibiales / Cuádriceps)

## Descripción del Proyecto

Este proyecto consiste en un dispositivo electrónico diseñado para la medición, visualización y análisis de la fuerza muscular, específicamente enfocado en la relación (ratio) entre los músculos Isquiotibiales y Cuádriceps. Es una herramienta orientada a la evaluación biomecánica y rehabilitación deportiva.

El sistema utiliza celdas de carga para medir la fuerza ejercida en tiempo real, procesa los datos mediante un microcontrolador ESP32 y presenta los resultados en una pantalla de alta resolución con una interfaz gráfica moderna. Además, transmite los datos vía Bluetooth para su registro y análisis externo.

## Componentes de Hardware

### Microcontrolador

- **ESP32 DevKit V1**: Seleccionado por su doble núcleo (potencia de procesamiento), conectividad Bluetooth nativa y gran cantidad de pines GPIO.

### Sensores

- **2x Celdas de Carga (Tipo S)**: Transductores que convierten la fuerza física en una señal eléctrica diferencial.
- **2x Módulos HX711**: Amplificadores de instrumentación de 24-bits que digitalizan la señal de las celdas de carga para que el ESP32 pueda leerla con alta precisión.

### Visualización

- **Pantalla TFT ST7789 (170x320 píxeles)**: Pantalla IPS a color de alta densidad. Se eligió por su excelente ángulo de visión y capacidad de refresco rápido.

### Interfaz Física

- **Botón de Tara**: Para calibrar el cero de los sensores antes de la medición.
- **LEDs Indicadores**:
  - LED Estado (Heartbeat).
  - LED Bluetooth (Indica conexión activa).

## Diagrama de Conexiones (Pinout)

| Componente | Pin Dispositivo | Pin ESP32 | Descripción |
| :--- | :--- | :--- | :--- |
| **Sensor Isquios** | DAT | GPIO 19 | Datos HX711 |
| | CLK | GPIO 5 | Reloj HX711 |
| **Sensor Cuads** | DAT | GPIO 21 | Datos HX711 |
| | CLK | GPIO 22 | Reloj HX711 |
| **Pantalla ST7789** | MOSI (SDA) | GPIO 13 | *Verificar config TFT_eSPI |
| | SCLK (SCL) | GPIO 14 | *Verificar config TFT_eSPI |
| | CS | (No conectado) | Chip Select (Opcional) |
| | DC | GPIO 26 | Data/Command |
| | RST | GPIO 27 | Reset |
| **Controles** | Botón Tara | GPIO 32 | Pull-up interno activado |
| | LED Estado | GPIO 4 | Indicador de funcionamiento |
| | LED Bluetooth | GPIO 23 | Indicador de conexión |


## Arquitectura de Software y Librerías

El firmware está desarrollado en C++ utilizando el framework Arduino sobre PlatformIO. Se han seleccionado librerías específicas para garantizar rendimiento y estabilidad:

### 1. `TFT_eSPI` (Bodmer)

- **Por qué se usa**: Es la librería gráfica más rápida disponible para el ESP32. Permite el uso de **Sprites** (buffers de memoria de video), lo que elimina el parpadeo ("flicker") al actualizar la pantalla.
- **Implementación**: Se dibuja toda la interfaz (medidores, textos, arcos) en un Sprite de 320x170 píxeles en la memoria RAM y luego se envía de golpe a la pantalla (DMA), logrando animaciones fluidas.

### 2. `HX711_ADC` (Olkal)

- **Por qué se usa**: A diferencia de la librería HX711 estándar, esta versión es **no bloqueante**. Permite leer los sensores sin detener el resto del programa (como la actualización de la pantalla o el Bluetooth).
- **Características**: Incluye algoritmos de suavizado (smoothing) para reducir el ruido eléctrico y estabilizar la lectura del peso.

### 3. `BluetoothSerial` (Nativa)

- **Por qué se usa**: Implementa el protocolo Bluetooth Clásico (SPP - Serial Port Profile), permitiendo que el dispositivo se comunique con teléfonos Android o PCs como si fuera un cable serial. Ideal para enviar datos de telemetría en tiempo real.

### 4. `Preferences` (Nativa)

- **Por qué se usa**: Reemplaza a la antigua EEPROM. Permite guardar los factores de calibración en la memoria no volátil (NVS) del ESP32, asegurando que la calibración no se pierda al apagar el dispositivo.

## Lógica de Funcionamiento

1. **Inicialización**: Al encender, el sistema carga la calibración guardada, inicia la pantalla y tara (pone a cero) los sensores.
2. **Bucle Principal (Loop)**:
    - **Lectura**: Lee los sensores HX711.
    - **Filtrado (Deadzone)**: Se aplica un umbral mínimo de **3 Newtons**. Cualquier fuerza menor a esto se considera ruido y se muestra como 0N.
    - **Cálculo**: Convierte la señal eléctrica a Newtons usando el factor de calibración y la constante de gravedad (9.81).
    - **Ratio H:Q**: Calcula la relación `Fuerza Isquios / Fuerza Cuádriceps`.
        - *Ejemplo*: Si Isquios = 100N y Cuads = 200N, Ratio = 0.5.
    - **Visualización**: Dibuja dos medidores circulares ("Gauges") y el valor numérico central. El color del ratio cambia según si está en un rango saludable o no.
    - **Transmisión**: Envía un paquete de datos binarios por Bluetooth para ser graficado en una App externa.

## Instrucciones de Uso

1. **Encendido**: Conectar el dispositivo a una fuente USB o batería.
2. **Tara**: Asegurarse de que no haya carga en los sensores y presionar el **Botón de Tara**. La pantalla indicará "Tarando...".
3. **Medición**: Aplicar fuerza en los sensores. Los gráficos se llenarán en tiempo real.
4. **Conexión App**:
    - Buscar dispositivo Bluetooth "ESP32_Fuerza_HQ".
    - Conectar desde la aplicación de análisis.
    - El LED de Bluetooth se encenderá fijo.

## Calibración (Técnica)

El sistema permite calibración remota vía comandos Serial/Bluetooth:

- `t`: Tarar sensores.
- `i=XXXX.X`: Establecer factor de calibración para Isquios.
- `q=XXXX.X`: Establecer factor de calibración para Cuádriceps.
- `save`: Guardar configuración en memoria permanente.

---

*Proyecto desarrollado por Alexis Ocando para Andrea Zunino*
