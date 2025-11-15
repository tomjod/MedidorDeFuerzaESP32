/*
 * PROYECTO: Medidor de Fuerza H:Q (Isquios/Cuádriceps)
 * MCU: ESP32 DevKit V1
 * PANTALLA: ST7789 170x320
 * SENSORES: 2x Celdas de Carga "S" + 2x SparkFun HX711
 * CONTROLES: 1x Botón (Tara), 2x LEDs (Estado, BT)
 *
 * *** VERSIÓN 2.1 (Pines de LED reasignados) ***
 * - LED de Estado (Heartbeat) movido a GPIO 4
 * - LED de Bluetooth movido a GPIO 23
 */
#include "BluetoothSerial.h" // Librería para Bluetooth

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
 
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// --- LIBRERÍAS ---
#include <Arduino.h>
#include <HX711_ADC.h> // Librería avanzada para los sensores
#include <TFT_eSPI.h>  // Librería optimizada para la pantalla
#include <SPI.h>

// --- CONFIGURACIÓN DE PINES (SENSORES) ---
const int PIN_ISQUIOS_DAT = 19;
const int PIN_ISQUIOS_CLK = 5;
const int PIN_CUADS_DAT = 21;
const int PIN_CUADS_CLK = 22;

// --- CONFIGURACIÓN DE PINES (CONTROLES) ---
const int PIN_BTN_TARE = 32;
const int PIN_LED_STATUS = 4;  // <-- CAMBIADO (antes 33)
const int PIN_LED_BT = 23;     // <-- CAMBIADO (antes 25)

// --- OBJETOS GLOBALES ---
TFT_eSPI tft = TFT_eSPI();
BluetoothSerial SerialBT;
HX711_ADC sensorIsquios(PIN_ISQUIOS_DAT, PIN_ISQUIOS_CLK);
HX711_ADC sensorCuads(PIN_CUADS_DAT, PIN_CUADS_CLK);

// --- CALIBRACIÓN (¡REEMPLAZAR!) ---
const float CAL_FACTOR_ISQUIOS = 43082.0; 
const float CAL_FACTOR_CUADS = 43540.0;   

// --- VARIABLES GLOBALES (TIMERS) ---
unsigned long lastDisplayTime = 0;
const int DISPLAY_INTERVAL_MS = 100; // 10 FPS

unsigned long lastLedBlinkTime = 0;
unsigned long lastLedBtTime = 0;
const int LED_BLINK_INTERVAL_MS = 1000; // 1 segundo
bool ledStatusState = LOW;
bool ledBTState = LOW;
bool BTReady = false;
bool BTConnected = false;

// --- VARIABLES GLOBALES (BOTÓN DEBOUNCE) ---
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY_MS = 100;

// --- Declaraciones de funciones ---
void initDisplay();
void initSensors();
void initControls();
void initBluetooth();
void tareSensors();
void updateDisplay(float fuerzaIsquios, float fuerzaCuads);
void handleTareButton();
void handleLEDs();
void handleBluetooth();
void handleSensorDisplay();
void updateBTdisplay();


//=================================================================
//  SETUP: Se ejecuta una vez al inicio
//=================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Medidor de Fuerza H:Q (v2.1)...");

  initControls(); // Inicializar pines de LEDs y Botón
  initDisplay();
  initSensors();

  // Iniciar Bluetooth
  initBluetooth();
}

//=================================================================
//  LOOP: Se ejecuta continuamente
//=================================================================
void loop() {
  
  // 1. Manejar el botón de tara (con debounce)
  handleTareButton();

  // 2. Manejar comandos por Bluetooth
  handleBluetooth();
  
  // 3. Manejar los LEDs de estado
  handleLEDs();
  
  // 4. Manejar la lectura de sensores y la actualización de pantalla (con timer)
  handleSensorDisplay();
}

//=================================================================
//  FUNCIONES DE INICIALIZACIÓN
//=================================================================

void initControls() {
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_LED_BT, OUTPUT);
  // Usamos el PULLUP interno del ESP32. El botón debe conectarse a GND.
  pinMode(PIN_BTN_TARE, INPUT_PULLUP); 
}

void initDisplay() {
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); 
  tft.setTextSize(2);
  tft.println("Iniciando...");
  delay(1000);
}

void initSensors() {
  Serial.println("Iniciando sensores...");
  tft.println("Iniciando sensores...");

  sensorIsquios.begin();
  sensorIsquios.setGain(128);
  sensorIsquios.setCalFactor(CAL_FACTOR_ISQUIOS);
  sensorIsquios.start(2000, true); // 2s estabilización y tara
  if (sensorIsquios.getTareTimeoutFlag()) {
    Serial.println("Error S1 Timeout");
    tft.println("Error S1 Timeout");
  }

  sensorCuads.begin();
  sensorCuads.setGain(128);
  sensorCuads.setCalFactor(CAL_FACTOR_CUADS);
  sensorCuads.start(2000, true); // 2s estabilización y tara
  if (sensorCuads.getTareTimeoutFlag()) {
    Serial.println("Error S2 Timeout");
    tft.println("Error S2 Timeout");
  }

  // Esperar a que ambos estén listos
  unsigned long startWait = millis();
  while(millis() - startWait < 3000) { 
    sensorIsquios.update();
    sensorCuads.update();
    if(sensorIsquios.getDataSetStatus() && sensorCuads.getDataSetStatus()) break;
    delay(10);
  }

  Serial.println("Sensores tarados y listos.");
  tft.fillScreen(TFT_BLACK); 
}

void initBluetooth() {
  if (SerialBT.begin("ESP32_Fuerza_HQ")) {
    BTReady = true;
    Serial.println("Bluetooth iniciado. Listo para conectar.");
  }
  updateBTdisplay();
}

//=================================================================
//  FUNCIONES DE TAREAS (LLAMADAS DESDE EL LOOP)
//=================================================================

/**
 * @brief Revisa el botón de tara con debounce de software.
 * Llama a tareSensors() si se presiona.
 */
void handleTareButton() {
  int reading = digitalRead(PIN_BTN_TARE);
  

  // Si el estado del botón cambió (ruido o presión), resetea el timer de debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    lastButtonState = reading;
  }

  // Si ha pasado suficiente tiempo desde el último cambio...
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
 
    // Si el nuevo estado es PRESIONADO (LOW porque usamos PULLUP)
    if (lastButtonState == LOW) {
      tareSensors();
    }
    
  }
}

/**
 * @brief Revisa comandos entrantes (Serial y Bluetooth)
 */
void handleBluetooth() {
  // También revisamos el Serial por si acaso
  if (Serial.available() > 0) {
    if (Serial.read() == 't') {
      tareSensors();
    }
  }
  
  // Revisa comandos de Bluetooth
  if (SerialBT.available() > 0) {
    if (SerialBT.read() == 't') {
      tareSensors();
    }
    // Aquí puedes agregar más comandos (ej: 'c' para calibrar)
  }
}

/**
 * @brief Actualiza los LEDs de estado (Heartbeat y BT)
 */
void handleLEDs() {
  // 1. LED de Bluetooth: Sólido ON si está conectado, OFF si no.
  if (SerialBT.connected()) {
    digitalWrite(PIN_LED_BT, HIGH);
  } else {
    if (millis() - lastLedBtTime >= (LED_BLINK_INTERVAL_MS - 500)) {
      lastLedBtTime = millis();
      ledBTState = !ledBTState; // Invierte el estado
      digitalWrite(PIN_LED_BT, ledBTState );
    }
  }

  // 2. LED de Estado: Heartbeat no-bloqueante
  /*if (millis() - lastLedBlinkTime >= LED_BLINK_INTERVAL_MS) {
    lastLedBlinkTime = millis();
    ledStatusState = !ledStatusState; // Invierte el estado
    digitalWrite(PIN_LED_STATUS, ledStatusState);
  }*/

  if (!ledStatusState) {
    digitalWrite(PIN_LED_STATUS, HIGH);
    ledStatusState = true;
  } 
}

/**
 * @brief Lee sensores y actualiza la pantalla (controlado por timer)
 */
void handleSensorDisplay() {
  // Controlar el refresco de pantalla para evitar parpadeo (Flicker)
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = millis();

    // Llama a update() (no-bloqueante)
    sensorIsquios.update();
    sensorCuads.update();

    // Obtiene el último valor promediado
    float fuerzaIsquios = sensorIsquios.getData();
    float fuerzaCuads = sensorCuads.getData();

    // Actualizar la pantalla
    updateDisplay(fuerzaIsquios, fuerzaCuads);
  }
}

void updateBTdisplay() {
  if (BTReady && !BTConnected) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK); 
    tft.setCursor(160, 150);
    tft.println("BT waiting...");
  } else if (BTConnected) {
    tft.setTextSize(2);
    tft.setTextColor(TFT_BLUE, TFT_BLACK); 
    tft.setCursor(100, 20);
    tft.println("BT connected");
  }
}

//=================================================================
//  FUNCIONES DE ACCIÓN
//=================================================================

/**
 * @brief Pone a cero (tara) ambos sensores
 */
void tareSensors() {
  Serial.println("Tarando sensores...");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.println("Tarando...");
  
  sensorIsquios.tare(); // Esta función es bloqueante
  sensorCuads.tare();   // Esta función es bloqueante

  delay(500); 
  tft.fillScreen(TFT_BLACK);
  Serial.println("Tara completa.");
  
  // Enviar confirmación por BT
  if(SerialBT.connected()) {
    SerialBT.println("Tara Completa");
  }
}


/**
 * @brief Dibuja los datos en la pantalla ST7789
 * (Esta función no cambió)
 */
void updateDisplay(float fuerzaIsquios, float fuerzaCuads) {

  // --- FUERZA ISQUIOTIBIALES ---
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK); 
  tft.setCursor(10, 20);
  tft.print("Isquios:");
  tft.fillRect(10, 80, 200, 40, TFT_BLACK); 
  tft.setCursor(10, 50);
  tft.print(max(0.0f, fuerzaIsquios), 2);
  tft.print(" Kg");

  // --- FUERZA CUÁDRICEPS ---
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(10,100);
  tft.print("Cuads:");
  tft.fillRect(10, 190, 200, 40, TFT_BLACK);
  tft.setCursor(10, 130);
  tft.print(max(0.0f, fuerzaCuads), 2); // 2 decimales
  tft.print(" Kg");

  // --- RATIO H:Q ---
  float ratio = 0.0;
  if (fuerzaCuads > 0.01) { 
    ratio = (fuerzaIsquios / fuerzaCuads);
  }
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 270);
  tft.print("Ratio H:Q: ");
  tft.fillRect(140, 270, 100, 30, TFT_BLACK);
  tft.setCursor(140, 270);
  tft.print(ratio, 2);
}