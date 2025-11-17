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
#include <Preferences.h> // Para guardar calibración en NVS


// --- CONFIGURACIÓN DE PINES (SENSORES) ---
const int PIN_ISQUIOS_DAT = 19;
const int PIN_ISQUIOS_CLK = 5;
const int PIN_CUADS_DAT = 21;
const int PIN_CUADS_CLK = 22;

// --- CONFIGURACIÓN DE PINES (CONTROLES) ---
const int PIN_BTN_TARE = 32;
const int PIN_LED_STATUS = 4;  
const int PIN_LED_BT = 23;    

// --- OBJETOS GLOBALES ---
TFT_eSPI tft = TFT_eSPI();
BluetoothSerial SerialBT;
HX711_ADC sensorIsquios(PIN_ISQUIOS_DAT, PIN_ISQUIOS_CLK);
HX711_ADC sensorCuads(PIN_CUADS_DAT, PIN_CUADS_CLK);
Preferences preferences;

// --- CALIBRACIÓN (¡REEMPLAZAR!) ---
volatile float CAL_FACTOR_ISQUIOS = 43082.0; 
volatile float CAL_FACTOR_CUADS = 43540.0;   

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
void drawStaticUI();
void loadCalibration(); 
void saveCalibration(); 


//=================================================================
//  SETUP: Se ejecuta una vez al inicio
//=================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Medidor de Fuerza H:Q (v2.1)...");

  // Cargar calibración guardada de la NVS
  preferences.begin("fuerza-hq", false); // false = read/write
  loadCalibration();

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
}

void initBluetooth() {
  Serial.println("Iniciando Bluetooth...");
  if (SerialBT.begin("ESP32_Fuerza_HQ")) {
    BTReady = true;
    Serial.println("Bluetooth iniciado. Listo para conectar.");
    tft.println("BT Iniciado");
  }
  updateBTdisplay();
  tft.fillScreen(TFT_BLACK);
  drawStaticUI();
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
  char cmdChar = ' ';
  String cmdString = "";

  // Revisar Serial (PC)
  if (Serial.available() > 0) {
    cmdString = Serial.readStringUntil('\n');
    cmdChar = cmdString.charAt(0); // Para compatibilidad con 't'
  }
  
  // Revisar Bluetooth (Teléfono)
  if (SerialBT.available() > 0) {
    cmdString = SerialBT.readStringUntil('\n');
    cmdChar = cmdString.charAt(0); // Para compatibilidad con 't'
  }

  // Procesar el comando
  cmdString.trim(); // Limpiar espacios en blanco

  if (cmdChar == 't') { // Comando de Tara
    tareSensors();
  }
  else if (cmdString.startsWith("i=")) { // Ajustar Isquios
    float nuevoFactor = cmdString.substring(2).toFloat();
    if (nuevoFactor > 1000) { // Un chequeo de seguridad
      CAL_FACTOR_ISQUIOS = nuevoFactor;
      sensorIsquios.setCalFactor(CAL_FACTOR_ISQUIOS);
      Serial.print("Nuevo factor Isquios: ");
      Serial.println(CAL_FACTOR_ISQUIOS);
    }
  }
  else if (cmdString.startsWith("q=")) { // Ajustar Cuádriceps
    float nuevoFactor = cmdString.substring(2).toFloat();
    if (nuevoFactor > 1000) { // Un chequeo de seguridad
      CAL_FACTOR_CUADS = nuevoFactor;
      sensorCuads.setCalFactor(CAL_FACTOR_CUADS);
      Serial.print("Nuevo factor Cuads: ");
      Serial.println(CAL_FACTOR_CUADS);
    }
  }
  else if (cmdString == "save") { // Guardar calibración
    saveCalibration();
  }
  else if (cmdString == "load") { // Recargar calibración (por si acaso)
    loadCalibration();
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
  tft.fillRect(10, 80, 200, 40, TFT_BLACK); 
  tft.setCursor(10, 50);
  tft.print(max(0.0f, fuerzaIsquios), 2);
  tft.setTextSize(2);
  tft.print(" Kg");


  // --- FUERZA CUÁDRICEPS ---
  tft.setTextSize(3);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.fillRect(10, 190, 200, 40, TFT_BLACK);
  tft.setCursor(tft.width() - 120, 50);
  tft.print(max(0.0f, fuerzaCuads), 2); // 2 decimales
  tft.setTextSize(2);
  tft.print(" Kg");


  // --- RATIO H:Q (Hamstring-to-Quadriceps ratio)--- 
  float ratio = 0.0;
  if (fuerzaCuads > 0.01) { 
    ratio = (fuerzaIsquios / fuerzaCuads);
  }
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int ratioLabelX = tft.width() / 2 - 80;
  // 2. Definimos el Y de la línea (el mismo que en drawStaticUI)
  int ratioLabelY = 120;
  // 3. Calculamos dónde termina la etiqueta "Ratio H:Q: " (12 chars * 12 px)
  int ratioValueX = ratioLabelX + (12 * 12);

  // 4. Borra SÓLO el área del número
  tft.fillRect(ratioValueX, ratioLabelY, 170 - ratioValueX, 20, TFT_BLACK);
  // 5. Pone el cursor al inicio del número
  tft.setCursor(ratioValueX, ratioLabelY);
  // 6. Imprime el número
  tft.print(ratio, 2);

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(5, tft.height() - 10); // Abajo a la izquierda
  tft.print("F1:");
  tft.print(CAL_FACTOR_ISQUIOS, 0);
  tft.print(" F2:");
  tft.print(CAL_FACTOR_CUADS, 0);
}

/**
 * @brief Dibuja la UI estática (etiquetas) una sola vez.
 * Esto previene el parpadeo (flicker).
 */
void drawStaticUI() {
  tft.fillScreen(TFT_BLACK); // Limpia la pantalla

  // --- ETIQUETA ISQUIOTIBIALES ---
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK); 
  tft.setCursor(10, 10);
  tft.print("Isquios"); 
  tft.setCursor(10, 50);


  // --- ETIQUETA CUÁDRICEPS ---
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(tft.width() - 120, 10);
  tft.print("Cuads");


  // --- ETIQUETA RATIO H:Q ---
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Usamos las mismas coordenadas base que en updateDisplay
  int ratioLabelX = tft.width() / 2 - 80;
  int ratioLabelY = 120;
  tft.setCursor(ratioLabelX, ratioLabelY);
  tft.print("Ratio H:Q: ");
}

/**
 * @brief Carga los factores de calibración desde la NVS.
 * Si no existen, usa los valores por defecto.
 * Aplica los factores a los sensores.
 */
void loadCalibration() {
  Serial.println("Cargando calibracion desde NVS...");
  // Carga el valor, o usa el default (el valor actual) si no se encuentra
  CAL_FACTOR_ISQUIOS = preferences.getFloat("cal_isquios", CAL_FACTOR_ISQUIOS);
  CAL_FACTOR_CUADS = preferences.getFloat("cal_cuads", CAL_FACTOR_CUADS);

  Serial.print("Factor Isquios: "); Serial.println(CAL_FACTOR_ISQUIOS);
  Serial.print("Factor Cuads: "); Serial.println(CAL_FACTOR_CUADS);

  // Aplicar los factores a los sensores
  sensorIsquios.setCalFactor(CAL_FACTOR_ISQUIOS);
  sensorCuads.setCalFactor(CAL_FACTOR_CUADS);
}

/**
 * @brief Guarda los factores de calibración actuales en la NVS.
 */
void saveCalibration() {
  Serial.println("Guardando calibracion en NVS...");
  preferences.putFloat("cal_isquios", CAL_FACTOR_ISQUIOS);
  preferences.putFloat("cal_cuads", CAL_FACTOR_CUADS);

  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, tft.height() / 2); // Centrado
  tft.print("¡Calibracion Guardada!");
  delay(1000);
  drawStaticUI(); // Redibujar la UI
}