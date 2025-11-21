/*
 * PROYECTO: Medidor de Fuerza H:Q (Isquios/Cuádriceps)
 * MCU: ESP32 DevKit V1
 * PANTALLA: ST7789 170x320
 * SENSORES: 2x Celdas de Carga "S" + 2x SparkFun HX711
 * CONTROLES: 1x Botón (Tara), 2x LEDs (Estado, BT)
 *
 * AUTOR: Ing. Alexis Ocando
 */
#include "BluetoothSerial.h" 

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
TFT_eSprite sprite = TFT_eSprite(&tft);
BluetoothSerial SerialBT;
HX711_ADC sensorIsquios(PIN_ISQUIOS_DAT, PIN_ISQUIOS_CLK);
HX711_ADC sensorCuads(PIN_CUADS_DAT, PIN_CUADS_CLK);
Preferences preferences;

// --- CALIBRACIÓN ---
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
bool lastBTState = false;
float ratio = 0.0;

// --- VARIABLES GLOBALES (BOTÓN DEBOUNCE) ---
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE_DELAY_MS = 100;

// --- DEFINICIÓN DEL PAQUETE BINARIO ---
#define STX 0x02 // STX
#define ETX 0x03 // ETX
#define PAYLOAD_LEN 12 // 3 floats * 4 bytes/float = 12 bytes

const float GRAVITY = 9.81; // Factor de conversión Kg -> Newtons

// --- Declaraciones de funciones ---
void initDisplay();
void initSensors();
void initControls();
void initBluetooth();
void tareSensors();
void handleTareButton();
void handleLEDs();
void handleBluetooth();
void handleSensorDisplay();
void updateBTdisplay();
void drawStaticUI();
void loadCalibration(); 
void saveCalibration(); 
void sendBinaryData(float f_isquios, float f_cuads, float f_ratio);
void updateDisplay(float fuerzaIsquios, float fuerzaCuads, float ratio);



//=================================================================
//  SETUP
//=================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Medidor de Fuerza H:Q (v2.1)...");

  // Cargo la calibración que tengo guardada
  preferences.begin("fuerza-hq", false); // false = read/write
  loadCalibration();

  initControls(); // LEDs y botón
  initDisplay();
  // BT
  initBluetooth();
  initSensors();

}

//=================================================================
//  LOOP
//=================================================================
void loop() {
  
  // 1. Botón de tara (con debounce)
  handleTareButton();

  // 2. Comandos por BT
  handleBluetooth();
  
  // 3. LEDs de estado
  handleLEDs();
  
  // 4. Leer sensores y actualizar pantalla
  handleSensorDisplay();

  // 5. Estado de BT
  updateBTdisplay();

  delay(10);
}

//=================================================================
//  FUNCIONES DE INICIALIZACIÓN
//=================================================================

void initControls() {
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_LED_BT, OUTPUT);
  pinMode(PIN_BTN_TARE, INPUT_PULLUP); 
}

void initDisplay() {
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); 
  tft.setTextSize(2);
  tft.println("Iniciando...");  
  sprite.createSprite(320, 170);
  delay(1000);
}

void initSensors() {
  Serial.println("Iniciando sensores...");
  tft.println("Iniciando sensores...");

  sensorIsquios.begin();
  sensorIsquios.setGain(128);
  sensorIsquios.setCalFactor(CAL_FACTOR_ISQUIOS);
  sensorIsquios.start(2000, true); // 2s para estabilizar
  if (sensorIsquios.getTareTimeoutFlag()) {
    Serial.println("Error S1 Timeout");
    tft.println("Error S1 Timeout");
  }

  sensorCuads.begin();
  sensorCuads.setGain(128);
  sensorCuads.setCalFactor(CAL_FACTOR_CUADS);
  sensorCuads.start(2000, true); // 2s para estabilizar
  if (sensorCuads.getTareTimeoutFlag()) {
    Serial.println("Error S2 Timeout");
    tft.println("Error S2 Timeout");
  }

  // Espero a que estén listos
  unsigned long startWait = millis();
  while(millis() - startWait < 3000) { 
    sensorIsquios.update();
    sensorCuads.update();
    if(sensorIsquios.getDataSetStatus() && sensorCuads.getDataSetStatus()) break;
    delay(10);
  }

  Serial.println("Sensores tarados y listos.");
  delay(1000);
  tft.fillScreen(TFT_BLACK);
}

void initBluetooth() {
  Serial.println("Iniciando Bluetooth...");
  tft.println("Iniciando Bluetooth...");
  if (SerialBT.begin("ESP32_Fuerza_HQ")) {
    Serial.println("Bluetooth iniciado. Listo para conectar.");
  }
  delay(1000);
}

//=================================================================
//  FUNCIONES PRINCIPALES
//=================================================================

// Reviso el botón de tara con debounce
void handleTareButton() {
  int reading = digitalRead(PIN_BTN_TARE);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    lastButtonState = reading;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (lastButtonState == LOW) {
      tareSensors();
    }
  }
}

// Reviso comandos que llegan por Serial o BT
void handleBluetooth() {
  char cmdChar = ' ';
  String cmdString = "";

  // Desde la PC
  if (Serial.available() > 0) {
    cmdString = Serial.readStringUntil('\n');
    cmdChar = cmdString.charAt(0); // Para compatibilidad con 't'
  }
  
  // Desde el teléfono
  if (SerialBT.available() > 0) {
    cmdString = SerialBT.readStringUntil('\n');
    cmdChar = cmdString.charAt(0); // Para compatibilidad con 't'
  }

  // Proceso el comando
  cmdString.trim();

  if (cmdChar == 't') { // Comando de Tara
    tareSensors();
  }
  else if (cmdString.startsWith("i=")) { // Ajustar Isquios
    float nuevoFactor = cmdString.substring(2).toFloat();
    if (nuevoFactor > 1000) { 
      CAL_FACTOR_ISQUIOS = nuevoFactor;
      sensorIsquios.setCalFactor(CAL_FACTOR_ISQUIOS); 
      Serial.print("Nuevo factor Isquios: ");
      Serial.println(CAL_FACTOR_ISQUIOS);
    }
  }
  else if (cmdString.startsWith("q=")) { // Ajustar Cuádriceps
    float nuevoFactor = cmdString.substring(2).toFloat();
    if (nuevoFactor > 1000) { 
      CAL_FACTOR_CUADS = nuevoFactor;
      sensorCuads.setCalFactor(CAL_FACTOR_CUADS);
      Serial.print("Nuevo factor Cuads: ");
      Serial.println(CAL_FACTOR_CUADS);
    }
  }
  else if (cmdString == "save") { // Guardar
    saveCalibration();
  }
  else if (cmdString == "load") { // Recargar
    loadCalibration();
  }
}

// LEDs de estado
void handleLEDs() {
  // LED BT: encendido si está conectado
  if (SerialBT.connected()) {
    digitalWrite(PIN_LED_BT, HIGH);
  } else {
    if (millis() - lastLedBtTime >= (LED_BLINK_INTERVAL_MS - 500)) {
      lastLedBtTime = millis();
      ledBTState = !ledBTState;
      digitalWrite(PIN_LED_BT, ledBTState );
    }
  }

  if (!ledStatusState) {
    digitalWrite(PIN_LED_STATUS, HIGH);
    ledStatusState = true;
  } 
}

// Leo sensores, actualizo pantalla y mando data por BT
void handleSensorDisplay() {
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = millis();
    sensorIsquios.update();
    sensorCuads.update();

    // Obtengo el valor y lo paso a Newtons
    float rawIsquios = sensorIsquios.getData() * GRAVITY;
    float rawCuads = sensorCuads.getData() * GRAVITY;

    // Deadzone de 3N para filtrar ruido
    float fuerzaIsquios = (rawIsquios > 3.0) ? rawIsquios : 0.0f;
    float fuerzaCuads = (rawCuads > 3.0) ? rawCuads : 0.0f;

    // Ratio H:Q

    float ratio = 0.0;
    if (fuerzaCuads > 3.0) { 
      ratio = fuerzaIsquios / fuerzaCuads;
    }

    // Actualizo pantalla
    updateDisplay(fuerzaIsquios, fuerzaCuads, ratio);

    // Mando por BT
    sendBinaryData(fuerzaIsquios, fuerzaCuads, ratio);

  }
}

//=================================================================
//  FUNCIONES DE ACCIÓN
//=================================================================

// Tara de los sensores
void tareSensors() {
  Serial.println("Tarando sensores...");
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(tft.width() / 2 - 50, tft.height() / 2 - 10);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Tarando...");
  
  sensorIsquios.tare();
  sensorCuads.tare();

  delay(500); 
  tft.fillScreen(TFT_BLACK);
  Serial.println("Tara completa.");
  
  // Confirmo por BT
  if(SerialBT.connected()) {
    SerialBT.println("Tara Completa");
    SerialBT.write(0x06); // ACK
    SerialBT.flush();
  }
}

// Dibujo un gauge circular
void drawGauge(int x, int y, int r, float val, float maxVal, uint16_t color, String label, String units) {
  int ir = r - 10;
  int startAngle = 135;
  int endAngle = 45;
  
  // Fondo
  sprite.drawSmoothArc(x, y, r, ir, startAngle, endAngle, TFT_DARKGREY, TFT_BLACK, true);

  // Mapeo el valor a un ángulo (270 grados de recorrido)
  
  float percentage = val / maxVal;
  if (percentage > 1.0) percentage = 1.0;
  
  int arcLength = 270;
  int currentEndAngle = startAngle + (int)(percentage * arcLength);
  if (currentEndAngle >= 360) currentEndAngle -= 360;

  // Arco con el valor actual
  if (val > 0.5) { 
      sprite.drawSmoothArc(x, y, r, ir, startAngle, currentEndAngle, color, TFT_BLACK, true);
  }

  // Valor
  sprite.setTextDatum(MC_DATUM); // Middle Center
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setTextSize(3);
  sprite.drawString(String(val, 0), x, y);
  
  // Unidades
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  sprite.drawString(units, x, y + 20);

  // Título
  sprite.setTextSize(2);
  sprite.setTextColor(color, TFT_BLACK);
  sprite.drawString(label, x, y - r - 15);
}

// Dibujo todo en la pantalla
void updateDisplay(float fuerzaIsquios, float fuerzaCuads, float ratio) {
  
  sprite.fillSprite(TFT_BLACK);

  // Dimensiones de la pantalla
  int w = 320;
  int h = 170;
  
  // Posiciones de los gauges
  int r = 55; // Radio
  int yCenter = h / 2 + 10;
  int xLeft = 70;
  int xRight = w - 70;

  // 1. Gauge Isquios (Izquierda)
  drawGauge(xLeft, yCenter, r, fuerzaIsquios, 1000.0, TFT_CYAN, "ISQUIOS", "N");

  // 2. Gauge Cuads (Derecha)
  drawGauge(xRight, yCenter, r, fuerzaCuads, 1000.0, TFT_ORANGE, "CUADS", "N");

  // 3. Ratio (Centro)
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  
  sprite.setTextSize(1);
  sprite.drawString("RATIO H:Q", w/2, yCenter - 20);
  
  sprite.setTextSize(4);
  // Color según el rango (0.6 es bueno)
  uint16_t ratioColor = TFT_WHITE;
  if(ratio >= 0.55 && ratio <= 0.75) ratioColor = TFT_GREEN;
  else if (ratio > 0.0) ratioColor = TFT_RED;
  
  sprite.setTextColor(ratioColor, TFT_BLACK);
  sprite.drawFloat(ratio, 2, w/2, yCenter + 10);

  // 4. Estado Bluetooth
  sprite.setTextSize(1);
  sprite.setTextDatum(TR_DATUM); // Top Right
  if (SerialBT.connected()) {
    sprite.setTextColor(TFT_BLUE, TFT_BLACK);
    sprite.drawString("BT: ON", w - 5, 5);
  } else {
    sprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
    sprite.drawString("BT: --", w - 5, 5);
  }

  // Mando todo a la pantalla
  sprite.pushSprite(0, 0);
}


void sendBinaryData(float f_isquios, float f_cuads, float f_ratio) {
  if (!SerialBT.connected()) {
    return;
  }
  
  // buffer del payload (12 bytes)
  byte payload[PAYLOAD_LEN];
  
  // Copio los floats al buffer
  memcpy(payload, &f_isquios, 4);
  memcpy(payload + 4, &f_cuads, 4);
  memcpy(payload + 8, &f_ratio, 4);

  // Checksum
  byte checksum = 0;
  for (int i = 0; i < PAYLOAD_LEN; i++) {
    checksum ^= payload[i];
  }

  // Mando el paquete
  SerialBT.write(STX);
  SerialBT.write(PAYLOAD_LEN);
  SerialBT.write(payload, PAYLOAD_LEN);
  SerialBT.write(checksum);
  SerialBT.write(ETX);
}

// Cargo la calibración desde la memoria
void loadCalibration() {
  Serial.println("Cargando calibracion desde NVS...");
  // Si no hay nada guardado, uso los valores por defecto
  CAL_FACTOR_ISQUIOS = preferences.getFloat("cal_isquios", CAL_FACTOR_ISQUIOS);
  CAL_FACTOR_CUADS = preferences.getFloat("cal_cuads", CAL_FACTOR_CUADS);

  Serial.print("Factor Isquios: "); Serial.println(CAL_FACTOR_ISQUIOS);
  Serial.print("Factor Cuads: "); Serial.println(CAL_FACTOR_CUADS);

  // Aplico los factores
  sensorIsquios.setCalFactor(CAL_FACTOR_ISQUIOS);
  sensorCuads.setCalFactor(CAL_FACTOR_CUADS);
}

// Guardo la calibración en la memoria
void saveCalibration() {
  Serial.println("Guardando calibracion en NVS...");
  preferences.putFloat("cal_isquios", CAL_FACTOR_ISQUIOS);
  preferences.putFloat("cal_cuads", CAL_FACTOR_CUADS);

  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, tft.height() / 2);
  tft.print("¡Calibracion Guardada!");
  delay(1000);
}