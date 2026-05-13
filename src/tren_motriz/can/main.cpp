/*
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA TREN MOTRIZ
 *
 *   Código actualizado del AMR1 para Feather RP2040 con CAN + OLED.
 *   Librerias: Adafruit MCP2515, Adafruit SSD1306, Wire.
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP2515.h>

// --- OLED ---
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// --- CAN BUS ---
#define CAN_BAUDRATE    250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

#define ID_CMD_PWM         0x300
#define ID_CMD_RELE        0x310
#define ID_CMD_DIR         0x320
#define ID_FB_PWM          0x301
#define ID_FB_RELE         0x311
#define ID_FB_DIR          0x321
#define ID_PARO_EMERGENCIA 0x210

// --- PINES DE HARDWARE ---
#define MOTOR_PWM      A1
#define RELAY_ENABLE   4
#define MOTOR_DIR_REV  11
#define CONFIG_L1      12   //  restaurado — activa transistor L1 constantemente
#define CONFIG_L2      13   //  restaurado — transistor L2 siempre apagado

// --- VARIABLES DE ESTADO ---
int  velocidadObj    = 0;
int  velocidadActual = 0;
bool relayState      = false;
bool direccion       = true;
bool modoEmergencia  = false;

const int pwmMin = 0;
const int pwmMax = 255;

// =====================================================
void enviarMensajeCAN(uint32_t id, uint8_t valor) {
  if (!canOK) return;
  valor = constrain(valor, 0, 100);
  mcp.beginPacket(id);
  mcp.write(valor);
  mcp.endPacket();
}

void activarParoEmergencia() {
  modoEmergencia  = true;
  velocidadObj    = 0;
  velocidadActual = 0;
  relayState      = false;

  analogWrite(MOTOR_PWM, 0);
  digitalWrite(RELAY_ENABLE, LOW);

  enviarMensajeCAN(ID_FB_PWM,  0);
  enviarMensajeCAN(ID_FB_RELE, 0);
}

void rampaPWM(int objetivo) {
  objetivo = constrain(objetivo, 0, 100);
  int pwmOut = map(objetivo, 0, 100, pwmMin, pwmMax);
  velocidadActual = pwmOut;
  analogWrite(MOTOR_PWM, velocidadActual);
}

// =====================================================
void updateOLED() {
  display.clearDisplay();

  // Header
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - TREN MOTRIZ"));
  display.setTextColor(SSD1306_WHITE);

  if (modoEmergencia) {
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.print(F("EMERG!"));
    display.setTextSize(1);
  } else {
    display.setCursor(0, 20);
    display.print(F("Vel: ")); display.print(velocidadObj); display.print(F(" %"));
  }

  display.setCursor(0, 35);
  display.print(F("Rele: ")); display.print(relayState ? F("ON") : F("OFF"));
  display.setCursor(64, 35);
  display.print(F("Dir: ")); display.print(direccion ? F("FWD") : F("REV"));

  // Estado CAN
  display.setCursor(0, 44);
  display.print(canOK ? F("CAN OK") : F("CAN ERR"));

  // Barra visual velocidad
  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int fill = map(velocidadObj, 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);

  display.display();
}

// =====================================================
void setup() {
  Serial.begin(115200);

  // --- HARDWARE INIT ---
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(RELAY_ENABLE, OUTPUT);
  pinMode(MOTOR_DIR_REV, OUTPUT);
  pinMode(CONFIG_L1, OUTPUT);   //  restaurado
  pinMode(CONFIG_L2, OUTPUT);   //  restaurado

  digitalWrite(CONFIG_L1, HIGH);  //  3.3V → transistor ON → 5V a L1 siempre
  digitalWrite(CONFIG_L2, LOW);   //  0V   → transistor OFF → 0V en L2 siempre
  digitalWrite(RELAY_ENABLE, LOW);
  digitalWrite(MOTOR_DIR_REV, HIGH);  // FIX: HIGH=FWD, alineado con direccion=true
  analogWrite(MOTOR_PWM, 0);
  analogWriteFreq(10000);             // FIX: frecuencia PWM explícita
  analogWriteRange(255);

  // --- OLED INIT ---
  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print(F("TREN MOTRIZ INIT..."));
    display.display();
    delay(500);
  }

  // --- CAN INIT ---
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, LOW);
  pinMode(PIN_CAN_RESET, OUTPUT);
  digitalWrite(PIN_CAN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_CAN_RESET, HIGH);
  delay(10);

  if (mcp.begin(CAN_BAUDRATE)) {
    canOK = true;
  }

  // Splash final con estado CAN
  if (displayOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print(canOK ? F("CAN OK") : F("CAN ERR"));
    display.display();
    delay(500);
  }
}

// =====================================================
void loop() {
  // 1. Revisar CAN Bus
  if (canOK) {
    int len = mcp.parsePacket();
    if (len > 0) {
      uint32_t id = mcp.packetId();

      // Paro Emergencia — tiene prioridad siempre
      if (id == ID_PARO_EMERGENCIA && mcp.available()) {
        byte em = mcp.read();
        if (em == 1) activarParoEmergencia();
        else if (em == 0) modoEmergencia = false;
      }

      if (!modoEmergencia) {
        if (id == ID_CMD_PWM && mcp.available()) {
          velocidadObj = mcp.read();
          velocidadObj = constrain(velocidadObj, 0, 100);
          enviarMensajeCAN(ID_FB_PWM, velocidadObj);
        }
        else if (id == ID_CMD_RELE && mcp.available()) {
          relayState = (mcp.read() > 0);
          if (!relayState) {             // FIX: apagar motor antes de abrir relé
            velocidadObj    = 0;
            velocidadActual = 0;
            analogWrite(MOTOR_PWM, 0);
          }
          digitalWrite(RELAY_ENABLE, relayState ? HIGH : LOW);
          enviarMensajeCAN(ID_FB_RELE, relayState);
        }
        else if (id == ID_CMD_DIR && mcp.available()) {
          bool nuevaDir = (mcp.read() > 0);
          if (nuevaDir != direccion) {   // FIX: parar antes de cambiar dirección
            velocidadObj    = 0;
            velocidadActual = 0;
            analogWrite(MOTOR_PWM, 0);
            delay(150);
            direccion = nuevaDir;
          }
          digitalWrite(MOTOR_DIR_REV, direccion ? HIGH : LOW);
          enviarMensajeCAN(ID_FB_DIR, direccion);
        }
      }
    }
  }

  // 2. Control PWM
  if (!modoEmergencia) {
    rampaPWM(velocidadObj);
  }

  // 3. OLED a 10Hz
  static unsigned long lastOLED = 0;
  if (millis() - lastOLED >= 100) {
    lastOLED = millis();
    if (displayOK) updateOLED();
  }
}
