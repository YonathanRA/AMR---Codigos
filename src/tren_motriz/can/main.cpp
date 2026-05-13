/*
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA TREN MOTRIZ  v2.4
 *
 *   Combinación de:
 *   - Mejoras de seguridad de la versión actual:
 *     * Pre-stop motor antes de cambiar dirección (delay 150ms)
 *     * Pre-stop PWM antes de abrir relé (evita arco)
 *     * MOTOR_DIR_REV alineado con direccion=true al boot
 *   - Funciones críticas recuperadas:
 *     * WATCHDOG 500ms con modoSeguro automático
 *     * masterOffline + OLED dedicado
 *     * Drenado completo de cola CAN (while + continue)
 *
 *   Marchas L1=HIGH, L2=LOW hardcoded (MARCHA_1 fija)
 *
 *   CAN:
 *     RX 0x300 → velocidad
 *     RX 0x310 → relay enable
 *     RX 0x320 → reversa
 *     RX 0x210 → emergencia
 *     TX 0x301/311/321 → feedback
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
#define CONFIG_L1      12
#define CONFIG_L2      13

// --- WATCHDOG ---
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long lastCanRx = 0;
bool masterOffline = false;

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

void entrarModoSeguro() {
  velocidadObj    = 0;
  velocidadActual = 0;
  relayState      = false;
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(RELAY_ENABLE, LOW);
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

  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - TREN MOTRIZ"));
  display.setTextColor(SSD1306_WHITE);

  if (masterOffline) {
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.print(F("!! MASTER OFFLINE"));
    display.setCursor(0, 28);
    unsigned long offlineMs = millis() - lastCanRx;
    display.print(F("Sin CAN: "));
    display.print(offlineMs / 1000);
    display.print('.');
    display.print((offlineMs % 1000) / 100);
    display.print(F("s"));

    display.setCursor(0, 40);
    display.print(F("MODO SEGURO:"));
    display.setCursor(0, 50);
    display.print(F("Vel=0 Relay=OFF"));

    display.display();
    return;
  }

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

  display.setCursor(0, 44);
  display.print(canOK ? F("CAN OK") : F("CAN ERR"));

  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int fill = map(velocidadObj, 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);

  display.display();
}

// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(RELAY_ENABLE, OUTPUT);
  pinMode(MOTOR_DIR_REV, OUTPUT);
  pinMode(CONFIG_L1, OUTPUT);
  pinMode(CONFIG_L2, OUTPUT);

  // Marchas hardcoded
  digitalWrite(CONFIG_L1, HIGH);
  digitalWrite(CONFIG_L2, LOW);

  digitalWrite(RELAY_ENABLE, LOW);
  digitalWrite(MOTOR_DIR_REV, HIGH);   // alineado con direccion=true (FWD)
  analogWrite(MOTOR_PWM, 0);
  analogWriteFreq(10000);
  analogWriteRange(255);

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print(F("TREN MOTRIZ v2.4"));
    display.setCursor(10, 32);
    display.print(F("Watchdog 500ms"));
    display.display();
    delay(500);
  }

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

  if (displayOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print(canOK ? F("CAN OK") : F("CAN ERR"));
    display.display();
    delay(500);
  }

  lastCanRx = millis();
  masterOffline = true;
}

// =====================================================
void loop() {
  unsigned long now = millis();

  // Drenar TODOS los paquetes CAN pendientes
  while (canOK) {
    int len = mcp.parsePacket();
    if (len <= 0) break;                   // no hay más paquetes → salir del while
    uint32_t id = mcp.packetId();

    // Resetear watchdog si llega ID relevante
    bool idRelevante = (id == ID_CMD_PWM       ||
                        id == ID_CMD_RELE      ||
                        id == ID_CMD_DIR       ||
                        id == ID_PARO_EMERGENCIA);
    if (idRelevante) {
      lastCanRx = now;
      if (masterOffline) {
        masterOffline = false;
      }
    }

    // ─── EMERGENCIA: prioridad máxima ─────────────────
    if (id == ID_PARO_EMERGENCIA && mcp.available()) {
      byte em = mcp.read();
      if (em == 1) activarParoEmergencia();
      else if (em == 0) modoEmergencia = false;
      continue;   // procesado, busca siguiente paquete
    }

    // ─── Si offline o emergencia, drenar y saltar ─────
    if (modoEmergencia || masterOffline) {
      while (mcp.available()) mcp.read();
      continue;   // ignorar comandos durante emergencia/offline
    }

    // ─── Comando velocidad ─────────────────────────────
    if (id == ID_CMD_PWM && mcp.available()) {
      velocidadObj = mcp.read();
      velocidadObj = constrain(velocidadObj, 0, 100);
      enviarMensajeCAN(ID_FB_PWM, velocidadObj);
      continue;
    }

    // ─── Comando relé ─────────────────────────────────
    if (id == ID_CMD_RELE && mcp.available()) {
      relayState = (mcp.read() > 0);
      if (!relayState) {                  // apagar motor antes de abrir relé
        velocidadObj    = 0;
        velocidadActual = 0;
        analogWrite(MOTOR_PWM, 0);
      }
      digitalWrite(RELAY_ENABLE, relayState ? HIGH : LOW);
      enviarMensajeCAN(ID_FB_RELE, relayState);
      continue;
    }

    // ─── Comando dirección (reversa) ──────────────────
    if (id == ID_CMD_DIR && mcp.available()) {
      bool nuevaDir = (mcp.read() > 0);
      if (nuevaDir != direccion) {        // parar antes de invertir
        velocidadObj    = 0;
        velocidadActual = 0;
        analogWrite(MOTOR_PWM, 0);
        delay(150);
        direccion = nuevaDir;
      }
      digitalWrite(MOTOR_DIR_REV, direccion ? HIGH : LOW);
      enviarMensajeCAN(ID_FB_DIR, direccion);
      continue;
    }

    // ─── ID no relevante: drenar bytes ───────────────
    while (mcp.available()) mcp.read();
  }

  // Verificar watchdog
  if (!masterOffline && (now - lastCanRx > WATCHDOG_TIMEOUT_MS)) {
    masterOffline = true;
    entrarModoSeguro();
  }

  // Control PWM (solo si activo)
  if (!modoEmergencia && !masterOffline) {
    rampaPWM(velocidadObj);
  }

  // OLED a 10Hz
  static unsigned long lastOLED = 0;
  if (now - lastOLED >= 100) {
    lastOLED = now;
    if (displayOK) updateOLED();
  }
}