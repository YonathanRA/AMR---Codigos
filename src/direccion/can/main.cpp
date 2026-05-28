/*
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA DIRECCIÓN  v2.4
 *
 *   Cambios v2.4:
 *   - EEPROM corregida: SIZE=14, MAGIC=0xCA52, ADDR_MID=6, ADDR_RIGHT=10
 *     (int = 4 bytes en RP2040, no 2 como AVR)
 *   - Calibración default: L=1242  M=2005  R=2865 (RAW típico centro: 2046)
 *   - TOLERANCIA_PCT: 2 → 4 (zona muerta más amplia vs EMI)
 *   - Tolerancia mínima absoluta: 8 → 25 raw
 *   - Filtro promedio móvil: N=6 → N=12 (mejor anti-EMI tren motriz)
 *   - Inicialización SEGURA: driver deshabilitado hasta que todo
 *     esté listo, filtro completamente lleno antes de leer pos.
 *
 *   Funcionalidad heredada:
 *   - Watchdog 500ms con modo seguro automático
 *   - Drenado completo de cola CAN (while + continue)
 *   - EEPROM con calibración persistente
 *   - Paro de emergencia CAN 0x210
 *
 *   PINOUT:
 *     R_EN + L_EN  → PIN 5   (enable BTS7960 / driver)
 *     RPWM         → PIN 6   (PWM A)
 *     LPWM         → PIN 9   (PWM B)
 *     Potenciómetro→ PIN A0  (ADC 12 bits, 0-4095)
 *
 *   CAN:
 *     RX 0x100 → comando dirección (0-100%)
 *                  0   = IZQUIERDA (rawLeft  = 1242)
 *                  100 = DERECHA   (rawRight = 2865)
 *     RX 0x210 → emergencia (0/1)
 *     TX 0x101 → feedback posición (0-100%)
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP2515.h>

// ── Pines ──────────────────────────────────────────────────────
#define PIN_EN    5
#define PIN_PWM_A 6
#define PIN_PWM_B 9
#define PIN_POT   A0

// ── OLED ───────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// ── CAN BUS ────────────────────────────────────────────────────
#define CAN_BAUDRATE    250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

#define ID_CMD_DIR         0x100
#define ID_FB_DIR          0x101
#define ID_PARO_EMERGENCIA 0x210

// ── EEPROM (corregida: int=4 bytes en RP2040) ──────────────────
#define EEPROM_SIZE  14       // 2(magic) + 4(left) + 4(mid) + 4(right)
#define MAGIC_VAL    0xCA52   // valor nuevo, invalida EEPROM vieja corrupta
#define ADDR_MAGIC   0
#define ADDR_LEFT    2
#define ADDR_MID     6        // FIX: int = 4 bytes, no 2
#define ADDR_RIGHT   10       // FIX: corrido 4 bytes desde MID

// ── Puntos de calibración (RAW ADC, 0-4095) ────────────────────
int rawLeft  = 1242;   // posición IZQUIERDA
int rawMid   = 2005;   // posición MEDIA (centro)
int rawRight = 2865;   // posición DERECHA

// ── Control ────────────────────────────────────────────────────
const int TOLERANCIA_PCT     = 4;    // % error aceptable (era 2)
const int TOLERANCIA_MIN_RAW = 25;   // raw mínimo absoluto (era 8)

int  cmdRecibido    = 50;    // último comando del master en %
int  rawObjetivo    = 0;     // objetivo en RAW
bool modoEmergencia = false;

// ── WATCHDOG ───────────────────────────────────────────────────
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long lastCanRx = 0;
bool masterOffline = false;

// ── Filtro promedio móvil (anti-EMI) ───────────────────────────
const int N = 12;             // era 6
int  buf[N];
long suma  = 0;
int  idx   = 0;
bool lleno = false;

// ── Flag de init completo ──────────────────────────────────────
bool initCompleto = false;

// =====================================================
// EEPROM
// =====================================================
void eepromLoad() {
  EEPROM.begin(EEPROM_SIZE);
  uint16_t magic;
  EEPROM.get(ADDR_MAGIC, magic);
  if (magic == MAGIC_VAL) {
    EEPROM.get(ADDR_LEFT,  rawLeft);
    EEPROM.get(ADDR_MID,   rawMid);
    EEPROM.get(ADDR_RIGHT, rawRight);
  }
}

// =====================================================
// LECTURA POT FILTRADA
// =====================================================
int leerPot() {
  int v = analogRead(PIN_POT);
  suma -= buf[idx];
  buf[idx] = v;
  suma += v;
  idx++;
  if (idx >= N) { idx = 0; lleno = true; }
  return lleno ? (suma / N) : (suma / idx);
}

int rawToPct(int raw) {
  int lo = min(rawLeft, rawRight);
  int hi = max(rawLeft, rawRight);
  raw = constrain(raw, lo, hi);
  return (int)map(raw, rawLeft, rawRight, 0, 100);
}

int pctToRaw(int pct) {
  pct = constrain(pct, 0, 100);
  return (int)map(pct, 0, 100, rawLeft, rawRight);
}

int tolRaw() {
  return max(abs(rawRight - rawLeft) * TOLERANCIA_PCT / 100, TOLERANCIA_MIN_RAW);
}

// =====================================================
// MOTOR
// =====================================================
void motorStop()        { analogWrite(PIN_PWM_A, 0); analogWrite(PIN_PWM_B, 0); }
void motorFwd(byte pwm) { analogWrite(PIN_PWM_B, 0); analogWrite(PIN_PWM_A, pwm); }
void motorRev(byte pwm) { analogWrite(PIN_PWM_A, 0); analogWrite(PIN_PWM_B, pwm); }

void entrarModoSeguro() {
  motorStop();
}

void mover() {
  // Protección: no mover hasta que init esté completo
  // ni si hay emergencia o master offline
  if (!initCompleto || modoEmergencia || masterOffline) {
    motorStop();
    return;
  }

  int raw   = leerPot();
  int error = rawObjetivo - raw;

  if (abs(error) <= tolRaw()) {
    motorStop();
    return;
  }

  int  rango = abs(rawRight - rawLeft);
  byte pwm   = (byte)constrain(map(abs(error), 0, rango, 135, 195), 0, 255);

  if (error > 0) motorFwd(pwm);
  else           motorRev(pwm);
}

// =====================================================
// CAN
// =====================================================
void enviarFeedbackCAN() {
  if (!canOK) return;
  int raw = leerPot();
  byte pos = rawToPct(raw);
  mcp.beginPacket(ID_FB_DIR);
  mcp.write(pos);
  mcp.endPacket();
}

// =====================================================
// OLED
// =====================================================
void updateOLED() {
  display.clearDisplay();

  // Header
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - DIRECCION"));
  display.setTextColor(SSD1306_WHITE);

  // ── MASTER OFFLINE: prioridad máxima ────────────────────────
  if (masterOffline) {
    display.setCursor(0, 16);
    display.print(F("!! MASTER OFFLINE"));
    display.setCursor(0, 28);
    unsigned long offlineMs = millis() - lastCanRx;
    display.print(F("Sin CAN: "));
    display.print(offlineMs / 1000);
    display.print('.');
    display.print((offlineMs % 1000) / 100);
    display.print('s');

    display.setCursor(0, 40);
    display.print(F("MODO SEGURO:"));
    display.setCursor(0, 50);
    display.print(F("Motor detenido"));

    display.display();
    return;
  }

  // ── EMERGENCIA ──────────────────────────────────────────────
  if (modoEmergencia) {
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.print(F("EMERG!"));
    display.setTextSize(1);
  } else {
    // ── Estado normal ─────────────────────────────────────────
    int raw = leerPot();
    int pos = rawToPct(raw);

    display.setCursor(0, 16);
    display.print(F("CMD: "));
    display.print(cmdRecibido);
    display.print('%');

    display.setCursor(64, 16);
    display.print(F("POS: "));
    display.print(pos);
    display.print('%');

    display.setCursor(0, 28);
    display.print(F("ERR: "));
    int err = cmdRecibido - pos;
    if (err > 0) display.print('+');
    display.print(err);

    display.setCursor(64, 28);
    display.print(F("RAW:"));
    display.print(raw);
  }

  display.setCursor(0, 42);
  display.print(canOK ? F("CAN OK") : F("CAN ERR"));
  display.setCursor(64, 42);
  display.print(F("TOL:"));
  display.print(tolRaw());

  // Barra visual de posición
  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int posVisual = rawToPct(leerPot());
  int fill = map(posVisual, 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);

  // Marker de objetivo
  int targetX = map(cmdRecibido, 0, 100, 1, 127);
  display.drawLine(targetX, 53, targetX, 64, SSD1306_WHITE);

  display.display();
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  // ── Hardware init seguro ───────────────────────────────────
  pinMode(PIN_EN,    OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);

  // Driver DESHABILITADO al boot (evita movimientos espurios)
  digitalWrite(PIN_EN,    LOW);
  digitalWrite(PIN_PWM_A, LOW);
  digitalWrite(PIN_PWM_B, LOW);

  analogWriteFreq(10000);
  analogWriteRange(255);
  analogReadResolution(12);

  motorStop();
  delay(50);   // estabilización tras configurar PWMs

  // ── Pre-llenar filtro COMPLETAMENTE antes de cualquier decisión ─
  for (int i = 0; i < N; i++) {
    buf[i] = analogRead(PIN_POT);
    suma += buf[i];
    delay(3);
  }
  lleno = true;

  // ── EEPROM (sobreescribe defaults si hay calibración guardada) ─
  eepromLoad();

  // ── Posición objetivo inicial = posición actual estabilizada ─
  rawObjetivo = leerPot();
  cmdRecibido = rawToPct(rawObjetivo);

  // ── OLED ────────────────────────────────────────────────────
  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 14);
    display.print(F("DIRECCION v2.4"));
    display.setCursor(10, 26);
    display.print(F("L=")); display.print(rawLeft);
    display.print(F(" M="));  display.print(rawMid);
    display.setCursor(10, 36);
    display.print(F("R=")); display.print(rawRight);
    display.print(F(" Tol=")); display.print(TOLERANCIA_PCT); display.print('%');
    display.setCursor(10, 48);
    display.print(F("Watchdog 500ms"));
    display.display();
    delay(800);
  }

  // ── CAN ─────────────────────────────────────────────────────
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
    display.setCursor(10, 36);
    display.print(F("Esperando master..."));
    display.display();
    delay(500);
  }

  // ── Habilitar driver SOLO al final, todo listo ─────────────
  digitalWrite(PIN_EN, HIGH);

  // Watchdog arranca offline (espera primer comando del master)
  lastCanRx = millis();
  masterOffline = true;

  // Flag de init completo: ahora mover() puede ejecutarse
  // (pero masterOffline aún lo bloquea hasta primer CAN)
  initCompleto = true;
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  // Drenar TODOS los paquetes CAN pendientes en cada ciclo
  while (canOK) {
    int len = mcp.parsePacket();
    if (len <= 0) break;
    uint32_t id = mcp.packetId();

    // Reset de watchdog con IDs relevantes
    bool idRelevante = (id == ID_CMD_DIR || id == ID_PARO_EMERGENCIA);
    if (idRelevante) {
      lastCanRx = now;
      if (masterOffline) {
        masterOffline = false;
      }
    }

    // ── EMERGENCIA: prioridad máxima ──────────────────────────
    if (id == ID_PARO_EMERGENCIA && mcp.available()) {
      byte em = mcp.read();
      if (em == 1) {
        modoEmergencia = true;
        motorStop();
      } else if (em == 0) {
        modoEmergencia = false;
      }
      enviarFeedbackCAN();
      continue;
    }

    // ── Si offline o emergencia, drenar y saltar ──────────────
    if (modoEmergencia || masterOffline) {
      while (mcp.available()) mcp.read();
      continue;
    }

    // ── Comando de dirección ──────────────────────────────────
    if (id == ID_CMD_DIR && mcp.available()) {
      cmdRecibido = mcp.read();
      cmdRecibido = constrain(cmdRecibido, 0, 100);
      rawObjetivo = pctToRaw(cmdRecibido);
      enviarFeedbackCAN();
      continue;
    }

    // ── ID no relevante: drenar bytes ─────────────────────────
    while (mcp.available()) mcp.read();
  }

  // Verificar watchdog
  if (!masterOffline && (now - lastCanRx > WATCHDOG_TIMEOUT_MS)) {
    masterOffline = true;
    entrarModoSeguro();
  }

  // Control del motor
  mover();

  // Feedback CAN periódico cada 100 ms (solo si master online)
  static unsigned long lastFeedback = 0;
  if (now - lastFeedback >= 100) {
    lastFeedback = now;
    if (!masterOffline) enviarFeedbackCAN();
  }

  // OLED a 10 Hz
  static unsigned long lastOLED = 0;
  if (now - lastOLED >= 100) {
    lastOLED = now;
    if (displayOK) updateOLED();
  }
}