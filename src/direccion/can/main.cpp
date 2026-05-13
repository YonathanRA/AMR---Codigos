/**
 * ================================================================
 * CONTROL DE DIRECCIÓN - ACTUADOR LINEAL  v2.2
 * Feather RP2040 CAN + Puente H BTS7960
 * ================================================================
 *
 * Mejoras v2.2:
 *   - WATCHDOG: si master no manda comandos en 500ms → quieto
 *   - Modo seguro: motor OFF, congela posición actual
 *   - Rearme: cuando llega comando 0x100 del master
 *
 * CAN:
 *   RX  0x100 → posición objetivo
 *   RX  0x210 → emergencia (cuenta para watchdog)
 *   TX  0x101 → telemetría posición actual
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP2515.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
#define PIN_EN    5
#define PIN_PWM_A 6
#define PIN_PWM_B 9
#define PIN_FEEDBACK A0

#define CAN_BAUDRATE  250000UL
#define FILTRO_ID     0x100
#define ID_TELEMETRY  0x101
#define ID_PARO_EMERGENCIA 0x210

Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

const int RAW_MIN = 1200;
const int RAW_MAX = 2740;

byte posicionObjetivo;
const int TOLERANCIA = 1;

// 🆕 WATCHDOG
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long lastCanRx = 0;
bool masterOffline = false;

const int N = 6;
int  bufferLecturas[N];
long sumaLecturas = 0;
int  indice = 0;
bool bufferLleno = false;

byte    posActualUI    = 0;
int     errorUI        = 0;
byte    pwmUI          = 0;
int8_t  direccionUI    = 0;
unsigned long lastTelemetry = 0;
unsigned long lastUI        = 0;
unsigned long lastDebug     = 0;
unsigned long cmdsRecibidos = 0;

// ================================================================
int leerPotFiltrado() {
  int nueva = analogRead(PIN_FEEDBACK);
  sumaLecturas -= bufferLecturas[indice];
  bufferLecturas[indice] = nueva;
  sumaLecturas += nueva;
  indice++;
  if (indice >= N) { indice = 0; bufferLleno = true; }
  return bufferLleno ? (sumaLecturas / N) : (sumaLecturas / indice);
}

byte lecturaToPorcentaje(int lecturaRaw) {
  lecturaRaw = constrain(lecturaRaw, RAW_MIN, RAW_MAX);
  int porcentaje = map(lecturaRaw, RAW_MIN, RAW_MAX, 0, 100);
  return (byte)constrain(porcentaje, 0, 100);
}

void motorStop() {
  analogWrite(PIN_PWM_A, 0);
  analogWrite(PIN_PWM_B, 0);
}

void motorAvanzar(byte pwmValor) {
  analogWrite(PIN_PWM_B, 0);
  analogWrite(PIN_PWM_A, pwmValor);
}

void motorRetraer(byte pwmValor) {
  analogWrite(PIN_PWM_A, 0);
  analogWrite(PIN_PWM_B, pwmValor);
}

void moverActuadorHaciaObjetivo() {
  // 🆕 Si master offline, no hacer nada (motor congelado)
  if (masterOffline) {
    motorStop();
    pwmUI = 0;
    direccionUI = 0;
    return;
  }

  byte posicionActual = lecturaToPorcentaje(leerPotFiltrado());
  int  error = (int)posicionObjetivo - (int)posicionActual;

  posActualUI = posicionActual;
  errorUI     = error;

  if (abs(error) <= TOLERANCIA) {
    motorStop();
    pwmUI = 0;
    direccionUI = 0;
    return;
  }

  byte pwmValor = map(abs(error), 0, 100, 135, 195);
  pwmValor = constrain(pwmValor, 0, 255);
  pwmUI = pwmValor;

  if (error > 0) {
    motorAvanzar(pwmValor);
    direccionUI = +1;
  } else {
    motorRetraer(pwmValor);
    direccionUI = -1;
  }
}

void enviarFeedbackCAN() {
  if (!canOK) return;
  byte posicionActual = lecturaToPorcentaje(leerPotFiltrado());
  mcp.beginPacket(ID_TELEMETRY);
  mcp.write(posicionActual);
  mcp.endPacket();
}

// ================================================================
//  OLED
// ================================================================
void drawBar(const char *label, int val, int minV, int maxV, int y) {
  const int BAR_X = 30, BAR_W = 70, BAR_H = 7;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, y);
  display.print(label);
  display.drawRect(BAR_X, y, BAR_W, BAR_H, SSD1306_WHITE);
  int fill = map(val, minV, maxV, 0, BAR_W - 2);
  fill = constrain(fill, 0, BAR_W - 2);
  if (fill > 0) display.fillRect(BAR_X + 1, y + 1, fill, BAR_H - 2, SSD1306_WHITE);
  display.setCursor(BAR_X + BAR_W + 2, y);
  display.print(val);
}

void drawBarSigned(const char *label, int val, int rangeAbs, int y) {
  const int BAR_X = 30, BAR_W = 70, BAR_H = 7;
  const int BAR_MID = BAR_X + BAR_W / 2;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, y);
  display.print(label);
  display.drawRect(BAR_X, y, BAR_W, BAR_H, SSD1306_WHITE);
  display.drawLine(BAR_MID, y, BAR_MID, y + BAR_H - 1, SSD1306_WHITE);
  int v = constrain(val, -rangeAbs, rangeAbs);
  int half = (BAR_W - 2) / 2;
  int fill = map(abs(v), 0, rangeAbs, 0, half);
  fill = constrain(fill, 0, half);
  if (fill > 0) {
    if (v > 0)      display.fillRect(BAR_MID + 1, y + 1, fill, BAR_H - 2, SSD1306_WHITE);
    else if (v < 0) display.fillRect(BAR_MID - fill, y + 1, fill, BAR_H - 2, SSD1306_WHITE);
  }
  display.setCursor(BAR_X + BAR_W + 2, y);
  if (v > 0) display.print('+');
  display.print(v);
}

void drawTag(int x, int y, const char *txt, bool active) {
  int w = 6 * strlen(txt) + 2;
  if (active) {
    display.fillRect(x, y - 1, w, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(x, y - 1, w, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(x + 1, y);
  display.print(txt);
  display.setTextColor(SSD1306_WHITE);
}

void updateOLED() {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("DIR CTRL"));
  display.setCursor(72, 2);
  display.print(masterOffline ? F("[OFFL]") : F("[LIVE]"));
  display.setCursor(110, 2);
  display.print(F("0x101"));
  display.setTextColor(SSD1306_WHITE);

  // 🆕 Pantalla de MASTER OFFLINE
  if (masterOffline) {
    display.setCursor(0, 18);
    display.print(F("!! MASTER OFFLINE"));
    display.setCursor(0, 30);
    unsigned long offlineMs = millis() - lastCanRx;
    display.print(F("Sin CAN: "));
    display.print(offlineMs / 1000);
    display.print('.');
    display.print((offlineMs % 1000) / 100);
    display.print(F("s"));

    display.setCursor(0, 44);
    display.print(F("MODO SEGURO"));
    display.setCursor(0, 54);
    display.print(F("Posicion congelada"));

    display.display();
    return;
  }

  drawBar      ("OBJ ", posicionObjetivo, 0, 100, 14);
  drawBar      ("ACT ", posActualUI,      0, 100, 25);
  drawBarSigned("ERR ", errorUI,          100,    36);

  display.drawLine(0, 46, 127, 46, SSD1306_WHITE);

  drawTag(0, 49, " EN ", true);

  const char *movTxt;
  bool movActive;
  if (direccionUI > 0)      { movTxt = " EXT> "; movActive = true; }
  else if (direccionUI < 0) { movTxt = "<RET ";  movActive = true; }
  else                      { movTxt = " STOP "; movActive = false; }
  drawTag(32, 49, movTxt, movActive);

  display.setCursor(80, 49);
  display.setTextColor(SSD1306_WHITE);
  display.print(canOK ? F("CAN OK") : F("CAN ER"));

  display.setCursor(0, 57);
  display.print(F("PWM="));
  display.print(pwmUI);

  display.setCursor(64, 57);
  display.print(F("RX="));
  display.print(cmdsRecibidos);

  display.display();
}

void splashOLED() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawRect(3, 3, 122, 58, SSD1306_WHITE);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 12);
  display.print(F("DIR CTRL"));
  display.setTextSize(1);
  display.setCursor(20, 34);
  display.print(F("Feather + BTS"));
  display.setCursor(28, 48);
  display.print(F("v2.2 + watchdog"));
  display.display();
}

// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_EN,    OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);

  analogWriteFreq(10000);
  analogWriteRange(255);

  motorStop();
  digitalWrite(PIN_EN, HIGH);

  analogReadResolution(12);

  for (int i = 0; i < N; i++) {
    bufferLecturas[i] = analogRead(PIN_FEEDBACK);
    sumaLecturas += bufferLecturas[i];
    delay(2);
  }
  bufferLleno = true;

  byte posicionInicial = lecturaToPorcentaje(leerPotFiltrado());
  posicionObjetivo = posicionInicial;
  posActualUI      = posicionInicial;

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    splashOLED();
    delay(1500);
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

  enviarFeedbackCAN();

  // 🆕 Watchdog inicia en offline
  lastCanRx = millis();
  masterOffline = true;
}

void loop() {
  unsigned long now = millis();

  if (canOK) {
    int packetSize = mcp.parsePacket();
    if (packetSize > 0) {
      uint32_t id = mcp.packetId();

      // 🆕 IDs relevantes para dirección
      bool idRelevante = (id == FILTRO_ID || id == ID_PARO_EMERGENCIA);
      if (idRelevante) {
        lastCanRx = now;
        if (masterOffline) {
          masterOffline = false;
        }
      }

      if (id == FILTRO_ID && mcp.available()) {
        byte nuevoObjetivo = mcp.read();
        posicionObjetivo = constrain(nuevoObjetivo, 0, 100);
        cmdsRecibidos++;
        enviarFeedbackCAN();
      } else {
        while (mcp.available()) mcp.read();
      }
    }
  }

  // 🆕 Verificar watchdog
  if (!masterOffline && (now - lastCanRx > WATCHDOG_TIMEOUT_MS)) {
    masterOffline = true;
    motorStop();
  }

  moverActuadorHaciaObjetivo();

  if (now - lastTelemetry >= 50) {
    lastTelemetry = now;
    enviarFeedbackCAN();
  }

  if (now - lastUI >= 100) {
    lastUI = now;
    if (displayOK) updateOLED();
  }
}