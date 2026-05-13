/**
 * ================================================================
 * CONTROL DE DIRECCIÓN v2.1
 * Feather RP2040 CAN + Puente H BTS7960
 * ================================================================
 *
 * FIX v2.1:
 *   - Telemetría movida de 0x201 → 0x101
 *     (antes chocaba con feedback de frenos y saturaba el bus)
 *
 * PINOUT BTS7960 → Feather:
 *   R_EN + L_EN (juntos) → PIN 5  Feather  (enable puente)
 *   RPWM                 → PIN 6  Feather  (PWM A)
 *   LPWM                 → PIN 9  Feather  (PWM B)
 *
 * FEEDBACK:
 *   Potenciómetro        → PIN A0 Feather  (ADC 12 bits)
 *
 * CAN:
 *   RX  ID 0x100 → posición objetivo (1 byte, 0-100 %)
 *   TX  ID 0x101 → telemetría: posición actual (1 byte, 0-100 %)  ← FIX
 *
 * OLED SSD1306 128x64 → I2C (SDA/SCL)
 * ================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP2515.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
//  PINOUT
// ================================================================
#define PIN_EN    5      // R_EN + L_EN (BTS7960 enable)
#define PIN_PWM_A 6      // RPWM (BTS7960)
#define PIN_PWM_B 9      // LPWM (BTS7960)

#define PIN_FEEDBACK A0  // Potenciómetro de feedback

// ================================================================
//  CAN
// ================================================================
#define CAN_BAUDRATE  250000UL
#define FILTRO_ID     0x100   // Recibe posición objetivo (comando del master)
#define ID_TELEMETRY  0x101   // ← FIX: antes 0x201 (chocaba con frenos)

Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

// ================================================================
//  OLED
// ================================================================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// ================================================================
//  CALIBRACIÓN POTENCIÓMETRO (ADC 12 bits = 0-4095)
//  Original Arduino (10 bits): RAW_MIN=432, RAW_MAX=847
//  Escalado x4 para 12 bits:   RAW_MIN=1728, RAW_MAX=3388
//  AJUSTA con los valores reales que veas en el monitor Serial.
// ================================================================
const int RAW_MIN = 1200;   // IZQUIERDA (~1198-1201)
const int RAW_MAX = 2740;   // DERECHA   (~2738-2743)

// ================================================================
//  CONTROL
// ================================================================
byte posicionObjetivo;
const int TOLERANCIA = 1;

// ================================================================
//  FILTRO PROMEDIO MÓVIL (igual al de Yona)
// ================================================================
const int N = 6;
int  bufferLecturas[N];
long sumaLecturas = 0;
int  indice = 0;
bool bufferLleno = false;

// ================================================================
//  ESTADO PARA DEBUG / OLED
// ================================================================
byte    posActualUI    = 0;
int     errorUI        = 0;
byte    pwmUI          = 0;
int8_t  direccionUI    = 0;    // -1 = retrae, +1 = extiende, 0 = stop
unsigned long lastTelemetry = 0;
unsigned long lastUI        = 0;
unsigned long lastDebug     = 0;
unsigned long lastCanRx     = 0;
unsigned long cmdsRecibidos = 0;

// ================================================================
//  LECTURA Y FILTRADO
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

// ================================================================
//  MOTOR (BTS7960)
// ================================================================
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

// ================================================================
//  LÓGICA DE CONTROL
// ================================================================
void moverActuadorHaciaObjetivo() {
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

// ================================================================
//  TELEMETRÍA CAN
// ================================================================
void enviarFeedbackCAN() {
  if (!canOK) return;
  byte posicionActual = lecturaToPorcentaje(leerPotFiltrado());
  mcp.beginPacket(ID_TELEMETRY);
  mcp.write(posicionActual);
  mcp.endPacket();
}

// ================================================================
//  OLED HELPERS
// ================================================================
void drawBar(const char *label, int val, int minV, int maxV, int y) {
  const int BAR_X = 30;
  const int BAR_W = 70;
  const int BAR_H = 7;

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, y);
  display.print(label);

  display.drawRect(BAR_X, y, BAR_W, BAR_H, SSD1306_WHITE);
  int fill = map(val, minV, maxV, 0, BAR_W - 2);
  fill = constrain(fill, 0, BAR_W - 2);
  if (fill > 0)
    display.fillRect(BAR_X + 1, y + 1, fill, BAR_H - 2, SSD1306_WHITE);

  display.setCursor(BAR_X + BAR_W + 2, y);
  display.print(val);
}

void drawBarSigned(const char *label, int val, int rangeAbs, int y) {
  const int BAR_X = 30;
  const int BAR_W = 70;
  const int BAR_H = 7;
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

  // Header invertido
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("DIR CTRL"));

  bool canActive = canOK && (millis() - lastCanRx < 1500);
  display.setCursor(72, 2);
  display.print(canActive ? F("[LIVE]") : F("[IDLE]"));

  display.setCursor(110, 2);
  display.print(F("0x101"));

  display.setTextColor(SSD1306_WHITE);

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
  display.print(F("v2.1 fix CAN"));
  display.display();
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) delay(10);

  Serial.println(F("============================================"));
  Serial.println(F(" CONTROL DIRECCION - Feather + BTS7960"));
  Serial.println(F(" v2.1 - Fix CAN ID 0x201 -> 0x101"));
  Serial.println(F("============================================"));
  Serial.print  (F(" EN    pin    : ")); Serial.println(PIN_EN);
  Serial.print  (F(" PWM_A pin    : ")); Serial.println(PIN_PWM_A);
  Serial.print  (F(" PWM_B pin    : ")); Serial.println(PIN_PWM_B);
  Serial.print  (F(" Feedback pin : ")); Serial.println(PIN_FEEDBACK);
  Serial.print  (F(" CAN RX  ID   : 0x")); Serial.println(FILTRO_ID, HEX);
  Serial.print  (F(" CAN TX  ID   : 0x")); Serial.println(ID_TELEMETRY, HEX);
  Serial.println(F("--------------------------------------------"));

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

  Serial.print(F("[INIT] Pos inicial: "));
  Serial.print(posicionInicial); Serial.println(F(" %"));

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    splashOLED();
    Serial.println(F("[INIT] OLED OK"));
    delay(1500);
  } else {
    Serial.println(F("[INIT] OLED FAIL"));
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
    Serial.println(F("[INIT] CAN OK"));
  } else {
    Serial.println(F("[INIT] CAN FAIL"));
  }

  enviarFeedbackCAN();
  Serial.println(F("[INICIO] Esperando comandos en 0x100...\n"));
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  if (canOK) {
    int packetSize = mcp.parsePacket();
    if (packetSize > 0 && mcp.packetId() == FILTRO_ID) {
      if (mcp.available()) {
        byte nuevoObjetivo = mcp.read();
        posicionObjetivo = constrain(nuevoObjetivo, 0, 100);
        lastCanRx = now;
        cmdsRecibidos++;
        enviarFeedbackCAN();
      }
    }
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

  if (now - lastDebug >= 500) {
    lastDebug = now;
    int raw = leerPotFiltrado();
    Serial.print(F("RAW="));   Serial.print(raw);
    Serial.print(F(" | OBJ=")); Serial.print(posicionObjetivo); Serial.print('%');
    Serial.print(F(" | ACT=")); Serial.print(posActualUI);      Serial.print('%');
    Serial.print(F(" | ERR=")); Serial.print(errorUI);
    Serial.print(F(" | PWM=")); Serial.print(pwmUI);
    Serial.print(F(" | DIR="));
    if      (direccionUI > 0) Serial.print(F("EXT"));
    else if (direccionUI < 0) Serial.print(F("RET"));
    else                      Serial.print(F("STOP"));
    Serial.print(F(" | RX="));  Serial.print(cmdsRecibidos);
    Serial.print(F(" | CAN=")); Serial.println(canOK ? F("OK") : F("ER"));
  }
}