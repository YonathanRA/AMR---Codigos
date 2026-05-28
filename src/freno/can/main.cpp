/*
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA FRENOS  v2.6
 *
 *   Cambios v2.6:
 *   - Freno PROPORCIONAL: cmd 0-100 mapea posición 100-0
 *     (antes binario: solo frenaba si cmd >= 90)
 *
 *   Funcionalidad:
 *   - Tolerancia 8 (zona muerta amplia, frenado rápido)
 *   - Timeout fuera de rango 2 seg → errorPot
 *   - Detección paro físico vía ACT_FLT (botón rojo)
 *   - Modo recuperación tras desactivar enables
 *   - Watchdog 1500ms (aplica freno si master se cae)
 *   - Drenado completo de cola CAN
 *
 *   ESCALA INVERTIDA:
 *   27%  = extendido (freno/emergencia) → ADC 868
 *   100% = retraído  (normal)           → ADC 815
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP2515.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

#define CAN_BAUDRATE    250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

#define ID_CMD_FRENO    0x200
#define ID_FB_FRENO     0x201
#define ID_EMERGENCIA   0x210

#define PIN_PWM         6
#define PIN_DIR         9
#define PIN_FEEDBACK    A0
#define ACT_FLT         5

const int RAW_MIN = 815;
const int RAW_MAX = 868;

#define POS_NORMAL       100
#define POS_FRENO          0
#define POS_EMERGENCIA     0

byte posicionObjetivo = POS_NORMAL;
const int TOLERANCIA = 8;
const unsigned long TIMEOUT_FUERA_RANGO_MS = 2000;

const unsigned long WATCHDOG_TIMEOUT_MS = 1500;
unsigned long lastCanRx = 0;
bool masterOffline = false;

bool modoEmergencia   = false;
byte modoFreno        = 0;     // 0 = sin freno, 1-100 = freno proporcional
bool errorPot         = false;
bool paroFisico       = false;
bool modoRecuperacion = false;

unsigned long tFueraDeRango = 0;

const int N = 5;
int bufferLecturas[N];
long sumaLecturas = 0;
int indice        = 0;
bool bufferLleno  = false;

byte ultimoPWM       = 0;
byte ultimaDireccion = 0;
bool fueraDeRango    = false;

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
  int porcentaje = map(lecturaRaw, RAW_MAX, RAW_MIN, 0, 100);
  return (byte)constrain(porcentaje, 0, 100);
}

bool detectarParoFisico() {
  return (digitalRead(ACT_FLT) == LOW);
}

void enviarFeedbackCAN() {
  if (!canOK) return;
  byte posicionActual = lecturaToPorcentaje(leerPotFiltrado());
  mcp.beginPacket(ID_FB_FRENO);
  mcp.write(posicionActual);
  mcp.endPacket();
}

void actualizarObjetivo() {
  if (masterOffline) {
    posicionObjetivo = POS_FRENO;
    return;
  }
  if (modoEmergencia) {
    posicionObjetivo = POS_EMERGENCIA;
  } else {
    // cmd=0 → POS_NORMAL(100), cmd=100 → POS_FRENO(0)
    posicionObjetivo = (byte)constrain(100 - (int)modoFreno, POS_FRENO, POS_NORMAL);
  }
}

void motorOff() {
  analogWrite(PIN_PWM, 0);
  ultimoPWM = 0;
}

// ================================================================
void moverActuadorHaciaObjetivo() {
  paroFisico = detectarParoFisico();
  int lectura = leerPotFiltrado();
  unsigned long now = millis();
  bool enRango = (lectura >= RAW_MIN && lectura <= RAW_MAX);

  // Paro físico
  if (paroFisico) {
    motorOff();
    fueraDeRango = !enRango;
    tFueraDeRango = 0;
    return;
  }

  // Recuperación
  if (modoRecuperacion) {
    if (paroFisico) {
      motorOff();
      return;
    }
    if (enRango) {
      modoRecuperacion = false;
      errorPot         = false;
      fueraDeRango     = false;
      tFueraDeRango    = 0;
      motorOff();
      return;
    }
    digitalWrite(PIN_DIR, HIGH);
    analogWrite(PIN_PWM, 160);
    ultimoPWM = 160;
    ultimaDireccion = 1;
    return;
  }

  // Error pot persistente
  if (errorPot) {
    motorOff();
    if (modoEmergencia || masterOffline) {
      if (enRango) {
        errorPot = false;
      } else {
        modoRecuperacion = true;
      }
    }
    return;
  }

  // Fuera de rango sin paro físico
  if (!enRango) {
    if (!fueraDeRango) tFueraDeRango = now;
    fueraDeRango = true;
    if (now - tFueraDeRango > TIMEOUT_FUERA_RANGO_MS) {
      errorPot = true;
      motorOff();
      return;
    }
    if (lectura < RAW_MIN) {
      digitalWrite(PIN_DIR, LOW);
      analogWrite(PIN_PWM, 160);
      ultimoPWM = 160;
      ultimaDireccion = 0;
    } else {
      digitalWrite(PIN_DIR, HIGH);
      analogWrite(PIN_PWM, 160);
      ultimoPWM = 160;
      ultimaDireccion = 1;
    }
    return;
  }

  fueraDeRango  = false;
  tFueraDeRango = 0;

  byte posicionActual = lecturaToPorcentaje(lectura);
  int  error = posicionObjetivo - posicionActual;

  if (abs(error) <= TOLERANCIA) {
    motorOff();
    return;
  }

  if (error > 0) {
    digitalWrite(PIN_DIR, HIGH);
    ultimaDireccion = 1;
  } else {
    digitalWrite(PIN_DIR, LOW);
    ultimaDireccion = 0;
  }

  byte pwmValor = map(abs(error), TOLERANCIA, 100, 140, 255);
  pwmValor = constrain(pwmValor, 140, 255);
  analogWrite(PIN_PWM, pwmValor);
  ultimoPWM = pwmValor;
}

// ================================================================
void updateOLED() {
  display.clearDisplay();

  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - FRENOS"));
  display.setTextColor(SSD1306_WHITE);

  if (masterOffline && !paroFisico) {
    display.setCursor(0, 16);
    display.print(F("!! MASTER OFFLINE"));
    display.setCursor(0, 28);
    unsigned long offlineMs = millis() - lastCanRx;
    display.print(F("Sin CAN: "));
    display.print(offlineMs / 1000);
    display.print('.');
    display.print((offlineMs % 1000) / 100);
    display.print(F("s"));
    display.setCursor(0, 42);
    display.print(F("FRENO APLICADO"));
    display.setCursor(0, 52);
    display.print(F("(seguridad auto)"));
    display.display();
    return;
  }

  if (paroFisico && !modoRecuperacion) {
    display.setTextSize(2);
    display.setCursor(8, 18);
    display.print(F("PARO FIS"));
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(F("Boton rojo activo"));
    display.setCursor(0, 50);
    if ((millis() / 500) % 2 == 0) display.print(F("(esperando...)"));
  }
  else if (modoRecuperacion) {
    display.setCursor(0, 16);
    display.print(F("MODO: RECUPERACION"));
    display.setCursor(0, 28);
    display.print(F("Retrayendo a normal"));
    display.setCursor(0, 40);
    display.print(F("ACT_FLT: "));
    display.print(paroFisico ? F("FAULT") : F("OK"));
    display.setCursor(0, 50);
    display.print(F("Pot: "));
    display.print(leerPotFiltrado());
  }
  else if (modoEmergencia) {
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.print(F("EMERG!"));
    display.setTextSize(1);
  }
  else if (fueraDeRango && !errorPot) {
    display.setCursor(0, 18);
    display.print(F("FUERA RANGO"));
    display.setCursor(0, 28);
    display.print(ultimaDireccion ? F("Retrayendo...") : F("Extendiendo..."));
    unsigned long restanteMs = TIMEOUT_FUERA_RANGO_MS - (millis() - tFueraDeRango);
    display.setCursor(0, 38);
    display.print(F("Timeout: "));
    display.print(restanteMs / 1000);
    display.print('.');
    display.print((restanteMs % 1000) / 100);
    display.print('s');
  }
  else {
    display.setCursor(0, 14);
    display.print(F("MODO: "));
    if (modoFreno > 0) {
      display.print(F("FRENO "));
      display.print(modoFreno);
      display.print('%');
    } else {
      display.print(F("NORMAL"));
    }

    display.setCursor(0, 24);
    display.print(F("FLT:  "));
    display.print(digitalRead(ACT_FLT) == LOW ? F("FAULT") : F("OK"));

    display.setCursor(0, 34);
    display.print(F("DIR:  "));
    display.print(ultimaDireccion ? F("RETR ") : F("EXTND"));

    display.setCursor(0, 44);
    display.print(F("PWM:  "));
    display.print(ultimoPWM);
  }

  byte actual;
  if (errorPot && !modoRecuperacion) {
    actual = 100;
  } else {
    actual = lecturaToPorcentaje(leerPotFiltrado());
  }
  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int fill = map(actual, 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);

  if (errorPot && !modoRecuperacion && !paroFisico && !masterOffline) {
    int boxX = 14, boxY = 18, boxW = 100, boxH = 28;
    display.fillRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);
    display.drawRect(boxX + 2, boxY + 2, boxW - 4, boxH - 4, SSD1306_BLACK);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(1);
    display.setCursor(boxX + 28, boxY + 4);
    display.print(F("!!ERROR!!"));
    display.setCursor(boxX + 6, boxY + 14);
    display.print(F("Pot fuera rango"));
    display.setCursor(boxX + 12, boxY + 22);
    if ((millis() / 500) % 2 == 0) display.print(F("Desactiva enable"));
    display.setTextColor(SSD1306_WHITE);
  }

  display.display();
}

// ================================================================
void setup() {
  Serial.begin(115200);

  analogWriteFreq(10000);
  analogWriteRange(255);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(ACT_FLT, INPUT_PULLUP);
  analogWrite(PIN_PWM, 0);
  digitalWrite(PIN_DIR, LOW);

  for (int i = 0; i < N; i++) {
    bufferLecturas[i] = analogRead(PIN_FEEDBACK);
    sumaLecturas += bufferLecturas[i];
    delay(2);
  }
  bufferLleno = true;
  posicionObjetivo = POS_NORMAL;

  Wire.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.print(F("FRENOS v2.6 INIT"));
    display.setCursor(10, 32);
    display.print(F("Watchdog 1500ms"));
    display.display();
    delay(800);
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

  lastCanRx = millis();
  masterOffline = true;
}

// ================================================================
void loop() {
  unsigned long now = millis();

  while (canOK) {
    int len = mcp.parsePacket();
    if (len <= 0) break;
    uint32_t id = mcp.packetId();

    bool idRelevante = (id == ID_CMD_FRENO || id == ID_EMERGENCIA);
    if (idRelevante) {
      lastCanRx = now;
      if (masterOffline) {
        masterOffline = false;
        actualizarObjetivo();
      }
    }

    if (id == ID_EMERGENCIA && mcp.available()) {
      byte em = mcp.read();
      if (em == 1) {
        modoEmergencia = true;
        modoFreno      = false;
      } else if (em == 0) {
        modoEmergencia = false;
      }
      actualizarObjetivo();
      enviarFeedbackCAN();
    }
    else if (id == ID_CMD_FRENO && !modoEmergencia && mcp.available()) {
      byte cmd = mcp.read();
      modoFreno = constrain(cmd, 0, 100);
      actualizarObjetivo();
      enviarFeedbackCAN();
    }
    else {
      while (mcp.available()) mcp.read();
    }
  }

  if (!masterOffline && (now - lastCanRx > WATCHDOG_TIMEOUT_MS)) {
    masterOffline = true;
    actualizarObjetivo();
  }

  moverActuadorHaciaObjetivo();

  static unsigned long lastOLED = 0;
  if (now - lastOLED >= 100) {
    lastOLED = now;
    if (displayOK) updateOLED();
  }
}