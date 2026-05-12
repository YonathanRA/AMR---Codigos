/*
 * ================================================================
 *   FEATHER RP2040 CAN — SUBSISTEMA FRENOS  v2.4
 *
 *   Mejoras v2.4:
 *   - Detección de PARO FÍSICO vía ACT_FLT del driver Pololu
 *     (cuando los relés del botón rojo cortan señal al driver)
 *   - Si ACT_FLT está en FAULT y pot fuera de rango → es paro físico
 *     (NO marca error, espera que se suelte)
 *   - Modo RECUPERACIÓN: cuando operador desactiva enable tras
 *     un error, retrae el actuador hasta entrar en rango
 *   - Si vuelve el paro físico durante recuperación → para inmediato
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

#define ID_CMD_FRENO    0x200
#define ID_FB_FRENO     0x201
#define ID_EMERGENCIA   0x210

// --- PINES DE HARDWARE ---
#define PIN_PWM         6
#define PIN_DIR         9
#define PIN_FEEDBACK    A0
#define ACT_FLT         5     // 🆕 USADO AHORA: LOW = fault del driver Pololu

// --- CALIBRACIÓN POTENCIÓMETRO ---
const int RAW_MIN = 815;
const int RAW_MAX = 868;

// --- POSICIONES OBJETIVO ---
#define POS_NORMAL       100
#define POS_FRENO          0
#define POS_EMERGENCIA     0

// --- CONTROL ---
byte posicionObjetivo = POS_NORMAL;
const int TOLERANCIA = 8;
const unsigned long TIMEOUT_FUERA_RANGO_MS = 2000;

// --- ESTADOS ---
bool modoEmergencia  = false;      // CAN 0x210 (señal del master/control remoto)
bool modoFreno       = false;      // CAN 0x200
bool errorPot        = false;      // Pot dañado (timeout sin recuperación)
bool paroFisico      = false;      // 🆕 Botón rojo apretado (detectado por ACT_FLT)
bool modoRecuperacion = false;     // 🆕 Retraer tras error/paro

unsigned long tFueraDeRango = 0;

// --- FILTRO MEDIA MÓVIL ---
const int N = 5;
int bufferLecturas[N];
long sumaLecturas = 0;
int indice        = 0;
bool bufferLleno  = false;

// --- ESTADO OLED ---
byte ultimoPWM       = 0;
byte ultimaDireccion = 0;
bool fueraDeRango    = false;

// =====================================================
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

// 🆕 Detección de paro físico vía pin FLT del driver Pololu
bool detectarParoFisico() {
  return (digitalRead(ACT_FLT) == LOW);
}

// =====================================================
void enviarFeedbackCAN() {
  if (!canOK) return;
  byte posicionActual = lecturaToPorcentaje(leerPotFiltrado());
  mcp.beginPacket(ID_FB_FRENO);
  mcp.write(posicionActual);
  mcp.endPacket();
}

void actualizarObjetivo() {
  if (modoEmergencia)   posicionObjetivo = POS_EMERGENCIA;
  else if (modoFreno)   posicionObjetivo = POS_FRENO;
  else                  posicionObjetivo = POS_NORMAL;
}

void motorOff() {
  analogWrite(PIN_PWM, 0);
  ultimoPWM = 0;
}

// =====================================================
//   LÓGICA PRINCIPAL DE CONTROL
// =====================================================
void moverActuadorHaciaObjetivo() {
  // Actualizar estado del paro físico cada ciclo
  paroFisico = detectarParoFisico();

  int lectura = leerPotFiltrado();
  unsigned long now = millis();
  bool enRango = (lectura >= RAW_MIN && lectura <= RAW_MAX);

  // ════════════════════════════════════════════════════════
  //   CASO 1: PARO FÍSICO ACTIVO
  // ════════════════════════════════════════════════════════
  // El botón rojo está apretado → relés cortan señal al driver.
  // El actuador físicamente extiende al máximo solo.
  // Nosotros NO debemos hacer nada, solo esperar.
  if (paroFisico) {
    motorOff();
    fueraDeRango = !enRango;  // solo para OLED
    tFueraDeRango = 0;

    // Si estábamos en errorPot, NO lo quitamos aquí
    // (el operador tiene que decidir entrar en recuperación)
    return;
  }

  // ════════════════════════════════════════════════════════
  //   CASO 2: MODO RECUPERACIÓN
  // ════════════════════════════════════════════════════════
  // El operador soltó el paro Y desactivó los enables.
  // Retraer hasta entrar en rango, luego volver a normal.
  if (modoRecuperacion) {
    // Si vuelve el paro físico durante recuperación → parar (seguridad)
    if (paroFisico) {
      motorOff();
      return;
    }

    if (enRango) {
      // 🎉 Logramos volver al rango → salir de recuperación
      modoRecuperacion = false;
      errorPot         = false;
      fueraDeRango     = false;
      tFueraDeRango    = 0;
      motorOff();
      return;
    }

    // Forzar retracción (sentido = HIGH, según tu código)
    digitalWrite(PIN_DIR, HIGH);
    analogWrite(PIN_PWM, 160);
    ultimoPWM = 160;
    ultimaDireccion = 1;
    return;
  }

  // ════════════════════════════════════════════════════════
  //   CASO 3: ERROR DE POT (persistente)
  // ════════════════════════════════════════════════════════
  // Motor apagado, esperando que operador inicie recuperación.
  // Se inicia cuando: enables OFF (modoEmergencia activo desde master)
  if (errorPot) {
    motorOff();

    // 🆕 Disparador de modo recuperación:
    // Si el master indica enable OFF (modoEmergencia=true desde CAN 0x210)
    // y el pot ya está en rango, salir directo del error.
    // Si NO está en rango, entrar a modo recuperación.
    if (modoEmergencia) {
      if (enRango) {
        // Pot ya volvió por sí solo (raro pero posible)
        errorPot = false;
      } else {
        // Iniciar recuperación
        modoRecuperacion = true;
      }
    }
    return;
  }

  // ════════════════════════════════════════════════════════
  //   CASO 4: OPERACIÓN NORMAL
  // ════════════════════════════════════════════════════════

  // ── Fuera de rango pero SIN paro físico ─────────────────
  if (!enRango) {
    if (!fueraDeRango) {
      tFueraDeRango = now;
    }
    fueraDeRango = true;

    // Timeout → error real de pot
    if (now - tFueraDeRango > TIMEOUT_FUERA_RANGO_MS) {
      errorPot = true;
      motorOff();
      return;
    }

    // Intento de recuperación automática suave
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

  // En rango — limpiar timer
  fueraDeRango  = false;
  tFueraDeRango = 0;

  // Control normal por error
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

// =====================================================
//   OLED
// =====================================================
void updateOLED() {
  display.clearDisplay();

  // Header
  display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1 - FRENOS"));
  display.setTextColor(SSD1306_WHITE);

  // ── Mensajes de estado (prioridad: paro físico > recuperación > error > emergencia) ──

  if (paroFisico && !modoRecuperacion) {
    // 🆕 PARO FÍSICO — no es error, es estado esperado
    display.setTextSize(2);
    display.setCursor(8, 18);
    display.print(F("PARO FIS"));
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(F("Boton rojo activo"));
    display.setCursor(0, 50);
    if ((millis() / 500) % 2 == 0) {
      display.print(F("(esperando...)"));
    }
  }
  else if (modoRecuperacion) {
    // 🆕 RECUPERACIÓN
    display.setTextSize(1);
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
    display.print(modoFreno ? F("FRENO") : F("NORMAL"));

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

  // Barra de posición
  byte actual;
  if (errorPot && !modoRecuperacion) {
    actual = 100;   // barra llena en error
  } else {
    actual = lecturaToPorcentaje(leerPotFiltrado());
  }
  display.drawRect(0, 54, 128, 10, SSD1306_WHITE);
  int fill = map(actual, 0, 100, 0, 126);
  if (fill > 0) display.fillRect(1, 55, fill, 8, SSD1306_WHITE);

  // ── Recuadro de error (solo si errorPot y NO recuperación) ──
  if (errorPot && !modoRecuperacion && !paroFisico) {
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
    if ((millis() / 500) % 2 == 0) {
      display.print(F("Desactiva enable"));
    }
    display.setTextColor(SSD1306_WHITE);
  }

  display.display();
}

// =====================================================
void setup() {
  Serial.begin(115200);

  analogWriteFreq(10000);
  analogWriteRange(255);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(ACT_FLT, INPUT_PULLUP);   // 🆕 pull-up por si flota
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
    display.print(F("FRENOS v2.4 INIT"));
    display.setCursor(10, 34);
    display.print(F("Det. paro fisico"));
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
  while (canOK) {
    int len = mcp.parsePacket();
    if (len <= 0) break;
    uint32_t id = mcp.packetId();

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
      modoFreno = (cmd >= 90);
      actualizarObjetivo();
      enviarFeedbackCAN();
    }
    else {
      while (mcp.available()) mcp.read();
    }
  }

  moverActuadorHaciaObjetivo();

  static unsigned long lastOLED = 0;
  if (millis() - lastOLED >= 100) {
    lastOLED = millis();
    if (displayOK) updateOLED();
  }
}