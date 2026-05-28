/**
 * ================================================================
 * AMR1 - CONTROL MASTER DASHBOARD PRO v2.4
 * Hardware: Adafruit Feather RP2040 CAN
 * ================================================================
 *
 * NUEVO v2.4:
 *   - INTERLOCK FRENO-MOTOR: si freno > 7%, valorVel se fuerza a 0
 *   - Relé queda activo (más rápida liberación)
 *   - Se mantiene throttle/freno excluyentes desde v2.3
 *
 * MAPA CAN:
 *   TX 0x100 → Comando dirección
 *   TX 0x200 → Comando freno PROPORCIONAL (0-100%)
 *   TX 0x210 → Emergencia (broadcast)
 *   TX 0x300 → Velocidad
 *   TX 0x310 → Enable relay
 *   TX 0x320 → Reversa
 *   TX 0x330 → High gear (informativo)
 *   RX 0x101 → Feedback dirección
 */

#include "hardware/gpio.h"
#include <Adafruit_GFX.h>
#include <Adafruit_MCP2515.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ================================================================
//  SBUS PINOUT
// ================================================================
#define SBUS_PIN_RX  1
#define BAUD_SBUS    100000

// --- CAN BUS ---
#define CAN_BAUDRATE 250000
Adafruit_MCP2515 mcp(PIN_CAN_CS);
bool canOK = false;

// --- OLED ---
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// ================================================================
//  CANALES SBUS
// ================================================================
#define CH_THROTTLE  0
#define CH_STEERING  1
#define CH_EMERG_A   2
#define CH_ENABLE    4
#define CH_GEAR_REV  6

#define SBUS_LOW   500
#define SBUS_HIGH  1500
#define CH7_HIGH   1300
#define CH7_LOW    700

// Zonas de la palanca de throttle
#define THROTTLE_MIN          172
#define THROTTLE_FRENA_TOP    910
#define THROTTLE_DEAD_BOT     911
#define THROTTLE_DEAD_TOP     1100
#define THROTTLE_ACEL_BOT     1101
#define THROTTLE_MAX          1811

#define VEL_MAX_LOW   120
#define VEL_MAX_HIGH  200

// 🆕 INTERLOCK FRENO-MOTOR
#define UMBRAL_FRENO_BLOQUEO  7   // si freno > este valor → motor bloqueado

// ================================================================
//  CAN IDs
// ================================================================
#define CAN_TX_DIR        0x100
#define CAN_TX_BRAKE      0x200
#define CAN_TX_EMERG      0x210
#define CAN_TX_VEL        0x300
#define CAN_TX_ENABLE     0x310
#define CAN_TX_REVERSA    0x320
#define CAN_TX_HIGHGEAR   0x330
#define CAN_RX_DIR_FB     0x101

// ================================================================
//  GLOBAL STATE
// ================================================================
struct SBUS_State {
  uint16_t ch[16];
  bool failsafe;
  unsigned long last_update;
  int valorVel;
  int valorDir;
  int valorFreno;
  int valorEnable;
  int valorReversa;
  bool highGear;
  bool motorBloqueado;   // 🆕 indica si el interlock está activo
};

volatile SBUS_State vehicle;

byte           dirFeedback        = 50;
unsigned long  lastDirFeedback    = 0;
unsigned long  cntFeedbackRx      = 0;
bool           dirFeedbackOnline  = false;

// ================================================================
//  SBUS DECODING (CORE 1)
// ================================================================
void parseSBUS(uint8_t *packet) {
  vehicle.ch[0]  = ((packet[1]    | packet[2]  << 8)                    & 0x07FF);
  vehicle.ch[1]  = ((packet[2]>>3 | packet[3]  << 5)                    & 0x07FF);
  vehicle.ch[2]  = ((packet[3]>>6 | packet[4]  << 2 | packet[5]  << 10) & 0x07FF);
  vehicle.ch[3]  = ((packet[5]>>1 | packet[6]  << 7)                    & 0x07FF);
  vehicle.ch[4]  = ((packet[6]>>4 | packet[7]  << 4)                    & 0x07FF);
  vehicle.ch[5]  = ((packet[7]>>7 | packet[8]  << 1 | packet[9]  << 9)  & 0x07FF);
  vehicle.ch[6]  = ((packet[9]>>2 | packet[10] << 6)                    & 0x07FF);
  vehicle.ch[7]  = ((packet[10]>>5| packet[11] << 3)                    & 0x07FF);
  vehicle.ch[8]  = ((packet[12]   | packet[13] << 8)                    & 0x07FF);
  vehicle.ch[9]  = ((packet[13]>>3| packet[14] << 5)                    & 0x07FF);
  vehicle.ch[10] = ((packet[14]>>6| packet[15] << 2 | packet[16] << 10) & 0x07FF);
  vehicle.ch[11] = ((packet[16]>>1| packet[17] << 7)                    & 0x07FF);
  vehicle.ch[12] = ((packet[17]>>4| packet[18] << 4)                    & 0x07FF);
  vehicle.ch[13] = ((packet[18]>>7| packet[19] << 1 | packet[20] << 9)  & 0x07FF);
  vehicle.ch[14] = ((packet[20]>>2| packet[21] << 6)                    & 0x07FF);
  vehicle.ch[15] = ((packet[21]>>5| packet[22] << 3)                    & 0x07FF);

  vehicle.failsafe = packet[23] & 0x08;

  vehicle.valorDir = map(vehicle.ch[CH_STEERING], 172, 1811, 100, 0);

  uint16_t v7 = vehicle.ch[CH_GEAR_REV];
  if (v7 > CH7_HIGH) {
    vehicle.valorReversa = 1;
    vehicle.highGear     = false;
  } else if (v7 < CH7_LOW) {
    vehicle.valorReversa = 0;
    vehicle.highGear     = true;
  } else {
    vehicle.valorReversa = 0;
    vehicle.highGear     = false;
  }

  // ────────────────────────────────────────────────────────
  // LÓGICA DE THROTTLE + FRENO PROPORCIONAL
  // ────────────────────────────────────────────────────────
  uint16_t throttle = vehicle.ch[CH_THROTTLE];

  if (throttle >= THROTTLE_ACEL_BOT) {
    // Zona ACELERA
    int volMax = (vehicle.highGear && !vehicle.valorReversa)
                   ? VEL_MAX_HIGH
                   : VEL_MAX_LOW;
    vehicle.valorVel   = map(throttle, THROTTLE_ACEL_BOT, THROTTLE_MAX, 50, volMax);
    vehicle.valorFreno = 0;
  }
  else if (throttle <= THROTTLE_FRENA_TOP) {
    // Zona FRENA proporcional
    vehicle.valorVel   = 0;
    vehicle.valorFreno = map(throttle, THROTTLE_MIN, THROTTLE_FRENA_TOP, 100, 1);
  }
  else {
    // Zona MUERTA
    vehicle.valorVel   = 0;
    vehicle.valorFreno = 0;
  }

  bool emerg_ch3  = (vehicle.ch[CH_EMERG_A] > SBUS_HIGH);
  bool enable_ch5 = (vehicle.ch[CH_ENABLE]  > SBUS_HIGH);

  // En emergencia/enable OFF: freno máximo
  if (emerg_ch3 || !enable_ch5) {
    vehicle.valorFreno   = 100;
    vehicle.valorVel     = 0;
    vehicle.valorReversa = 0;
    vehicle.valorEnable  = 0;
  } else {
    vehicle.valorEnable = 1;
  }

  // ────────────────────────────────────────────────────────
  // 🆕 INTERLOCK FRENO-MOTOR
  // Si hay freno aplicado (> umbral), el motor NO puede moverse
  // ────────────────────────────────────────────────────────
  if (vehicle.valorFreno > UMBRAL_FRENO_BLOQUEO) {
    vehicle.valorVel       = 0;
    vehicle.motorBloqueado = true;
  } else {
    vehicle.motorBloqueado = false;
  }

  if (!vehicle.failsafe)
    vehicle.last_update = millis();
}

void setup1() {
  Serial1.begin(BAUD_SBUS, SERIAL_8E2);
  gpio_set_inover(SBUS_PIN_RX, 1);
}

void loop1() {
  static uint8_t buf[25];
  static int idx = 0;
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (idx == 0 && c != 0x0F) continue;
    buf[idx++] = c;
    if (idx == 25) {
      if (buf[24] == 0x00) parseSBUS(buf);
      idx = 0;
    }
  }
}

// ================================================================
//  CAN RX
// ================================================================
void leerCANEntrante() {
  if (!canOK) return;
  while (true) {
    int packetSize = mcp.parsePacket();
    if (packetSize <= 0) break;

    long id = mcp.packetId();
    if (id == CAN_RX_DIR_FB && mcp.available()) {
      byte fb = mcp.read();
      dirFeedback     = constrain(fb, 0, 100);
      lastDirFeedback = millis();
      cntFeedbackRx++;
    } else {
      while (mcp.available()) mcp.read();
    }
  }
  dirFeedbackOnline = (millis() - lastDirFeedback) < 1500;
}

// ================================================================
//  CAN TX
// ================================================================
void sendCANTelemetry() {
  if (!canOK) return;

  bool signalLost = vehicle.failsafe || (millis() - vehicle.last_update > 500);
  bool hayEmerg = signalLost || !vehicle.valorEnable;

  mcp.beginPacket(CAN_TX_DIR);
  mcp.write((byte)vehicle.valorDir);
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_EMERG);
  mcp.write((byte)(hayEmerg ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_BRAKE);
  byte cmdFreno = hayEmerg ? 100 : (byte)vehicle.valorFreno;
  mcp.write(cmdFreno);
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_VEL);
  mcp.write((byte)map(vehicle.valorVel, 0, VEL_MAX_HIGH, 0, 100));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_ENABLE);
  mcp.write((byte)(vehicle.valorEnable ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_REVERSA);
  mcp.write((byte)(vehicle.valorReversa ? 1 : 0));
  mcp.endPacket();

  mcp.beginPacket(CAN_TX_HIGHGEAR);
  mcp.write((byte)(vehicle.highGear ? 1 : 0));
  mcp.endPacket();
}

// ================================================================
//  OLED
// ================================================================
void drawBar(int x, int y, int w, int h, int val, int valMax) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = ((w - 2) * val) / valMax;
  fill = constrain(fill, 0, w - 2);
  if (fill > 0)
    display.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE);
}

void drawDirBar(int x, int y, int w, int h) {
  const int MID = x + w / 2;

  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.drawLine(MID, y, MID, y + h - 1, SSD1306_WHITE);

  int q1 = x + w / 4;
  int q3 = x + (w * 3) / 4;
  display.drawPixel(q1, y, SSD1306_WHITE);
  display.drawPixel(q1, y + h - 1, SSD1306_WHITE);
  display.drawPixel(q3, y, SSD1306_WHITE);
  display.drawPixel(q3, y + h - 1, SSD1306_WHITE);

  int cmdX = x + 1 + ((w - 2) * vehicle.valorDir) / 100;
  cmdX = constrain(cmdX, x + 1, x + w - 2);
  display.fillTriangle(
    cmdX - 3, y - 5,
    cmdX + 3, y - 5,
    cmdX,     y - 1,
    SSD1306_WHITE
  );

  if (dirFeedbackOnline) {
    int fbX = x + 1 + ((w - 2) * dirFeedback) / 100;
    fbX = constrain(fbX, x + 1, x + w - 2);
    if (fbX >= MID) {
      display.fillRect(MID, y + 2, fbX - MID, h - 4, SSD1306_WHITE);
    } else {
      display.fillRect(fbX, y + 2, MID - fbX, h - 4, SSD1306_WHITE);
    }
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x + w/2 - 2, y + 2);
    display.print('?');
  }
}

void drawTag(int x, int y, const char *txt, bool active) {
  int w = 6 * strlen(txt) + 2;
  if (active) {
    display.fillRect(x, y - 1, w, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(x, y - 1, w, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(x + 1, y + 1);
  display.print(txt);
  display.setTextColor(SSD1306_WHITE);
}

void updateOLED() {
  display.clearDisplay();
  bool signalLost = vehicle.failsafe || (millis() - vehicle.last_update > 500);

  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(F("AMR1"));

  display.setCursor(40, 2);
  if (signalLost)              display.print(F("NO SIGNAL"));
  else if (!vehicle.valorEnable) display.print(F("  SAFE   "));
  else                         display.print(F("  LIVE   "));

  display.setCursor(106, 2);
  if (!canOK)                  display.print(F("CAN-X"));
  else if (dirFeedbackOnline)  display.print(F("LINK*"));
  else                         display.print(F("LINK?"));

  display.setTextColor(SSD1306_WHITE);

  if (signalLost) {
    display.setTextSize(2);
    display.setCursor(14, 20);
    display.print(F("TX LOST!"));
    display.setTextSize(1);
    display.setCursor(8, 44);
    display.print(F("Sistema en HALT"));
    display.setCursor(8, 54);
    display.print(F("Espera senal..."));
    display.display();
    return;
  }

  // Fila VEL
  display.setCursor(0, 14);
  display.print(F("VEL"));
  drawBar(20, 14, 78, 8, vehicle.valorVel, VEL_MAX_HIGH);

  display.setCursor(102, 14);
  int velPct = (vehicle.valorVel * 100) / VEL_MAX_HIGH;
  if (velPct < 10) display.print(F("  "));
  else if (velPct < 100) display.print(F(" "));
  display.print(velPct);
  display.print('%');

  // Fila FRENO proporcional
  display.setCursor(0, 24);
  display.print(F("BRK"));
  drawBar(20, 24, 78, 6, vehicle.valorFreno, 100);
  display.setCursor(102, 24);
  if (vehicle.valorFreno < 10) display.print(F("  "));
  else if (vehicle.valorFreno < 100) display.print(F(" "));
  display.print(vehicle.valorFreno);
  display.print('%');

  // Fila DIR
  display.setCursor(0, 34);
  display.print(F("DIR"));
  drawDirBar(20, 33, 78, 8);

  display.setCursor(102, 34);
  if (vehicle.valorDir < 10) display.print(F("  "));
  else if (vehicle.valorDir < 100) display.print(F(" "));
  display.print(vehicle.valorDir);

  // Fila FB
  display.setCursor(0, 44);
  if (dirFeedbackOnline) {
    int err = (int)vehicle.valorDir - (int)dirFeedback;
    display.print(F("FB:"));
    display.print(dirFeedback);
    display.print('%');

    display.setCursor(48, 44);
    display.print(F("ERR:"));
    if (err > 0) display.print('+');
    display.print(err);

    display.setCursor(102, 44);
    if (abs(err) <= 3) display.print(F("OK "));
    else if (abs(err) <= 10) display.print(F("..."));
    else display.print(F("!!!"));
  } else {
    display.print(F("FB: sin link al actuador"));
  }

  display.drawLine(0, 52, 127, 52, SSD1306_WHITE);

  drawTag(0, 54, " EN ", vehicle.valorEnable);

  const char *gearTxt;
  bool gearHL;
  if (vehicle.valorReversa)   { gearTxt = " REV "; gearHL = true; }
  else if (vehicle.highGear)  { gearTxt = "F-HI "; gearHL = true; }
  else                        { gearTxt = "F-LO "; gearHL = false; }
  drawTag(32, 54, gearTxt, gearHL);

  // 🆕 Tag de freno cambia a "LOCK" si el motor está bloqueado por interlock
  bool brk = vehicle.valorFreno > 0;
  const char *brkTxt;
  if (vehicle.motorBloqueado) brkTxt = "LOCK";  // bloqueo activo
  else if (brk)               brkTxt = "BRK";   // freno aplicado pero motor disponible
  else                        brkTxt = "---";
  drawTag(70, 54, brkTxt, brk);

  display.setCursor(98, 56);
  display.setTextColor(SSD1306_WHITE);
  display.print(F("R"));
  if (cntFeedbackRx % 1000 < 100) display.print('0');
  if (cntFeedbackRx % 1000 < 10) display.print('0');
  display.print(cntFeedbackRx % 1000);

  display.display();
}

void splashOLED() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawRect(3, 3, 122, 58, SSD1306_WHITE);

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 12);
  display.print(F("AMR1 CTRL"));

  display.setTextSize(1);
  display.setCursor(16, 34);
  display.print(F("Dashboard PRO v2.4"));
  display.setCursor(10, 48);
  display.print(F("Interlock activo"));

  display.display();
}

// ================================================================
//  DEBUG SERIAL
// ================================================================
void debugChannels() {
  Serial.print(F("CH1="));   Serial.print(vehicle.ch[CH_THROTTLE]);
  Serial.print(F(" CH7="));  Serial.print(vehicle.ch[CH_GEAR_REV]);
  Serial.print(F(" | DIR_CMD=")); Serial.print(vehicle.valorDir);
  Serial.print(F(" DIR_FB="));    Serial.print(dirFeedback);
  Serial.print(dirFeedbackOnline ? F("(ON)") : F("(--)"));
  Serial.print(F(" | VEL=")); Serial.print(vehicle.valorVel);
  Serial.print(F(" BRK=")); Serial.print(vehicle.valorFreno); Serial.print('%');
  if (vehicle.motorBloqueado) Serial.print(F(" [LOCK]"));
  Serial.print(F(" | "));
  if      (vehicle.valorReversa) Serial.print(F("REV "));
  else if (vehicle.highGear)     Serial.print(F("F-HI"));
  else                           Serial.print(F("F-LO"));
  Serial.print(F(" | EN=")); Serial.print(vehicle.valorEnable);
  Serial.print(F(" | RX=")); Serial.println(cntFeedbackRx);
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(115200);

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
}

void loop() {
  static unsigned long lastUI    = 0;
  static unsigned long lastCAN   = 0;
  static unsigned long lastDebug = 0;
  unsigned long now = millis();

  leerCANEntrante();

  if (now - lastCAN >= 20) {
    lastCAN = now;
    sendCANTelemetry();
  }

  if (now - lastUI >= 100) {
    lastUI = now;
    if (displayOK) updateOLED();
  }

  if (now - lastDebug >= 500) {
    lastDebug = now;
    debugChannels();
  }
}