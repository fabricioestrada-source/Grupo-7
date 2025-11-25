/*
  ESP32 - Sistema completo (Opción C - DIVIDIDO)
  - OLED SH1106 128x64 (Adafruit_SH110X)
  - 4 vibradores PWM (ledc) (todos controlados por la app)
  - Bluetooth SPP: "nivel 1","nivel 2","nivel 3","apagado"
    + parser avanzado: "ALL,pct" "Vx,pct" "STATUS"
  - MPU6050 (movimiento)
  - 2 x SHT31 (0x44, 0x45) con fallback (no NaN)
  - Telemetría por BT en CSV
  - Todos los vibradores APAGADOS al inicio
  - Lectura SHT31 cada 100 ms con suavizado
  - Diseño: OPCIÓN C — Pantalla dividida (arriba sensores/mov, abajo vibradores solo %)
  - Ref image (upload): /mnt/data/c656a815-9eb9-4b2e-84e4-e9baeff357e4.jpg
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_SH110X.h>
#include "BluetoothSerial.h"
#include <math.h>

// ========== HARDWARE / LIBRARIES ==========
Adafruit_MPU6050 mpu;
Adafruit_SHT31 sht1 = Adafruit_SHT31(); // 0x44
Adafruit_SHT31 sht2 = Adafruit_SHT31(); // 0x45
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire); // 128x64
BluetoothSerial SerialBT;

// ========== PINS ==========
#define VIBRA1 25
#define VIBRA2 32
#define VIBRA3 33
#define VIBRA4 26

// ========== PWM (ledc) ==========
const int PWM_FREQ = 5000;
const int PWM_RES  = 8; // 8 bits -> 0..255
const int CH1 = 0;
const int CH2 = 1;
const int CH3 = 2;
const int CH4 = 3;

// ========== VARIABLES ==========
unsigned long previousMillis = 0;
const long DISPLAY_INTERVAL = 200; // refresh display / telemetry interval
// SHT sampling interval (fast)
const unsigned long SHT_INTERVAL_MS = 100; // 100 ms
unsigned long lastShtMillis = 0;

// niveles en porcentaje 0..100
int nivel1 = 0, nivel2 = 0, nivel3 = 0, nivel4 = 0;

// init flags
bool displayOK = false, mpuOK = false, sht1OK = false, sht2OK = false;

// fallback last valid values
float lastT1 = 22.0, lastH1 = 45.0;
float lastT2 = 22.0, lastH2 = 45.0;

// smoothed live readings
float smoothT1 = 22.0, smoothH1 = 45.0;
float smoothT2 = 22.0, smoothH2 = 45.0;
float accelX = 0, accelY = 0, accelZ = 0;

// smoothing alpha (0..1). Lower -> smoother (slower)
const float ALPHA = 0.25f; // adjust between 0.15 and 0.35 for stability

// I2C addrs
const uint8_t SHT1_ADDR = 0x44;
const uint8_t SHT2_ADDR = 0x45;

// Bluetooth name
const char* BTNAME = "ESP32_VIBRA_C";

// ---------- PROTOTYPES ----------
void updateDisplayOptionC(const sensors_event_t &a, float t1, float h1, float t2, float h2);
int pctToDuty(int pct);
float readTempFallback(Adafruit_SHT31 &sht, bool &flag, uint8_t addr, float &lastVal);
float readHumFallback(Adafruit_SHT31 &sht, bool &flag, uint8_t addr, float &lastVal);
void setVibration(int motor, int pct);
void setAllVibration(int pct);
void handleBT(String cmd);
void sendTelemetry(const sensors_event_t &a, float t1, float h1, float t2, float h2);

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Inicio ESP32 - Opcion C (128x64) - Fast SHT100ms");

  // PWM setup and attach pins
  ledcSetup(CH1, PWM_FREQ, PWM_RES);
  ledcSetup(CH2, PWM_FREQ, PWM_RES);
  ledcSetup(CH3, PWM_FREQ, PWM_RES);
  ledcSetup(CH4, PWM_FREQ, PWM_RES);

  ledcAttachPin(VIBRA1, CH1);
  ledcAttachPin(VIBRA2, CH2);
  ledcAttachPin(VIBRA3, CH3);
  ledcAttachPin(VIBRA4, CH4);

  // ensure off at start
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
  ledcWrite(CH3, 0);
  ledcWrite(CH4, 0);

  // I2C
  Wire.begin();

  // Display 128x64
  if (!display.begin(0x3C, true)) {
    Serial.println("OLED no detectada/incorrecta");
    displayOK = false;
  } else {
    displayOK = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("PANEL VIBRA - DIVIDIDO (C)");
    display.println("Ref: /mnt/data/c656a815-9eb9-4b2e-84e4-e9baeff357e4.jpg");
    display.display();
    delay(400);
  }

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 NO detectado");
    mpuOK = false;
  } else {
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  }

  // SHT31 inits (attempt once; keep fallback)
  if (!sht1.begin(SHT1_ADDR)) { Serial.println("SHT1 NO (0x44)"); sht1OK = false; }
  else { sht1OK = true; float t = sht1.readTemperature(); if (!isnan(t)) { lastT1 = t; smoothT1 = t; } float h = sht1.readHumidity(); if (!isnan(h)) { lastH1 = h; smoothH1 = h; } Serial.println("SHT1 OK"); }
  if (!sht2.begin(SHT2_ADDR)) { Serial.println("SHT2 NO (0x45)"); sht2OK = false; }
  else { sht2OK = true; float t = sht2.readTemperature(); if (!isnan(t)) { lastT2 = t; smoothT2 = t; } float h = sht2.readHumidity(); if (!isnan(h)) { lastH2 = h; smoothH2 = h; } Serial.println("SHT2 OK"); }

  // Bluetooth
  if (!SerialBT.begin(BTNAME)) Serial.println("Bluetooth init fail");
  else { Serial.print("Bluetooth iniciado: "); Serial.println(BTNAME); }

  previousMillis = millis();
  lastShtMillis = millis();
}

// ================= LOOP =================
void loop() {
  // handle BT commands
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    Serial.print("BT CMD: "); Serial.println(cmd);
    handleBT(cmd);
  }

  // periodic SHT sampling (fast)
  unsigned long now = millis();
  if (now - lastShtMillis >= SHT_INTERVAL_MS) {
    lastShtMillis = now;

    // read SHT1
    float t1 = NAN, h1 = NAN;
    if (sht1OK) {
      t1 = sht1.readTemperature();
      h1 = sht1.readHumidity();
    } else {
      // try re-init
      sht1OK = sht1.begin(SHT1_ADDR);
      if (sht1OK) { t1 = sht1.readTemperature(); h1 = sht1.readHumidity(); }
    }

    if (!isnan(t1)) {
      smoothT1 = smoothT1 * (1 - ALPHA) + t1 * ALPHA;
      lastT1 = smoothT1;
    }
    if (!isnan(h1)) {
      smoothH1 = smoothH1 * (1 - ALPHA) + h1 * ALPHA;
      lastH1 = smoothH1;
    }

    // read SHT2
    float t2 = NAN, h2 = NAN;
    if (sht2OK) {
      t2 = sht2.readTemperature();
      h2 = sht2.readHumidity();
    } else {
      sht2OK = sht2.begin(SHT2_ADDR);
      if (sht2OK) { t2 = sht2.readTemperature(); h2 = sht2.readHumidity(); }
    }

    if (!isnan(t2)) {
      smoothT2 = smoothT2 * (1 - ALPHA) + t2 * ALPHA;
      lastT2 = smoothT2;
    }
    if (!isnan(h2)) {
      smoothH2 = smoothH2 * (1 - ALPHA) + h2 * ALPHA;
      lastH2 = smoothH2;
    }
  }

  // Periodic display + telemetry update at DISPLAY_INTERVAL
  unsigned long now2 = millis();
  if (now2 - previousMillis >= DISPLAY_INTERVAL) {
    previousMillis = now2;

    // read accel for display/telemetry
    sensors_event_t a, g, tempEvent;
    if (mpuOK) mpu.getEvent(&a, &g, &tempEvent);
    else { a.acceleration.x = a.acceleration.y = a.acceleration.z = 0; }

    // update accel vars
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;
    accelZ = a.acceleration.z;

    // send telemetry
    sendTelemetry(a, lastT1, lastH1, lastT2, lastH2);

    // update display
    if (displayOK) updateDisplayOptionC(a, lastT1, lastH1, lastT2, lastH2);
  }
}

// ========== HELPERS ==========
int pctToDuty(int pct) {
  if (pct <= 0) return 0;
  if (pct >= 100) return 255;
  return map(pct, 0, 100, 0, 255);
}

void setVibration(int motor, int pct) {
  int duty = pctToDuty(pct);
  switch (motor) {
    case 1: ledcWrite(CH1, duty); nivel1 = pct; break;
    case 2: ledcWrite(CH2, duty); nivel2 = pct; break;
    case 3: ledcWrite(CH3, duty); nivel3 = pct; break;
    case 4: ledcWrite(CH4, duty); nivel4 = pct; break;
  }
}

void setAllVibration(int pct) {
  setVibration(1, pct); setVibration(2, pct); setVibration(3, pct); setVibration(4, pct);
}

void handleBT(String cmd) {
  String s = cmd; s.trim();
  String slo = s; slo.toLowerCase();

  // simple app commands
  if (slo == "nivel 1") { setAllVibration(50); SerialBT.println("ACK,NIVEL,1"); return; }
  if (slo == "nivel 2") { setAllVibration(75); SerialBT.println("ACK,NIVEL,2"); return; }
  if (slo == "nivel 3") { setAllVibration(100); SerialBT.println("ACK,NIVEL,3"); return; }
  if (slo == "apagado") { setAllVibration(0); SerialBT.println("ACK,OFF"); return; }

  // advanced parser (ALL,pct or Vx,pct or STATUS)
  s.toUpperCase();
  if (s == "STATUS") { SerialBT.printf("STATUS,%d,%d,%d,%d\n", nivel1, nivel2, nivel3, nivel4); return; }
  int comma = s.indexOf(',');
  if (comma > 0) {
    String left = s.substring(0, comma);
    String right = s.substring(comma + 1);
    int pct = right.toInt();
    if (left == "ALL") { setAllVibration(pct); SerialBT.printf("ACK,ALL,%d\n", pct); return; }
    if (left.startsWith("V")) {
      int motor = left.substring(1).toInt();
      if (motor >= 1 && motor <= 4) { setVibration(motor, pct); SerialBT.printf("ACK,V%d,%d\n", motor, pct); return; }
    }
  }
  SerialBT.println("ERR,UNKNOWN_CMD");
}

void sendTelemetry(const sensors_event_t &a, float t1, float h1, float t2, float h2) {
  // TELE,accX,accY,accZ,t1,h1,t2,h2
  SerialBT.print("TELE,");
  SerialBT.print(a.acceleration.x, 3); SerialBT.print(",");
  SerialBT.print(a.acceleration.y, 3); SerialBT.print(",");
  SerialBT.print(a.acceleration.z, 3); SerialBT.print(",");
  SerialBT.print(t1, 2); SerialBT.print(",");
  SerialBT.print(h1, 1); SerialBT.print(",");
  SerialBT.print(t2, 2); SerialBT.print(",");
  SerialBT.print(h2, 1); SerialBT.print("\n");

  // also USB
  Serial.print("TELE,");
  Serial.print(a.acceleration.x, 3); Serial.print(",");
  Serial.print(a.acceleration.y, 3); Serial.print(",");
  Serial.print(a.acceleration.z, 3); Serial.print(",");
  Serial.print(t1, 2); Serial.print(",");
  Serial.print(h1, 1); Serial.print(",");
  Serial.print(t2, 2); Serial.print(",");
  Serial.print(h2, 1); Serial.print("\n");
}

// SHT31 fallback reads (unused now because we sample fast and smooth; kept for re-init attempts)
float readTempFallback(Adafruit_SHT31 &sht, bool &flag, uint8_t addr, float &lastVal) {
  if (!flag) {
    flag = sht.begin(addr);
    if (!flag) return lastVal;
  }
  float t = sht.readTemperature();
  if (isnan(t)) {
    flag = sht.begin(addr);
    if (!flag) return lastVal;
    t = sht.readTemperature();
  }
  if (!isnan(t)) lastVal = t;
  return lastVal;
}
float readHumFallback(Adafruit_SHT31 &sht, bool &flag, uint8_t addr, float &lastVal) {
  if (!flag) {
    flag = sht.begin(addr);
    if (!flag) return lastVal;
  }
  float h = sht.readHumidity();
  if (isnan(h)) {
    flag = sht.begin(addr);
    if (!flag) return lastVal;
    h = sht.readHumidity();
  }
  if (!isnan(h)) lastVal = h;
  return lastVal;
}

// ========== DISPLAY LAYOUT: OPCIÓN C (Dividida) - SIN CUADROS EN VIBRACION ==========
void updateDisplayOptionC(const sensors_event_t &a, float t1, float h1, float t2, float h2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // --- TOP SECTION (0..31 px): sensors + movement ---
  // header
  display.setCursor(0, 0);
  display.print("PANEL VIBRA");
  display.setCursor(92, 0);
  display.print("BT:");
  if (SerialBT.hasClient()) display.print("ON"); else display.print("OFF");

  // small separator line
  display.drawFastHLine(0, 10, 128, SH110X_WHITE);

  // sensors line (y = 12)
  display.setCursor(0, 12);
  display.print("T1:");
  display.print(t1, 1);
  display.print("C ");
  display.print("H1:");
  display.print((int)round(h1));
  display.print("%");

  // movement (right side)
  float accel = sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
  display.setCursor(92, 12);
  display.print("M:");
  display.print(accel, 2);
  display.print("g");

  // sensors 2 (y = 24)
  display.setCursor(0, 24);
  display.print("T2:");
  display.print(t2, 1);
  display.print("C ");
  display.print("H2:");
  display.print((int)round(h2));
  display.print("%");

  // draw horizontal divider between sections
  display.drawFastHLine(0, 31, 128, SH110X_WHITE);

  // --- BOTTOM SECTION (32..63 px): vibrators area (SIN CAJAS) ---
  // Title for bottom section
  display.setCursor(4, 33);
  display.print("VIBRADORES");

  // Layout: show only percentages, arranged 2x2
  int leftX = 8;
  int rightX = 72; // adjust to fit on right half
  int row0 = 44;
  int row1 = 54;

  // V1 (left, row0) - percentage only
  display.setCursor(leftX, row0);
  display.print("M1: ");
  display.print(nivel1);
  display.print("%");

  // V2 (left, row1)
  display.setCursor(leftX, row1);
  display.print("M2: ");
  display.print(nivel2);
  display.print("%");

  // V3 (right, row0)
  display.setCursor(rightX, row0);
  display.print("M3: ");
  display.print(nivel3);
  display.print("%");

  // V4 (right, row1)
  display.setCursor(rightX, row1);
  display.print("M4: ");
  display.print(nivel4);
  display.print("%");

  // footer (small)
  display.setCursor(4, 62);
  display.setTextSize(1);
  display.print("Modo: APP (nivel 1/2/3, apagado)");

  display.display();
}


