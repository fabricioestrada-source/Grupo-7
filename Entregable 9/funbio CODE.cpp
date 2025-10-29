#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MPU6050.h>

// --- PINES ---
const int VIBRATION_MOTOR_1_PIN = 12; // D12
const int VIBRATION_MOTOR_2_PIN = 13; // D13
const int VIBRATION_MOTOR_3_PIN = 14; // D14
const int ROCKET_SWITCH_PIN = 27;     // D27
const int HEATER_RELAY_PIN = 26;      // D26
const int BATTERY_PIN = 34;           // PIN ADC PARA LEER BATERÍA (Requiere divisor de voltaje!)

// --- PARÁMETROS BATERÍA (AJUSTAR SEGÚN TU DIVISOR Y BATERÍA) ---
// Asumiendo un divisor 1:1 (ej. R1=100k, R2=100k) y Vref de 3.3V
const float ADC_VREF = 3.3;
const float VOLTAGE_DIVIDER_RATIO = 2.0; // (R1+R2)/R2. (100k+100k)/100k = 2.0
const float V_BAT_MAX = 4.2; // Voltaje batería 100%
const float V_BAT_MIN = 3.0; // Voltaje batería 0%

// --- PARÁMETROS PWM MOTORES ---
const int freq = 5000;
const int resolution = 8;
const int channel0 = 0, channel1 = 1, channel2 = 2;

// --- NIVELES DE VIBRACIÓN (PWM 0-255) ---
const int INTENSITY_50_PERCENT = 127;
const int INTENSITY_75_PERCENT = 191;
const int INTENSITY_100_PERCENT = 255;
int currentVibrationIntensity = INTENSITY_50_PERCENT;

// --- NIVELES DE TEMPERATURA (°C) - ACTUALIZADOS ---
const float TEMP_LEVEL_1 = 34.0;
const float TEMP_LEVEL_2 = 36.0;
const float TEMP_LEVEL_3 = 38.0;
float currentTargetTemperature = TEMP_LEVEL_1;

// --- DURACIÓN DE NIVELES (en milisegundos) ---
const unsigned long LEVEL_1_DURATION = 11UL * 60 * 1000; // 11 minutos
const unsigned long LEVEL_2_DURATION = 11UL * 60 * 1000; // 11 minutos
const unsigned long LEVEL_3_DURATION = 8UL * 60 * 1000;  // 8 minutos

// --- LÍMITES DE SEGURIDAD ---
const float MAX_TEMP_LIMIT = 42.0;
const float MAX_HUMID_LIMIT = 75.0;

// --- ESTADO DEL SISTEMA ---
int currentLevel = 1;
unsigned long lastLevelChange = 0;
bool systemActive = false;

// --- OBJETOS DE SENSORES Y PANTALLA ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Configuración PWM motores
  ledcSetup(channel0, freq, resolution); ledcAttachPin(VIBRATION_MOTOR_1_PIN, channel0);
  ledcSetup(channel1, freq, resolution); ledcAttachPin(VIBRATION_MOTOR_2_PIN, channel1);
  ledcSetup(channel2, freq, resolution); ledcAttachPin(VIBRATION_MOTOR_3_PIN, channel2);

  // Configuración de pines de E/S
  pinMode(ROCKET_SWITCH_PIN, INPUT_PULLUP);
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT); // Pin de batería como entrada
  digitalWrite(HEATER_RELAY_PIN, LOW); // Apagado al inicio

  // Inicialización de Pantalla OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed")); for (;;) ;
  }
  display.display(); delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Inicialización SHT31
  if (!sht31.begin(0x44)) {
    Serial.println(F("Couldn't find SHT31 sensor!")); while (1) delay(1);
  }

  // Inicialización Acelerómetro MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip"); while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// *** FUNCIÓN DE APAGADO (Muestra alertas) ***
void performShutdown(String message) {
  systemActive = false; // Asegurarse de que el sistema esté marcado como inactivo
  
  // Apagar todos los componentes
  ledcWrite(channel0, 0);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, 0);
  digitalWrite(HEATER_RELAY_PIN, LOW);
  
  // Mostrar mensaje en la pantalla OLED
  display.clearDisplay();
  display.setTextSize(2); // Texto más grande para el mensaje final
  display.setCursor(5, 20); // Centrar mensaje
  display.print(message);
  display.display();
  Serial.println("SHUTDOWN: " + message);
}

void loop() {
  unsigned long currentTime = millis(); // Obtener el tiempo actual una vez por ciclo

  // 1. VERIFICAR EL INTERRUPTOR GENERAL (ROCKET SWITCH)
  if (digitalRead(ROCKET_SWITCH_PIN) == LOW) { // Sistema activado
    if (!systemActive) {
      systemActive = true;
      Serial.println("System ON");
      currentLevel = 1;
      currentVibrationIntensity = INTENSITY_50_PERCENT;
      currentTargetTemperature = TEMP_LEVEL_1;
      lastLevelChange = currentTime; // Iniciar el temporizador del nivel 1
      Serial.println("Level 1 started: 50% Vib, 34C Target (11 min)");
    }
  } else { // Sistema desactivado
    if (systemActive) {
      systemActive = false;
      Serial.println("System OFF (Manual)");
      performShutdown("System OFF");
    }
  }

  // 2. LÓGICA PRINCIPAL (SOLO SI EL SISTEMA ESTÁ ACTIVO)
  if (systemActive) {
    
    // 2A. Leer sensores
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float accelerationMagnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));

    // 2B. Leer Batería
    int adc_val = analogRead(BATTERY_PIN);
    float battery_voltage = (adc_val * (ADC_VREF / 4095.0)) * VOLTAGE_DIVIDER_RATIO;
    int battery_percentage = ((battery_voltage - V_BAT_MIN) / (V_BAT_MAX - V_BAT_MIN)) * 100.0;
    if (battery_percentage > 100) battery_percentage = 100;
    if (battery_percentage < 0) battery_percentage = 0;

    // 2C. *** VERIFICACIÓN DE SEGURIDAD ***
    if (t > MAX_TEMP_LIMIT) {
      performShutdown("ALERTA TEMP!");
      return; // Salir del loop() inmediatamente
    }
    if (h > MAX_HUMID_LIMIT) {
      performShutdown("ALERTA HUM!");
      return; // Salir del loop() inmediatamente
    }

    // 2D. Lógica de cambio de nivel por tiempo
    if (currentLevel == 1 && (currentTime - lastLevelChange >= LEVEL_1_DURATION)) {
      currentLevel = 2;
      currentVibrationIntensity = INTENSITY_75_PERCENT;
      currentTargetTemperature = TEMP_LEVEL_2;
      lastLevelChange = currentTime;
      Serial.println("Level 2 started: 75% Vib, 36C Target (11 min)");
    } 
    else if (currentLevel == 2 && (currentTime - lastLevelChange >= LEVEL_2_DURATION)) {
      currentLevel = 3;
      currentVibrationIntensity = INTENSITY_100_PERCENT;
      currentTargetTemperature = TEMP_LEVEL_3;
      lastLevelChange = currentTime;
      Serial.println("Level 3 started: 100% Vib, 38C Target (8 min)");
    }
    else if (currentLevel == 3 && (currentTime - lastLevelChange >= LEVEL_3_DURATION)) {
      performShutdown("CICLO FIN"); // Ciclo completado
      return; // Salir del loop()
    }

    // 2E. Controlar actuadores
    ledcWrite(channel0, currentVibrationIntensity);
    ledcWrite(channel1, currentVibrationIntensity);
    ledcWrite(channel2, currentVibrationIntensity);
    
    bool heaterOn = (t < currentTargetTemperature);
    digitalWrite(HEATER_RELAY_PIN, heaterOn ? HIGH : LOW);

    // 2F. Actualizar Pantalla OLED (Nuevo Layout)
    display.clearDisplay();
    display.setTextSize(1);
    
    // Linea 1: Temp Sensed, Hum Sensed
    display.setCursor(0, 0);
    display.print("T:"); display.print(t, 1); "C ");
    display.setCursor(64, 0); // Mitad de pantalla
    display.print("H:"); display.print(h, 1); "%");

    // Linea 2: Vib Sensed, Bat Level
    display.setCursor(0, 10);
    display.print("Vib:"); display.print(accelerationMagnitude, 1);
    display.setCursor(64, 10);
    display.print("Bat:"); display.print(battery_percentage); "%");

    // Linea 3: Separador
    display.drawFastHLine(0, 19, 128, SSD1306_WHITE);

    // Linea 4: Nivel actual
    display.setCursor(0, 22);
    display.print("Nivel: "); display.print(currentLevel);
    if (currentLevel == 1) display.print(" (50%)");
    else if (currentLevel == 2) display.print(" (75%)");
    else display.print(" (100%)");

    // Linea 5: Temp Target, Heater Status
    display.setCursor(0, 32);
    display.print("Tgt: "); display.print(currentTargetTemperature, 0); "C");
    display.setCursor(64, 32);
    display.print("Htr: "); display.print(heaterOn ? "ON" : "OFF");

    // Linea 6: Tiempo restante
    unsigned long timeInLevel = currentTime - lastLevelChange;
    unsigned long totalDuration = (currentLevel == 1) ? LEVEL_1_DURATION : (currentLevel == 2) ? LEVEL_2_DURATION : LEVEL_3_DURATION;
    unsigned long remainingTimeMs = totalDuration - timeInLevel;
    int remainingMinutes = remainingTimeMs / 60000;
    int remainingSeconds = (remainingTimeMs % 60000) / 1000;

    display.setCursor(0, 42);
    display.print("Tiempo: ");
    if (remainingMinutes < 10) display.print("0");
    display.print(remainingMinutes);
    display.print(":");
    if (remainingSeconds < 10) display.print("0");
    display.print(remainingSeconds);

    display.display();
  } else {
    // Si el sistema está inactivo (apagado manual o por alerta),
    // no hacer nada activo, solo esperar.
  }

  // 3. RETARDO
  delay(500); // Actualizar pantalla y sensores dos veces por segundo
}