#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MPU6050.h>

// --- LIBRERIAS BLUETOOTH LOW ENERGY (BLE) ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- PINES ---
const int VIBRATION_MOTOR_1_PIN = 12; // D12
const int VIBRATION_MOTOR_2_PIN = 13; // D13
const int VIBRATION_MOTOR_3_PIN = 14; // D14
const int ROCKET_SWITCH_PIN = 27;     // D27
const int HEATER_RELAY_PIN = 26;      // D26
const int BATTERY_PIN = 34;           // PIN ADC PARA LEER BATERÍA (Requiere divisor de voltaje!)

// --- PARÁMETROS BATERÍA (AJUSTAR SEGÚN TU DIVISOR Y BATERÍA) ---
const float ADC_VREF = 3.3;
const float VOLTAGE_DIVIDER_RATIO = 2.0; // (R1+R2)/R2. (100k+100k)/100k = 2.0 (ejemplo)
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

// --- NIVELES DE TEMPERATURA (°C) ---
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
bool bleControlled = false; // Nuevo: Para saber si el control es por BLE o por temporizador

// --- OBJETOS DE SENSORES Y PANTALLA ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MPU6050 mpu;

// --- OBJETOS Y UUIDS PARA BLUETOOTH ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UUID para nuestro servicio principal
#define CONTROL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8" // UUID para característica de control (desde App a ESP32)
#define SENSOR_CHAR_UUID    "ae0ca201-381c-43c3-ae4a-115f576e2c34" // UUID para característica de sensores (desde ESP32 a App)

BLEServer* pServer = NULL;
BLECharacteristic* pControlCharacteristic = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- CLASE CALLBACK PARA EVENTOS BLE DE CONEXIÓN ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE: Dispositivo conectado");
      bleControlled = true; // El sistema ahora está bajo control BLE
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE: Dispositivo desconectado");
      bleControlled = false; // El sistema ya no está bajo control BLE
      BLEDevice::startAdvertising(); // Volver a anunciar para reconexión
      Serial.println("BLE: Reiniciando el anuncio");
    }
};

// --- CLASE CALLBACK PARA RECIBIR DATOS DESDE LA APP (CONTROL) ---
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.print("BLE: Comando recibido: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();

        // --- Lógica para procesar comandos desde la App ---
        char command = rxValue[0]; // Tomamos el primer caracter como comando

        if (command == 'O') { // Comando 'O' para encender (ON)
          if (!systemActive) {
            systemActive = true;
            currentLevel = 1;
            currentVibrationIntensity = INTENSITY_50_PERCENT;
            currentTargetTemperature = TEMP_LEVEL_1;
            lastLevelChange = millis(); // Reiniciar tiempo si se inicia por BLE
            Serial.println("System ON (BLE)");
          }
        } else if (command == 'F') { // Comando 'F' para apagar (OFF)
          if (systemActive) {
            performShutdown("System OFF (BLE)");
          }
        } else if (command == '1') { // Comando '1' para Nivel 1
          currentLevel = 1;
          currentVibrationIntensity = INTENSITY_50_PERCENT;
          currentTargetTemperature = TEMP_LEVEL_1;
          lastLevelChange = millis(); // Reiniciar tiempo si se cambia por BLE
          Serial.println("BLE: Nivel 1 (50% Vib, 34C Target)");
        } else if (command == '2') { // Comando '2' para Nivel 2
          currentLevel = 2;
          currentVibrationIntensity = INTENSITY_75_PERCENT;
          currentTargetTemperature = TEMP_LEVEL_2;
          lastLevelChange = millis();
          Serial.println("BLE: Nivel 2 (75% Vib, 36C Target)");
        } else if (command == '3') { // Comando '3' para Nivel 3
          currentLevel = 3;
          currentVibrationIntensity = INTENSITY_100_PERCENT;
          currentTargetTemperature = TEMP_LEVEL_3;
          lastLevelChange = millis();
          Serial.println("BLE: Nivel 3 (100% Vib, 38C Target)");
        }
        // Puedes añadir más comandos aquí, por ejemplo, para ajustar individualmente vibración o temp
      }
    }
};

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
    Serial.println(F("SSD1306 allocation failed")); while (1) delay(1);
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

  // --- CONFIGURACIÓN BLUETOOTH ---
  Serial.println("Configurando BLE...");
  BLEDevice::init("ESP32_TheraVibe"); // Nombre del dispositivo BLE
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Crear el servicio BLE
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Crear la característica de control (Write sin respuesta)
  pControlCharacteristic = pService->createCharacteristic(
                      CONTROL_CHAR_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pControlCharacteristic->setCallbacks(new MyCallbacks());

  // Crear la característica de sensores (Read y Notify)
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHAR_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pSensorCharacteristic->addDescriptor(new BLE2902()); // Descriptor para notificaciones

  pService->start(); // Iniciar el servicio
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Ayuda a la conexión rápida
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE: Esperando una conexión de cliente...");
}

// *** FUNCIÓN DE APAGADO (Muestra alertas) ***
void performShutdown(String message) {
  systemActive = false;
  
  ledcWrite(channel0, 0);
  ledcWrite(channel1, 0);
  ledcWrite(channel2, 0);
  digitalWrite(HEATER_RELAY_PIN, LOW);
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(5, 20);
  display.print(message);
  display.display();
  Serial.println("SHUTDOWN: " + message);
}

void loop() {
  unsigned long currentTime = millis();

  // 1. Manejo de conexión/desconexión BLE
  if (deviceConnected) {
    // Si se acaba de conectar, asegurar que bleControlled sea true
    if (!oldDeviceConnected) {
        bleControlled = true;
        oldDeviceConnected = true;
        Serial.println("BLE: Conexión establecida. Control por BLE activado.");
    }
  } else if (oldDeviceConnected) {
    // Si se acaba de desconectar
    bleControlled = false;
    oldDeviceConnected = false;
    Serial.println("BLE: Desconexión detectada. Control por BLE desactivado.");
    // Después de desconectarse, si el sistema estaba activo, ponerlo en el modo por tiempo automático
    // Opcional: podrías apagarlo o dejarlo como estaba antes de la conexión BLE
    // Por simplicidad, volvemos al modo de temporizador si estaba activo antes.
    if(systemActive) {
       // Podrías reiniciar el lastLevelChange aquí para que empiece un nivel nuevo al desconectarse BLE
       // o mantener el progreso. Por ahora, si no hay control BLE, usará el temporizador.
       Serial.println("BLE: Volviendo a control por temporizador.");
    }
  }

  // 2. VERIFICAR EL INTERRUPTOR GENERAL (ROCKET SWITCH) - Siempre tiene prioridad si el sistema no está activo por BLE
  if (!bleControlled && digitalRead(ROCKET_SWITCH_PIN) == LOW) { // Solo si no está controlado por BLE
    if (!systemActive) {
      systemActive = true;
      currentLevel = 1;
      currentVibrationIntensity = INTENSITY_50_PERCENT;
      currentTargetTemperature = TEMP_LEVEL_1;
      lastLevelChange = currentTime;
      Serial.println("System ON (Rocket Switch)");
    }
  } else if (!bleControlled && digitalRead(ROCKET_SWITCH_PIN) == HIGH) { // Apagado manual
    if (systemActive) {
      performShutdown("System OFF (Manual)");
    }
  }
  
  // 3. LÓGICA PRINCIPAL (SOLO SI EL SISTEMA ESTÁ ACTIVO)
  if (systemActive) {
    
    // 3A. Leer sensores
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float accelerationMagnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));

    // 3B. Leer Batería
    int adc_val = analogRead(BATTERY_PIN);
    float battery_voltage = (adc_val * (ADC_VREF / 4095.0)) * VOLTAGE_DIVIDER_RATIO;
    int battery_percentage = ((battery_voltage - V_BAT_MIN) / (V_BAT_MAX - V_BAT_MIN)) * 100.0;
    if (battery_percentage > 100) battery_percentage = 100;
    if (battery_percentage < 0) battery_percentage = 0;

    // 3C. *** VERIFICACIÓN DE SEGURIDAD ***
    if (t > MAX_TEMP_LIMIT) {
      performShutdown("ALERTA TEMP!");
      return;
    }
    if (h > MAX_HUMID_LIMIT) {
      performShutdown("ALERTA HUM!");
      return;
    }

    // 3D. Lógica de cambio de nivel por tiempo (SOLO SI NO ESTÁ CONTROLADO POR BLE)
    if (!bleControlled) {
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
        performShutdown("CICLO FIN");
        return;
      }
    }

    // 3E. Controlar actuadores
    ledcWrite(channel0, currentVibrationIntensity);
    ledcWrite(channel1, currentVibrationIntensity);
    ledcWrite(channel2, currentVibrationIntensity);
    
    bool heaterOn = (t < currentTargetTemperature);
    digitalWrite(HEATER_RELAY_PIN, heaterOn ? HIGH : LOW);

    // 3F. Actualizar Pantalla OLED
    display.clearDisplay();
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.print("T:"); display.print(t, 1); "C ");
    display.setCursor(64, 0);
    display.print("H:"); display.print(h, 1); "%");

    display.setCursor(0, 10);
    display.print("Vib:"); display.print(accelerationMagnitude, 1);
    display.setCursor(64, 10);
    display.print("Bat:"); display.print(battery_percentage); "%");

    display.drawFastHLine(0, 19, 128, SSD1306_WHITE);

    display.setCursor(0, 22);
    display.print("Nivel: "); display.print(currentLevel);
    if (currentLevel == 1) display.print(" (50%)");
    else if (currentLevel == 2) display.print(" (75%)");
    else display.print(" (100%)");

    display.setCursor(0, 32);
    display.print("Tgt: "); display.print(currentTargetTemperature, 0); "C");
    display.setCursor(64, 32);
    display.print("Htr: "); display.print(heaterOn ? "ON" : "OFF");

    // Mostrar el tiempo restante o "BLE Control"
    display.setCursor(0, 42);
    if (bleControlled) {
      display.print("BLE Control ON");
    } else {
      unsigned long timeInLevel = currentTime - lastLevelChange;
      unsigned long totalDuration = (currentLevel == 1) ? LEVEL_1_DURATION : (currentLevel == 2) ? LEVEL_2_DURATION : LEVEL_3_DURATION;
      unsigned long remainingTimeMs = totalDuration - timeInLevel;
      int remainingMinutes = remainingTimeMs / 60000;
      int remainingSeconds = (remainingTimeMs % 60000) / 1000;

      display.print("Tiempo: ");
      if (remainingMinutes < 10) display.print("0");
      display.print(remainingMinutes);
      display.print(":");
      if (remainingSeconds < 10) display.print("0");
      display.print(remainingSeconds);
    }
    
    display.display();

    // 3G. Enviar datos de sensores por BLE (Notificación)
    if (deviceConnected) {
      // Formatear los datos como una cadena para enviar
      String sensorData = String(t, 1) + "," + String(h, 1) + "," + String(accelerationMagnitude, 1) + "," + String(battery_percentage);
      pSensorCharacteristic->setValue(sensorData.c_str());
      pSensorCharacteristic->notify(); // Enviar la notificación a la App
    }

  } else {
    // Si el sistema está inactivo, pero BLE está conectado, aún mostrar "BLE Control" o un mensaje de inactividad
    if(bleControlled && !display.getRotation()){ // Evitar borrar pantalla si está en un mensaje de alerta
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0); display.print("System OFF");
        display.setCursor(0, 10); display.print("BLE Control ON");
        display.display();
    }
  }

  // 4. RETARDO
  delay(500);
}