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
const int VIBRATION_MOTOR_1_PIN = 32; // D32
const int VIBRATION_MOTOR_2_PIN = 33; // D33
const int VIBRATION_MOTOR_3_PIN = 25; // D25
const int ROCKET_SWITCH_PIN = 35;     // D35
const int HEATER_RELAY_PIN = 26;      // D26

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

// --- LÍMITES DE SEGURIDAD (MODIFICADOS) ---
const float MAX_TEMP_LIMIT = 40.0; // Límite ajustado a 40C
const float MAX_HUMID_LIMIT = 75.0; // Límite de 75% HR

// --- ESTADO DEL SISTEMA ---
int currentLevel = 1;
bool systemActive = false;

// --- OBJETOS DE SENSORES Y PANTALLA ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MPU6050 mpu;

// --- OBJETOS Y UUIDS PARA BLUETOOTH ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SENSOR_CHAR_UUID    "ae0ca201-381c-43c3-ae4a-115f576e2c34"

BLEServer* pServer = NULL;
BLECharacteristic* pControlCharacteristic = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;

// --- CLASE CALLBACK PARA EVENTOS BLE DE CONEXIÓN ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE: Dispositivo conectado");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE: Dispositivo desconectado");
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

        char command = rxValue[0];

        if (command == '1') { // Comando '1' para Nivel 1
          currentLevel = 1;
          currentVibrationIntensity = INTENSITY_50_PERCENT;
          currentTargetTemperature = TEMP_LEVEL_1;
          Serial.println("BLE: Nivel 1 (50% Vib, 34C Target)");
        } else if (command == '2') { // Comando '2' para Nivel 2
          currentLevel = 2;
          currentVibrationIntensity = INTENSITY_75_PERCENT;
          currentTargetTemperature = TEMP_LEVEL_2;
          Serial.println("BLE: Nivel 2 (75% Vib, 36C Target)");
        } else if (command == '3') { // Comando '3' para Nivel 3
          currentLevel = 3;
          currentVibrationIntensity = INTENSITY_100_PERCENT;
          currentTargetTemperature = TEMP_LEVEL_3;
          Serial.println("BLE: Nivel 3 (100% Vib, 38C Target)");
        }
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
  // Se eliminó pinMode de batería
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
  BLEDevice::init("ESP32_TheraVibe");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pControlCharacteristic = pService->createCharacteristic(
                           CONTROL_CHAR_UUID,
                           BLECharacteristic::PROPERTY_WRITE
                         );
  pControlCharacteristic->setCallbacks(new MyCallbacks());

  pSensorCharacteristic = pService->createCharacteristic(
                           SENSOR_CHAR_UUID,
                           BLECharacteristic::PROPERTY_READ |
                           BLECharacteristic::PROPERTY_NOTIFY
                         );
  pSensorCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
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

  // 1. VERIFICAR EL INTERRUPTOR GENERAL (ROCKET SWITCH)
  if (digitalRead(ROCKET_SWITCH_PIN) == LOW) { // Switch en posición ON
    if (!systemActive) {
      systemActive = true;
      currentLevel = 1; // Inicia en Nivel 1 por defecto
      currentVibrationIntensity = INTENSITY_50_PERCENT;
      currentTargetTemperature = TEMP_LEVEL_1;
      Serial.println("System ON (Rocket Switch)");
    }
  } else { // Switch en posición OFF
    if (systemActive) {
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

    // 2B. Lectura de Batería (ELIMINADA)

    // 2C. *** VERIFICACIÓN DE SEGURIDAD ***
    if (t > MAX_TEMP_LIMIT) {
      performShutdown("ALERTA TEMP!");
      return;
    }
    if (h > MAX_HUMID_LIMIT) {
      performShutdown("ALERTA HUM!");
      return;
    }

    // 2D. Lógica de cambio de nivel por tiempo (ELIMINADA)
    
    // 2E. Controlar actuadores
    ledcWrite(channel0, currentVibrationIntensity);
    ledcWrite(channel1, currentVibrationIntensity);
    ledcWrite(channel2, currentVibrationIntensity);
    
    bool heaterOn = (t < currentTargetTemperature);
    digitalWrite(HEATER_RELAY_PIN, heaterOn ? HIGH : LOW);

    // 2F. Actualizar Pantalla OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    // Fila 1: Temperatura
    display.setCursor(0, 0);
    display.print("T: ");
    display.print(t, 1);
    display.print(" C");

    // Fila 2: Humedad
    display.setCursor(0, 22);
    display.print("H: ");
    display.print(h, 1);
    display.print(" %");

    // Fila 3: Vibración y Nivel
    display.setCursor(0, 44);
    display.print("V: ");
    display.print(accelerationMagnitude, 1);
    
    display.setCursor(80, 44);
    display.print(" L:");
    display.print(currentLevel);
    
    display.display();

    // 2G. Enviar datos de sensores por BLE (Notificación)
    if (deviceConnected) {
      // Formato: T,H,Vib
      String sensorData = String(t, 1) + "," + String(h, 1) + "," + String(accelerationMagnitude, 1);
      pSensorCharacteristic->setValue(sensorData.c_str());
      pSensorCharacteristic->notify();
    }

  }

  // 3. RETARDO
  delay(500);
}