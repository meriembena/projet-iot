#include <Wire.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ArduinoJson.h>
// ========== CONFIGURATION WiFi et MQTT ==========
const char* ssid = "Redmi";
const char* password = "Mmmm123@";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

// Topics MQTT
const char* topic_health = "health/monitor/data";
const char* topic_buzzer = "health/monitor/buzzer";


// Client WiFi et MQTT
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// ========== PINS ==========
#define ECG_OUTPUT A0
#define LO_PLUS 10
#define LO_MINUS 11
#define BUZZER_PIN 9
#define GSR_PIN A2 

// BUZZER - Fréquence fixe
#define BUZZER_FREQ 2000
bool buzzerState = false;

// ========== CAPTEURS ==========
MAX30105 particleSensor;

// MPU6050
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float pitch = 0, roll = 0, temperature = 0;
bool fallDetected = false;
bool mpu6050_ok = false;
String currentPosition = "Unknown";

// MAX30101
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
bool max30101_ok = false;

// GSR (Galvanic Skin Response)
int gsrValue = 0;
int gsrAverage = 0;
String stressLevel = "Unknown";

// Timing
unsigned long lastPublish = 0;
const long publishInterval = 2000;

// ========== CALLBACK MQTT - CONTRÔLE BUZZER ==========
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  
  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);
  
  //  BUZZER CONTRÔLÉ PAR NODE-RED
  if (String(topic) == topic_buzzer) {
    if (msg == "ON") {
      tone(BUZZER_PIN, BUZZER_FREQ);
      buzzerState = true;
      Serial.print("Buzzer activé (");
      Serial.print(BUZZER_FREQ);
      Serial.println(" Hz)");
    } else if (msg == "OFF") {
      noTone(BUZZER_PIN);
      buzzerState = false;
      Serial.println("Buzzer désactivé");
    }
  }
}

// ========== WiFi ==========
void setup_wifi() {
  Serial.println("\n=================================");
  Serial.print("WiFi: ");
  Serial.println(ssid);
  
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Module WiFi absent!");
    while (true);
  }

  WiFi.begin(ssid, password);
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n WiFi ÉCHEC!");
    while (true);
  }

  Serial.println("\n WiFi OK!");
  Serial.print(" IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("=================================");
}

// ========== MQTT ==========
void reconnect() {
  while (!client.connected()) {
    Serial.print(" MQTT...");
    String clientId = "UnoR4-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println(" OK!");
      client.subscribe(topic_buzzer);
      Serial.println(" Abonné: health/monitor/buzzer");
      client.publish(topic_health, "{\"status\":\"online\"}");
    } else {
      Serial.print(" Code:");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  // Pins
  pinMode(ECG_OUTPUT, INPUT);
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(GSR_PIN, INPUT);
  noTone(BUZZER_PIN);
  
  // Test buzzer
  Serial.println(" Test buzzer (2000 Hz)...");
  tone(BUZZER_PIN, BUZZER_FREQ, 200);
  delay(300);
  Serial.println(" Buzzer OK\n");
  
  // WiFi
  setup_wifi();
  
  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(600);
  
  // I2C
  Serial.println(" I2C...");
  Wire.begin();
  delay(100);
  
  // SCAN I2C
  Serial.println(" Scan I2C:");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x57) Serial.println(" - MAX30101");
      else if (addr == 0x68) Serial.println(" - MPU6050");
      else Serial.println();
      count++;
    }
  }
  Serial.print("Total: ");
  Serial.print(count);
  Serial.println(" périphériques\n");
  
  // MPU6050
  Serial.println(" MPU6050...");
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  byte mpu_error = Wire.endTransmission(true);
  
  if (mpu_error == 0) {
    Serial.println(" MPU6050 OK!");
    mpu6050_ok = true;
  } else {
    Serial.print(" MPU6050 erreur:");
    Serial.println(mpu_error);
    mpu6050_ok = false;
  }
  
  // MAX30101
  Serial.println("\n MAX30101...");
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(" MAX30101 absent!");
    max30101_ok = false;
  } else {
    Serial.println("MAX30101 OK!");
    particleSensor.setup(0xFF, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0xFF);
    particleSensor.setPulseAmplitudeIR(0xFF);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("Config: LEDs MAX, Mode Red+IR");
    max30101_ok = true;
  }
  
  //  GSR Sensor
  Serial.println("\n GSR (Stress Sensor)...");
  int testGSR = analogRead(GSR_PIN);
  if (testGSR > 0) {
    Serial.print(" GSR OK! Valeur test: ");
    Serial.println(testGSR);
  } else {
    Serial.println("GSR: Vérifier connexions");
  }
  
  delay(1000);
}

// ========== LOOP ==========
void loop() {
  // WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" WiFi perdu!");
    setup_wifi();
  }
  
  // MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  unsigned long currentMillis = millis();
  
  // ===== ECG =====
  int ecgValue = 0;
  bool electrodesConnected = true;
  
  if ((digitalRead(LO_PLUS) == 1) || (digitalRead(LO_MINUS) == 1)) {
    electrodesConnected = false;
  } else {
    ecgValue = analogRead(ECG_OUTPUT);
  }
  
  // ===== GSR (Galvanic Skin Response) - STRESS =====
  long gsrSum = 0;
  for(int i = 0; i < 10; i++) {
    gsrValue = analogRead(GSR_PIN);
    gsrSum += gsrValue;
    delay(5);
  }
  gsrAverage = gsrSum / 10;
  
  // Déterminer niveau de stress
  // Valeurs typiques: 200-600 (calme), 600-900 (normal), 900+ (stressé)
  if (gsrAverage < 300) {
    stressLevel = "TRES_CALME";
  } else if (gsrAverage < 600) {
    stressLevel = "CALME";
  } else if (gsrAverage < 900) {
    stressLevel = "NORMAL";
  } else if (gsrAverage < 1200) {
    stressLevel = "STRESS_LEGER";
  } else if (gsrAverage < 1500) {
    stressLevel = "STRESS_MOYEN";
  } else {
    stressLevel = "STRESS_ELEVE";
  }
  
  // ===== MPU6050 - POSITION =====
  if (mpu6050_ok) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    byte error = Wire.endTransmission(false);
    
    if (error == 0) {
      Wire.requestFrom(MPU_addr, 14, true);
      
      if (Wire.available() >= 14) {
        AcX = Wire.read() << 8 | Wire.read();
        AcY = Wire.read() << 8 | Wire.read();
        AcZ = Wire.read() << 8 | Wire.read();
        Tmp = Wire.read() << 8 | Wire.read();
        GyX = Wire.read() << 8 | Wire.read();
        GyY = Wire.read() << 8 | Wire.read();
        GyZ = Wire.read() << 8 | Wire.read();
        
        pitch = atan2(AcY, sqrt((long)AcX * AcX + (long)AcZ * AcZ)) * 180.0 / PI;
        roll = atan2(-AcX, AcZ) * 180.0 / PI;
        temperature = Tmp / 340.0 + 36.53;
        
        fallDetected = false;
        
        if (abs(pitch) > 70 || abs(roll) > 70) {
          currentPosition = "CHUTE";
          fallDetected = true;
        }
        else if (roll > 25 && abs(pitch) < 45) {
          currentPosition = "DROITE";
        }
        else if (roll < -25 && abs(pitch) < 45) {
          currentPosition = "GAUCHE";
        }
        else if (pitch > 25 && abs(roll) < 45) {
          currentPosition = "AVANT";
        }
        else if (pitch < -25 && abs(roll) < 45) {
          currentPosition = "ARRIERE";
        }
        else if (abs(pitch) < 20 && abs(roll) < 20) {
          currentPosition = "NORMAL";
        }
        else {
          currentPosition = "INTERMEDIAIRE";
        }
      }
    }
  }
  
  // ===== MAX30101 - OXYMETRE =====
  long irValue = 0;
  long redValue = 0;
  bool fingerDetected = false;
  
  if (max30101_ok) {
    irValue = particleSensor.getIR();
    redValue = particleSensor.getRed();
    fingerDetected = (irValue > 10000);
    
    if (fingerDetected && checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      
      beatsPerMinute = 60 / (delta / 1000.0);
      
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }
  
  // ===== PUBLICATION MQTT =====
  if (currentMillis - lastPublish >= publishInterval) {
    lastPublish = currentMillis;
    
    StaticJsonDocument<600> doc;
    
    doc["timestamp"] = millis();
    doc["deviceId"] = "uno-r4-health-001";
    
    // ECG
    JsonObject ecg = doc.createNestedObject("ecg");
    ecg["value"] = ecgValue;
    ecg["connected"] = electrodesConnected;
    
    // Oxymètre
    JsonObject oximeter = doc.createNestedObject("oximeter");
    oximeter["heartRate"] = beatAvg;
    oximeter["instantBPM"] = beatsPerMinute;
    oximeter["fingerDetected"] = fingerDetected;
    oximeter["irValue"] = irValue;
    oximeter["redValue"] = redValue;
    
    // Gyroscope
    JsonObject gyro = doc.createNestedObject("gyroscope");
    gyro["accelX"] = AcX;
    gyro["accelY"] = AcY;
    gyro["accelZ"] = AcZ;
    gyro["gyroX"] = GyX;
    gyro["gyroY"] = GyY;
    gyro["gyroZ"] = GyZ;
    gyro["pitch"] = pitch;
    gyro["roll"] = roll;
    gyro["temperature"] = temperature;
    gyro["position"] = currentPosition;
    gyro["fallDetected"] = fallDetected;
    
    // GSR (Stress)
    JsonObject gsr = doc.createNestedObject("gsr");
    gsr["value"] = gsrAverage;
    gsr["raw"] = gsrValue;
    gsr["stressLevel"] = stressLevel;
    
    // Alertes
    JsonObject alerts = doc.createNestedObject("alerts");
    alerts["fall"] = fallDetected;
    alerts["hrHigh"] = (beatAvg > 100);
    alerts["hrLow"] = (beatAvg < 50 && beatAvg > 0);
    alerts["ecgDisconnected"] = !electrodesConnected;
    alerts["highStress"] = (gsrAverage > 1200);
    
    // Publier
    char jsonBuffer[600];
    serializeJson(doc, jsonBuffer);
    
    bool published = client.publish(topic_health, jsonBuffer);
    
    if (published) {
      Serial.print("║  BPM: ");
      Serial.print(beatAvg);
      Serial.println(fingerDetected ? " ✓            ║" : " ✗            ║");
      Serial.print("║ ECG: ");
      Serial.print(ecgValue);
      Serial.println(electrodesConnected ? " ✓           ║" : " ✗           ║");
      Serial.print("║  Temp: ");
      Serial.print(temperature, 1);
      Serial.println(" °C        ║");
      Serial.print("║ Position: ");
      Serial.print(currentPosition);
      for (int i = currentPosition.length(); i < 13; i++) Serial.print(" ");
      Serial.println("║");
      Serial.print("║ GSR: ");
      Serial.print(gsrAverage);
      Serial.print(" (");
      Serial.print(stressLevel);
      Serial.println(")   ║");
      Serial.print("║ IR:");
      Serial.print(irValue);
      Serial.print(" R:");
      Serial.print(redValue);
      Serial.println("  ║");
      Serial.print("║ Buzzer: ");
      Serial.println(buzzerState ? "ON              ║" : "OFF             ║");
      Serial.println("╚════════════════════════════════╝");
      
      if (fallDetected) Serial.println("Chute détectée");
      if (beatAvg > 100) Serial.println("FC haute");
      if (beatAvg < 50 && beatAvg > 0) Serial.println("FC basse");
      if (gsrAverage > 1200) Serial.println("Stress élevé détecté");
      Serial.println();
      
    } else {
      Serial.println("MQTT ÉCHEC!\n");
    }
  }
  
  delay(20);
}
