#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h> // Needed for I2C to read IMU
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <IMU_RoboticsUB.h>   // Nom de la llibreria custom


// Device ID
const char *deviceId = "G1_Gri";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Vibration motor settings
const int vibrationPin = 23; // Pin for the vibration motor

// Botons
const int PIN_S1 = 14;
const int PIN_S2 = 27;
int s1Status = HIGH;
int s2Status = HIGH;

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 13); // IP of receiver ESP32
IPAddress receiverComputerIP(192, 168, 1, 15); // IP of PC
const int udpPort = 12345;
WiFiUDP udp;

// IMU object
IMU imu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void updateOrientation() {
  imu.ReadSensor();
  // Obté els angles (roll, pitch, yaw) via GetRPW()
  float* rpw = imu.GetRPW();
  Gri_roll  = rpw[0];
  Gri_pitch = rpw[1];
  Gri_yaw   = rpw[2];
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

void receiveTorquesUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[512];
    int len = udp.read(incomingPacket, 512);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, incomingPacket);

    if (!error) {
      float Torque_roll1 = doc["Torque_roll1"] | 0.0;
      float Torque_pitch = doc["Torque_pitch"] | 0.0;
      float Torque_yaw   = doc["Torque_yaw"]   | 0.0;

      // Calcula el total de torque
      float totalTorque = abs(Torque_roll1) + abs(Torque_pitch) + abs(Torque_yaw);

      // Converteix el torque a valor PWM (0–255)
      int vibrationValue = constrain(totalTorque * 3.0, 0, 255);
      ledcWrite(0, vibrationValue); // Aplica PWM al motor vibrador

      Serial.print("Total torque: ");
      Serial.print(totalTorque);
      Serial.print(" -> Vibration PWM: ");
      Serial.println(vibrationValue);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  // Inicialitza IMU (amb DMP)
  imu.Install();

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  // Configure PWM for the vibration motor
  ledcSetup(0, 5000, 8);         // Channel 0, 5kHz, 8-bit resolution
  ledcAttachPin(vibrationPin, 0); // Connect vibration motor to PWM channel 0
  Serial.println("Vibration motor ready.");

}

void loop() {
  updateOrientation();
  sendOrientationUDP();
  receiveTorquesUDP();
  delay(10);
}
