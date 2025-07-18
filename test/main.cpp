// Libraries Packages
#include <Arduino.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h> // MQTT Library
#include <ArduinoJson.h>  // JSON Library

// WiFi credentials
const char* ssid = "Awang";
const char* password = "Awang8888";

// MQTT Broker details
const char* mqtt_server = "broker.hivemq.com"; // Your local broker IP or public broker
const char* mqtt_topic_publish = "Smart-Kitchen/data"; // Topic to publish data
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client (espClient);

// === Define All Pins using ESP8266 GPIO numbers ===
// Note: D0-D8 on NodeMCU map to GPIOs as:
// D0=16, D1=5, D2=4, D3=0, D4=2, D5=14, D6=12, D7=13, D8=15

// Automatic Door
#define SERVO_PIN D6          // GPIO14 (NodeMCU D6)
#define TRIGGER_PIN D7        // GPIO13 (NodeMCU D7)
#define ECHO_PIN D8           // GPIO15 (NodeMCU D8)

// Smoke Detector
// CRITICAL NOTE: ESP8266 has only ONE analog pin (A0).
// If both GAS_SENSOR_PIN and TEMPERATURE_SENSOR_PIN are analog,
// they cannot be read simultaneously without an external multiplexer.
// Assuming for compilation fix, but verify your hardware setup.
#define GAS_SENSOR_PIN A0     // Analog Input (NodeMCU A0)
#define BUZZER_PIN D1         // GPIO5 (NodeMCU D1)

// Temperature Detector
#define TEMPERATURE_SENSOR_PIN A0  // Analog Input (NodeMCU A0) - Conflicts with GAS_SENSOR_PIN if both are analog
#define FAN_1_PIN D2          // GPIO4 (NodeMCU D2)
 
// Automatic Lights and Fan
#define PIR_SENSOR_PIN D0     // GPIO16 (NodeMCU D0, INPUT only, no interrupt on D0)
#define LED_PIN D4            // GPIO2 (NodeMCU D4)
#define FAN_2_PIN D5          // GPIO14 (NodeMCU D5) - Note: This does not conflict with BUZZER_PIN (D1/GPIO5)

// Global variables for sensor readings
Servo servo_5;
float cabinet_distance_cm = 0.0;
int pir_state = 0;
int gas_value = 0;
float temperature_celsius = 0.0;

// Function to read ultrasonic distance
// Returns duration of pulse in microseconds, with a timeout
long readUltrasonicDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  // Use a timeout of 20000 microseconds (20ms) for typical distances up to ~340cm
  // Max distance with 30000 timeout is about 5 meters (30000 * 0.0343 / 2)
  return pulseIn(echoPin, HIGH, 30000);
}

// Function to reconnect to MQTT broker
void reconnectMqtt() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // You can subscribe to topics here if needed (e.g., for commands)
      // client.subscribe("Smart-Kitchen/commands");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Servo
  servo_5.attach(SERVO_PIN);
  servo_5.write(0); // Ensure door is closed initially

  // Initialize Ultrasonic Sensor Pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Smoke Detector / Buzzer Pins
  // GAS_SENSOR_PIN is A0 and doesn't need pinMode for analogRead
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially

  // Initialize Temperature Detector / Fan 1 Pins
  // TEMPERATURE_SENSOR_PIN is A0 and doesn't need pinMode for analogRead
  pinMode(FAN_1_PIN, OUTPUT);
  digitalWrite(FAN_1_PIN, LOW); // Ensure fan is off initially

  // Initialize PIR Sensor / LED / Fan 2 Pins
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_2_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure light is off initially
  digitalWrite(FAN_2_PIN, LOW); // Ensure fan is off initially

  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up MQTT client server
  client.setServer(mqtt_server, mqtt_port);
  // Optional: Set a callback for incoming MQTT messages if you subscribe to topics
  // client.setCallback(callback_function);
}

void loop() {
  // Check MQTT connection and reconnect if lost
  if (!client.connected()) {
    reconnectMqtt();
  }
  client.loop(); // Must be called regularly to process MQTT messages and maintain connection

  // === Automatic Door (Ultrasonic Sensor) ===
  // Calculate distance in cm (speed of sound ~0.0343 cm/µs; distance = time * speed / 2)
  long duration = readUltrasonicDistance(TRIGGER_PIN, ECHO_PIN);
  if (duration > 0) { // Check if a valid pulse was received
    cabinet_distance_cm = 0.01715 * duration; // Corrected factor for cm
  } else {
    cabinet_distance_cm = 999.0; // Indicate out of range or no reading
  }
  Serial.print("Distance: "); Serial.print(cabinet_distance_cm); Serial.println(" cm");

  // Open door if object is close, otherwise close it
  if (cabinet_distance_cm < 15.0 && cabinet_distance_cm > 0.0) { // Check for valid distance range
    servo_5.write(90); // Open door (adjust angle as needed)
    // Removed delay(5000); here to avoid blocking loop for other sensors.
    // If you need the door to stay open for a duration, implement a non-blocking timer.
  } else {
    servo_5.write(0); // Close door (adjust angle as needed)
  }

  // === Automatic Lights and Fan (PIR Sensor) ===
  pir_state = digitalRead(PIR_SENSOR_PIN);
  Serial.print("PIR State: "); Serial.println(pir_state == HIGH ? "MOTION DETECTED" : "NO MOTION");
  digitalWrite(LED_PIN, pir_state); // Turn LED on/off with PIR
  digitalWrite(FAN_2_PIN, pir_state); // Turn Fan 2 on/off with PIR

  // === Temperature Detector (Analog Sensor) ===
  int rawTemp = analogRead(TEMPERATURE_SENSOR_PIN);
  // Assuming a linear temperature sensor (e.g., TMP36) connected to A0
  // ESP8266 ADC reads 0-1023 for 0-1.0V or 0-3.3V depending on VCC/attenuation
  // For TMP36, output is (mV) = (Temp_C * 10) + 500. Vout = (Temp_C + 50) / 100
  // Voltage = rawTemp * (ADC_Reference_Voltage / 1024.0)
  // Assuming ADC_Reference_Voltage = 3.3V (ESP8266 typical)
  float voltage = rawTemp * (3.3 / 1024.0); // Use 1024 for 10-bit ADC
  temperature_celsius = (voltage - 0.5) * 100.0; // For TMP36 sensor (adjust if different sensor)
  Serial.print("Temperature: "); Serial.print(temperature_celsius); Serial.println(" °C");
  digitalWrite(FAN_1_PIN, temperature_celsius >= 30.0 ? HIGH : LOW);

  // === Smoke Detector (Analog Gas Sensor) ===
  gas_value = analogRead(GAS_SENSOR_PIN);
  Serial.print("Gas Value: "); Serial.println(gas_value);
  // Adjust threshold (220) based on your specific gas sensor and calibration
  digitalWrite(BUZZER_PIN, gas_value >= 220 ? HIGH : LOW);

  // === Publish All Sensor Data via MQTT ===
  StaticJsonDocument<200> doc; // Create a JSON document (adjust size as needed)
  doc["distance_cm"] = cabinet_distance_cm;
  doc["pir_motion"] = (pir_state == HIGH); // true/false for boolean
  doc["gas_raw_value"] = gas_value;
  doc["temperature_celsius"] = temperature_celsius;

  char jsonBuffer[200]; // Buffer to hold the serialized JSON string
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  Serial.print("Publishing MQTT message: ");
  Serial.println(jsonBuffer);
  client.publish(mqtt_topic_publish, jsonBuffer);

  delay(2000); // Delay before next loop iteration (adjust as needed, avoids flooding MQTT)
}
