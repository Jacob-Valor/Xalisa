#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "Jacob";
const char* password = "93007401";

// MQTT settings
const char* mqtt_server = "YOUR_HIVEMQ_SERVER_IP";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Device_001";

// Topics
const char* sensor_topic = "sensors/temperature";
const char* control_topic = "control/led";

WiFiClient espClient;
PubSubClient client(espClient);

// Sensor simulation
float temperature = 25.0;
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 5000; // 5 seconds

// LED control
const int ledPin = 2;

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("WiFi connected");
    
    // Setup MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(onMqttMessage);
    
    connectToMqtt();
}

void loop() {
    if (!client.connected()) {
        connectToMqtt();
    }
    client.loop();
    
    // Publish sensor data
    if (millis() - lastSensorRead > sensorInterval) {
        publishSensorData();
        lastSensorRead = millis();
    }
}

void connectToMqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(mqtt_client_id)) {
            Serial.println("connected");
            client.subscribe(control_topic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void publishSensorData() {
    // Simulate temperature sensor
    temperature += random(-10, 10) / 10.0;
    
    // Create JSON payload
    StaticJsonDocument<200> doc;
    doc["device_id"] = mqtt_client_id;
    doc["temperature"] = temperature;
    doc["humidity"] = random(40, 80);
    doc["timestamp"] = millis();
    
    char buffer[256];
    serializeJson(doc, buffer);
    
    client.publish(sensor_topic, buffer);
    Serial.println("Sensor data published: " + String(buffer));
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.println("Message received [" + String(topic) + "]: " + message);
    
    if (String(topic) == control_topic) {
        if (message == "ON") {
            digitalWrite(ledPin, HIGH);
            Serial.println("LED turned ON");
        } else if (message == "OFF") {
            digitalWrite(ledPin, LOW);
            Serial.println("LED turned OFF");
        }
    }
}