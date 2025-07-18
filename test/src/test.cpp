/*
 * ═══════════════════════════════════════════════════════════════════════════════════════
 *                           SMART HOME AUTOMATION SYSTEM
 * ═══════════════════════════════════════════════════════════════════════════════════════
 * Description: Complete Arduino-based smart home system with multiple sensors and controls
 * Author: TEAM GROUP 5 of Micro-LABS
 * Date: 2025
 * Board: Arduino Uno + NodeMCU (ESP8266)
 * ═══════════════════════════════════════════════════════════════════════════════════════
 */

#include <Servo.h>
#include <DHT.h>

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════════════

// System Settings
const unsigned long SYSTEM_CYCLE_DELAY = 1000;  // Main loop delay (ms)
const unsigned long SENSOR_READ_INTERVAL = 2000; // DHT sensor read interval
const unsigned long DISPLAY_UPDATE_INTERVAL = 5000; // Status display update interval

// Door Control Settings
const int DOOR_OPEN_ANGLE = 45;
const int DOOR_CLOSE_ANGLE = 150;
const int DISTANCE_THRESHOLD_CM = 20;
const int MIN_VALID_DISTANCE = 2;    // Minimum valid distance (cm)
const int MAX_VALID_DISTANCE = 400;  // Maximum valid distance (cm)

// Smoke Detection Settings
const float RL = 10.0;               // Load resistance in kΩ
const int DANGER_THRESHOLD = 700;    // High gas concentration threshold
const int WARNING_THRESHOLD = 400;   // Warning gas concentration threshold
const int CALIBRATION_SAMPLES = 100;
const unsigned long WARMUP_TIME = 120000; // 2 minutes warmup

// Temperature Control Settings
const float TEMPERATURE_THRESHOLD = 30.0; // Celsius
const float HUMIDITY_THRESHOLD = 70.0;    // Percentage

// Motion Detection Settings
const unsigned long LIGHT_ON_DURATION = 10000; // 10 seconds

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════════════

// Automatic Door System
const int SERVO_PIN = 6;
const int TRIGGER_PIN = 7;
const int ECHO_PIN = 8;

// Smoke Detection System
const int MQ_PIN = A0;
const int BUZZER_PIN = 4;

// Automatic Lighting System
const int PIR_SENSOR_PIN = 9;
const int LED_PIN = 3;

// Temperature Control System
const int DHT_PIN = A1;
const int MOTOR_PIN = 11;

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════════════════

// Object Instances
Servo doorServo;
DHT dhtSensor(DHT_PIN, DHT11);

// System State Variables
bool systemInitialized = false;
bool doorOpen = false;
bool motorRunning = false;
bool lightsOn = false;
bool smokeDetected = false;
bool motionDetected = false;

// Sensor Calibration
float R0 = 10.0; // Default R0 value, will be calibrated

// Timing Variables
unsigned long lastDHTRead = 0;
unsigned long lastStatusDisplay = 0;
unsigned long lightOnTime = 0;

// Sensor Data Storage
struct SensorData {
    float temperature = 0.0;
    float humidity = 0.0;
    float heatIndex = 0.0;
    int distance = 0;
    int gasLevel = 0;
    bool motion = false;
};

SensorData currentReadings;

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════════════

void printSeparator(char symbol = '═', int length = 80) {
    for (int i = 0; i < length; i++) {
        Serial.print(symbol);
    }
    Serial.println();
}

void printTitle(const char* title) {
    Serial.println();
    printSeparator();
    Serial.print("║");
    int padding = (78 - strlen(title)) / 2;
    for (int i = 0; i < padding; i++) Serial.print(" ");
    Serial.print(title);
    for (int i = 0; i < padding; i++) Serial.print(" ");
    if (strlen(title) % 2 == 1) Serial.print(" ");
    Serial.println("║");
    printSeparator();
}

void printStatus(const char* system, const char* status, bool isActive = false) {
    Serial.print("║ ");
    Serial.print(system);
    Serial.print(": ");
    
    // Add padding to align status
    int padding = 20 - strlen(system);
    for (int i = 0; i < padding; i++) Serial.print(" ");
    
    if (isActive) {
        Serial.print("🟢 ");
    } else {
        Serial.print("🔴 ");
    }
    
    Serial.print(status);
    Serial.println(" ║");
}

void printSensorReading(const char* label, float value, const char* unit) {
    Serial.print("║ ");
    Serial.print(label);
    Serial.print(": ");
    
    int padding = 15 - strlen(label);
    for (int i = 0; i < padding; i++) Serial.print(" ");
    
    Serial.print(value, 1);
    Serial.print(" ");
    Serial.print(unit);
    Serial.println(" ║");
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   SYSTEM INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════════════════

void initializeSystem() {
    printTitle("SMART HOME SYSTEM INITIALIZING");
    
    // Initialize Serial Communication
    Serial.println("║ Initializing Serial Communication...                                    ║");
    delay(500);
    
    // Initialize Pins
    Serial.println("║ Configuring GPIO Pins...                                                ║");
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(PIR_SENSOR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    delay(500);
    
    // Initialize Servo
    Serial.println("║ Initializing Servo Motor...                                             ║");
    doorServo.attach(SERVO_PIN);
    doorServo.write(DOOR_CLOSE_ANGLE);
    delay(1000);
    
    // Initialize DHT Sensor
    Serial.println("║ Initializing Temperature & Humidity Sensor...                           ║");
    dhtSensor.begin();
    delay(2000);
    
    // Initialize other components
    Serial.println("║ Initializing Motion Sensor...                                           ║");
    delay(500);
    
    Serial.println("║ Initializing Gas Sensor...                                              ║");
    delay(500);
    
    // System Ready
    Serial.println("║ All Systems Initialized Successfully!                                   ║");
    printSeparator();
    Serial.println("║                           SYSTEM READY                                  ║");
    printSeparator();
    
    systemInitialized = true;
    delay(2000);
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   AUTOMATIC DOOR SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════════════

void automaticDoorControl() {
    // Clear trigger pin
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    
    // Send pulse
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    // Read echo
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
    
    if (duration == 0) {
        currentReadings.distance = -1; // Timeout error
        return;
    }
    
    // Calculate distance
    int distance = duration * 0.0343 / 2;
    
    // Validate reading
    if (distance < MIN_VALID_DISTANCE || distance > MAX_VALID_DISTANCE) {
        currentReadings.distance = -1; // Invalid reading
        return;
    }
    
    currentReadings.distance = distance;
    
    // Door control logic
    if (distance < DISTANCE_THRESHOLD_CM) {
        if (!doorOpen) {
            doorServo.write(DOOR_OPEN_ANGLE);
            doorOpen = true;
            Serial.println("🚪 Door: OPENING");
            delay(500);
        }
    } else {
        if (doorOpen) {
            doorServo.write(DOOR_CLOSE_ANGLE);
            doorOpen = false;
            Serial.println("🚪 Door: CLOSING");
            delay(500);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   SMOKE DETECTION SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════════════

void smokeDetectionSystem() {
    int sensorValue = analogRead(MQ_PIN);
    currentReadings.gasLevel = sensorValue;
    
    // Calculate gas concentration ratio
    float voltage = sensorValue * (5.0 / 1023.0);
    float RS = ((5.0 * RL) / voltage) - RL;
    float ratio = RS / R0;
    
    // Determine alert level
    if (sensorValue >= DANGER_THRESHOLD) {
        smokeDetected = true;
        dangerAlarm();
        Serial.println("🚨 GAS ALERT: DANGER LEVEL - EVACUATE IMMEDIATELY!");
    } else if (sensorValue >= WARNING_THRESHOLD) {
        smokeDetected = true;
        warningAlarm();
        Serial.println("⚠️  GAS ALERT: Warning Level - Ventilation Recommended");
    } else {
        smokeDetected = false;
        noAlarm();
    }
}

void warningAlarm() {
    tone(BUZZER_PIN, 1000, 200);
    delay(300);
    noTone(BUZZER_PIN);
}

void dangerAlarm() {
    tone(BUZZER_PIN, 2000, 100);
    delay(100);
    tone(BUZZER_PIN, 2000, 100);
    delay(100);
}

void noAlarm() {
    noTone(BUZZER_PIN);
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   TEMPERATURE CONTROL SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════════════

void temperatureControlSystem() {
    unsigned long currentTime = millis();
    
    // Read sensor only at specified intervals
    if (currentTime - lastDHTRead >= SENSOR_READ_INTERVAL) {
        lastDHTRead = currentTime;
        
        float humidity = dhtSensor.readHumidity();
        float temperature = dhtSensor.readTemperature();
        float temperatureF = dhtSensor.readTemperature(true);
        
        // Check for valid readings
        if (isnan(humidity) || isnan(temperature) || isnan(temperatureF)) {
            Serial.println("❌ DHT Sensor: Reading Error!");
            return;
        }
        
        // Store readings
        currentReadings.temperature = temperature;
        currentReadings.humidity = humidity;
        currentReadings.heatIndex = dhtSensor.computeHeatIndex(temperature, humidity, false);
        
        // Motor control logic
        if (temperature > TEMPERATURE_THRESHOLD) {
            if (!motorRunning) {
                digitalWrite(MOTOR_PIN, HIGH);
                motorRunning = true;
                Serial.println("🌡️  Temperature Control: Fan ON");
            }
        } else {
            if (motorRunning) {
                digitalWrite(MOTOR_PIN, LOW);
                motorRunning = false;
                Serial.println("🌡️  Temperature Control: Fan OFF");
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   AUTOMATIC LIGHTING SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════════════

void automaticLightingSystem() {
    bool currentMotion = digitalRead(PIR_SENSOR_PIN);
    unsigned long currentTime = millis();
    
    if (currentMotion && !motionDetected) {
        // Motion detected
        motionDetected = true;
        currentReadings.motion = true;
        digitalWrite(LED_PIN, HIGH);
        lightsOn = true;
        lightOnTime = currentTime;
        Serial.println("💡 Motion Detected: Lights ON");
    } else if (!currentMotion) {
        motionDetected = false;
        currentReadings.motion = false;
    }
    
    // Auto turn off lights after specified duration
    if (lightsOn && (currentTime - lightOnTime >= LIGHT_ON_DURATION)) {
        digitalWrite(LED_PIN, LOW);
        lightsOn = false;
        Serial.println("💡 Auto Timer: Lights OFF");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   SYSTEM STATUS DISPLAY
// ═══════════════════════════════════════════════════════════════════════════════════════

void displaySystemStatus() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastStatusDisplay >= DISPLAY_UPDATE_INTERVAL) {
        lastStatusDisplay = currentTime;
        
        // Clear screen (optional - comment out if not needed)
        // Serial.write(27); Serial.print("[2J"); Serial.write(27); Serial.print("[H");
        
        printTitle("SMART HOME SYSTEM STATUS");
        
        // System Status
        Serial.println("║                              SYSTEM STATUS                              ║");
        printSeparator('─');
        printStatus("Door System", doorOpen ? "OPEN" : "CLOSED", doorOpen);
        printStatus("Smoke Detector", smokeDetected ? "ALERT" : "NORMAL", !smokeDetected);
        printStatus("Temperature Control", motorRunning ? "FAN ON" : "FAN OFF", motorRunning);
        printStatus("Lighting System", lightsOn ? "LIGHTS ON" : "LIGHTS OFF", lightsOn);
        
        Serial.println("║                                                                          ║");
        Serial.println("║                             SENSOR READINGS                             ║");
        printSeparator('─');
        
        // Sensor Readings
        if (currentReadings.distance >= 0) {
            printSensorReading("Distance", currentReadings.distance, "cm");
        } else {
            Serial.println("║ Distance:       ERROR - Invalid Reading                                 ║");
        }
        
        printSensorReading("Temperature", currentReadings.temperature, "°C");
        printSensorReading("Humidity", currentReadings.humidity, "%");
        printSensorReading("Heat Index", currentReadings.heatIndex, "°C");
        printSensorReading("Gas Level", currentReadings.gasLevel, "ppm");
        
        Serial.print("║ Motion:         ");
        Serial.print(currentReadings.motion ? "DETECTED" : "NONE");
        Serial.println("                                            ║");
        
        printSeparator();
        Serial.println("║                            SYSTEM RUNNING                               ║");
        printSeparator();
        Serial.println();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════════════
//                                   MAIN PROGRAM
// ═══════════════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(9600);
    
    // Wait for serial connection
    while (!Serial) {
        delay(10);
    }
    
    delay(1000);
    initializeSystem();
}

void loop() {
    if (!systemInitialized) {
        return;
    }
    
    // Execute all system functions
    automaticDoorControl();
    smokeDetectionSystem();
    temperatureControlSystem();
    automaticLightingSystem();
    displaySystemStatus();
    
    // Main loop delay
    delay(SYSTEM_CYCLE_DELAY);
}