/*
 * MQ Gas Sensor Tester with Buzzer Alarm
 * Reads analog value from MQ sensor and triggers buzzer when gas is detected
 * 
 * Connections:
 * MQ Sensor VCC -> Arduino 5V
 * MQ Sensor GND -> Arduino GND
 * MQ Sensor AOUT -> Arduino A0
 * Buzzer (+) -> Arduino D8
 * Buzzer (-) -> Arduino GND
 */

const int MQ_PIN = A0;       // MQ sensor analog pin
const int BUZZER_PIN = 5;    // Buzzer digital pin

// Threshold values (adjust based on your needs)
const int WARNING_THRESHOLD = 540;   // Moderate gas level
const int DANGER_THRESHOLD = 564;    // High gas level

// Calibration values
float RL = 10.0;  // Load resistance in kilo-ohms
float R0 = 10.0;  // Will be calculated during calibration

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("MQ Gas Sensor with Buzzer Alarm");
  Serial.println("-------------------------------");
  
  // Uncomment to calibrate sensor
  // calibrate();
}

void loop() {
  int sensorValue = analogRead(MQ_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  float RS = ((5.0 * RL) / voltage) - RL;
  float ratio = RS / R0;

  // Print sensor data
  Serial.print("Raw: ");
  Serial.print(sensorValue);
  Serial.print(" | Ratio: ");
  Serial.print(ratio);
  
  // Check gas levels and trigger buzzer
  if (sensorValue >= DANGER_THRESHOLD) {
    Serial.println(" - DANGER! High gas concentration!");
    dangerAlarm();  // Continuous fast beeping
  } 
  else if (sensorValue >= WARNING_THRESHOLD) {
    Serial.println(" - Warning: Gas detected");
    warningAlarm();  // Intermittent beeping
  } 
  else {
    Serial.println(" - Normal");
    noAlarm();      // Turn off buzzer
  }

  delay(500);  // Shorter delay for more responsive buzzer
}

void warningAlarm() {
  // Intermittent beep (1 beep per second)
  tone(BUZZER_PIN, 1000, 200);  // 1kHz tone for 200ms
  delay(800);                   // 800ms pause
}

void dangerAlarm() {
  // Fast continuous beeping (5 beeps per second)
  tone(BUZZER_PIN, 2000, 100);  // 2kHz tone for 100ms
  delay(100);                   // 100ms pause
}

void noAlarm() {
  noTone(BUZZER_PIN);  // Turn off buzzer
}

void calibrate() {
  Serial.println("Calibrating MQ sensor...");
  Serial.println("Please ensure sensor is in clean air");
  Serial.println("Waiting for 2 minutes warm-up time");
  
  delay(120000); // Wait 2 minutes for sensor to warm up
  
  float sum = 0.0;
  int samples = 100;
  
  Serial.println("Sampling...");
  
  for (int i = 0; i < samples; i++) {
    int sensorValue = analogRead(MQ_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    float RS = ((5.0 * RL) / voltage) - RL;
    sum += RS;
    delay(100);
  }
  
  float RS_clean = sum / samples;
  R0 = RS_clean / 9.8;  // Adjust 9.8 based on your sensor's datasheet
  
  Serial.print("Calibration complete. R0 = ");
  Serial.print(R0);
  Serial.println("kΩ");
  Serial.println("-----------------");
}