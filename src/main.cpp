#include <Arduino.h>
#include <Servo.h>

// === Define All Pins using ESP8266 GPIO numbers ===
// Note: D0-D8 on NodeMCU map to GPIOs as:
// D0=16, D1=5, D2=4, D3=0, D4=2, D5=14, D6=12, D7=13, D8=15

// Automatic Door
#define SERVO_PIN D5          // GPIO14
#define TRIGGER_PIN D6        // GPIO12
#define ECHO_PIN D7           // GPIO13

// Smoke Detector
#define GAS_SENSOR_PIN A0     // Only A0 is usable on ESP-12E
#define BUZZER_PIN D8         // GPIO15

// Temperature Detector
#define TEMPERATURE_SENSOR_PIN A0  // A0 used again (multiplex if needed)
#define FAN_1_PIN D4          // GPIO2

// Automatic Lights and Fan
#define PIR_SENSOR_PIN D0     // GPIO16 (INPUT only, no interrupt)
#define LED_1_PIN D1          // GPIO5
#define LED_2_PIN D2          // GPIO4
#define LED_3_PIN D3          // GPIO0
#define FAN_2_PIN D8          // GPIO15 (shared with buzzer, avoid using both)

Servo servo_5;
float Cabinet = 0.0;
int PIRS = 0;
int Gas = 0;
float Temps = 0.0;

long readUltrasonicDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  return pulseIn(echoPin, HIGH, 30000);  // timeout to avoid hang
}

void setup() {
  Serial.begin(115200);
  servo_5.attach(SERVO_PIN);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_1_PIN, OUTPUT);

  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(FAN_2_PIN, OUTPUT);
}

void loop() {
  // === Automatic Door ===
  Cabinet = 0.01723 * readUltrasonicDistance(TRIGGER_PIN, ECHO_PIN);
  Serial.print("Distance: "); Serial.println(Cabinet);
  if (Cabinet < 15) {
    servo_5.write(90);
    delay(5000);
  } else {
    servo_5.write(0);
  }

  // === Automatic Lights and Fan ===
  PIRS = digitalRead(PIR_SENSOR_PIN);
  Serial.print("PIR: "); Serial.println(PIRS);
  digitalWrite(LED_1_PIN, PIRS);
  digitalWrite(LED_2_PIN, PIRS);
  digitalWrite(LED_3_PIN, PIRS);
  digitalWrite(FAN_2_PIN, PIRS);

  // === Temperature Detector ===
  int rawTemp = analogRead(TEMPERATURE_SENSOR_PIN);
  Temps = (rawTemp * 3.3 / 1023.0) * 100.0;
  Serial.print("Temp: "); Serial.println(Temps);
  digitalWrite(FAN_1_PIN, Temps >= 30 ? HIGH : LOW);

  // === Smoke Detector ===
  Gas = analogRead(GAS_SENSOR_PIN);
  Serial.print("Gas: "); Serial.println(Gas);
  digitalWrite(BUZZER_PIN, Gas >= 220 ? HIGH : LOW);

  delay(500);
}
