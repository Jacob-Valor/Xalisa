// Include the Servo library for controlling servo motors
#include <Servo.h>

// Define pin assignments for the ultrasonic sensor
const int trigPin = 8;  // Trigger pin of the ultrasonic sensor
const int echoPin = 9; // Echo pin of the ultrasonic sensor

// Define pin assignment for the servo motor
const int servoPin = 3; // Digital pin connected to the servo motor signal wire

// Create a Servo object
Servo myServo;

// Define constants for servo motor angles
const int doorOpenAngle = 45;   // Angle for the door to be open (adjust as needed)
const int doorCloseAngle = 150;   // Angle for the door to be closed (adjust as needed)

// Define the distance threshold in centimeters
// If an object is closer than this distance, the door will open
const int distanceThresholdCm = 20; // Adjust this value based on your needs

void setup() {
  // Initialize serial communication for debugging purposes
  Serial.begin(9600);

  // Set the ultrasonic sensor pins as input/output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach the servo motor to the specified pin
  myServo.attach(servoPin);

  // Ensure the door starts in the closed position
  myServo.write(doorCloseAngle);
  Serial.println("Door is initially closed.");
  delay(1000); // Give the servo time to move
}

void loop() {
  // Measure the distance using the ultrasonic sensor
  long duration, distanceCm;

  // Clear the trigPin by setting it LOW for a moment
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin to HIGH for 10 microseconds to send a pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the pulse on the echoPin
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  // Speed of sound in air is approximately 343 meters/second or 0.0343 cm/microsecond
  // Distance = (duration * speed of sound) / 2 (because the sound travels to the object and back)
  distanceCm = duration * 0.0343 / 2;

  // Print the distance to the Serial Monitor for debugging
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  // Logic to control the door based on distance
  if (distanceCm < distanceThresholdCm && distanceCm > 0) { // Add distanceCm > 0 to avoid false positives for 0 distance
    // If an object is detected within the threshold, open the door
    if (myServo.read() != doorOpenAngle) { // Only move if not already open
      myServo.write(doorOpenAngle);
      Serial.println("Door Opening...");
      delay(500); // Small delay to allow servo to move
    }
  } else {
    // If no object is detected or it's too far, close the door
    if (myServo.read() != doorCloseAngle) { // Only move if not already closed
      myServo.write(doorCloseAngle);
      Serial.println("Door Closing...");
      delay(500); // Small delay to allow servo to move
    }
  }

  // Add a small delay before the next measurement to avoid erratic readings
  delay(100);
}
