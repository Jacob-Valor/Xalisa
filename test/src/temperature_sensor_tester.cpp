// Include the DHT sensor library
// You might need to install this library if you haven't already.
// In Arduino IDE, go to Sketch > Include Library > Manage Libraries...
// Search for "DHT sensor library" by Adafruit and install it.
#include "DHT.h"

// Define the pin where the DHT11 data pin is connected
// For example, if connected to digital pin 2:
#define DHTPIN A1

// Define the digital pin for the motor control
// You can change this to any available digital pin, e.g., pin 7
#define MOTORPIN 11

// Define the type of DHT sensor you are using
// DHT11, DHT22, or DHT21
#define DHTTYPE DHT11

// Initialize DHT sensor.
// Parameter 1: Pin number
// Parameter 2: DHT type
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // Start serial communication at 9600 baud rate
  // This is used to print the sensor readings to your computer's Serial Monitor
  Serial.begin(9600);
  Serial.println("DHT11 Test Program with Motor Control");

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT sensor initialized.");

  // Set the motor pin as an OUTPUT
  pinMode(MOTORPIN, OUTPUT);
  Serial.println("Motor pin configured as OUTPUT.");
}

void loop() {
  // Sensor readings can take a bit of time, so wait a few seconds between readings.
  // A delay of 2 seconds (2000 milliseconds) is recommended for DHT11.
  delay(2000);

  // Read humidity as a percentage (0-100%)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return; // Exit loop iteration and try again after delay
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius
  float hic = dht.computeHeatIndex(t, h, false);

  // Print the readings to the Serial Monitor
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");

  // --- Motor Control Logic ---
  if (t > 30.0) { // Check if temperature in Celsius is greater than 37.0
    digitalWrite(MOTORPIN, HIGH); // Turn the motor ON
    Serial.println("Temperature > 37C. Motor ON.");
  } else {
    digitalWrite(MOTORPIN, LOW); // Turn the motor OFF
    Serial.println("Temperature <= 37C. Motor OFF.");
  }
}
