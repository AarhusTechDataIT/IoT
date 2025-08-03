/***************************************************************************
  BME280 & HC-SR04 Ultrasonic Distance Calculator

  This sketch reads temperature, humidity, and pressure from a BME280 sensor,
  calculates the precise speed of sound, and then uses that value with an
  HC-SR04 ultrasonic sensor to accurately measure distance.

  It demonstrates how environmental factors affect the speed of sound and,
  consequently, the accuracy of ultrasonic distance measurements.

  Dependencies:
  - Adafruit BME280 Library: https://github.com/adafruit/Adafruit_BME280_Library
  - Adafruit Unified Sensor Driver: https://github.com/adafruit/Adafruit_Sensor

  To use, install the above libraries through the Arduino Library Manager.
  Then, wire your sensors to the Arduino's pins as defined below.
***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <math.h> // For exp() and sqrt() functions

// --- Pin Definitions for HC-SR04 ---
const int trans_pin = 10; // Trig pin of the HC-SR04
const int recv_pin = 11;  // Echo pin of the HC-SR04

// Create a BME280 sensor object
Adafruit_BME280 bme;

// --- Physical Constants ---
const float R = 8.31446;      // Universal gas constant in J/(mol·K)
const float M_d = 0.028965;   // Molar mass of dry air in kg/mol
const float M_w = 0.018016;   // Molar mass of water vapor in kg/mol
const float gamma_d = 1.400;  // Heat capacity ratio of dry air
const float gamma_w = 1.327;  // Heat capacity ratio of water vapor

void setup() {
  // Initialize Serial communication at 9600 bits per second.
  Serial.begin(9600);
  while (!Serial); // Wait for Serial port to connect (needed for native USB)

  Serial.println(F("BME280 & HC-SR04 Distance Calculator"));
  Serial.println(F("------------------------------------"));

  // --- Initialize BME280 Sensor ---
  unsigned status = bme.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF indicates a wiring or I2C address problem.\n");
    while (1) delay(10); // Halt forever if sensor not found.
  }

  // --- Initialize HC-SR04 Pins ---
  pinMode(trans_pin, OUTPUT);
  pinMode(recv_pin, INPUT);
}

void loop() {
  // --- Step 1: Read BME280 Sensor Data ---
  float temp_c = bme.readTemperature();          // Temperature in degrees Celsius
  float rel_hum = bme.readHumidity();            // Relative humidity as a percentage (%)
  float pressure_pa = bme.readPressure();        // Absolute pressure in Pascals (Pa)

  // Check if any sensor readings failed.
  if (isnan(temp_c) || isnan(rel_hum) || isnan(pressure_pa)) {
    Serial.println(F("Failed to read from BME280 sensor!"));
    return;
  }

  // --- Step 2: Calculate Comprehensive Speed of Sound ---
  float temp_k = temp_c + 273.15;
  float p_sat = 610.94 * exp((17.625 * temp_c) / (temp_c + 243.04));
  float p_v = (rel_hum / 100.0) * p_sat;
  float x_w = p_v / pressure_pa;
  float x_d = 1.0 - x_w;
  float M_a = x_d * M_d + x_w * M_w;
  float gamma_a = x_d * gamma_d + x_w * gamma_w;
  float speed_comprehensive = sqrt((gamma_a * R * temp_k) / M_a); // Speed in m/s

  // --- Step 3: Measure Distance with HC-SR04 ---
  // Clear the trans_pin by setting it LOW
  digitalWrite(trans_pin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by sending a HIGH pulse for 10 microseconds
  digitalWrite(trans_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trans_pin, LOW);

  // Read the echoPin. pulseIn() returns the duration of the pulse in microseconds.
  long duration_us = pulseIn(recv_pin, HIGH);

  // --- Step 4: Calculate Distance ---
  // The sound travels to the object and back, so we divide the duration by 2.
  // Distance = (Speed * Time)
  // Speed is in m/s, Duration is in µs. We need to convert units.
  // Distance in cm = (speed_comprehensive * (duration_us / 1,000,000.0) / 2.0) * 100.0
  // Simplified: Distance in cm = (speed_comprehensive * duration_us) / 20000.0
  float distance_cm = (speed_comprehensive * duration_us) / 20000.0;


  // --- Step 5: Print Results to Serial Monitor ---
  Serial.println(F("Sensor Readings:"));
  Serial.print(F("  Temperature: ")); Serial.print(temp_c); Serial.println(F(" °C"));
  Serial.print(F("  Humidity:    ")); Serial.print(rel_hum); Serial.println(F(" %"));
  Serial.print(F("  Pressure:    ")); Serial.print(pressure_pa / 100.0); Serial.println(F(" hPa"));
  Serial.println();

  Serial.println(F("Calculated Values:"));
  Serial.print(F("  Speed of Sound: "));
  Serial.print(speed_comprehensive);
  Serial.println(F(" m/s"));

  Serial.print(F("  Measured Distance: "));
  if (duration_us == 0) {
    Serial.println(F("Out of range"));
  } else {
    Serial.print(distance_cm);
    Serial.println(F(" cm"));
  }
  
  Serial.println(F("------------------------------------"));

  // Wait for 2 seconds before taking the next reading.
  delay(2000);
}
