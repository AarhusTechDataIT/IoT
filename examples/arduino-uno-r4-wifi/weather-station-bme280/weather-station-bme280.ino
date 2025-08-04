// Include necessary libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Define the sea level pressure for altitude calculation (in hPa)
#define SEALEVELPRESSURE_HPA (1013.25)

// Create an Adafruit_BME280 object
Adafruit_BME280 bme; // Uses I2C communication

void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
  while (!Serial); // Wait for the serial port to connect

  Serial.println("BME280 Test");

  // Initialize the BME280 sensor at I2C address 0x76
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or address!");
    while (1); // Stop execution if the sensor is not found
  }

  Serial.println("-- Weather Station --");
  Serial.println();
}

void loop() {
  // Read sensor values
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pres = bme.readPressure() / 100.0F; // Convert from Pascals to hPa
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // Print the values to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");

  Serial.print("Humidity:    ");
  Serial.print(hum);
  Serial.println(" %");

  Serial.print("Pressure:    ");
  Serial.print(pres);
  Serial.println(" hPa");

  Serial.print("Altitude:    ");
  Serial.print(alt);
  Serial.println(" m");

  Serial.println(); // Add a blank line for readability

  // Wait 2 seconds before taking the next reading
  delay(2000);
}