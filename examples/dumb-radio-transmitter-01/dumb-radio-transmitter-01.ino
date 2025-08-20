// TRANSMITTER CODE
#include <RH_ASK.h> // Include the RadioHead ASK Library
#include <SPI.h>    // Not actually used but required by the library

// Create an instance of the driver
RH_ASK driver;

int messageCounter = 0;

void setup() {
  // Initialize the ASK driver
  if (!driver.init()) {
    Serial.begin(9600);
    Serial.println("initialization failed");
  }
}

void loop() {
  // Create a message string
  String str = "Hello World! #" + String(messageCounter);
  
  // Convert the String to a char array to send it
  char msg[str.length() + 1];
  str.toCharArray(msg, str.length() + 1);

  // Send the message
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent(); // Wait for the message to be sent

  // Print to serial for debugging (optional)
  // Serial.begin(9600); 
  // Serial.println("Sent: " + String(msg));

  messageCounter++;

  delay(1000); // Wait 1 second before sending the next one
}