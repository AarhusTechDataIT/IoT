// RECEIVER CODE
#include <RH_ASK.h> // Include the RadioHead ASK Library
#include <SPI.h>    // Not actually used but required by the library

// Create an instance of the driver
RH_ASK driver;

void setup() {
  Serial.begin(9600); // Start serial communication for debugging

  // Initialize the ASK driver
  if (!driver.init()) {
    Serial.println("Initialization failed");
  }
  
  Serial.println("Receiver is ready...");
}

void loop() {
  // Create a buffer to hold the incoming message
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  // Check if a message has been received
  if (driver.recv(buf, &buflen)) {
    //
    // THE FIX IS HERE 👇
    //
    buf[buflen] = '\0'; // Add a null terminator to the end of the received data

    // Message received!
    Serial.print("Message Received: ");
    Serial.println((char*)buf);
  }
}