/*
  Transmitter code using the rc-switch library.
  This will send a simple code that the sniffer can understand.
*/

#include <RCSwitch.h>

// Create an instance of the library
RCSwitch mySwitch = RCSwitch();

void setup() {
  // Set up the transmitter. The pin is D12.
  // The transmitter is on the default pin for RCSwitch (10), but we will set it explicitly.
  // Note: Most online examples use default pins. Setting it yourself is good practice.
  // The library does not use pin numbers for the transmitter pin in the setup.
  // It is set directly in the send command or defaults to a specific pin if not set.
  // Let's assume you've wired to pin D12 and need to ensure it works.
  // To avoid confusion with library defaults, it's often easiest to wire to the default pin
  // or use a library that allows explicit pin setting.
  // However, the standard RCSwitch library uses Interrupts for receiving, but not transmitting.
  // Let's create a simple version that should work with most default setups.
  // We will assume pin D10 is the default. If your wiring is on D12, please move it to D10.

    // Transmitter is connected to Arduino Pin #10  
  mySwitch.enableTransmit(10);
  
  // Optional set protocol (default is 1, will work for most outlets)
  // mySwitch.setProtocol(2);

  // Optional set pulse length.
  // mySwitch.setPulseLength(320);
  
  // Optional set number of transmission repetitions.
  // mySwitch.setRepeatTransmit(15);
}

void loop() {
  // Send a simple code. The value is 12345 and it's 24 bits long.
  mySwitch.send(12345, 24);

  // Wait a couple of seconds before sending again
  delay(1000);
}