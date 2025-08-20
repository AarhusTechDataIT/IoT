/*
  433 MHz RF Sniffer / Scanner
  
  Listens for any decodable 433 MHz traffic and prints the
  received code to the Serial Monitor.
  
  Uses the rc-switch library.
*/

#include <RCSwitch.h>

// Create an instance of the RCSwitch library
RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  
  // The receiver is connected to the Arduino's interrupt pin 0, which is digital pin 2.
  mySwitch.enableReceive(0); 
  
  Serial.println("RF Sniffer is ready. Listening for codes...");
  Serial.println("-----------------------------------------");
}

void loop() {
  // Check if a signal is available
  if (mySwitch.available()) {
    
    // Print the received code's value
    Serial.print("Received Value: ");
    Serial.print(mySwitch.getReceivedValue());

    // Print the bit length
    Serial.print(" / Bit Length: ");
    Serial.print(mySwitch.getReceivedBitlength());

    // Print the protocol used
    Serial.print(" / Protocol: ");
    Serial.println(mySwitch.getReceivedProtocol());
    
    Serial.println("-----------------------------------------");

    // Reset the receiver for the next signal
    mySwitch.resetAvailable();
  }
}