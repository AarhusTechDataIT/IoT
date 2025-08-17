// ESP8266 D1 Mini and MRF89XAM8A radio module

// Wemos D1 Mini	MRF89XAM8A Pin	Function
// 3V3	          10 (Vin)	      Power
// GND	          1, 11, 12 (GND)	Ground
// D5 (GPIO14)	  6 (SCK)	        SPI Clock in
// D6 (GPIO12)	  7 (SDO)	        SPI Data Out (MISO)
// D7 (GPIO13)	  5 (SDI)	        SPI Data In (MOSI)
// D3 (GPIO0)   	4 (IRQ1)	      Interrupt 1 output
//                                Interrupt 0 output, not connected
// D1 (GPIO5)	    3 (CSCON)	      Configuration chip select in
// D2 (GPIO4)	    8 (CSDATA)	    Data chip select in

#include <SPI.h>
#include <RH_MRF89.h>

// Explicitly define your pins.
// We must use GPIO5 as the primary CS for the library to auto-select GPIO4.
#define RFM_CS_PIN 5   // D1 on the D1 mini (for CSCON)
#define RFM_IRQ_PIN 0  // D3 on the D1 mini (for IRQ1)

// Tell the driver which pins you've chosen
RH_MRF89 mrf89(RFM_CS_PIN, RFM_IRQ_PIN);

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;  // Wait for serial port to connect

  Serial.println("Attempting to initialize MRF89XAM8A on ESP8266...");

  if (mrf89.init()) {
    Serial.println("✅ init success!");
  } else {
    Serial.println("❌ init failed. Check wiring.");
  }

  mrf89.printRegisters();  // See if the registers changed

  // Set the frequency to match the client
  if (mrf89.setFrequency(868.0)) {
    Serial.println("✅ setFrequency success!");
  } else {
    Serial.println("❌ setFrequency failed.");
  }
}

void loop() {

  // Attempt to send

  Serial.println("Sending message...");

  uint8_t data[] = "Hello, World!";
  mrf89.send(data, sizeof(data));

  mrf89.waitPacketSent();

  delay(2000);

  // Attempt to receive

  Serial.println("Listening for incoming message...");

  uint8_t buf[RH_MRF89_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (mrf89.recv(buf, &len)) {
    Serial.print("Received message: ");
    Serial.println((char*)buf);
  }

  delay(2000);
}
