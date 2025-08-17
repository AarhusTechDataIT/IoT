#include <SPI.h>
#include <RH_MRF89.h>

#define CSCON_PIN  5  // D1
#define IRQ_PIN    0  // D3

RH_MRF89 rf(CSCON_PIN, IRQ_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // IRQ pull-up
  pinMode(IRQ_PIN, INPUT_PULLUP);

  // SPI setup
  SPI.begin();
  SPI.setFrequency(2000000);

  // Init radio
  if (!rf.init()) { Serial.println("init failed"); while (1); }
  rf.setFrequency(868.0);
  rf.setModeRx();

  // Dump regs
  Serial.println("\n-- Register Dump 0x00–0x07 --");
  for (uint8_t r = 0; r <= 0x07; r++) {
    uint8_t v = rf.spiRead(r);
    Serial.print("0x"); Serial.print(r, HEX);
    Serial.print(": 0x"); Serial.println(v, HEX);
  }
}

void loop() { /* done */ }