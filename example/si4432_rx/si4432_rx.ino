#include "si4432.h"

#define RESPONCE 1

Si4432 radio(10, 7, 2); // CS, SDN, IRQ


void setup() {
  delay(1000);
  Serial.begin(115200);
  if (radio.init() == false) {
    Serial.println("SI4432 not installed");
    while(1);
  }
  radio.setBaudRate(70);
  radio.setFrequency(433);
  radio.readAll();

  radio.startListening();
}

void loop() {
  byte rxBuf[64] = {0};
  byte rxLen = 0;

  bool recv = radio.isPacketReceived();
  if (recv) {
    radio.getPacketReceived(&rxLen, rxBuf);
    Serial.print(millis());
    Serial.print(" rxLen:");
    Serial.println(rxLen, DEC);

    for (int i = 0; i < rxLen; ++i) {
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    for (int i = 0; i < rxLen; ++i) {
      char c = rxBuf[i];
      if (c > 0x20 && c < 0xFF) {
        Serial.print(c);
      } else {
        Serial.print(" ");
      }
    }
    Serial.println();

#if RESPONCE
    byte txLen;
    byte txBuf[64];
    for (int i = 0; i < rxLen; ++i) {
      char c = rxBuf[i];
      if (islower(c)) {
        txBuf[i] = toupper(c);
      } else {
        txBuf[i] = tolower(c);
      }
    }
    txLen = rxLen;
    
    delay(30);
    Serial.println("Sending response... ");
    radio.sendPacket(txLen, txBuf);
#endif

    radio.startListening(); // restart the listening.
  } else {
    //Serial.println("No packet this cycle");
  }

}
