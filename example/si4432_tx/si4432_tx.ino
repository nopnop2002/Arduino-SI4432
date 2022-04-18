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
  byte txBuf[64];
  sprintf(txBuf,"Hello World %lu", millis());
  byte txLen = strlen((char *)txBuf);

  bool sent = radio.sendPacket(txLen, txBuf);
  //Serial.print("sent=");
  //Serial.println(sent);
  if (sent) {
    
#if RESPONCE
    byte rxLen;
    byte rxBuf[64];
    unsigned long startMillis = millis();
    Serial.println("startListening");
    radio.startListening(); // restart the listening.
    while(1) {
      bool recv = radio.isPacketReceived();
      if (recv) {
        radio.getPacketReceived(&rxLen, rxBuf);
        unsigned long receiveMillis = millis();
        unsigned long elaspedMills = receiveMillis - startMillis;
        Serial.print(receiveMillis);
        Serial.print(" rxLen:");
        Serial.print(rxLen, DEC);
        Serial.print(" elasped:");
        Serial.print(elaspedMills);
        Serial.println();
        
        for (int i = 0; i < rxLen; ++i) {
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
    
        for (int i = 0; i < txLen; ++i) {
          char c = txBuf[i];
          if (c > 0x20 && c < 0xFF) {
            Serial.print(c);
          } else {
            Serial.print(" ");
          }
        }
        Serial.print("-->");
        for (int i = 0; i < rxLen; ++i) {
          char c = rxBuf[i];
          if (c > 0x20 && c < 0xFF) {
            Serial.print(c);
          } else {
            Serial.print(" ");
          }
        }
        Serial.println();
        break;
      } // end if

      if ( (millis() - startMillis) > 1000) {
        Serial.println("No responce within 1000 ms");
        break;
      }
      
    } // end while
#endif

  } else {
    Serial.println("sendPacket Fail");
    delay(10000);
  }

  delay(1000);

}
