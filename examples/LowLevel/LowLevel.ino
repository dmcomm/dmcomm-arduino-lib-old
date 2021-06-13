// Program for 5V AVR Arduino boards.
// Trades courage egg with Japanese D-3 every 10 seconds.

#include <DMComm.h>

const byte pinAnalog = A3;
const byte pinOut = A2;
const byte pinNotOE = A1;

const int logBufSize = 1000;

DMComm::DComAnalog dcom(DMComm::BOARD_5V, 10, pinAnalog, pinOut, pinNotOE);
byte logBuffer[logBufSize];

void setup() {
    Serial.begin(9600);
    dcom.begin();
    dcom.setLogBuffer(logBuffer, logBufSize);
}

void loop() {
    dcom.beginComm(DMComm::PROTOCOL_V);
    dcom.sendPacket(0x8C0F);
    int8_t ret = dcom.receivePacket();
    if (ret == 0) {
        dcom.sendPacket(0x480F);
        dcom.receivePacket();
    }
    if (ret == 0) {
        Serial.println("done");
    } else {
        Serial.println("nope");
    }
    delay(10000);
}
