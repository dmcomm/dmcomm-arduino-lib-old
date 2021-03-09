#include <DMComm.h>

const byte pinAnalog = A3;
const byte pinOut = A2;
const byte pinNotOE = A1;
const byte pinLed = 13;

const int logBufSize = 1000;

DMComm dmcomm(pinAnalog, pinOut, pinNotOE);
byte logBuffer[logBufSize];

void setup() {
    Serial.begin(9600);
    dmcomm.begin();
    dmcomm.setPinLed(pinLed);
    dmcomm.setSerial(Serial);
    dmcomm.setLogBuffer(logBuffer, logBufSize);
}

void loop() {
    dmcomm.loop();
}

