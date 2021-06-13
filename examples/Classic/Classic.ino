// Program for 5V AVR Arduino boards.
// Aiming for same behaviour as non-library version of DMComm.

#include <DMComm.h>

const byte pinAnalog = A3;
const byte pinOut = A2;
const byte pinNotOE = A1;
const byte pinLed = 13;

const int logBufSize = 1000;

DMComm::DComAnalog dcom(DMComm::BOARD_5V, 10, pinAnalog, pinOut, pinNotOE);
DMComm::Controller controller(dcom);
byte logBuffer[logBufSize];

void setup() {
    Serial.begin(9600);
    dcom.begin();
    dcom.setLogBuffer(logBuffer, logBufSize);
    controller.setPinLed(pinLed);
    controller.setSerial(Serial);
}

void loop() {
    controller.loop();
}
