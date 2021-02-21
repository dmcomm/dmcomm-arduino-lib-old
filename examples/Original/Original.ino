#include <DMComm.h>

const byte pinAnalog = A3;
const byte pinOut = A2;
const byte pinNotOE = A1;
const byte pinLed = 13;

const int logSize = 1000;

DMComm dmcomm(pinAnalog, pinOut, pinNotOE);
byte logBuffer[logSize];

void setup() {
    Serial.begin(9600);
    dmcomm.begin();
}

void loop() {

}

