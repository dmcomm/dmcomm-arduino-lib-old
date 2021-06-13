// Program for M5Stack Basic with A-Com.
// Aiming for similar behaviour to non-library version of DMComm.

#include <M5Stack.h>
#include <DMComm.h>

const byte pinAnalog = 35;
const byte pinOut = 26;

const int logBufSize = 1000;

DMComm::DComAnalog dcom(DMComm::BOARD_3V3, 10, pinAnalog, pinOut);
DMComm::Controller controller(dcom);
byte logBuffer[logBufSize];

void setup() {
    Serial.begin(9600); //non-default baud rate; set SerialEnable=false in M5.begin
    dcom.begin();
    dcom.setLogBuffer(logBuffer, logBufSize);
    controller.setSerial(Serial);
    M5.begin(true, false, false);
    M5.Power.begin();
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.println("Hello DMComm");
    analogReadResolution(10); //since the config in DComAnalog doesn't work yet
    dacWrite(25, 0); //stop the noise
}

void loop() {
    controller.loop();
}
