// Program for M5Stack Basic with A-Com.
// Classic USB, Bluetooth, or standalone DMOG punchbag.

#include <M5Stack.h>
#include <BluetoothSerial.h>
#include <DMComm.h>

const byte pinAnalog = 35;
const byte pinOut = 26;

const int logBufSize = 1000;

uint8_t digirom[] = "V1-FB04-FD02";

enum State {STATE_MENU, STATE_SERIAL, STATE_STANDALONE};

DMComm::DComAnalog dcom(DMComm::BOARD_3V3, 10, pinAnalog, pinOut);
DMComm::Controller controller(dcom);
byte logBuffer[logBufSize];
State state;
BluetoothSerial SerialBT;

void display(const char * str) {
    M5.Lcd.clear(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println(str);
}

void setup() {
    M5.begin(true, false, false);
    M5.Power.begin();
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(3);
    analogReadResolution(10); //since the config in DComAnalog doesn't work yet
    dacWrite(25, 0); //stop the noise
    dcom.begin();
    dcom.setLogBuffer(logBuffer, logBufSize);
    display("*DMComm*\nA:USB\nB:Bluetooth\nC:DMOG punchbag");
    state = STATE_MENU;
}

void loop() {
    M5.update();
    if (state == STATE_MENU) {
        if (M5.BtnA.wasReleased()) {
            Serial.begin(9600); //non-default baud rate; set SerialEnable=false in M5.begin
            controller.setSerial(Serial);
            display("DMComm USB");
            state = STATE_SERIAL;
        } else if (M5.BtnB.wasReleased()) {
            SerialBT.begin("DMComm-ESP32");
            controller.setSerial(SerialBT);
            display("DMComm Bluetooth");
            state = STATE_SERIAL;
        } else if (M5.BtnC.wasReleased()) {
            controller.execute(digirom);
            display("DMComm\nDMOG punchbag\nPress B to battle");
            state = STATE_STANDALONE;
        }
    } else if (state == STATE_SERIAL) {
        controller.loop();
    } else {
        //STATE_STANDALONE
        if (M5.BtnB.wasReleased()) {
            controller.doComm();
        }
    }
}
