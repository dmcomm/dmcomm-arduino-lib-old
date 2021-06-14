/* This file is part of the DMComm project by BladeSabre. License: MIT. */

#include <Arduino.h>
#include "DMComm.h"

#define SERIAL_WRITE_MAYBE(value) if (serial_ != NULL) { serial_->write(value); }
#define SERIAL_PRINT_MAYBE(value) if (serial_ != NULL) { serial_->print(value); }

namespace DMComm {

Controller::Controller(DComAnalog& prongInterface) : prongInterface_(&prongInterface)
{}

Controller::~Controller() {}

void Controller::loop() {
    if (serial_ == NULL) {
        return;
    }
    uint8_t length = readCommand();
    if (length > 0) {
        serial_->print(F("got "));
        serial_->print(length, DEC);
        serial_->print(F(" bytes: "));
        serial_->write(commandBuffer_, length);
        serial_->print(F(" -> "));
        execute(commandBuffer_);
        //TODO add conditional delay after execute has a return value
    }
    doComm();
}

int8_t Controller::execute(uint8_t command[]) {
    //TODO fix handling of parameter
    uint8_t i = 0;
    while (commandBuffer_[i] != '\0') {
        i ++;
    }
    if (commandBuffer_[0] == 't' || commandBuffer_[0] == 'T') {
        SERIAL_PRINT_MAYBE(F("[test voltages]\n"));
        //TODO scanVoltages(true);
    }
    if ((commandBuffer_[0] == 'd' || commandBuffer_[0] == 'D') && i >= 2) {
        if (commandBuffer_[1] == '0' || commandBuffer_[1] == 'o' || commandBuffer_[1] == 'O') {
            SERIAL_PRINT_MAYBE(F("debug off "));
            //TODO debugMode_ = DEBUG_OFF;
        } else if (commandBuffer_[1] == '1' || commandBuffer_[1] == 'd' || commandBuffer_[1] == 'D') {
            SERIAL_PRINT_MAYBE(F("debug digital "));
            //TODO debugMode_ = DEBUG_DIGITAL;
        } else if (commandBuffer_[1] == '2' || commandBuffer_[1] == 'a' || commandBuffer_[1] == 'A') {
            SERIAL_PRINT_MAYBE(F("debug analog "));
            //TODO debugMode_ = DEBUG_ANALOG;
        }
        if (i >= 5 && commandBuffer_[2] == '-') {
            //TODO setupTrigger(commandBuffer_[3], commandBuffer_[4]);
        } else {
            //TODO setupTrigger(' ', ' ');
        }
        /*TODO if (debugMode_ != DEBUG_OFF) {
            SERIAL_PRINT_MAYBE(F("trigger="));
            //TODO serialPrintTrigger();
            SERIAL_WRITE_MAYBE(' ');
        }*/
    }
    commCommandActive_ = true;
    if (commandBuffer_[0] == 'v' || commandBuffer_[0] == 'V') {
        configIndex_ = PROTOCOL_V;
        SERIAL_WRITE_MAYBE('V');
    } else if (commandBuffer_[0] == 'x' || commandBuffer_[0] == 'X') {
        configIndex_ = PROTOCOL_X;
        SERIAL_WRITE_MAYBE('X');
    } else if (commandBuffer_[0] == 'y' || commandBuffer_[0] == 'Y') {
        configIndex_ = PROTOCOL_Y;
        SERIAL_WRITE_MAYBE('Y');
    } else {
        configIndex_ = PROTOCOL_V;
        commCommandActive_ = false;
    }
    if (i < 2 || (i < 5 && commandBuffer_[1] != '0')) {
        commCommandActive_ = false;
    }
    if (commCommandActive_) {
        if (commandBuffer_[1] == '0') {
            listenOnly_ = true;
            goFirst_ = false;
            SERIAL_WRITE_MAYBE('0');
        } else if (commandBuffer_[1] == '1') {
            listenOnly_ = false;
            goFirst_ = true;
            SERIAL_WRITE_MAYBE('1');
        } else if (commandBuffer_[1] == '2') {
            listenOnly_ = false;
            goFirst_ = false;
            SERIAL_WRITE_MAYBE('2');
        } else {
            commCommandActive_ = false;
            SERIAL_WRITE_MAYBE('?');
        }
    }
    if (i < 7 && !listenOnly_) {
        commCommandActive_ = false;
    }
    if (commCommandActive_ && !listenOnly_) {
        numPackets_ = 0;
        for(i = 2; commandBuffer_[i] != '\0'; i ++) {
            if (commandBuffer_[i] == '-') {
                numPackets_ ++;
            }
        }
        SERIAL_PRINT_MAYBE(F("-["));
        SERIAL_PRINT_MAYBE(numPackets_);
        SERIAL_PRINT_MAYBE(F(" packets]"));
    }
    if (!commCommandActive_) {
        SERIAL_PRINT_MAYBE(F("(paused)"));
    }
    SERIAL_WRITE_MAYBE('\n');
    
    return 0; //TODO
}

int8_t Controller::doComm() {
    prongInterface_->beginComm(configIndex_);
    if (commCommandActive_) { //TODO && doTick(true) == HIGH
        if (listenOnly_) {
            commListen();
        } else {
            commBasic();
        }
        /*TODO if (debugMode_ != DEBUG_OFF) {
            SERIAL_PRINT_MAYBE(F("p:timing="));
            SERIAL_WRITE_MAYBE(CONF_BYTE(protocolSymbol));
            SERIAL_PRINT_MAYBE(F(" threshold="));
            //TODO Serial.print(dm_times.sensor_threshold);
            SERIAL_PRINT_MAYBE(F(" trigger="));
            //TODO serialPrintTrigger();
            SERIAL_WRITE_MAYBE('\n');
        }
        if (debugMode_ == DEBUG_DIGITAL) {
            SERIAL_PRINT_MAYBE(F("d:"));
        }
        if (debugMode_ == DEBUG_ANALOG) {
            SERIAL_PRINT_MAYBE(F("a:"));
        }
        if (debugMode_ != DEBUG_OFF) {
            for (uint16_t i = 0; i < logSize_; i ++) {
                serialPrintHex(logBuffer_[i], 2);
                SERIAL_WRITE_MAYBE(' ');
            }
            SERIAL_WRITE_MAYBE('\n');
        }*/
        if (goFirst_) {
            delay(DMCOMM_GOFIRST_REPEAT_MILLIS);
        }
    } else {
        delay(DMCOMM_INACTIVE_DELAY_MILLIS);
    }
    return 0; //TODO
}

void Controller::setPinLed(uint8_t pinLed) {
    pinLed_ = pinLed;
    pinModeMaybe(pinLed_, OUTPUT);
}

void Controller::setSerial(Stream& serial) {
    serial_ = &serial;
}

void Controller::serialPrintHex(uint16_t number, uint8_t numDigits) {
    const uint8_t maxDigits = 4;
    int8_t i;
    int8_t digits[maxDigits];
    if (serial_ == NULL) {
        return;
    }
    if (numDigits > maxDigits) {
        numDigits = maxDigits;
    }
    for (i = 0; i < numDigits; i ++) {
        digits[i] = val2hex((byte)number);
        number /= 0x10;
    }
    for (i = numDigits - 1; i >= 0; i --) {
        serial_->write(digits[i]);
    }
}

uint8_t Controller::readCommand() {
    unsigned long timeStart; //same type as millis()
    unsigned long time;
    int incomingInt; //same type as Stream.read()
    uint8_t incomingByte;
    uint8_t i = 0;
    
    if (serial_->available() == 0) {
        return 0;
    }
    timeStart = millis();
    do {
        do {
            incomingInt = serial_->read();
            time = millis() - timeStart;
            if (time > DMCOMM_SERIAL_TIMEOUT_MILLIS) {
                serial_->println(F("too late"));
                return 0;
            }
        } while (incomingInt == -1);
        incomingByte = incomingInt;
        if (incomingByte != '\r' && incomingByte != '\n') {
            commandBuffer_[i] = incomingByte;
            i += 1;
        }
    } while (incomingByte != '\r' && incomingByte != '\n' && i < DMCOMM_COMMAND_BUFFER_SIZE - 1);
    if (incomingByte != '\r' && incomingByte != '\n') {
        serial_->println(F("too long"));
        return 0;
    }
    commandBuffer_[i] = '\0';
    return i;
}

void Controller::ledOn() {
    digitalWriteMaybe(pinLed_, HIGH);
}

void Controller::ledOff() {
    digitalWriteMaybe(pinLed_, LOW);
}

void Controller::commListen() {
    int8_t result;
    result = receivePacketAndReport(listenTimeoutTicks_);
    ledOff();
    while (result == 0 || result >= 13) {
        result = receivePacketAndReport(0);
    }
    prongInterface_->delayTicks(endedCaptureTicks_);
    ledOn();
    SERIAL_WRITE_MAYBE('\n');
}

void Controller::commBasic() {
    int8_t bufCur = 2;
    int8_t result;
    if (!goFirst_) {
        if (receivePacketAndReport(listenTimeoutTicks_)) {
            SERIAL_WRITE_MAYBE('\n');
            return;
        }
    }
    ledOff();
    while (1) {
        result = sendPacketAndReport(commandBuffer_ + bufCur);
        if (result <= 0) {
            //finished or error
            break;
        }
        bufCur += result;
        if (receivePacketAndReport(0)) {
            break;
        }
    }
    prongInterface_->delayTicks(endedCaptureTicks_);
    ledOn();
    SERIAL_WRITE_MAYBE('\n');
}

int8_t Controller::sendPacketAndReport(uint8_t digitsToSend[]) {
    int8_t result = prongInterface_->sendPacket(digitsToSend);
    switch (result) {
    case 0:
        //nothing to do
        break;
    case -1:
        //packet error
        SERIAL_PRINT_MAYBE(F("s:?"));
        break;
    default:
        //OK
        if (serial_ != NULL) {
            serial_->print(F("s:"));
            serialPrintHex(prongInterface_->getBitsSent(), 4);
            serial_->write(' ');
        }
    }
    return result;
}

int8_t Controller::receivePacketAndReport(uint16_t timeoutTicks) {
    int8_t result = prongInterface_->receivePacket(timeoutTicks);
    uint16_t bitsReceived = prongInterface_->getBitsReceived();
    switch (result) {
    case 0:
        //OK
        SERIAL_PRINT_MAYBE(F("r:"));
        serialPrintHex(bitsReceived, 4);
        SERIAL_WRITE_MAYBE(' ');
        break;
    case 16:
        //opp didn't release at end of packet
        SERIAL_PRINT_MAYBE(F("r:"));
        serialPrintHex(bitsReceived, 4);
        SERIAL_PRINT_MAYBE(F("t "));
        break;
    case -4:
        //got nothing
        SERIAL_PRINT_MAYBE(F("t "));
        break;
    case -3:
        SERIAL_PRINT_MAYBE(F("t:-3 "));
        break;
    case -2:
        SERIAL_PRINT_MAYBE(F("t:-2 "));
        break;
    case -1:
        SERIAL_PRINT_MAYBE(F("t:-1 "));
        break;
    default:
        //got broken packet
        if (serial_ != NULL) {
            serial_->print(F("t:"));
            serial_->print(result, DEC);
            serial_->write(':');
            serialPrintHex(bitsReceived, 4);
            serial_->write(' ');
        }
    }
    return result;
}

} /* namespace DMComm */
