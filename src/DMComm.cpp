#include <Arduino.h>
#include "DMComm.h"

static void pinModeMaybe(uint8_t pin, uint8_t mode) {
    if (pin != DMCOMM_NO_PIN) {
        pinMode(pin, mode);
    }
}

static void digitalWriteMaybe(uint8_t pin, uint8_t val) {
    if (pin != DMCOMM_NO_PIN) {
        digitalWrite(pin, val);
    }
}

DMComm::DMComm(uint8_t pinAnalog, uint8_t pinOut, uint8_t pinNotOE) :
    pinAnalog_(pinAnalog), pinOut_(pinOut), pinNotOE_(pinNotOE), pinLed_(DMCOMM_NO_PIN),
    boardVoltage_(BOARD_5V), debugMode_(DEBUG_OFF), debugTrigger_(0),
    serial_(NULL), logBuffer_(NULL), logBufferLength_(0), logSize_(0)
{}

DMComm::~DMComm() {
    end();
}

void DMComm::begin() {
    pinMode(pinAnalog_, INPUT);
    pinModeMaybe(pinOut_, OUTPUT);
    pinModeMaybe(pinNotOE_, OUTPUT);
    pinModeMaybe(pinLed_, OUTPUT);
    
}

void DMComm::end() {
}
