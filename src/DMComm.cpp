#include <Arduino.h>
#include "DMComm.h"

#define CONF_BYTE(name) pgm_read_byte_near(name + configIndex_)
#define CONF_WORD(name) pgm_read_word_near(name + configIndex_)

static const uint8_t  logicHighLevel[]  PROGMEM = {HIGH, HIGH, LOW};
static const uint8_t  logicLowLevel[]   PROGMEM = {LOW, LOW, HIGH};
static const uint8_t  invertBitRead[]   PROGMEM = {false, false, true};
static const uint8_t  sensorThreshold[] PROGMEM = {37, 37, 25}; // 1.3V and 1.9V approx
static const uint16_t preHighTicks[]    PROGMEM = {15, 15, 25};
static const uint16_t preLowTicks[]     PROGMEM = {295, 300, 200};
static const uint16_t startHighTicks[]  PROGMEM = {10, 11, 55};
static const uint16_t startLowTicks[]   PROGMEM = {4, 8, 30};
static const uint16_t bit1HighTicks[]   PROGMEM = {13, 20, 7};
static const uint16_t bit1LowTicks[]    PROGMEM = {8, 8, 22};
static const uint16_t bit0HighTicks[]   PROGMEM = {5, 8, 20};
static const uint16_t bit0LowTicks[]    PROGMEM = {15, 20, 8};
static const uint16_t bit1HighMinTicks[] PROGMEM = {9, 13, 15};
static const uint16_t sendRecoveryTicks[] PROGMEM = {2, 2, 1};
static const uint16_t timeoutReplyTicks[] PROGMEM = {500, 500, 500};
static const uint16_t timeoutBitsTicks[] PROGMEM = {1000, 1000, 1000};
static const uint16_t timeoutBitTicks[] PROGMEM = {25, 35, 100};

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
    boardVoltage_(BOARD_5V), readResolution_(10), debugMode_(DEBUG_OFF), debugTrigger_(0),
    serial_(NULL), logBuffer_(NULL), logBufferLength_(0), logSize_(0),
    configIndex_(PROTOCOL_V), receivedBits_(0),
    listenTimeoutTicks_(15000), endedCaptureTicks_(2500)
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

void DMComm::beginComm(ToyProtocol protocol) {
    //TODO
    configIndex_ = protocol;
}

int8_t DMComm::receivePacket(uint32_t timeoutMicros) {
    //TODO
    receivedBits_ = 0xAAAA;
    return 0;
}

uint16_t DMComm::getReceivedBits() {
    return receivedBits_;
}

void DMComm::sendPacket(uint16_t bitsToSend) {
    //TODO
}

int8_t DMComm::sendPacket(uint8_t digitsToSend[]) {
    //TODO
    return 0;
}

int8_t DMComm::execute(uint8_t command[]) {
    //TODO
    return 0;
}

void DMComm::setPinLed(uint8_t pinLed) {
    //TODO
}

void DMComm::setSerial(Stream& serial) {
    serial_ = &serial;
}

void DMComm::setLogBuffer(uint8_t buffer[], uint16_t length) {
    logBuffer_ = buffer;
    logBufferLength_ = length;
}

void DMComm::configureDebug(DebugMode debugMode) {
    debugMode_ = debugMode;
}

void DMComm::configureDebug(DebugMode debugMode, uint8_t trigger) {
    debugMode_ = debugMode;
    debugTrigger_ = trigger;
}

void DMComm::configureDebug(DebugMode debugMode, uint8_t packetNum, uint8_t ab) {
    debugMode_ = debugMode;
    //TODO
}

uint16_t DMComm::getLogSize() {
    return logSize_;
}

void DMComm::configureAnalog(BoardVoltage boardVoltage, uint8_t readResolution) {
    boardVoltage_ = boardVoltage;
    readResolution_ = readResolution;
}
