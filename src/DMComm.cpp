#include <Arduino.h>
#include "DMComm.h"

#define CONF_BYTE(name) pgm_read_byte_near(name + configIndex_)
#define CONF_WORD(name) pgm_read_word_near(name + configIndex_)
#define SERIAL_WRITE_MAYBE(value) if (serial_ != NULL) { serial_->write(value); }
#define SERIAL_PRINT_MAYBE(value) if (serial_ != NULL) { serial_->print(value); }

static const uint8_t  protocolSymbol[]  PROGMEM = {'V', 'X', 'Y'};
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
    listenTimeoutTicks_(15000), endedCaptureTicks_(2500),
    commCommandActive_(false)
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

void DMComm::loop() {
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

int8_t DMComm::execute(uint8_t command[]) {
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
            debugMode_ = DEBUG_OFF;
        } else if (commandBuffer_[1] == '1' || commandBuffer_[1] == 'd' || commandBuffer_[1] == 'D') {
            SERIAL_PRINT_MAYBE(F("debug digital "));
            debugMode_ = DEBUG_DIGITAL;
        } else if (commandBuffer_[1] == '2' || commandBuffer_[1] == 'a' || commandBuffer_[1] == 'A') {
            SERIAL_PRINT_MAYBE(F("debug analog "));
            debugMode_ = DEBUG_ANALOG;
        }
        if (i >= 5 && commandBuffer_[2] == '-') {
            //TODO setupTrigger(commandBuffer_[3], commandBuffer_[4]);
        } else {
            //TODO setupTrigger(' ', ' ');
        }
        if (debugMode_ != DEBUG_OFF) {
            SERIAL_PRINT_MAYBE(F("trigger="));
            //TODO serialPrintTrigger();
            SERIAL_WRITE_MAYBE(' ');
        }
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

int8_t DMComm::doComm() {
    beginComm(configIndex_);
    if (commCommandActive_ && doTick(true) == HIGH) {
        if (listenOnly_) {
            //TODO commListen();
        } else {
            //TODO commBasic();
        }
        if (debugMode_ != DEBUG_OFF) {
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
                //TODO serialPrintHex(logBuf[i], 2);
                SERIAL_WRITE_MAYBE(' ');
            }
            SERIAL_WRITE_MAYBE('\n');
        }
        if (goFirst_) {
            delay(DMCOMM_GOFIRST_REPEAT_MILLIS);
        }
    } else {
        delay(DMCOMM_INACTIVE_DELAY_MILLIS);
    }
    return 0; //TODO
}

void DMComm::beginComm(ToyProtocol protocol) {
    configIndex_ = protocol;
    //TODO startLog();
    //TODO busRelease();
    //TODO ...
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


uint8_t DMComm::readCommand() {
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

uint8_t DMComm::doTick(bool first) {
    return HIGH; //TODO
}

void DMComm::ledOn() {
    digitalWriteMaybe(pinLed_, HIGH);
}

void DMComm::ledOff() {
    digitalWriteMaybe(pinLed_, LOW);
}

void DMComm::busDriveLow() {
    digitalWriteMaybe(pinOut_, CONF_BYTE(logicLowLevel));
    digitalWriteMaybe(pinNotOE_, LOW);
}

void DMComm::busDriveHigh() {
    digitalWriteMaybe(pinOut_, CONF_BYTE(logicHighLevel));
    digitalWriteMaybe(pinNotOE_, LOW);
}

void DMComm::busRelease() {
    digitalWriteMaybe(pinNotOE_, HIGH);
    digitalWriteMaybe(pinOut_, CONF_BYTE(logicHighLevel));
    //pinOut_ is "don't care" on D-Com, but matters for A-Com
}
