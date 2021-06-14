/* This file is part of the DMComm project by BladeSabre. License: MIT. */

#include <Arduino.h>
#include "DMComm.h"

#define CONF_BYTE(name) pgm_read_byte_near(name + configIndex_)
#define CONF_WORD(name) pgm_read_word_near(name + configIndex_)

namespace DMComm {

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

DComAnalog::DComAnalog(BoardVoltage boardVoltage, uint8_t readResolution,
        uint8_t pinAnalog, uint8_t pinOut, uint8_t pinNotOE) :
    boardVoltage_(boardVoltage), readResolution_(readResolution),
    pinAnalog_(pinAnalog), pinOut_(pinOut), pinNotOE_(pinNotOE)
{}

DComAnalog::~DComAnalog() {
    end();
}

void DComAnalog::begin() {
    pinMode(pinAnalog_, INPUT);
    pinModeMaybe(pinOut_, OUTPUT);
    pinModeMaybe(pinNotOE_, OUTPUT);
}

void DComAnalog::end() {}

void DComAnalog::beginComm(ToyProtocol protocol) {
    configIndex_ = protocol;
    startLog();
    busRelease();
    bitsReceived_ = 0;
    checksum_ = 0;
}

int8_t DComAnalog::receivePacket(uint16_t timeoutTicks) {
    uint8_t i, r;
    if (timeoutTicks == 0) {
        timeoutTicks = CONF_WORD(timeoutReplyTicks);
    }
    addLogEvent(DMCOMM_LOG_OPP_ENTER_WAIT);
    if (busWait(LOW, timeoutTicks)) {
        addLogEvent(DMCOMM_LOG_OPP_EXIT_FAIL);
        return -4;
    }
    logPacketIndex_ ++;
    addLogEvent(DMCOMM_LOG_OPP_INIT_PULLDOWN);
    if (busWait(HIGH, CONF_WORD(timeoutBitsTicks))) {
        addLogEvent(DMCOMM_LOG_OPP_EXIT_FAIL);
        return -3;
    }
    addLogEvent(DMCOMM_LOG_OPP_START_BIT_HIGH);
    if (busWait(LOW, CONF_WORD(timeoutBitTicks))) {
        addLogEvent(DMCOMM_LOG_OPP_EXIT_FAIL);
        return -2;
    }
    addLogEvent(DMCOMM_LOG_OPP_START_BIT_LOW);
    if (busWait(HIGH, CONF_WORD(timeoutBitTicks))) {
        addLogEvent(DMCOMM_LOG_OPP_EXIT_FAIL);
        return -1;
    }
    addLogEvent(DMCOMM_LOG_OPP_BITS_BEGIN_HIGH);
    for (i = 0; i < 16; i ++) {
        r = receiveBit();
        if (r == 2 && i == 15) {
            //opp didn't release at end of packet
            addLogEvent(DMCOMM_LOG_OPP_EXIT_FAIL);
            return 16;
        } else if (r != 0) {
            //packet failed
            r = i + r - 1; //number of bits received
            bitsReceived_ >>= (16-r);
            addLogEvent(DMCOMM_LOG_OPP_EXIT_FAIL);
            return r;
        }
    }
    addLogEvent(DMCOMM_LOG_OPP_EXIT_OK);
    return 0;
}

uint16_t DComAnalog::getBitsReceived() {
    return bitsReceived_;
}

uint16_t DComAnalog::getBitsSent() {
    return bitsSent_;
}

void DComAnalog::sendPacket(uint16_t bitsToSend) {
    uint8_t i;
    bitsSent_ = bitsToSend;
    
    logPacketIndex_ ++;
    addLogEvent(DMCOMM_LOG_SELF_ENTER_DELAY);
    delayTicks(CONF_WORD(preHighTicks));
    
    addLogEvent(DMCOMM_LOG_SELF_INIT_PULLDOWN);
    busDriveLow();
    delayTicks(CONF_WORD(preLowTicks));
    
    addLogEvent(DMCOMM_LOG_SELF_START_BIT_HIGH);
    busDriveHigh();
    delayTicks(CONF_WORD(startHighTicks));
    
    addLogEvent(DMCOMM_LOG_SELF_START_BIT_LOW);
    busDriveLow();
    delayTicks(CONF_WORD(startLowTicks));
    
    busDriveHigh();
    for (i = 0; i < 16; i ++) {
        sendBit(bitsToSend & 1);
        bitsToSend >>= 1;
    }
    delayTicks(CONF_WORD(sendRecoveryTicks));
    
    addLogEvent(DMCOMM_LOG_SELF_RELEASE);
    busRelease();
}

int8_t DComAnalog::sendPacket(uint8_t digitsToSend[]) {
    uint16_t bitsToSend;
    uint16_t bitsReceived = bitsReceived_;
    int8_t digits[4];
    int8_t dCur;
    int8_t dCurChk = -1;
    int8_t chkTarget = -1;
    int8_t bufCur = 1;
    uint8_t b1, b2;
    int8_t val1, val2;
    if (digitsToSend[0] == '\0') {
        return 0;
    }
    //require first character dash
    if (digitsToSend[0] != '-') {
        return -1;
    }
    //unpack bits received into digits
    for (dCur = 3; dCur >= 0; dCur --) {
        digits[dCur] = bitsReceived & 0xF;
        bitsReceived >>= 4;
    }
    for (dCur = 0; dCur < 4; dCur ++) {
        b1 = digitsToSend[bufCur];
        if (b1 == '\0') {
            return -1;
        }
        bufCur ++;
        b2 = digitsToSend[bufCur];
        val1 = hex2val(b1);
        val2 = hex2val(b2);
        if (b1 == '@' || b1 == '^') {
            //expect a hex digit to follow
            if (val2 == -1) {
                return -1;
            }
            if (b1 == '@') {
                //here is check digit
                dCurChk = dCur;
                chkTarget = val2;
            } else {
                //xor received bits with new value
                digits[dCur] ^= val2;
            }
            bufCur ++; //extra digit taken
        } else {
            //expect this to be a hex digit
            if (val1 == -1) {
                return -1;
            }
            //store (overwrite) digit
            digits[dCur] = val1;
        }
        if (b1 != '@') {
            //update checksum
            checksum_ += digits[dCur];
            checksum_ &= 0xF;
        }
    }
    if (dCurChk != -1) {
        //we have a check digit
        digits[dCurChk] = (chkTarget - checksum_) & 0xF;
        checksum_ = chkTarget;
    }
    //pack digits into bitsToSend
    bitsToSend = 0;
    for (dCur = 0; dCur < 4; dCur ++) {
        bitsToSend <<= 4;
        bitsToSend |= digits[dCur];
    }
    sendPacket(bitsToSend);
    return bufCur;
}

void DComAnalog::delayTicks(uint16_t ticks) {
    uint16_t i;
    for (i = 0; i < ticks; i ++) {
        doTick();
    }
}

void DComAnalog::setLogBuffer(uint8_t buffer[], uint16_t length) {
    logBuffer_ = buffer;
    logBufferLength_ = length;
}

uint16_t DComAnalog::getLogSize() {
    return logSize_;
}

void DComAnalog::configureDebug(DebugMode debugMode) {
    debugMode_ = debugMode;
}

void DComAnalog::configureDebug(DebugMode debugMode, uint8_t trigger) {
    debugMode_ = debugMode;
    debugTrigger_ = trigger;
}

void DComAnalog::configureDebug(DebugMode debugMode, uint8_t packetNum, uint8_t ab) {
    debugMode_ = debugMode;
    //TODO
}



uint8_t DComAnalog::doTick(bool first) {
    static unsigned long prevMicros = 0; //same type as micros()
    static uint16_t ticks = 0;
    
    uint16_t sensorValue;
    uint8_t sensorLevelScaled;
    uint8_t sensorLevelDigital;
    uint8_t sensorLevelForLog;
    uint8_t ticksMissed = 0;
    
    sensorValue = analogRead(pinAnalog_);
    
    //counts missed ticks for log (but delayTicks does not currently account for them)
    while (ticksMissed <= DMCOMM_LOG_MAX_TICKS_MISSED && micros() - prevMicros > DMCOMM_TICK_MICROS) {
        prevMicros += DMCOMM_TICK_MICROS;
        ticks ++;
        ticksMissed ++;
    }
    if (ticksMissed <= DMCOMM_LOG_MAX_TICKS_MISSED) {
        while(micros() - prevMicros < DMCOMM_TICK_MICROS);
        prevMicros += DMCOMM_TICK_MICROS;
        ticks ++;
    } else {
        prevMicros = micros();
    }
    if (ticksMissed != 0 && !first) {
        if (ticksMissed > DMCOMM_LOG_MAX_TICKS_MISSED) {
            ticksMissed = 0;
        }
        addLogEvent(DMCOMM_LOG_PREFIX_TICK_OVERRUN | ticksMissed);
    }
    
    sensorLevelScaled = scaleSensorValue(sensorValue);
    
    if (sensorLevelScaled >= CONF_BYTE(sensorThreshold)) {
        sensorLevelDigital = HIGH;
    } else {
        sensorLevelDigital = LOW;
    }
    
    if (debugMode_ == DEBUG_ANALOG) {
        sensorLevelForLog = sensorLevelScaled;
    } else {
        sensorLevelForLog = sensorLevelDigital;
    }
    
    if (sensorLevelForLog != logPrevSensorLevel_) {
        addLogTime();
        logTicksSame_ = 1;
    } else {
        logTicksSame_ ++;
    }
    logPrevSensorLevel_ = sensorLevelForLog;
    
    if (sensorLevelDigital == HIGH) {
        return CONF_BYTE(logicHighLevel);
    } else {
        return CONF_BYTE(logicLowLevel);
    }
}

void DComAnalog::busDriveLow() {
    digitalWriteMaybe(pinOut_, CONF_BYTE(logicLowLevel));
    digitalWriteMaybe(pinNotOE_, LOW);
}

void DComAnalog::busDriveHigh() {
    digitalWriteMaybe(pinOut_, CONF_BYTE(logicHighLevel));
    digitalWriteMaybe(pinNotOE_, LOW);
}

void DComAnalog::busRelease() {
    digitalWriteMaybe(pinNotOE_, HIGH);
    digitalWriteMaybe(pinOut_, CONF_BYTE(logicHighLevel));
    //pinOut_ is "don't care" on D-Com, but matters for A-Com
}

void DComAnalog::startLog() {
    logSize_ = 0;
    logTicksSame_ = 0;
    logPacketIndex_ = 0;
}

void DComAnalog::addLogByte(uint8_t b) {
    if (logPacketIndex_ >= debugTrigger_ && logSize_ < logBufferLength_) {
        logBuffer_[logSize_] = b;
        logSize_ ++;
    }
}

void DComAnalog::addLogTime() {
    if (logTicksSame_ == 0) {
        return;
    }
    if (debugMode_ == DEBUG_ANALOG) {
        addLogByte(logPrevSensorLevel_);
        logTicksSame_ --;
    } else {
        byte logPrefix = (logPrevSensorLevel_ == LOW) ? DMCOMM_LOG_PREFIX_LOW : DMCOMM_LOG_PREFIX_HIGH;
        addLogByte((logTicksSame_ & DMCOMM_LOG_MAX_COUNT) | logPrefix);
        logTicksSame_ >>= DMCOMM_LOG_COUNT_BITS;
    }
    while (logTicksSame_ > 0) {
        addLogByte((logTicksSame_ & DMCOMM_LOG_MAX_COUNT) | DMCOMM_LOG_PREFIX_AGAIN);
        logTicksSame_ >>= DMCOMM_LOG_COUNT_BITS;
    }
}

void DComAnalog::addLogEvent(uint8_t b) {
    addLogTime();
    addLogByte(b);
}

uint8_t DComAnalog::scaleSensorValue(uint16_t sensorValue) {
    //TODO account for readResolution_
    if (boardVoltage_ == BOARD_3V3) {
        sensorValue = sensorValue / 16;
    } else {
        sensorValue = sensorValue * 3 / 32;
    }
    if (sensorValue > 63) {
        sensorValue = 63;
    }
    return (uint8_t)sensorValue;
}



void DComAnalog::sendBit(uint16_t bit) {
    addLogEvent(bit ? DMCOMM_LOG_SELF_SEND_BIT_1 : DMCOMM_LOG_SELF_SEND_BIT_0);
    delayTicks(bit ? CONF_WORD(bit1HighTicks) : CONF_WORD(bit0HighTicks));
    busDriveLow();
    delayTicks(bit ? CONF_WORD(bit1LowTicks) : CONF_WORD(bit0LowTicks));
    busDriveHigh();
}

uint16_t DComAnalog::busWaitTimed(uint8_t level, uint16_t timeoutTicks) {
    uint16_t ticksPassed = 0;
    uint8_t logicLevel;
    do {
        logicLevel = doTick();
        ticksPassed ++;
    } while (logicLevel != level && ticksPassed <= timeoutTicks);
    return ticksPassed;
}

bool DComAnalog::busWait(uint8_t level, uint16_t timeoutTicks) {
    uint16_t ticksPassed = busWaitTimed(level, timeoutTicks);
    return (ticksPassed > timeoutTicks);
}

uint8_t DComAnalog::receiveBit() {
    uint16_t ticksPassed;
    uint16_t timeoutTicks = CONF_WORD(timeoutBitTicks);
    bool bit0 = false;
    ticksPassed = busWaitTimed(LOW, timeoutTicks);
    if (ticksPassed > timeoutTicks) {
        return 1;
    }
    if (ticksPassed > CONF_WORD(bit1HighMinTicks)) {
        bit0 = true;
    }
    if (CONF_BYTE(invertBitRead)) {
        bit0 = !bit0;
    }
    bitsReceived_ >>= 1;
    if (bit0) {
        addLogEvent(DMCOMM_LOG_OPP_GOT_BIT_1);
        bitsReceived_ |= 0x8000;
    } else {
        addLogEvent(DMCOMM_LOG_OPP_GOT_BIT_0);
    }
    if (busWait(HIGH, timeoutTicks)) {
        return 2;
    }
    return 0;
}

} /* namespace DMComm */
