/* This file is part of the DMComm project by BladeSabre. License: MIT. */

#ifndef DCOMANALOG_H_
#define DCOMANALOG_H_

namespace DMComm {

class DComAnalog {

public:
    /**
     * Create the object, taking no action on the hardware.
     * The analog input needs to be configured to read values of the correct size.
     * @param boardVoltage see BoardVoltage.
     * @param readResolution should be 10 or more, matching the value set with analogReadResolution().
     * Normally 10, but note that ESP32 defaults to 12 bits.
     * @param pinAnalog the pin number for the analog input.
     * @param pinOut the pin number for the main output.
     * @param pinNotOE the pin number for notOE (to include D-Com support).
     * Use DMCOMM_NO_PIN (default) to disable (for A-Com support only).
     */
    DComAnalog(BoardVoltage boardVoltage, uint8_t readResolution,
        uint8_t pinAnalog, uint8_t pinOut, uint8_t pinNotOE=DMCOMM_NO_PIN);

    /**
     * Destructor.
     */
    ~DComAnalog();

    /**
     * Configure the pins for use.
     */
    void begin();

    /**
     * Currently does nothing.
     */
    void end();

    /**
     * Reset the system before starting a communication sequence.
     * @param protocol see ToyProtocol.
     */
    void beginComm(ToyProtocol protocol);

    /**
     * Receive a single packet and store the resulting bits.
     * @param timeoutTicks the number of ticks to wait for the packet (length DMCOMM_TICK_MICROS).
     * If 0 (default), wait the appropriate time for a reply.
     * @return 0 if successful; positive for the number of bits received before failure;
     * negative for failures at different stages before receiving any bits.
     */
    int8_t receivePacket(uint16_t timeoutTicks=0);

    /**
     * Get the bits from the most recent receive attempt.
     * @return the 16 bits received. If less than 16 were received, the result is right-aligned.
     */
    uint16_t getBitsReceived();

    /**
     * Get the bits from the most recent send.
     * @return the 16 bits sent.
     */
    uint16_t getBitsSent();

    /**
     * Send the 16 bits specified.
     * @param bitsToSend the 16 bits to send.
     */
    void sendPacket(uint16_t bitsToSend);

    /**
     * Send the packet specified. See the serial codes documentation for details.
     * @param digitsToSend a byte string describing the packet to send.
     * It should have enough characters to complete the packet or be null-terminated.
     * Extra characters after completing the packet are ignored.
     * @return the number of bytes consumed if successful; 0 if at the end of the string already;
     * -1 if failed to parse.
     */
    int8_t sendPacket(uint8_t digitsToSend[]);

    /*
     * Delay by specified number of ticks (with size DMCOMM_TICK_MICROS)
     * while logging each tick.
     */
    void delayTicks(uint16_t ticks);

    /**
     * Provide a buffer for storing the debug log. If this is not done, no logging will occur.
     */
    void setLogBuffer(uint8_t buffer[], uint16_t length);

    /**
     * Get the number of bytes currently stored in the debug log.
     */
    uint16_t getLogSize();

    /**
     * Set the debug mode with no trigger.
     */
    void configureDebug(DebugMode debugMode);

    /**
     * Set the debug mode and trigger. The trigger is activated at the start of a packet.
     * @param debugMode see DebugMode.
     * @param trigger the packet number counted from 1 in the result sequence.
     * 0 for no trigger, i.e. start right away.
     */
    void configureDebug(DebugMode debugMode, uint8_t trigger);

    /**
     * Set the debug mode and trigger. The trigger is activated at the start of a packet.
     * Packets are numbered 1A,1B,2A,2B... (corresponding to the protocol documentation).
     * @param debugMode see DebugMode.
     * @param packetNum the packet number counted from 1 on each side.
     * 0 for no trigger, i.e. start right away.
     * @param ab 'a' or 'b' (case insensitive). Ignored if packetNum is 0.
     */
    void configureDebug(DebugMode debugMode, uint8_t packetNum, uint8_t ab);

private:

    BoardVoltage boardVoltage_ = BOARD_5V;
    uint8_t readResolution_ = 10;
    uint8_t pinAnalog_ = DMCOMM_NO_PIN;
    uint8_t pinOut_ = DMCOMM_NO_PIN;
    uint8_t pinNotOE_ = DMCOMM_NO_PIN;
    DebugMode debugMode_ = DEBUG_OFF;
    uint8_t debugTrigger_ = 0;
    uint8_t *logBuffer_ = nullptr;
    uint16_t logBufferLength_ = 0;
    uint16_t logSize_ = 0;
    uint32_t logTicksSame_;
    uint8_t logPacketIndex_;
    uint8_t logPrevSensorLevel_;
    ToyProtocol configIndex_ = PROTOCOL_V;
    uint16_t bitsReceived_ = 0;
    uint16_t bitsSent_ = 0;
    uint8_t checksum_;

    /**
     * Read analog input and do logging, clocked by tick length.
     * @param first whether this is the first call of the current communication sequence
     * (so that we know whether to consider the time passed since the previous call).
     * @return the current logic level measured.
     */
    uint8_t doTick(bool first=false);

    /**
     * Drive the comm bus to logic low, according to the config.
     */
    void busDriveLow();

    /**
     * Drive the comm bus to logic high, according to the config.
     */
    void busDriveHigh();

    /**
     * Set the comm bus to listening, according to the config.
     */
    void busRelease();

    /**
     * Initialize logging for a new run.
     */
    void startLog();

    /**
     * Add specified byte to the log.
     */
    void addLogByte(uint8_t b);

    /**
     * Add current sensor level and time to the log (may be multiple bytes)
     * and initialize next timing.
     */
    void addLogTime();

    /**
     * Add specified byte to the log, making sure current timing information is logged first.
     */
    void addLogEvent(uint8_t b);

    /**
     * Scale ADC reading to 6 bits 0-3.3V, according to the config.
     */
    uint8_t scaleSensorValue(uint16_t sensorValue);

    /**
     * Send one bit (helper for sendBits).
     * @param bit 0 or 1.
     */
    void sendBit(uint16_t bit);

    /**
     * Wait until measured input equals level, or timeout in ticks.
     * @return the time taken in ticks.
     */
    uint16_t busWaitTimed(uint8_t level, uint16_t timeoutTicks);

    /**
     * Wait until measured input equals level, or timeout in ticks.
     * @return true if timeout was reached, false otherwise.
     */
    bool busWait(uint8_t level, uint16_t timeoutTicks);

    /**
     * Receive one bit, and rotate into `bitsReceived_` from the left.
     * @return 0 on success, 1 on bit error, 2 on error after receiving bit.
     */
    uint8_t receiveBit();
};

} /* namespace DMComm */

#endif /* DCOMANALOG_H_ */
