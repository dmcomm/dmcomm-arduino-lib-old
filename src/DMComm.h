
#ifndef DMCOMM_H_
#define DMCOMM_H_

#define DMCOMM_NO_PIN 0xFF

class DMComm {

public:
    /**
     * The board voltage. Fast division is used, so only pre-programmed voltages are available.
     */
    enum BoardVoltage {BOARD_5V, BOARD_3V3};
    
    /**
      * The debug mode: off, digital or analog.
      */
    enum DebugMode {DEBUG_OFF, DEBUG_DIGITAL, DEBUG_ANALOG};
    
    /**
     * Create the object, taking no action on the hardware.
     * @param pinAnalog the pin number for the analog input.
     * @param pinOut the pin number for the main output.
     * @param pinNotOE the pin number for notOE (to include D-Com support).
     * Use DMCOMM_NO_PIN (default) to disable (for A-Com support only).
     */
    DMComm(uint8_t pinAnalog, uint8_t pinOut, uint8_t pinNotOE=DMCOMM_NO_PIN);
    
    /**
     * Destructor.
     */
    ~DMComm();
    
    /**
     * Configure the pins for use.
     */
    void begin();
    
    /**
     * Does this need to do anything?
     */
    void end();
    
    /**
     * Reset the system before starting a communication sequence using the single-packet functions.
     * This is not required with the `execute` function.
     */
    void reset();
    
    /**
     * Receive a single packet and store the resulting bits.
     * @param timeoutMicros the number of microseconds to wait for the packet.
     * @return 0 if successful; positive for the number of bits received before failure;
     * negative for failures at different stages before receiving any bits.
     */
    int8_t receivePacket(uint32_t timeoutMicros);
    
    /**
     * Get the bits from the most recent receive attempt.
     * @return the 16 bits received. If less than 16 were received, the result is right-aligned.
     */
    uint16_t getReceivedBits();
    
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
    
    /**
     * Carry out the command specified. See the serial codes documentation for details.
     * TBD: Will this just do communication codes, or config codes too?
     * @param command a null-terminated byte string containing the command.
     * @return (not decided yet).
     */
    int8_t execute(uint8_t command[]);
    
    /**
     * Set a pin for LED output. Initially DMCOMM_NO_PIN, which disables it.
     */
    void setLedPin(uint8_t ledPin);
    
    /**
      * Provide a stream for reporting results. If this is not done, no results will be reported.
      */
    void setSerial(Stream serial);
    
    /**
     * Provide a buffer for storing the debug log. If this is not done, no logging will occur.
     */
    void setLogBuffer(uint8_t buffer[], uint16_t length);
    
    /**
     * Set the debug mode with no trigger.
     */
    void configureDebug(DebugMode debugMode);
    
    /**
     * Set the debug mode and trigger. The trigger is activated at the start of a packet.
     * Packets are numbered 1A,1B,2A,2B...
     * @param debugMode see DebugMode.
     * @param packetNum the packet number counted from 1 on each side.
     * 0 for no trigger, i.e. start right away.
     * @param ab 'a' or 'b' (case insensitive). Ignored if packetNum is 0.
     */
    void configureDebug(DebugMode debugMode, uint8_t packetNum, uint8_t ab);
    
    /**
     * Get the number of bytes currently stored in the debug log.
     */
    uint16_t getLogSize();
    
    /**
     * Configure the analog input to read values of the correct size.
     * Initially 5V with 10 bits, suitable for 5V AVR-based boards.
     * @param boardVoltage see BoardVoltage.
     * @param readResolution should be 10 or more, matching the value set with analogReadResolution().
     * Note that ESP32 defaults to 12 bits.
     */
    void configureAnalog(BoardVoltage boardVoltage, uint8_t readResolution);
}

#endif /* DMCOMM_H_ */
