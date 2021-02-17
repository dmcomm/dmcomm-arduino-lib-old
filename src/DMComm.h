
#ifndef DMCOMM_H_
#define DMCOMM_H_

class DMComm {

public:
    
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
     * @param command a null-terminated byte string containing the command.
     */
    int8_t execute(uint8_t command[]);
    
}

#endif /* DMCOMM_H_ */
