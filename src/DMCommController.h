/* This file is part of the DMComm project by BladeSabre. License: MIT. */

#ifndef DMCOMMCONTROLLER_H_
#define DMCOMMCONTROLLER_H_

namespace DMComm {

class Controller {

public:
    /**
     * TODO comment.
     */
    Controller(DComAnalog& prongInterface);

    /**
     * Destructor.
     */
    ~Controller();

    /**
     * Do one iteration of checking the serial for instructions and carrying them out.
     * (Does nothing if no serial is set.)
     */
    void loop();

    /**
     * Carry out the command specified. See the serial codes documentation for details.
     * Communication pattern codes configure the system to prepare for calling `doComm`.
     * Config codes are executed immmediately.
     * @param command a null-terminated byte string containing the command.
     * @return (not decided yet).
     */
    int8_t execute(uint8_t command[]);

    /**
     * Communicate with the toy as specified by the communication pattern code passed to
     * the most recent call of `execute`.
     * @return (not decided yet).
     */
    int8_t doComm();
    
    /**
     * Set a pin for LED output. Initially DMCOMM_NO_PIN, which disables it.
     */
    void setPinLed(uint8_t pinLed);

    /**
      * Specify the serial port for the loop function and for reporting results.
      * If this is not done, the loop function will do nothing and no results will be reported.
      */
    void setSerial(Stream& serial);

private:

    DComAnalog *prongInterface_ = nullptr;
    Stream *serial_ = nullptr;
    ToyProtocol configIndex_ = PROTOCOL_V;
    uint8_t pinLed_ = DMCOMM_NO_PIN;
    uint16_t listenTimeoutTicks_ = 15000;
    uint16_t endedCaptureTicks_ = 2500;
    bool commCommandActive_ = false;
    bool listenOnly_;
    bool goFirst_;
    uint8_t numPackets_;
    uint8_t commandBuffer_[DMCOMM_COMMAND_BUFFER_SIZE];

    /**
     * Print number onto serial as hex,
     * with specified number of digits up to 4 (with leading zeros).
     * If too few digits to display that number, will take the least significant.
     * (Does nothing if no serial is set.)
     */
    void serialPrintHex(uint16_t number, uint8_t numDigits);

    /**
     * Try to read from serial into command buffer.
     * Read until end-of-line and replace that with a null terminator.
     * Should only be called if serial is present.
     * @return 0 on failure, or a positive integer for the number of characters read.
     */
    uint8_t readCommand();

    /**
     * Turn the LED on if the pin is set, otherwise do nothing.
     */
    void ledOn();

    /**
     * Turn the LED off if the pin is set, otherwise do nothing.
     */
    void ledOff();

    /**
     * Just listen for sequences of incoming messages,
     * e.g. if only the first packet is wanted, or listening to 2 toys.
     */
    void commListen();

    /**
     * Interact with the toy on the other end of the connection.
     */
    void commBasic();
};

} /* namespace DMComm */

#endif /* DMCOMMCONTROLLER_H_ */
