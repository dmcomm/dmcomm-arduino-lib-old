/* This file is part of the DMComm project by BladeSabre. License: MIT. */

#ifndef DMCOMM_H_
#define DMCOMM_H_

#define DMCOMM_NO_PIN 0xFF
#define DMCOMM_TICK_MICROS 200
#define DMCOMM_COMMAND_BUFFER_SIZE 64
#define DMCOMM_SERIAL_TIMEOUT_MILLIS 6000
#define DMCOMM_GOFIRST_REPEAT_MILLIS 5000
#define DMCOMM_INACTIVE_DELAY_MILLIS 3000

#define DMCOMM_LOG_PREFIX_LOW    0b00000000
#define DMCOMM_LOG_PREFIX_HIGH   0b01000000
#define DMCOMM_LOG_PREFIX_AGAIN  0b10000000
#define DMCOMM_LOG_PREFIX_OTHER  0b11000000
#define DMCOMM_LOG_MAX_COUNT     0b00111111
#define DMCOMM_LOG_COUNT_BITS             6
#define DMCOMM_LOG_PREFIX_TICK_OVERRUN 0xF0
#define DMCOMM_LOG_MAX_TICKS_MISSED    0x0F

#define DMCOMM_LOG_OPP_ENTER_WAIT      0xC0
#define DMCOMM_LOG_OPP_INIT_PULLDOWN   0xC1
#define DMCOMM_LOG_OPP_START_BIT_HIGH  0xC2
#define DMCOMM_LOG_OPP_START_BIT_LOW   0xC3
#define DMCOMM_LOG_OPP_BITS_BEGIN_HIGH 0xC4
#define DMCOMM_LOG_OPP_GOT_BIT_0       0xC5
#define DMCOMM_LOG_OPP_GOT_BIT_1       0xC6
#define DMCOMM_LOG_OPP_EXIT_OK         0xC8
#define DMCOMM_LOG_OPP_EXIT_FAIL       0xC9

#define DMCOMM_LOG_SELF_ENTER_DELAY    0xE0
#define DMCOMM_LOG_SELF_INIT_PULLDOWN  0xE1
#define DMCOMM_LOG_SELF_START_BIT_HIGH 0xE2
#define DMCOMM_LOG_SELF_START_BIT_LOW  0xE3
#define DMCOMM_LOG_SELF_SEND_BIT_0     0xE5
#define DMCOMM_LOG_SELF_SEND_BIT_1     0xE6
#define DMCOMM_LOG_SELF_RELEASE        0xE7

namespace DMComm {

/**
 * The board voltage. Fast division is used, so only pre-programmed voltages are available.
 */
enum BoardVoltage {BOARD_5V, BOARD_3V3};

/**
 * The debug mode: off, digital or analog.
 */
enum DebugMode {DEBUG_OFF, DEBUG_DIGITAL, DEBUG_ANALOG};

/**
 * The low-level protocol for communicating with the toy.
 * V for 2-prong, X for 3-prong, Y for Xros Mini.
 */
enum ToyProtocol {PROTOCOL_V = 0, PROTOCOL_X = 1, PROTOCOL_Y = 2};

/**
 * TODO comment.
 */
void pinModeMaybe(uint8_t pin, uint8_t mode);

/**
 * TODO comment.
 */
void digitalWriteMaybe(uint8_t pin, uint8_t val);

/**
 * Return integer value of hex digit character, or -1 if not a hex digit.
 */
int8_t hex2val(int8_t hexdigit);

/**
 * Return uppercase hex digit character for lowest 4 bits of input byte.
 */
int8_t val2hex(int8_t value);

} /* namespace DMComm */

#include "DComAnalog.h"
#include "DMCommController.h"

#endif /* DMCOMM_H_ */
