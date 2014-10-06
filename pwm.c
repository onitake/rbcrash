/*
 * Rainbow Crash is Copyright (c) 2012-2014 Gregor Riepl <onitake@gmail.com>
 * All rights reserved.
 *
 * The project, all circuit designs, circuit board layouts, program code and
 * other associated documents are provided to you under the following conditions:
 *
 *     Redistributions of source code, drawings, parts lists and any other
 *     documents required for creating derivative works must retain the above
 *     copyright notice, this list of conditions and the following disclaimer.
 *
 *     Redistributions in binary form, printable vector graphics, images and
 *     any documents required for direct reproduction must reproduce the above
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the distribution.
 *
 *     Modified PCB layouts must use a logo or other marking that is clearly
 *     distinct from the design included in the original version, including
 *     board revision and/or copyright marker.
 *
 * THIS PROJECT IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "clocks.h"

#if CLOCK_DIV >= 256 && CLOCK_DIV % 256 == 0
#define CLOCK0_DIV (CLOCK_DIV / 256)
#define CLOCK1_DIV (CLOCK_DIV / 256)
#define CLOCK_COUNT 1
#else
#error Invalid clock divider for IR decoder, an integer multiple of 256 is required
#endif

#ifdef F_IR_MIN
#if F_CPU / CLOCK0_DIV / 256 < F_IR_MIN
#error PWM frequency is too small, decrease the divider
#endif
#endif
#ifdef F_IR_MAX
#if F_CPU / CLOCK0_DIV / 256 > F_IR_MAX
#error PWM frequency is too big, increase the divider
#endif
#endif

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)

#if (CLOCK0_DIV == 0)
#define CLOCK0_CSX 0
#elif (CLOCK0_DIV == 1)
#define CLOCK0_CSX _BV(CS00)
#elif (CLOCK0_DIV == 8)
#define CLOCK0_CSX _BV(CS01)
#elif (CLOCK0_DIV == 64)
#define CLOCK0_CSX (_BV(CS00) | _BV(CS01))
#elif (CLOCK0_DIV == 256)
#define CLOCK0_CSX _BV(CS02)
#elif (CLOCK0_DIV == 1024)
#define CLOCK0_CSX (_BV(CS00) | _BV(CS02))
#else
#error Unsupported divider for timer 0
#endif

#define CLOCK0_TCCRA TCCR0A
#define CLOCK0_TCCRB TCCR0B
#define CLOCK0_TIMSK TIMSK
#define CLOCK0_TCNT TCNT0
#define CLOCK0_REGA (_BV(WGM00) | _BV(WGM01) | _BV(COM0A0) | _BV(COM0A1) | _BV(COM0B0) | _BV(COM0B1))
#define CLOCK0_REGB CLOCK0_CSX
#define CLOCK0_INTS _BV(TOIE0)
#define CLOCK0_OVERFLOW_INTVECT TIMER0_OVF_vect

#if (CLOCK1_DIV == 0)
#define CLOCK1_CSX 0
#elif (CLOCK1_DIV == 1)
#define CLOCK1_CSX _BV(CS10)
#elif (CLOCK1_DIV == 2)
#define CLOCK1_CSX _BV(CS11)
#elif (CLOCK1_DIV == 4)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS11))
#elif (CLOCK1_DIV == 8)
#define CLOCK1_CSX _BV(CS12)
#elif (CLOCK1_DIV == 16)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS12))
#elif (CLOCK1_DIV == 32)
#define CLOCK1_CSX (_BV(CS11) | _BV(CS12))
#elif (CLOCK1_DIV == 64)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS11) | _BV(CS12))
#elif (CLOCK1_DIV == 128)
#define CLOCK1_CSX _BV(CS13)
#elif (CLOCK1_DIV == 256)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS13))
#elif (CLOCK1_DIV == 512)
#define CLOCK1_CSX (_BV(CS11) | _BV(CS13))
#elif (CLOCK1_DIV == 1024)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS11) | _BV(CS13))
#elif (CLOCK1_DIV == 2048)
#define CLOCK1_CSX (_BV(CS12) | _BV(CS13))
#elif (CLOCK1_DIV == 4096)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS12) | _BV(CS13))
#elif (CLOCK1_DIV == 8192)
#define CLOCK1_CSX (_BV(CS11) | _BV(CS12) | _BV(CS13))
#elif (CLOCK1_DIV == 16384)
#define CLOCK1_CSX (_BV(CS10) | _BV(CS11) | _BV(CS12) | _BV(CS13))
#else
#error Unsupported divider for timer 1
#endif

#define CLOCK1_TCCRA TCCR1
#define CLOCK1_TCCRB GTCCR
#define CLOCK1_TIMSK TIMSK
#define CLOCK1_TCNT TCNT1
#define CLOCK1_REGA CLOCK1_CSX
#define CLOCK1_REGB (_BV(PWM1B) | _BV(COM1B0))
#define CLOCK1_INTS 0
#define CLOCK1_OVERFLOW_INTVECT TIMER2_OVF_vect

#define PWM_OCR_RED OCR0A
#define PWM_OCR_GREEN OCR0B
#define PWM_OCR_BLUE OCR1B
#define PWM_PORT PORTB
#define PWM_DDR DDRB
#define PWM_PIN PINB
#define PWM_MASK (_BV(0) | _BV(1) | _BV(4))
#define PWM_INV_RED 1
#define PWM_INV_GREEN 1
#define PWM_INV_BLUE 0

#define IR_PIN PINB
#define IR_DDR DDRB
#define IR_PORT PORTB
#define IR_BIT _BV(DDB2)
#define IR_INT _BV(PCINT2)
#define IR_PCMSK PCMSK
#define IR_GIMSK GIMSK
#define IR_PCIE _BV(PCIE)
#define IR_INTVECT PCINT0_vect

#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)

#if (CLOCK0_DIV == 0)
#define CLOCK0_CSX 0
#elif (CLOCK0_DIV == 1)
#define CLOCK0_CSX _BV(CS00)
#elif (CLOCK0_DIV == 8)
#define CLOCK0_CSX _BV(CS01)
#elif (CLOCK0_DIV == 64)
#define CLOCK0_CSX (_BV(CS00) | _BV(CS01))
#elif (CLOCK0_DIV == 256)
#define CLOCK0_CSX _BV(CS02)
#elif (CLOCK0_DIV == 1024)
#define CLOCK0_CSX (_BV(CS00) | _BV(CS02))
#else
#error Unsupported divider for timer 0
#endif

#define CLOCK0_TCCRA TCCR0A
#define CLOCK0_TCCRB TCCR0B
#define CLOCK0_TIMSK TIMSK0
#define CLOCK0_TCNT TCNT0
#define CLOCK0_REGA (_BV(WGM00) | _BV(WGM01) | _BV(COM0A0) | _BV(COM0A1) | _BV(COM0B0) | _BV(COM0B1))
#define CLOCK0_REGB CLOCK0_CSX
#define CLOCK0_INTS _BV(TOIE0)
#define CLOCK0_OVERFLOW_INTVECT TIMER0_OVF_vect

#if (CLOCK1_DIV == 0)
#define CLOCK1_CSX 0
#elif (CLOCK1_DIV == 1)
#define CLOCK1_CSX _BV(CS20)
#elif (CLOCK1_DIV == 8)
#define CLOCK1_CSX _BV(CS21)
#elif (CLOCK1_DIV == 32)
#define CLOCK1_CSX (_BV(CS20) | _BV(CS21))
#elif (CLOCK1_DIV == 64)
#define CLOCK1_CSX _BV(CS22)
#elif (CLOCK1_DIV == 128)
#define CLOCK1_CSX (_BV(CS20) | _BV(CS22))
#elif (CLOCK1_DIV == 256)
#define CLOCK1_CSX (_BV(CS21) | _BV(CS22))
#elif (CLOCK1_DIV == 1024)
#define CLOCK1_CSX (_BV(CS20) | _BV(CS21) | _BV(CS22))
#else
#error Unsupported divider for timer 1
#endif

#define CLOCK1_TCCRA TCCR2A
#define CLOCK1_TCCRB TCCR2B
#define CLOCK1_TIMSK TIMSK2
#define CLOCK1_TCNT TCNT2
#define CLOCK1_REGA (_BV(WGM20) | _BV(WGM21) | _BV(COM2B0) | _BV(COM2B1))
#define CLOCK1_REGB CLOCK1_CSX
#define CLOCK1_INTS 0
#define CLOCK1_OVERFLOW_INTVECT TIMER2_OVF_vect

#define PWM_OCR_RED OCR2B
#define PWM_OCR_GREEN OCR0A
#define PWM_OCR_BLUE OCR0B
#define PWM_PORT PORTD
#define PWM_DDR DDRD
#define PWM_PIN PIND
#define PWM_MASK (_BV(3) | _BV(5) | _BV(6))
#define PWM_INV_RED 1
#define PWM_INV_GREEN 1
#define PWM_INV_BLUE 1

#define IR_PIN PIND
#define IR_DDR DDRD
#define IR_PORT PORTD
#define IR_BIT _BV(DDD2)
#define IR_INT _BV(PCINT18)
#define IR_PCMSK PCMSK2
#define IR_GIMSK PCICR
#define IR_PCIE _BV(PCIE2)
#define IR_INTVECT PCINT2_vect

#else
#error Unknown architecture, please add support for your MCU in pwm.c
#endif

#define COLOR_RED_MAX 32
#define COLOR_GREEN_MAX 32
#define COLOR_BLUE_MAX 32

#define COLOR_RED color_make(255, 0, 0)
#define COLOR_GREEN color_make(0, 255, 0)
#define COLOR_BLUE color_make(0, 0, 255)
#define COLOR_YELLOW color_make(255, 255, 0)
#define COLOR_CYAN color_make(0, 255, 255)
#define COLOR_MAGENTA color_make(255, 0, 255)
#define COLOR_BLACK color_make(0, 0, 0)
#define COLOR_WHITE color_make(255, 255, 255)

#define CHANGE_STATE(stat, clk) { global.state = stat; }
#define CLOCK_MATCH(symbol, diff) (CLOCK_##symbol##_MIN <= diff && diff < CLOCK_##symbol##_MAX)
#define DEBUG_ERROR(err, cod, clk)

#if CLOCK_16BIT
typedef uint16_t clock_t;
#else
typedef uint8_t clock_t;
#endif

typedef struct {
	uint8_t r, g, b;
} color_t;

enum {
	// No code received
	IR_CODE_NONE,
	// Sony IR codes (SIRC)
	IR_CODE_SONY12,
	IR_CODE_SONY15,
	IR_CODE_SONY20,
	// Philips IR code (RC5)
	IR_CODE_RC5,
	// NEC IR code
	IR_CODE_NEC,
#if ENABLE_DEBUG
	// Errors (only generated in debug mode)
	// Unknown/invalid pulse (clk contains the pulse length)
	IR_ERROR_PULSE,
	// Checksum mismatch or other transmission error
	IR_ERROR_TRANSMISSION,
	// Glitch detected
	IR_ERROR_GLITCH,
	// Unimplemented decoding routine
	IR_ERROR_UNIMPLEMENTED,
	// Invalid state (i.e. repeat without corresponding code to repeat)
	IR_ERROR_STATE,
	// Interrupt event overrun
	IR_ERROR_OVERRUN,
	// Debug events
	// Report state change (state contains the old and new states)
	IR_DEBUG_STATE,
#endif
};

typedef struct {
	uint8_t type;
	union {
#if ENABLE_SIRC
		struct {
			uint8_t code;
			uint8_t device;
			uint8_t extend;
		} sony;
#endif
		uint8_t rc5;
#if ENABLE_NEC
		struct {
			uint8_t device;
			uint8_t code;
			uint8_t repeat:1;
			uint8_t verified:1;
		} nec;
#endif
#if 0 && ENABLE_DEBUG
		struct {
			uint16_t code;
			clock_t clock;
			uint16_t line;
		} error;
		struct {
			uint8_t old;
			uint8_t new;
			clock_t clock;
			uint16_t line;
		} state;
#endif
	} code;
} irevent_t;

// State machine
enum {
	IR_STATE_IDLE,
	IR_STATE_START,
	IR_STATE_SIRC_START_ACK,
	IR_STATE_SIRC_BIT,
	IR_STATE_SIRC_ACK,
	IR_STATE_NEC_TYPE,
	IR_STATE_NEC_CMD_ACK,
	IR_STATE_NEC_RPT_ACK,
	IR_STATE_NEC_BIT,
	IR_STATE_NEC_SHORT_ACK,
	IR_STATE_NEC_LONG_ACK,
};

// Interrupt handler states
enum {
	IR_INTSTATE_IDLE,
	IR_INTSTATE_L_TO_H,
	IR_INTSTATE_H_TO_L,
	IR_INTSTATE_GLITCH,
	IR_INTSTATE_OVERRUN,
	IR_INTSTATE_DONE,
	IR_INTSTATE_SIRC_DONE,
	IR_INTSTATE_TIMEOUT,
};

const PROGMEM uint8_t PWM_LOOKUP[32] = { 0, 1, 4, 7, 11, 15, 20, 26, 32, 38, 44, 51, 58, 66, 74, 82, 90, 99, 108, 117, 126, 136, 145, 155, 166, 176, 187, 198, 209, 220, 232, 244, };

static volatile struct {
	// Current wall clock in tPWM/256 units
	clock_t clock;
	// Last port level (0 = L, 1 = H)
	uint8_t bitstate;
	// Interrupt handler state
	uint8_t intstate;
	// Flag telling if a bit change or glitch event is pending
	uint8_t pending;
	// Timestamp of previous interrupt event
	clock_t prevclock;
	// Timestamp of last interrupt event
	clock_t lastclock;
	// State machine state
	uint8_t state;
	// Detected code type (or IR_CODE_NONE if no code stored)
	uint8_t type;
	// Shift register
	uint32_t word;
	// Number of bits in shift register
	uint8_t bitcount;
	// Data store (for repeats)
	union {
		uint32_t init;
		struct {
			uint8_t device;
			uint8_t code;
			uint8_t verified:1;
		} nec;
	} data;
} global __attribute__((section(".noinit")));

static clock_t clock_now();

ISR(CLOCK0_OVERFLOW_INTVECT) {
	global.clock++;
}

ISR(IR_INTVECT) {
	// Read IR pin state first to avoid additional glitches
	uint8_t bit = IR_PIN & IR_BIT;
	
	// Interrupt glitch detection
	if (bit != global.bitstate) {
		// Record the bit change
		global.bitstate = bit;
		
		// Check if an event is already pending
		if (!global.pending) {
			// Copy the last clock event to the previous one
			global.prevclock = global.lastclock;
			// Get the current time
			global.lastclock = clock_now();
			
			// Post the event
			global.intstate = bit ? IR_INTSTATE_L_TO_H : IR_INTSTATE_H_TO_L;
		} else {
			// Report an overrun
			global.intstate = IR_INTSTATE_OVERRUN;
		}
	} else {
		// Check if an event is already pending
		if (!global.pending) {
			// Report a glitch
			global.intstate = IR_INTSTATE_GLITCH;
		} else {
			// Report an overrun
			global.intstate = IR_INTSTATE_OVERRUN;
		}
	}
	
	// Signal the new event
	global.pending = 1;
}

static void clock_init() {
	global.clock = 0;
	CLOCK0_TCNT = 0;
	CLOCK1_TCNT = 0;
#ifdef CLOCK0_TOP
	CLOCK0_TOP = 255;
#endif
#ifdef CLOCK1_TOP
	CLOCK1_TOP = 255;
#endif
	CLOCK0_TIMSK |= CLOCK0_INTS;
	CLOCK1_TIMSK |= CLOCK1_INTS;
	CLOCK0_TCCRA = CLOCK0_REGA;
	CLOCK0_TCCRB = CLOCK0_REGB;
	CLOCK1_TCCRA = CLOCK1_REGA;
	CLOCK1_TCCRB = CLOCK1_REGB;
}

static clock_t clock_now() {
	return global.clock / CLOCK_COUNT;
}

static color_t color_make(uint8_t r, uint8_t g, uint8_t b) {
	color_t c = { r, g, b };
	return c;
}

static void color_set(color_t unscaled) {
	color_t c = color_make(unscaled.r < COLOR_RED_MAX ? pgm_read_byte(&PWM_LOOKUP[unscaled.r]) : 255, unscaled.g < COLOR_GREEN_MAX ? pgm_read_byte(&PWM_LOOKUP[unscaled.g]) : 255, unscaled.b < COLOR_BLUE_MAX ? pgm_read_byte(&PWM_LOOKUP[unscaled.b]) : 255);
#if PWM_INV_RED
	PWM_OCR_RED = ~c.r;
#else
	PWM_OCR_RED = c.r;
#endif
#if PWM_INV_GREEN
	PWM_OCR_GREEN = ~c.g;
#else
	PWM_OCR_GREEN = c.g;
#endif
#if PWM_INV_BLUE
	PWM_OCR_BLUE = ~c.b;
#else
	PWM_OCR_BLUE = c.b;
#endif
}

static color_t color_get() {
	color_t ret;
#if PWM_INV_RED
	ret.r = ~PWM_OCR_RED;
#else
	ret.r = PWM_OCR_RED;
#endif
#if PWM_INV_GREEN
	ret.g = ~PWM_OCR_GREEN;
#else
	ret.g = PWM_OCR_GREEN;
#endif
#if PWM_INV_BLUE
	ret.b = ~PWM_OCR_BLUE;
#else
	ret.b = PWM_OCR_BLUE;
#endif
	return ret;
}

static void color_init() {
	color_set(COLOR_BLACK);
	PWM_PORT &= ~PWM_MASK;
	PWM_DDR |= PWM_MASK;
}

static void ir_init() {
	IR_DDR &= ~IR_BIT;
	IR_PORT |= IR_BIT;
	global.pending = 0;
	global.intstate = IR_INTSTATE_IDLE;
	global.lastclock = clock_now();
	global.prevclock = global.lastclock;
	// Initialize code detector state
	CHANGE_STATE(IR_STATE_IDLE, 0);
	global.type = IR_CODE_NONE;
	global.word = 0;
	global.bitcount = 0;
	global.data.init = 0;
	global.bitstate = IR_PIN & IR_BIT;
	IR_PCMSK |= IR_INT;
	IR_GIMSK |= IR_PCIE;
}

irevent_t ir_handle() {
	// Check if we have an event pending
	uint8_t pending;
	uint8_t intstate;
	clock_t clockdiff;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		pending = global.pending;
		intstate = global.intstate;
		global.pending = 0;
		global.intstate = IR_INTSTATE_IDLE;
		clockdiff = global.lastclock - global.prevclock;
	}
	if (pending) {
		/* Debug */ //{ irevent_t ev = { .type = IR_CODE_NEC, .code = { .nec = { .device = (uint8_t) (clockdiff >> 8), .code = (uint8_t) clockdiff } } }; return ev; }
		
		// Determine the type of interrupt event
		switch (intstate) {
			case IR_INTSTATE_IDLE:
				// Ignore
				break;
			case IR_INTSTATE_L_TO_H:
			case IR_INTSTATE_H_TO_L:
				switch (global.state) {
					case IR_STATE_IDLE:
						CHANGE_STATE(IR_STATE_START, clockdiff);
						break;
					case IR_STATE_START:
#if ENABLE_NEC
						if (CLOCK_MATCH(NEC_START, clockdiff)) {
							CHANGE_STATE(IR_STATE_NEC_TYPE, clockdiff);
						} else
#endif
#if ENABLE_SIRC
						if (CLOCK_MATCH(SIRC_START, clockdiff)) {
							CHANGE_STATE(IR_STATE_SIRC_START_ACK, clockdiff);
						} else
#endif
						{
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
#if ENABLE_NEC
					case IR_STATE_NEC_TYPE:
						if (CLOCK_MATCH(NEC_MODE_CMD, clockdiff)) {
							CHANGE_STATE(IR_STATE_NEC_CMD_ACK, clockdiff);
						} else if (CLOCK_MATCH(NEC_MODE_REP, clockdiff)) {
							CHANGE_STATE(IR_STATE_NEC_RPT_ACK, clockdiff);
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_NEC_CMD_ACK:
						if (CLOCK_MATCH(NEC_ACK, clockdiff)) {
							global.word = 0;
							global.bitcount = 0;
							global.type = IR_CODE_NONE;
							CHANGE_STATE(IR_STATE_NEC_BIT, clockdiff);
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_NEC_RPT_ACK:
						if (CLOCK_MATCH(NEC_ACK, clockdiff)) {
							if (global.type == IR_CODE_NEC) {
								CHANGE_STATE(IR_STATE_IDLE, clockdiff);
								{ irevent_t ev = { .type = IR_CODE_NEC, .code = { .nec = { .repeat = 1, .verified = global.data.nec.verified, .device = global.data.nec.device, .code = global.data.nec.code } } }; return ev; }
							} else {
								CHANGE_STATE(IR_STATE_IDLE, clockdiff);
								DEBUG_ERROR(IR_ERROR_STATE, IR_STATE_NEC_RPT_ACK, clockdiff);
							}
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_NEC_BIT:
						if (CLOCK_MATCH(NEC_LONG, clockdiff)) {
							CHANGE_STATE(IR_STATE_NEC_LONG_ACK, clockdiff);
						} else if (CLOCK_MATCH(NEC_SHORT, clockdiff)) {
							CHANGE_STATE(IR_STATE_NEC_SHORT_ACK, clockdiff);
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_NEC_LONG_ACK:
						if (CLOCK_MATCH(NEC_ACK, clockdiff)) {
							// Store 1
							global.word = (global.word << 1) | 1;
							global.bitcount++;
							// Check for completion
							if (global.bitcount >= 32) {
								global.type = IR_CODE_NEC;
								global.data.nec.device = global.word >> 24;
								global.data.nec.code = global.word >> 8;
#if VERIFY_CHECKSUM
								// Evaluate checksum
								if ((uint8_t) (global.word >> 24) == (uint8_t) ~(global.word >> 16) && (uint8_t) (global.word >> 8) == (uint8_t) ~global.word) {
									global.data.nec.verified = 1;
								}
#endif
								CHANGE_STATE(IR_STATE_IDLE, clockdiff);
								{ irevent_t ev = { .type = IR_CODE_NEC, .code = { .nec = { .repeat = 0, .verified = global.data.nec.verified, .device = global.data.nec.device, .code = global.data.nec.code } } }; return ev; }
							} else {
								CHANGE_STATE(IR_STATE_NEC_BIT, clockdiff);
							}
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_NEC_SHORT_ACK:
						if (CLOCK_MATCH(NEC_ACK, clockdiff)) {
							// Store 0
							global.word = global.word << 1;
							global.bitcount++;
							// Check for completion
							if (global.bitcount >= 32) {
								global.type = IR_CODE_NEC;
								global.data.nec.device = global.word >> 24;
								global.data.nec.code = global.word >> 8;
#if VERIFY_CHECKSUM
								// Evaluate checksum
								if ((uint8_t) (global.word >> 24) == (uint8_t) ~(global.word >> 16) && (uint8_t) (global.word >> 8) == (uint8_t) ~global.word) {
									global.data.nec.verified = 1;
								}
#endif
								CHANGE_STATE(IR_STATE_IDLE, clockdiff);
								{ irevent_t ev = { .type = IR_CODE_NEC, .code = { .nec = { .repeat = 0, .verified = global.data.nec.verified, .device = global.data.nec.device, .code = global.data.nec.code } } }; return ev; }
							} else {
								CHANGE_STATE(IR_STATE_NEC_BIT, clockdiff);
							}
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
#endif
#if ENABLE_SIRC
					case IR_STATE_SIRC_START_ACK:
						if (CLOCK_MATCH(SIRC_ACK, clockdiff)) {
							global.word = 0;
							global.bitcount = 0;
							global.type = IR_CODE_NONE;
							CHANGE_STATE(IR_STATE_SIRC_BIT, clockdiff);
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_SIRC_BIT:
						if (CLOCK_MATCH(SIRC_LONG, clockdiff)) {
							// Store 1
							global.word = (global.word >> 1) | 0x80000000;
							global.bitcount++;
							CHANGE_STATE(IR_STATE_SIRC_ACK, clockdiff);
						} else if (CLOCK_MATCH(SIRC_SHORT, clockdiff)) {
							// Store 0
							global.word = global.word >> 1;
							global.bitcount++;
							CHANGE_STATE(IR_STATE_SIRC_ACK, clockdiff);
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
					case IR_STATE_SIRC_ACK:
						clock_notimeout();
						if (CLOCK_MATCH(SIRC_ACK, clockdiff)) {
							CHANGE_STATE(IR_STATE_SIRC_BIT, clockdiff);
						} else {
							CHANGE_STATE(IR_STATE_IDLE, clockdiff);
							DEBUG_ERROR(IR_ERROR_PULSE, 0, clockdiff);
						}
						break;
#endif
					default:
						// Unknown previous state
						CHANGE_STATE(IR_STATE_IDLE, clockdiff);
						DEBUG_ERROR(IR_ERROR_STATE, global.state, clockdiff);
						break;
				}
				break;
#if ENABLE_SIRC
			case IR_INTSTATE_SIRC_DONE:
				// SIRC transmission complete
				CHANGE_STATE(IR_STATE_IDLE, clockdiff);
				// Check data length
				if (global.bitcount == 12) {
					irevent_t ev = { .type = IR_CODE_SONY12, .code = { .sony = { .device = global.word >> 27, .code = (global.word >> 20) & 0x7f, .extend = 0 } } };
					return ev;
				} else if (global.bitcount == 15) {
					irevent_t ev = { .type = IR_CODE_SONY15, .code = { .sony = { .device = global.word >> 24, .code = (global.word >> 17) & 0x7f, .extend = 0 } } };
					return ev;
				} else if (global.bitcount == 20) {
					irevent_t ev = { .type = IR_CODE_SONY20, .code = { .sony = { .extend = global.word >> 24, .device = (global.word >> 19) & 0x1f, .code = (ir_state.word >> 12) & 0x7f } } };
					return ev;
				} else {
					DEBUG_ERROR(IR_ERROR_TRANSMISSION, global.bitcount, clockdiff);
				}
				break;
#endif
			case IR_INTSTATE_GLITCH:
				CHANGE_STATE(IR_STATE_IDLE, clockdiff);
				DEBUG_ERROR(IR_ERROR_GLITCH, 0, 0);
				break;
			case IR_INTSTATE_OVERRUN:
				CHANGE_STATE(IR_STATE_IDLE, clockdiff);
				DEBUG_ERROR(IR_ERROR_OVERRUN, 0, 0);
				break;
			case IR_INTSTATE_TIMEOUT:
				CHANGE_STATE(IR_STATE_IDLE, clockdiff);
				DEBUG_ERROR(IR_ERROR_STATE, global.state, clockdiff);
				break;
		}
	}
	
	// No event handled or an error occured
	irevent_t ev = { .type = IR_CODE_NONE };
	return ev;
}

#if ENABLE_DEBUG

static uint8_t serial_rx_done() {
	return UCSR0A & _BV(RXC0);
}

static uint8_t serial_tx_ready() {
	return UCSR0A & _BV(UDRE0);
}

static void serial_write(char chr) {
	while (!serial_tx_ready());
	UDR0 = chr;
}

static char serial_read() {
	while (!serial_rx_done());
	return UDR0;
}

static int serial_putchar(char c, FILE *stream) {
	if (c == '\n') {
		serial_write('\r');
	}
	serial_write(c);
	return 0;
}

static FILE serial_stdout = FDEV_SETUP_STREAM(serial_putchar, NULL, _FDEV_SETUP_WRITE);

static void serial_init(uint16_t rate, uint8_t bits, uint8_t stop, uint8_t parity) {
	// Set baud rate
	uint16_t ubrr = (F_CPU / 16 / rate) - 1;
	UBRR0H = (uint8_t) (ubrr >> 8);
	UBRR0L = (uint8_t) ubrr;
	
	// Set data format
	uint8_t ucsrb = _BV(RXEN0) | _BV(TXEN0);
	uint8_t ucsrc = 0;
	switch (bits) {
		case 5:
			break;
		case 6:
			ucsrc |= _BV(UCSZ00);
			break;
		case 7:
			ucsrc |= _BV(UCSZ01);
			break;
		case 8:
		default:
			ucsrc |= _BV(UCSZ00) | _BV(UCSZ01);
			break;
		case 9:
			ucsrb |= _BV(UCSZ02);
			ucsrc |= _BV(UCSZ00) | _BV(UCSZ01);
			break;
	}
	switch (stop) {
		case 1:
		default:
			break;
		case 2:
			ucsrc |= _BV(USBS0);
			break;
	}
	switch (parity) {
		case 0:
		default:
			break;
		case 2:
			ucsrc |= _BV(UPM00) | _BV(UPM01);
			break;
		case 1:
			ucsrc |= _BV(UPM01);
			break;
	}
	UCSR0A = 0;
	UCSR0C = ucsrc;
	UCSR0B = ucsrb;
	
	// Send init data and wait for sync
	serial_write(0);
	_delay_ms(100);
	
	// Initialize stdout
	stdout = &serial_stdout;
}

#define debug(pformat, ...) printf_P(PSTR(pformat), __VA_ARGS__)

#else

#define debug(pformat, ...)

#endif

int main() {
	color_init();
	clock_init();
	ir_init();
#if ENABLE_DEBUG
	serial_init(38400, 8, 1, 0);
#endif
	
	uint8_t rbon = 0;
	//color_t color = color_make(1.0 * COLOR_RED_MAX, 0.4 * COLOR_GREEN_MAX, 0 * COLOR_BLUE_MAX);
	color_t color = COLOR_BLACK;
	color_set(color);

	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();

	debug("start=%u ack=%u cmd=%u rep=%u long=%u short=%u\n", CLOCK_NEC_START, CLOCK_NEC_ACK, CLOCK_NEC_MODE_CMD, CLOCK_NEC_MODE_REP, CLOCK_NEC_LONG, CLOCK_NEC_SHORT);

	while (1) {
		irevent_t ev = ir_handle();
		if (ev.type == IR_CODE_NEC) {
			debug("nec repeat=%u verified=%u device=%u code=%u\n", ev.code.nec.repeat, ev.code.nec.verified, ev.code.nec.device, ev.code.nec.code);
		}
#if ENABLE_DEBUG
		while (serial_rx_done()) {
			char c = serial_read();
			serial_write(c);
			if (c == '\r') serial_write('\n');
		}
#endif
#if ENABLE_SIRC
		if (ev.type == IR_CODE_SONY20 && ev.code.sony.device == 26) {
			switch (ev.code.sony.code) {
				case 92:
					// Square
					if (color.r < 31) color.r++;
					break;
				case 95:
					// Circle
					if (color.r > 0) color.r--;
					break;
				case 84:
					// Up
					if (color.g < 31) color.g++;
					break;
				case 86:
					// Down
					if (color.g > 0) color.g--;
					break;
				case 93:
					// Triangle
					if (color.b < 31) color.b++;
					break;
				case 94:
					// Cross
					if (color.b > 0) color.b--;
					break;
				case 11:
					// Ok
					rbon = 0;
					break;
				case 50:
					// Play
					rbon = 1;
					break;
			}
			color_set(color);
		}
#endif
#if ENABLE_NEC
		if (ev.type == IR_CODE_NEC && ev.code.nec.device == 225) {
			switch (ev.code.nec.code) {
				case 232:
					// Power
					if (!ev.code.nec.repeat) {
						color = COLOR_BLACK;
					}
					break;
				case 164:
					// Previous
					if (color.r > 0) color.r--;
					break;
				case 36:
					// Next
					if (color.r < COLOR_RED_MAX) color.r++;
					break;
				case 16:
					// Volume Up
					break;
				case 32:
					// Volume Down
					break;
				case 148:
					// Mute
					break;
				case 212:
					// Effect
					if (color.g > 0) color.g--;
					break;
				case 140:
					// Blank
					if (color.b > 0) color.b--;
					break;
				case 88:
					// Video
					if (color.g < COLOR_GREEN_MAX) color.g++;
					break;
				case 152:
					// Computer
					if (color.b < COLOR_BLUE_MAX) color.b++;
					break;
				case 200:
					// Up
					break;
				case 40:
					// Down
					break;
				case 64:
					// Menu
					break;
				case 76:
					// Select
					break;
			}
			color_set(color);
		}
#endif
		sleep_mode();
	}
	return 0;
}
