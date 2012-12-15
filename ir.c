/*
 * Rainbow Crash is Copyright (c) 2012 Gregor Riepl <onitake@gmail.com>
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

#if ENABLE_DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#endif
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ir.h"

// Pulse length definitions in microseconds
#define CLK_ZERO 0
#define CLK_SIRC_START 2440
#define CLK_SIRC_ACK 520
#define CLK_SIRC_LONG 1280
#define CLK_SIRC_SHORT 720
#define CLK_NEC_START 9000
#define CLK_NEC_ACK 480
#define CLK_NEC_MODE_CMD 4600
#define CLK_NEC_MODE_REP 2300
#define CLK_NEC_LONG 1760
#define CLK_NEC_SHORT 640

// Assign the shortest/longest pulse lengths here
#define CLK_DIV_MIN CLK_NEC_ACK
#define CLK_DIV_MAX CLK_NEC_START

// Clock recognition range (+/- percent)
#define CLK_EPSILON 20

// Size of event queue (a power of 2 is highly recommended, and keep RAM size in mind!)
#define IR_EVENT_QUEUE_SIZE 16

// Pin mask bits and port assignments
#if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny13A__)
#define IR_PIN PINB
#define IR_DDR DDRB
#define IR_PORT PORTB
#define IR0_BIT _BV(DDB4)
#define IR0_INT _BV(PCINT4)
#define IR_PCMSK PCMSK
#define IR_GIMSK GIMSK
#define IR_PCIE _BV(PCIE)
#define IR_INTVECT PCINT0_vect
#define TCCRA TCCR0A
#define TCCRB TCCR0B
#if (CLOCK_DIV == 0)
#define CSX 0
#elif (CLOCK_DIV == 1)
#define CSX _BV(CS00)
#elif (CLOCK_DIV == 8)
#define CSX _BV(CS01)
#elif (CLOCK_DIV == 64)
#define CSX _BV(CS00) | _BV(CS01)
#elif (CLOCK_DIV == 256)
#define CSX _BV(CS02)
#elif (CLOCK_DIV == 1024)
#define CSX _BV(CS20) | _BV(CS22)
#endif
#define TIMSK TIMSK0
#define TOIE _BV(TOIE0)
#define TCNT TCNT0
#define TIMER_INTVECT TIM0_OVF_vect
#elif defined(__AVR_ATtiny25__)
#define IR_PIN PINB
#define IR_DDR DDRB
#define IR_PORT PORTB
#define IR0_BIT _BV(DDB4)
#define IR0_INT _BV(PCINT4)
#define IR_PCMSK PCMSK
#define IR_GIMSK GIMSK
#define IR_PCIE _BV(PCIE)
#define IR_INTVECT PCINT0_vect
#define TCCRA TCCR0A
#define TCCRB TCCR0B
#if (CLOCK_DIV == 0)
#define CSX 0
#elif (CLOCK_DIV == 1)
#define CSX _BV(CS00)
#elif (CLOCK_DIV == 8)
#define CSX _BV(CS01)
#elif (CLOCK_DIV == 64)
#define CSX _BV(CS00) | _BV(CS01)
#elif (CLOCK_DIV == 256)
#define CSX _BV(CS02)
#elif (CLOCK_DIV == 1024)
#define CSX _BV(CS00) | _BV(CS02)
#endif
#define TOIE _BV(TOIE0)
#define TCNT TCNT0
#define TIMER_INTVECT TIM0_OVF_vect
#elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__)
#define IR_PIN PIND
#define IR_DDR DDRD
#define IR_PORT PORTD
#define IR0_BIT _BV(DDD3)
#define IR0_INT _BV(PCINT27)
#define IR_PCMSK PCMSK3
#define IR_GIMSK PCICR
#define IR_PCIE _BV(PCIE3)
#define IR_INTVECT PCINT3_vect
#define TCCRA TCCR0A
#define TCCRB TCCR0B
#if (CLOCK_DIV == 0)
#define CSX 0
#elif (CLOCK_DIV == 1)
#define CSX _BV(CS00)
#elif (CLOCK_DIV == 8)
#define CSX _BV(CS01)
#elif (CLOCK_DIV == 64)
#define CSX _BV(CS00) | _BV(CS01)
#elif (CLOCK_DIV == 256)
#define CSX _BV(CS02)
#elif (CLOCK_DIV == 1024)
#define CSX _BV(CS20) | _BV(CS22)
#endif
#define TIMSK TIMSK0
#define TOIE _BV(TOIE0)
#define TCNT TCNT0
#define TIMER_INTVECT TIMER0_OVF_vect
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define IR_PIN PIND
#define IR_DDR DDRD
#define IR_PORT PORTD
#define IR0_BIT _BV(DDD2)
#define IR0_INT _BV(PCINT18)
#define IR_PCMSK PCMSK2
#define IR_GIMSK PCICR
#define IR_PCIE _BV(PCIE2)
#define IR_INTVECT PCINT2_vect
#define TCCRA TCCR2A
#define TCCRB TCCR2B
#if (CLOCK_DIV == 0)
#define CSX 0
#elif (CLOCK_DIV == 1)
#define CSX _BV(CS20)
#elif (CLOCK_DIV == 8)
#define CSX _BV(CS21)
#elif (CLOCK_DIV == 32)
#define CSX _BV(CS20) | _BV(CS21)
#elif (CLOCK_DIV == 64)
#define CSX _BV(CS22)
#elif (CLOCK_DIV == 128)
#define CSX _BV(CS20) | _BV(CS22)
#elif (CLOCK_DIV == 256)
#define CSX _BV(CS21) | _BV(CS22)
#elif (CLOCK_DIV == 1024)
#define CSX _BV(CS20) | _BV(CS21) | _BV(CS22)
#endif
#define TIMSK TIMSK2
#define TOIE _BV(TOIE2);
#define TCNT TCNT2
#define TIMER_INTVECT TIMER2_OVF_vect
#endif

// Pulse length to clock conversion macros
#define USEC_TO_CLOCK(usec) (F_CPU / CLOCK_DIV * usec / 1000000)
#define USEC_TO_CLOCK_MIN(usec) (F_CPU / CLOCK_DIV * usec / 1000000 * (100 - CLK_EPSILON) / 100)
#define USEC_TO_CLOCK_MAX(usec) (F_CPU / CLOCK_DIV * usec / 1000000 * (100 + CLK_EPSILON) / 100)
#define USEC_PULSE_MATCH(usec, pulse) (USEC_TO_CLOCK_MIN(usec) <= pulse && pulse <= USEC_TO_CLOCK_MAX(usec))

// Check if the clock precision is sufficient
#if (USEC_TO_CLOCK(CLK_DIV_MIN) < 1)
#warning Clock not precise enough. Please decrease the divider.
#endif
#if CLOCK_16BIT
#if (USEC_TO_CLOCK(CLK_DIV_MAX) > 65535)
#warning Clock too precise. Please increase the divider.
#endif
#else
#if (USEC_TO_CLOCK(CLK_DIV_MAX) > 255)
#warning Clock range insufficient. Please enable 16bit mode.
#endif
#endif

// State/time/event diagram
// H IDLE: ?->START
// L START: SIRC_START->SIRC_START_ACK NEC_START->NEC_TYPE ?->IDLE
// H SIRC_START_ACK: SIRC_ACK->SIRC_BIT ?->IDLE
// L SIRC_BIT: SIRC_SHORT->SIRC_SHORT_ACK SIRC_LONG->SIRC_LONG_ACK ?->IDLE
// H SIRC_SHORT_ACK: SIRC_ACK->SIRC_BIT|IDLE ?->IDLE
// H SIRC_LONG_ACK: SIRC_ACK->SIRC_BIT ?->IDLE
// H NEC_TYPE: NEC_CMD->NEC_CMD_ACK NEC_RPT->NEC_RPT_ACK ?->IDLE
// L NEC_CMD_ACK: NEC_ACK->NEC_BIT ?->IDLE
// L NEC_RPT_ACK: NEC_ACK->NEC_BIT ?->IDLE
// H NEC_BIT: NEC_SHORT->NEC_SHORT_ACK NEC_LONG->NEC_LONG_ACK ?->IDLE
// L NEC_SHORT_ACK: NEC_ACK->NEC_BIT|IDLE ?->IDLE
// L NEC_LONG_ACK: NEC_ACK->NEC_BIT|IDLE ?->IDLE

// incomplete
// L NEC_SHORT_ACK -> NEC_ACK=append(0),count++,if(count==32){done,IDLE}else{NEC_BIT} ?=IDLE
// L NEC_LONG_ACK -> NEC_ACK=append(1),count++,if(count==32){done,IDLE}else{NEC_BIT} ?=IDLE
// H SIRC_SHORT_ACK -> SIRC_ACK=if(count<19){append(0),count++,SIRC_BIT}else{done,IDLE} ?=IDLE
// H SIRC_LONG_ACK -> SIRC_ACK=append(1),count++,SIRC_BIT ?=IDLE
#define IR_STATE_IDLE 0
#define IR_STATE_START 1
#define IR_STATE_SIRC_START_ACK 2
#define IR_STATE_SIRC_BIT 3
#define IR_STATE_SIRC_SHORT_ACK 4
#define IR_STATE_SIRC_LONG_ACK 5
#define IR_STATE_NEC_TYPE 6
#define IR_STATE_NEC_CMD_ACK 7
#define IR_STATE_NEC_RPT_ACK 8
#define IR_STATE_NEC_BIT 9
#define IR_STATE_NEC_SHORT_ACK 10
#define IR_STATE_NEC_LONG_ACK 11

// Enqueue one event
static inline void ir_event_push(IrEvent ev);
// Initialize the clock
static inline void clock_init();
// Get the current clock
static inline clock_t clock_now();

// IR module state
static struct {
	// Last port level (0 = L, 1 = H)
	volatile uint8_t bitstate;
	// Timestamp of last level change
	volatile clock_t lastclock;
	// State machine state
	volatile uint8_t state;
	// Detected code type (or IR_CODE_NONE if no code stored)
	uint8_t type;
	// Shift register
	uint32_t word;
	// Number of bits in shift register
	uint8_t bitcount;
	// Data store (for repeats)
	union {
		uint8_t sony8;
		struct {
			uint8_t device;
			uint8_t code;
		} nec;
	} data;
	// FIFO
	uint8_t in, out;
	IrEvent events[IR_EVENT_QUEUE_SIZE];
} ir_state;

#if ENABLE_DEBUG
#define DEBUG_PULSE_ERROR(clk) { \
	IrEvent ev = { .type = IR_ERROR_PULSE, .code = { .clock = clk } }; \
	ir_event_push(ev); \
}
#define CHANGE_STATE(stat) { \
	IrEvent ev = { .type = IR_DEBUG_STATE, .code = { .state = { .old = ir_state.state, .new = stat, .clock = clock_diff, .line = __LINE__ } } }; \
	ir_event_push(ev); \
	ir_state.state = stat; \
}
#else
#define DEBUG_PULSE_ERROR(clk)
#define CHANGE_STATE(stat) { \
	ir_state.state = stat; \
}
#endif

ISR(IR_INTVECT) {
	// Read IR pin state first to avoid glitches
	uint8_t bit = IR_PIN & IR0_BIT;
	
	// Get the real clock value
	clock_t clock_real = clock_now();
	// Get the delta from the previous clock event
	// Use the absolute value to work around clock wraps
	clock_t clock_diff = abs(clock_real - ir_state.lastclock);

	// Glitch detection
	if (bit != ir_state.bitstate) {
		// Pulse, evaluate and handle event
		switch (ir_state.state) {
		case IR_STATE_IDLE:
			CHANGE_STATE(IR_STATE_START);
			break;
		case IR_STATE_START:
			if (USEC_PULSE_MATCH(CLK_NEC_START, clock_diff)) {
				CHANGE_STATE(IR_STATE_NEC_TYPE);
			} else if (USEC_PULSE_MATCH(CLK_SIRC_START, clock_diff)) {
				CHANGE_STATE(IR_STATE_SIRC_START_ACK);
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_NEC_TYPE:
			if (USEC_PULSE_MATCH(CLK_NEC_MODE_CMD, clock_diff)) {
				CHANGE_STATE(IR_STATE_NEC_CMD_ACK);
			} else if (USEC_PULSE_MATCH(CLK_NEC_MODE_REP, clock_diff)) {
				CHANGE_STATE(IR_STATE_NEC_RPT_ACK);
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_NEC_CMD_ACK:
			if (USEC_PULSE_MATCH(CLK_NEC_ACK, clock_diff)) {
				ir_state.word = 0;
				ir_state.bitcount = 0;
				ir_state.type = IR_CODE_NONE;
				CHANGE_STATE(IR_STATE_NEC_BIT);
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_NEC_RPT_ACK:
			if (USEC_PULSE_MATCH(CLK_NEC_ACK, clock_diff)) {
				if (ir_state.type == IR_CODE_NEC) {
					IrEvent ev = { .type = IR_CODE_NEC, .code = { .nec = { .device = ir_state.data.nec.device, .code = ir_state.data.nec.code } } };
					ir_event_push(ev);
				} else {
#if ENABLE_DEBUG
					IrEvent ev = { .type = IR_ERROR_STATE };
					ir_event_push(ev);
#endif
				}
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
			}
			CHANGE_STATE(IR_STATE_IDLE);
			break;
		case IR_STATE_NEC_BIT:
			if (USEC_PULSE_MATCH(CLK_NEC_LONG, clock_diff)) {
				CHANGE_STATE(IR_STATE_NEC_LONG_ACK);
			} else if (USEC_PULSE_MATCH(CLK_NEC_SHORT, clock_diff)) {
				CHANGE_STATE(IR_STATE_NEC_SHORT_ACK);
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_NEC_LONG_ACK:
			if (USEC_PULSE_MATCH(CLK_NEC_ACK, clock_diff)) {
				// Store 1
				ir_state.word = (ir_state.word << 1) | 1;
				ir_state.bitcount++;
				// Check for completion
				if (ir_state.bitcount >= 32) {
					ir_state.data.nec.device = ir_state.word >> 24;
					ir_state.data.nec.code = ir_state.word >> 8;
					// Evaluate checksum
					if ((uint8_t) (ir_state.word >> 24) == (uint8_t) ~(ir_state.word >> 16) && (uint8_t) (ir_state.word >> 8) == (uint8_t) ~ir_state.word) {
						IrEvent ev = { .type = IR_CODE_NEC, .code = { .nec = { .device = ir_state.data.nec.device, .code = ir_state.data.nec.code } } };
						ir_event_push(ev);
					} else {
#if ENABLE_DEBUG
						IrEvent ev = { .type = IR_ERROR_CKSUM, .code = { .nec = { .device = ir_state.data.nec.device, .code = ir_state.data.nec.code } } };
						ir_event_push(ev);
#endif
					}
					CHANGE_STATE(IR_STATE_IDLE);
				} else {
					CHANGE_STATE(IR_STATE_NEC_BIT);
				}
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_NEC_SHORT_ACK:
			if (USEC_PULSE_MATCH(CLK_NEC_ACK, clock_diff)) {
				// Store 0
				ir_state.word = ir_state.word << 1;
				ir_state.bitcount++;
				// Check for completion
				if (ir_state.bitcount >= 32) {
					ir_state.data.nec.device = ir_state.word >> 24;
					ir_state.data.nec.code = ir_state.word >> 8;
					// Evaluate checksum
					if ((uint8_t) (ir_state.word >> 24) == (uint8_t) ~(ir_state.word >> 16) && (uint8_t) (ir_state.word >> 8) == (uint8_t) ~ir_state.word) {
						IrEvent ev = { .type = IR_CODE_NEC, .code = { .nec = { .device = ir_state.data.nec.device, .code = ir_state.data.nec.code } } };
						ir_event_push(ev);
					} else {
#if ENABLE_DEBUG
						IrEvent ev = { .type = IR_ERROR_CKSUM, .code = { .nec = { .device = ir_state.data.nec.device, .code = ir_state.data.nec.code } } };
						ir_event_push(ev);
#endif
					}
					CHANGE_STATE(IR_STATE_IDLE);
				} else {
					CHANGE_STATE(IR_STATE_NEC_BIT);
				}
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_SIRC_START_ACK:
			if (USEC_PULSE_MATCH(CLK_SIRC_ACK, clock_diff)) {
				CHANGE_STATE(IR_STATE_SIRC_BIT);
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_SIRC_BIT:
			if (USEC_PULSE_MATCH(CLK_SIRC_LONG, clock_diff)) {
				CHANGE_STATE(IR_STATE_SIRC_LONG_ACK);
			} else if (USEC_PULSE_MATCH(CLK_SIRC_SHORT, clock_diff)) {
				CHANGE_STATE(IR_STATE_SIRC_SHORT_ACK);
			} else {
				DEBUG_PULSE_ERROR(clock_diff);
				CHANGE_STATE(IR_STATE_IDLE);
			}
			break;
		case IR_STATE_SIRC_LONG_ACK:
#if ENABLE_DEBUG
			{
				IrEvent ev = { .type = IR_ERROR_STATE };
				ir_event_push(ev);
			}
#endif
			CHANGE_STATE(IR_STATE_IDLE);
			break;
		case IR_STATE_SIRC_SHORT_ACK:
#if ENABLE_DEBUG
			{
				IrEvent ev = { .type = IR_ERROR_STATE };
				ir_event_push(ev);
			}
#endif
			CHANGE_STATE(IR_STATE_IDLE);
			break;
		default:
			// Invalid transmission
			DEBUG_PULSE_ERROR(clock_diff);
			CHANGE_STATE(IR_STATE_IDLE);
			break;
		}

	} else {
		// Glitch, report
#if ENABLE_DEBUG
		IrEvent ev = { .type = IR_ERROR_GLITCH, .code = { .error = bit } };
		ir_event_push(ev);
#endif
		CHANGE_STATE(IR_STATE_IDLE);
	}

	// Update the level state
	ir_state.bitstate = bit;
	// Update the saved clock
	ir_state.lastclock = clock_real;
}

void ir_init() {
	// Initialize the clock
	clock_init();

	// Set IR port as input
	IR_DDR &= ~IR0_BIT;
	// Disable pullup on IR input
	IR_PORT &= ~IR0_BIT;

#if 0
	printf_P(PSTR("CLK_SIRC_START %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_SIRC_START), (uint16_t) USEC_TO_CLOCK_MAX(CLK_SIRC_START));
	printf_P(PSTR("CLK_SIRC_ACK %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_SIRC_ACK), (uint16_t) USEC_TO_CLOCK_MAX(CLK_SIRC_ACK));
	printf_P(PSTR("CLK_SIRC_LONG %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_SIRC_LONG), (uint16_t) USEC_TO_CLOCK_MAX(CLK_SIRC_LONG));
	printf_P(PSTR("CLK_SIRC_SHORT %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_SIRC_SHORT), (uint16_t) USEC_TO_CLOCK_MAX(CLK_SIRC_SHORT));
	printf_P(PSTR("CLK_NEC_START %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_NEC_START), (uint16_t) USEC_TO_CLOCK_MAX(CLK_NEC_START));
	printf_P(PSTR("CLK_NEC_ACK %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_NEC_ACK), (uint16_t) USEC_TO_CLOCK_MAX(CLK_NEC_ACK));
	printf_P(PSTR("CLK_NEC_MODE_CMD %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_NEC_MODE_CMD), (uint16_t) USEC_TO_CLOCK_MAX(CLK_NEC_MODE_CMD));
	printf_P(PSTR("CLK_NEC_MODE_REP %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_NEC_MODE_REP), (uint16_t) USEC_TO_CLOCK_MAX(CLK_NEC_MODE_REP));
	printf_P(PSTR("CLK_NEC_LONG %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_NEC_LONG), (uint16_t) USEC_TO_CLOCK_MAX(CLK_NEC_LONG));
	printf_P(PSTR("CLK_NEC_SHORT %u:%u\n"), (uint16_t) USEC_TO_CLOCK_MIN(CLK_NEC_SHORT), (uint16_t) USEC_TO_CLOCK_MAX(CLK_NEC_SHORT));
#endif

	// Initialize state structure
	ir_state.lastclock = 0;
	//ir_state.state = IR_STATE_IDLE;
	//clock_t clock_diff = 0;
	CHANGE_STATE(IR_STATE_IDLE);
	ir_state.type = IR_CODE_NONE;
	ir_state.word = 0;
	ir_state.bitcount = 0;
	ir_state.in = 0;
	ir_state.out = 0;

	// Read the IR port's initial logic level
	ir_state.bitstate = IR_PIN & IR0_BIT;

	// Enable the GPIO interrupt
	IR_PCMSK |= IR0_INT;
	IR_GIMSK |= IR_PCIE;
}

void ir_reset() {
	// Disable the GPIO interrupt (only for the pin, not globally)
	IR_PCMSK &= ~IR0_INT;
}

uint8_t ir_event_size() {
	// 0123456789abcdef
	//     |            0
	// -----i o-------- 14
	//    o-----i       6
	// io-------------- 15
	// fill = (in - out) % size
	return (ir_state.in - ir_state.out) % IR_EVENT_QUEUE_SIZE;
}

static inline void ir_event_push(IrEvent ev) {
	// Insert the new event and increment the input pointer
	ir_state.events[ir_state.in].type = ev.type;
	ir_state.events[ir_state.in].code = ev.code;
	ir_state.in = (ir_state.in + 1) % IR_EVENT_QUEUE_SIZE;
	// Queue is now full, discard the oldest event
	if (ir_state.in == ir_state.out) {
		ir_state.out = (ir_state.out + 1) % IR_EVENT_QUEUE_SIZE;
	}
}

IrEvent ir_event_pop() {
	IrEvent ret;
	// Check if the queue is empty first
	if (ir_state.in == ir_state.out) {
		ret.type = IR_CODE_NONE;
	} else {
		ret.type = ir_state.events[ir_state.out].type;
		ret.code = ir_state.events[ir_state.out].code;
		ir_state.out = (ir_state.out + 1) % IR_EVENT_QUEUE_SIZE;
	}
	return ret;
}

#if CLOCK_16BIT
// Global clock register (bitwise or with TCNT0 to get the current clock)
uint8_t clock;

ISR(TIMER_INTVECT) {
	// Timer overflow - increment the clock
	clock++;
}
#endif

static inline void clock_init() {
	// Reset the clock
#if CLOCK_16BIT
	clock = 0;
#endif
	TCNT = 0;

	// Disable waveform generation
	TCCRA = 0;

	// Set timer clock source
	TCCRB = CSX;
#if CLOCK_16BIT
	// Enable timer overflow interrupt, disable output compare
	TIMSK |= TOIE;
#endif
}

static inline clock_t clock_now() {
#if CLOCK_16BIT
	return (clock << 8) | TCNT;
#else
	return TCNT;
#endif
}
