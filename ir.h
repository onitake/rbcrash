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

#ifndef _IR_H
#define _IR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Full clock type
#if CLOCK_16BIT
typedef uint16_t clock_t;
#else
typedef uint8_t clock_t;
#endif

// Code queue entry

// No code received
#define IR_CODE_NONE 0
// Sony IR code (SIRC)
#define IR_CODE_SONY 1
// Philips IR code (RC5)
#define IR_CODE_RC5 2
// NEC IR code
#define IR_CODE_NEC 3
#if ENABLE_DEBUG
// Errors (only generated in debug mode)
// Unknown/invalid pulse (clk contains the pulse length)
#define IR_ERROR_PULSE 101
// Checksum mismatch
#define IR_ERROR_CKSUM 102
// Glitch detected
#define IR_ERROR_GLITCH 103
// Unimplemented decoding routine
#define IR_ERROR_INCOMPLETE 104
// Invalid state (i.e. repeat without corresponding code to repeat)
#define IR_ERROR_STATE 105
// Debug events
// Report state change (state contains the old and new states)
#define IR_DEBUG_STATE 201
#endif

typedef struct {
	uint8_t type;
	union {
		uint8_t sony8;
		uint8_t rc5;
		struct {
			uint8_t device;
			uint8_t code;
		} nec;
#if ENABLE_DEBUG
		clock_t clock;
		uint8_t error;
		struct {
			uint8_t old;
			uint8_t new;
			clock_t clock;
			uint16_t line;
		} state;
#endif
	} code;
} IrEvent;

// Initialize the IR state machine and arm the GPIO interrupt
// You have to call sei() to set the global interrupt flag before processing starts
void ir_init();
// Temporarily disable and reset the IR module
// Call ir_init again to reenable it
void ir_reset();
// Return the number of events in the queue
uint8_t ir_event_size();
// Read and discard the oldest entry from the event list
// Returns { .type = IR_CODE_NONE } if no code was queued
// Errors are queued up in the same way (if debugging is enabled)
IrEvent ir_event_pop();

#ifdef __cplusplus
}
#endif

#endif /*_IR_H*/
