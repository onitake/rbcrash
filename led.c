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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "led.h"

#define TPS_DDR DDRB
#define TPS_PIN PINB
#define TPS_PORT PORTB
#define TPS_BIT_R _BV(PB0)
#define TPS_BIT_G _BV(PB1)
#define TPS_BIT_B _BV(PB2)
#define TPS_MASK (TPS_BIT_R | TPS_BIT_G | TPS_BIT_B)

#define TPS_DEVICE_ADDRESS 0x72
#define TPS_T_ESDELAY 120
#define TPS_T_ESDETECT 300
#define TPS_T_START 20
#define TPS_T_EOS 20
#define TPS_T_SHORT 20
#define TPS_T_LONG 40
#define TPS_T_ACKN 512

static struct {
	Color color;
} global;

void es_one(uint8_t bit) {
	TPS_PORT &= ~bit;
	_delay_us(TPS_T_SHORT);
	TPS_PORT |= bit;
	_delay_us(TPS_T_LONG);
}

void es_zero(uint8_t bit) {
	TPS_PORT &= ~bit;
	_delay_us(TPS_T_LONG);
	TPS_PORT |= bit;
	_delay_us(TPS_T_SHORT);
}

void es_init() {
	TPS_PORT &= ~TPS_MASK;
	TPS_DDR |= TPS_MASK;

	TPS_PORT |= TPS_MASK;
	_delay_us(TPS_T_ESDELAY);
	TPS_PORT &= ~TPS_MASK;
	_delay_us(TPS_T_ESDETECT);
	TPS_PORT |= TPS_MASK;
}

uint8_t es_cmd(uint8_t bit, uint8_t ack, uint8_t address, uint8_t data) {
	TPS_PORT |= bit;
	_delay_us(TPS_T_START);

	int8_t i;
	for (i = 8; i > 0; i--) {
		TPS_DEVICE_ADDRESS & _BV(i - 1) ? es_one(bit) : es_zero(bit);
	}

	TPS_PORT &= ~bit;
	_delay_us(TPS_T_EOS);
	TPS_PORT |= bit;
	_delay_us(TPS_T_START);

	ack ? es_one(bit) : es_zero(bit);
	for (i = 2; i > 0; i--) {
		address & _BV(i - 1) ? es_one(bit) : es_zero(bit);
	}
	for (i = 5; i > 0; i--) {
		data & _BV(i - 1) ? es_one(bit) : es_zero(bit);
	}

	if (ack) {
		TPS_DDR &= ~bit;
		_delay_us(TPS_T_EOS);
		uint8_t status = TPS_PIN | bit;
		_delay_us(TPS_T_ACKN);
		TPS_DDR |= bit;
		return status;
	} else {
		TPS_PORT &= ~bit;
		_delay_us(TPS_T_EOS);
		TPS_PORT |= bit;
		return 0;
	}
}

void led_init() {
	global.color = COLOR_BLACK;

	es_init();
}

void led_set(Color c) {
	global.color = c;

	es_cmd(TPS_BIT_R, 0, 0, c.r);
	es_cmd(TPS_BIT_G, 0, 0, c.g);
	es_cmd(TPS_BIT_B, 0, 0, c.b);
}

Color led_get() {
	return global.color;
}


