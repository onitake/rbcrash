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

#include <avr/interrupt.h>
#include <util/delay.h>
#include "led.h"
#include "ir.h"

void rainbow() {
	uint8_t i;
	for (i = 0; i < 32; i++) {
		led_set2(31 - i, i, 0);
		_delay_ms(100);
	}
	for (i = 0; i < 32; i++) {
		led_set2(0, 31 - i, i);
		_delay_ms(100);
	}
	for (i = 0; i < 32; i++) {
		led_set2(i, 0, 31 - i);
		_delay_ms(100);
	}
}

int main() {
	led_init();
	ir_init();
	sei();

	while (1) {
		rainbow();
	}

	return 0;
}

