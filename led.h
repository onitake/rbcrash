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

#ifndef _LED_H
#define _LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COLOR_MAX 0x1f

typedef struct {
	uint16_t r:5;
	uint16_t g:5;
	uint16_t b:5;
} Color;

static inline Color color_make(uint8_t r, uint8_t g, uint8_t b) {
	Color c = { r, g, b };
	return c;
}

static inline Color color_blend(Color a, Color b, uint8_t f/*:3*/) {
	Color c;
	c.r = ((uint8_t) a.r * (7 - f) + (uint8_t) b.r * f) >> 3;
	c.g = ((uint8_t) a.g * (7 - f) + (uint8_t) b.g * f) >> 3;
	c.b = ((uint8_t) a.b * (7 - f) + (uint8_t) b.b * f) >> 3;
	return c;
}

static const Color COLOR_BLACK = { 0x00, 0x00, 0x00 };
static const Color COLOR_RED = { 0x1f, 0x00, 0x00 };
static const Color COLOR_GREEN = { 0x00, 0x1f, 0x00 };
static const Color COLOR_BLUE = { 0x00, 0x00, 0x1f };
static const Color COLOR_YELLOW = { 0x1f, 0x1f, 0x00 };
static const Color COLOR_CYAN = { 0x00, 0x1f, 0x1f };
static const Color COLOR_MAGENTA = { 0x1f, 0x00, 0x1f };
static const Color COLOR_WHITE = { 0x1f, 0x1f, 0x1f };

void led_init();
void led_set(Color c);
Color led_get();

static inline void led_set2(uint8_t r, uint8_t g, uint8_t b) {
	led_set(color_make(r, g, b));
}

#ifdef __cplusplus
}
#endif

#endif /*_LED_H*/
