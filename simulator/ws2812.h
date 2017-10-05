#ifndef WS2812_H
#define WS2812_H

#include <inttypes.h>

#include "simavr/sim_avr.h"
#include "simavr/sim_irq.h"

struct ws2812_color {
	uint8_t r, g, b;
};

typedef void (*ws2812_callback_t)(
		unsigned int num_leds,
		struct ws2812_color *colors);

struct ws2812 {
	unsigned int id;
	ws2812_callback_t callback;

	avr_t *avr;
	avr_irq_t *irq, *pin_irq;

	int sent;
	avr_cycle_count_t pulse_start;

	unsigned int cur_color;
	enum {G, R, B} cur_comp;
	unsigned short cur_bit;
	struct ws2812_color *colors;
	unsigned int colors_alloc;
};

struct ws2812 *ws2812_attach(struct avr_t *avr, uint8_t pin);
void ws2812_set_callback(struct ws2812 *w, ws2812_callback_t callback);
void ws2812_destroy(struct ws2812 *w);

#endif
