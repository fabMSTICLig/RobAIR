#include "ws2812.h"

#include <stdlib.h>
#include <stdio.h>
#include <strings.h>

#include "simavr/sim_time.h"

#include "arduino.h"

#define RESET_THRESH 50000
#define T0H_LOW  200
#define T0H_HIGH 500
#define T1H_LOW  550
#define T1H_HIGH 850

#define COLOR_ALLOC_INC 8

static uint8_t ws2812_id = 0;

static void push_bit(struct ws2812 *w, unsigned int bit)
{
	struct ws2812_color *col = w->colors + w->cur_color;
	uint8_t mask = 1 << w->cur_bit;

	switch (w->cur_comp) {
	case R:
		if (bit)
			col->r |= mask;
		else
			col->r &= ~mask;
		break;
	case G:
		if (bit)
			col->g |= mask;
		else
			col->g &= ~mask;
		break;
	case B:
		if (bit)
			col->b |= mask;
		else
			col->b &= ~mask;
		break;
	}
}

static void increment_cur(struct ws2812 *w)
{
	if (w->cur_bit > 0) {
		w->cur_bit--;
	} else if (w->cur_comp < B) {
		w->cur_bit = 7;
		w->cur_comp++;
	} else {
		w->cur_bit = 7;
		w->cur_comp = G;
		w->cur_color++;

		if (w->cur_color >= w->colors_alloc) {
			w->colors_alloc += COLOR_ALLOC_INC;
			w->colors = realloc(w->colors,
					w->colors_alloc * sizeof(*w->colors));
		}
	}
}

static void reset(struct ws2812 *w)
{
	w->cur_color = 0;
	w->cur_comp = G;
	w->cur_bit = 7;
}

static void ws2812_onirq(struct avr_irq_t *irq, uint32_t value, void *param)
{
	struct ws2812 *w = param;

	avr_cycle_count_t cycles = w->avr->cycle - w->pulse_start;
	uint64_t nsec = avr_cycles_to_nsec(w->avr, cycles);

	w->pulse_start = w->avr->cycle;

	if (nsec > RESET_THRESH) {
		if (!w->sent && w->cur_color > 0) {
			w->callback(w->cur_color, w->colors);
			w->sent = 1;
			reset(w);
		}
		return;
	}

	if (!value) {
		unsigned int bit;
		if (nsec >= T0H_LOW && nsec <= T0H_HIGH)
			bit = 0;
		else if (nsec >= T1H_LOW && nsec <= T1H_HIGH)
			bit = 1;
		else
			return;

		w->sent = 0;

		push_bit(w, bit);
		increment_cur(w);
	}
}

static avr_cycle_count_t ws2812_ontimer(
		struct avr_t *avr,
		avr_cycle_count_t when,
		void *param)
{
	struct ws2812 *w = param;

	avr_cycle_count_t cycles = w->avr->cycle - w->pulse_start;
	uint64_t nsec = avr_cycles_to_nsec(w->avr, cycles);

	if (nsec > RESET_THRESH && !w->sent && w->cur_color > 0) {
		w->callback(w->cur_color, w->colors);
		w->sent = 1;
		reset(w);
	}

	return avr->cycle + avr_usec_to_cycles(w->avr, RESET_THRESH / 1000);
}

struct ws2812 *ws2812_attach(struct avr_t *avr, uint8_t pin)
{
	struct ws2812 *w = malloc(sizeof(*w));

	w->id = ws2812_id++;
	w->avr = avr;
	w->callback = NULL;

	w->sent = 0;
	w->pulse_start = w->avr->cycle;

	w->colors_alloc = COLOR_ALLOC_INC;
	w->colors = calloc(w->colors_alloc, sizeof(*w->colors));
	reset(w);

	char irq_name[11];
	sprintf(irq_name, "WS2812_%d", w->id);

	const char *irq_names[] = {irq_name};

	w->irq = avr_alloc_irq(&avr->irq_pool, 0, 1, irq_names);
	avr_irq_register_notify(w->irq, ws2812_onirq, w);
	w->pin_irq = arduino_mega_digital_getirq(avr, pin);
	avr_connect_irq(w->pin_irq, w->irq);

	avr_cycle_timer_register_usec(avr,
			RESET_THRESH / 1000,
			ws2812_ontimer, w);

	return w;
}

void ws2812_set_callback(struct ws2812 *w, ws2812_callback_t callback)
{
	w->callback = callback;
}


void ws2812_destroy(struct ws2812 *w)
{
	avr_cycle_timer_cancel(w->avr, ws2812_ontimer, w);
	avr_unconnect_irq(w->pin_irq, w->irq);
	avr_irq_unregister_notify(w->irq, ws2812_onirq, w);
	avr_free_irq(w->irq, 1);
	free(w);
}
