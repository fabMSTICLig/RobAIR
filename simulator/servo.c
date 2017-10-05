#include "servo.h"

#include <stdlib.h>
#include <stdio.h>

#include "simavr/sim_time.h"

#include "arduino.h"

// Values for the Parallax Standard Servo
#define MIN_WIDTH 750
#define MAX_WIDTH 2250

static uint8_t servo_id = 0;

static void servo_onirq(struct avr_irq_t *irq, uint32_t value, void *param)
{
	struct servo *s = param;

	if (value) {
		s->started = 1;
		s->pulse_start = s->avr->cycle;
	} else if (s->started) {
		s->started = 0;

		if (s->callback == NULL)
			return;

		avr_cycle_count_t cycles = s->avr->cycle - s->pulse_start;
		uint32_t usec = avr_cycles_to_usec(s->avr, cycles);

		if (usec < MIN_WIDTH)
			usec = MIN_WIDTH;
		else if (usec > MAX_WIDTH)
			usec = MAX_WIDTH;

		int angle = (usec - MIN_WIDTH) * 180 / (MAX_WIDTH - MIN_WIDTH);
		if (angle != s->last_angle) {
			s->last_angle = angle;
			s->callback(angle);
		}
	}
}

struct servo *servo_attach(struct avr_t *avr, uint8_t pin)
{
	struct servo *s = malloc(sizeof(*s));

	s->id = servo_id++;
	s->avr = avr;
	s->callback = NULL;
	s->started = 0;
	s->last_angle = -1;

	char irq_name[10];
	sprintf(irq_name, "SERVO_%d", s->id);

	const char *irq_names[] = {irq_name};

	s->irq = avr_alloc_irq(&avr->irq_pool, 0, 1, irq_names);
	avr_irq_register_notify(s->irq, servo_onirq, s);
	s->pin_irq = arduino_mega_digital_getirq(avr, pin);
	avr_connect_irq(s->pin_irq, s->irq);

	return s;
}

void servo_set_callback(struct servo *s, servo_callback_t callback)
{
	s->callback = callback;
}

void servo_destroy(struct servo *s)
{
	avr_unconnect_irq(s->pin_irq, s->irq);
	avr_irq_unregister_notify(s->irq, servo_onirq, s);
	avr_free_irq(s->irq, 1);
	free(s);
}
