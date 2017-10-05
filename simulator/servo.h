#ifndef SERVO_H
#define SERVO_H

#include "simavr/sim_avr.h"
#include "simavr/sim_irq.h"

typedef void (*servo_callback_t)(int angle);

struct servo {
	unsigned int id;
	servo_callback_t callback;

	avr_t *avr;
	avr_irq_t *irq, *pin_irq;
	int started;
	avr_cycle_count_t pulse_start;
	int last_angle;
};

struct servo *servo_attach(struct avr_t *avr, uint8_t pin);
void servo_set_callback(struct servo *s, servo_callback_t callback);
void servo_destroy(struct servo *s);

#endif
