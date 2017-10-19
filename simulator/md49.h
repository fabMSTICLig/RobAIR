#ifndef MD49_H
#define MD49_H

#include "simavr/sim_avr.h"
#include "simavr/sim_irq.h"
#include "simavr/sim_time.h"

#define MD49_OUT_BUFFER_LEN 32

typedef void (*md49_callback_t)(int8_t speed1, int8_t speed2);

struct md49 {
	unsigned int id;
	char uart_name;
	md49_callback_t callback;

	avr_t *avr;
	avr_irq_t *irqs;

	unsigned int mode;
	unsigned int timeout;
	union {
		uint8_t u;
		int8_t s;
	} requested_speeds[2];
	int8_t target_speeds[2];
	int8_t effective_speeds[2];
	int16_t encoders[2];
	unsigned int accel;
	unsigned int is_accelerating;

	uint8_t out_buffer[MD49_OUT_BUFFER_LEN];
	unsigned int out_write, out_read;
	unsigned int uart_out_buffer_full;

	unsigned int need_sync;
	uint8_t cur_cmd;
	uint8_t data;
	unsigned int need_data;
};

struct md49 *md49_attach(struct avr_t *avr, short serial);
void md49_set_callback(struct md49 *m, md49_callback_t callback);
void md49_destroy(struct md49 *m);

#endif
