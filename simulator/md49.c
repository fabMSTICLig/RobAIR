#include "md49.h"
#include "md49_commands.h"

#include <stdlib.h>
#include <stdio.h>

#include "simavr/avr_uart.h"

#define IRQ_IN    0  // input for us, output for the uart
#define IRQ_OUT   1  // output for us, input for the uart
#define IRQ_XON   2
#define IRQ_XOFF  3

#define TIMEOUT_USEC 2000000
#define ACCEL_USEC     16000

//TODO: update encoders depending on speed

static uint8_t md49_id = 0;

static int out_buffer_full(struct md49 *m)
{
	return (m->out_write == m->out_read - 1) ||
		(m->out_write == MD49_OUT_BUFFER_LEN && m->out_read == 0);
}

static void send_byte(struct md49 *m, uint8_t byte)
{
	if (!m->uart_out_buffer_full) {
		avr_raise_irq(m->irqs + IRQ_OUT, byte);
	} else {
		if (out_buffer_full(m)) {
			fprintf(stderr, "ERROR: md49 uart buffer full\n");
			return;
		}

		m->out_buffer[m->out_write++] = byte;
		if (m->out_write >= MD49_OUT_BUFFER_LEN)
			m->out_write = 0;
	}
}

static void send_buffer(struct md49 *m)
{
	while (m->out_read != m->out_write && !m->uart_out_buffer_full) {
		avr_raise_irq(m->irqs + IRQ_OUT, m->out_buffer[m->out_read++]);
		if (m->out_read >= MD49_OUT_BUFFER_LEN)
			m->out_read = 0;
	}
}

static void send_encoder(struct md49 *m, int i)
{
	/* Encoders are 32 bits according to documentation, but 16 bits
	 * according to reality
	 */

	if (m->encoders[i] >= 0) {
		send_byte(m, 0x00);
		send_byte(m, 0x00);
	} else {
		send_byte(m, 0xff);
		send_byte(m, 0xff);
	}

	send_byte(m, m->encoders[i] >> 8);
	send_byte(m, m->encoders[i]);
}

static avr_cycle_count_t accelerate(
		struct avr_t *avr,
		avr_cycle_count_t when,
		void *param)
{
	struct md49 *m = param;

	int reached = 1;

	for (int i = 0 ; i <= 1 ; ++i) {
		int diff = m->target_speeds[i] - m->effective_speeds[i];

		if (diff > 0) {
			m->effective_speeds[i] +=
				(diff < m->accel ? diff : m->accel);
		} else if (diff < 0) {
			m->effective_speeds[i] +=
				(diff > -m->accel ? diff : -m->accel);
		}

		if (m->effective_speeds[i] != m->target_speeds[i])
			reached = 0;
	}

	m->callback(m->effective_speeds[0], m->effective_speeds[1]);

	m->is_accelerating = !reached;

	if (reached)
		return 0;
	else
		return when + avr_usec_to_cycles(avr, ACCEL_USEC);
}

static void update_target_speeds(struct md49 *m)
{
	int base, turn, left, right;

	if (!m->is_accelerating) {
		m->is_accelerating = 1;
		avr_cycle_timer_register_usec(m->avr, ACCEL_USEC,
				accelerate, m);
	}

	switch (m->mode)
	{
	case 0:
		m->target_speeds[0] = m->requested_speeds[0].u - 128;
		m->target_speeds[1] = m->requested_speeds[1].u - 128;
		return;
	default:
	case 1:
		m->target_speeds[0] = m->requested_speeds[0].s;
		m->target_speeds[1] = m->requested_speeds[1].s;
		return;
	case 2:
		base = m->requested_speeds[0].u - 128;
		turn = m->requested_speeds[1].u - 128;
		break;
	case 3:
		base = m->requested_speeds[0].s;
		turn = m->requested_speeds[1].s;
		break;
	}


	// For turn mode only

	if (base >= 0) {
		left = base + turn;
		right = base - turn;
	} else {
		left = base - turn;
		right = base + turn;
	}

	/* "if either motor is not able to achieve the required speed for the
	 * turn (beyong the maxiumum output), then the othor motor is
	 * automatically changed by the program to meet the required
	 * difference"
	 */
	if (left > INT8_MAX) {
		m->target_speeds[0] = INT8_MAX;
		m->target_speeds[1] = right - (INT8_MAX - left);
	} else if (left < INT8_MIN) {
		m->target_speeds[0] = INT8_MIN;
		m->target_speeds[1] = right + (INT8_MIN - left);
	} else if (right > INT8_MAX) {
		m->target_speeds[0] = left - (INT8_MAX - right);
		m->target_speeds[1] = INT8_MAX;
	} else if (right < INT8_MIN) {
		m->target_speeds[0] = left + (INT8_MIN - right);
		m->target_speeds[1] = INT8_MIN;
	} else {
		m->target_speeds[0] = left;
		m->target_speeds[1] = right;
	}
}

static void handle_command(struct md49 *m)
{
	switch (m->cur_cmd) {
	case MD49_CMD_GET_SPEED_1:
		send_byte(m, m->requested_speeds[0].u);
		break;
	case MD49_CMD_GET_SPEED_2:
		send_byte(m, m->requested_speeds[1].u);
		break;
	case MD49_CMD_GET_ENCODER_1:
		send_encoder(m, 0);
		break;
	case MD49_CMD_GET_ENCODER_2:
		send_encoder(m, 1);
		break;
	case MD49_CMD_GET_ENCODERS:
		send_encoder(m, 0);
		send_encoder(m, 1);
		break;
	case MD49_CMD_GET_VOLTS:
		send_byte(m, 24);
		break;
	case MD49_CMD_GET_CURRENT_1:
		send_byte(m, 25);
		break;
	case MD49_CMD_GET_CURRENT_2:
		send_byte(m, 25);
		break;
	case MD49_CMD_GET_VERSION:
		send_byte(m, MD49_VERSION);
		break;
	case MD49_CMD_GET_ACCELERATION:
		send_byte(m, m->accel);
		break;
	case MD49_CMD_GET_MODE:
		send_byte(m, m->mode);
		break;
	case MD49_CMD_GET_VI:
		send_byte(m, 25);
		send_byte(m, 10);
		send_byte(m, 0);
		break;
	case MD49_CMD_GET_ERROR:
		send_byte(m, 0);
		break;
	case MD49_CMD_SET_SPEED_1:
		m->requested_speeds[0].u = m->data;
		update_target_speeds(m);
		break;
	case MD49_CMD_SET_SPEED_2:
		m->requested_speeds[1].u = m->data;
		update_target_speeds(m);
		break;
	case MD49_CMD_SET_ACCELERATION:
		if (m->data < 1)
			m->accel = 1;
		else if (m->data > 10)
			m->accel = 10;
		else
			m->accel = m->data;
		break;
	case MD49_CMD_SET_MODE:
		m->mode = m->data;
		break;
	case MD49_CMD_RESET_ENCODERS:
		m->encoders[0] = 0;
		m->encoders[1] = 0;
		break;
	case MD49_CMD_DISABLE_REGULATOR:
		break;
	case MD49_CMD_ENABLE_REGULATOR:
		break;
	case MD49_CMD_DISABLE_TIMEOUT:
		m->timeout = 0;
		break;
	case MD49_CMD_ENABLE_TIMEOUT:
		m->timeout = 1;
		break;
	}
}

static avr_cycle_count_t on_timeout(
		struct avr_t *avr,
		avr_cycle_count_t when,
		void *param)
{
	struct md49 *m = param;

	if (!m->timeout)
		return 0;

	switch (m->mode) {
	case 0:
	case 2:
		m->requested_speeds[0].u = 128;
		m->requested_speeds[1].u = 128;
		break;
	case 1:
	case 3:
		m->requested_speeds[0].s = 0;
		m->requested_speeds[1].s = 0;
		break;
	}

	update_target_speeds(m);

	if (!m->is_accelerating) {
		m->is_accelerating = 1;
		avr_cycle_timer_register_usec(m->avr, ACCEL_USEC,
				accelerate, m);
	}

	return 0;
}

static void on_input(struct avr_irq_t *irq, uint32_t value, void *param)
{
	struct md49 *m = param;

	if (m->need_sync) {
		if (value == MD49_SYNC)
			m->need_sync = 0;
	} else if(m->need_data) {
		m->data = value;
		m->need_data = 0;
		handle_command(m);
		m->need_sync = 1;
	} else {
		m->cur_cmd = value;

		switch (value) {
		case MD49_CMD_SET_SPEED_1:
		case MD49_CMD_SET_SPEED_2:
		case MD49_CMD_SET_ACCELERATION:
		case MD49_CMD_SET_MODE:
			m->need_data = 1;
			break;
		case MD49_SYNC:
			break;
		default:
			handle_command(m);
			m->need_sync = 1;
		}
	}

	if (m->timeout)
		avr_cycle_timer_register_usec(m->avr, TIMEOUT_USEC,
				on_timeout, m);
}

static void on_xon(struct avr_irq_t *irq, uint32_t value, void *param)
{
	struct md49 *m = param;

	m->uart_out_buffer_full = 0;
	send_buffer(m);
}

static void on_xoff(struct avr_irq_t *irq, uint32_t value, void *param)
{
	struct md49 *m = param;

	m->uart_out_buffer_full = 1;
}


struct md49 *md49_attach(struct avr_t *avr, short serial)
{
	struct md49 *m = malloc(sizeof(*m));

	m->id = md49_id++;
	m->avr = avr;
	m->uart_name = '0' + serial;
	m->callback = NULL;

	m->mode = 0;
	m->timeout = 1;
	m->requested_speeds[0].u = 128;
	m->requested_speeds[1].u = 128;
	update_target_speeds(m);
	m->effective_speeds[0] = m->target_speeds[0];
	m->effective_speeds[1] = m->target_speeds[1];
	m->encoders[0] = 0;
	m->encoders[1] = 0;
	m->accel = 5;
	m->is_accelerating = 0;

	m->out_write = 0;
	m->out_read = 0;
	m->uart_out_buffer_full = 0;
	m->need_sync = 1;
	m->need_data = 0;

	char in_name[12], out_name[13], xon_name[13], xoff_name[14];
	sprintf(in_name, "MD49_%d_in", m->id);
	sprintf(out_name, "MD49_%d_out", m->id);
	sprintf(xon_name, "MD49_%d_xon", m->id);
	sprintf(xoff_name, "MD49_%d_xoff", m->id);

	const char *irq_names[] = {in_name, out_name, xon_name, xoff_name};

	m->irqs = avr_alloc_irq(&avr->irq_pool, 0, 4, irq_names);
	avr_irq_register_notify(m->irqs + IRQ_IN, on_input, m);
	avr_irq_register_notify(m->irqs + IRQ_XON, on_xon, m);
	avr_irq_register_notify(m->irqs + IRQ_XOFF, on_xoff, m);

	uint32_t ioctl_irq = AVR_IOCTL_UART_GETIRQ(m->uart_name);
	avr_irq_t *src, *dst, *xon, *xoff;
	src = avr_io_getirq(avr, ioctl_irq, UART_IRQ_OUTPUT);
	dst = avr_io_getirq(avr, ioctl_irq, UART_IRQ_INPUT);
	xon = avr_io_getirq(avr, ioctl_irq, UART_IRQ_OUT_XON);
	xoff = avr_io_getirq(avr, ioctl_irq, UART_IRQ_OUT_XOFF);

	avr_connect_irq(src, m->irqs + IRQ_IN);
	avr_connect_irq(m->irqs + IRQ_OUT, dst);
	avr_connect_irq(xon, m->irqs + IRQ_XON);
	avr_connect_irq(xoff, m->irqs + IRQ_XOFF);

	return m;
}

void md49_set_callback(struct md49 *m, md49_callback_t callback)
{
	m->callback = callback;
}

void md49_destroy(struct md49 *m)
{
	avr_cycle_timer_cancel(m->avr, accelerate, m);
	avr_cycle_timer_cancel(m->avr, on_timeout, m);

	uint32_t ioctl_irq = AVR_IOCTL_UART_GETIRQ(m->uart_name);
	avr_irq_t *src, *dst, *xon, *xoff;
	src = avr_io_getirq(m->avr, ioctl_irq, UART_IRQ_OUTPUT);
	dst = avr_io_getirq(m->avr, ioctl_irq, UART_IRQ_INPUT);
	xon = avr_io_getirq(m->avr, ioctl_irq, UART_IRQ_OUT_XON);
	xoff = avr_io_getirq(m->avr, ioctl_irq, UART_IRQ_OUT_XOFF);

	avr_unconnect_irq(src, m->irqs + IRQ_IN);
	avr_unconnect_irq(m->irqs + IRQ_OUT, dst);
	avr_unconnect_irq(xon, m->irqs + IRQ_XON);
	avr_unconnect_irq(xoff, m->irqs + IRQ_XOFF);

	avr_irq_unregister_notify(m->irqs + IRQ_IN, on_input, m);
	avr_irq_unregister_notify(m->irqs + IRQ_XON, on_xon, m);
	avr_irq_unregister_notify(m->irqs + IRQ_XOFF, on_xoff, m);

	avr_free_irq(m->irqs, 4);

	free(m);
}
