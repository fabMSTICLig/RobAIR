#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "simavr/sim_avr.h"
#include "simavr/sim_elf.h"
#include "simavr/sim_irq.h"
#include "simavr/sim_io.h"
#include "simavr/sim_gdb.h"
#include "simavr/avr_ioport.h"
#include "simavr/parts/uart_pty.h"

#include "gui.h"
#include "servo.h"
#include "md49.h"

#define PIN_HEAD 3
#define PIN_EYES 4
#define SERIAL_MOTORS 1

struct servo *head;
struct ws2812 *eyes;
struct md49 *motors;

void print_usage(char *argv0)
{
	fprintf(stderr, "Usage: %s [-g] path/to/robairarduino.elf\n", argv0);
}

void robair_setup(struct avr_t *avr)
{
	head = servo_attach(avr, PIN_HEAD);
	eyes = ws2812_attach(avr, PIN_EYES);
	motors = md49_attach(avr, SERIAL_MOTORS);
}

void robair_clean(void)
{
	servo_destroy(head);
	ws2812_destroy(eyes);
	md49_destroy(motors);
}

int main(int argc, char **argv)
{
	int debug = 0;
	char *elf_path;

	if (argc < 2) {
		print_usage(argv[0]);
		return 1;
	}

	if (strcmp(argv[1], "-g") == 0)
		debug = 1;

	if (debug && argc < 3) {
		print_usage(argv[0]);
		return 1;
	}

	if (debug)
		elf_path = argv[2];
	else
		elf_path = argv[1];

	elf_firmware_t elf;
	elf_read_firmware(elf_path, &elf);
	elf.frequency = 16000000;

	avr_t *avr;
	avr = avr_make_mcu_by_name("atmega2560");
	avr_init(avr);
	avr_load_firmware(avr, &elf);

	robair_setup(avr);

	if (debug) {
		avr->gdb_port = 1234;
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}

	uart_pty_t uart_pty;
	uart_pty_init(avr, &uart_pty);
	uart_pty_connect(&uart_pty, '0');

	gui_init();

	struct gui_data_sources gui_srcs = {
		.head = head,
		.eyes = eyes,
		.motors = motors
	};

	gui_attach(&gui_srcs);

	int state = cpu_Running;
	while (state != cpu_Done && state != cpu_Crashed && gui_is_active())
		state = avr_run(avr);

	gui_deinit();

	robair_clean();

	unlink("/tmp/simavr-uart0");
}
