#include <stdlib.h>
#include <stdio.h>

#include "simavr/sim_avr.h"
#include "simavr/sim_elf.h"
#include "simavr/sim_irq.h"
#include "simavr/sim_io.h"
#include "simavr/avr_ioport.h"
#include "simavr/parts/uart_pty.h"

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "Usage: %s path/to/robairarduino.elf\n", argv[0]);
		return 1;
	}

	elf_firmware_t elf;
	elf_read_firmware(argv[1], &elf);

	avr_t *avr;
	avr = avr_make_mcu_by_name("atmega2560");
	avr_init(avr);
	avr_load_firmware(avr, &elf);

	uart_pty_t uart_pty;
	uart_pty_init(avr, &uart_pty);
	uart_pty_connect(&uart_pty, '0');

	int state = cpu_Running;
	while (state != cpu_Done && state != cpu_Crashed)
		state = avr_run(avr);
}
