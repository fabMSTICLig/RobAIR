#include "arduino_helpers.h"

#include "simavr/sim_irq.h"
#include "simavr/avr_ioport.h"

avr_irq_t *arduino_mega_digital_getirq(avr_t *avr, uint8_t pin)
{
	char port = digital_pin_to_port[pin];
	int bit = digital_pin_to_bit_mask[pin];

	return avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(port), bit);
}
