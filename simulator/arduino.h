#ifndef ARDUINO_H
#define ARDUINO_H

#include "simavr/sim_avr.h"
#include "simavr/sim_irq.h"

avr_irq_t *arduino_mega_digital_getirq(avr_t *avr, uint8_t pin);

#endif
