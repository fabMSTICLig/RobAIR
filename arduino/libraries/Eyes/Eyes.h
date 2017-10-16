#ifndef EYES_H
#define EYES_H

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define NUMPIXELS 70
#define LINEWIDTH 14

#define EYESVIDE          0
#define EYESSTRAIGHT      1
#define EYESRIGHT         2
#define EYESLEFT          3
#define EYESTOP           4
#define EYESBOTTOM        5
#define EYESEXCLAMATIONS  6
#define EYESINTEROGATIONS 7
#define EYESSTOP          8
#define EYESHELLO         9
#define EYESERROR         10

class Eyes {
public:
	Eyes(int pin);
	~Eyes(void);
	void begin(void);
	int display_void(void);
	int display_stop(void);
	int setMatrice(uint8_t *mat);
	int setMatrice(int id);

private:
	Adafruit_NeoPixel *pixels;
};

#endif
