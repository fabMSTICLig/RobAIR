#ifndef EYES_H
#define EYES_H

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <robairmain/EyesAnim.h>

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
	~Eyes();
	void begin();
	int display_void();
	int display_stop();
	int setMatrice(uint8_t *mat);
	void setMatrice(uint32_t *mat);
	int setMatrice(int id);

	void setAnimation(const robairmain::EyesAnim &anim);
	void animation_step(void);

private:
	Adafruit_NeoPixel * pixels;
	enum {STILL, ANIMATION} mode;

	robairmain::EyesAnim animation;
	unsigned int anim_cur_frame;
	unsigned long anim_last_change;

	int mat_id_to_pixel_id(int i);
	uint32_t scale_color(uint32_t color, int max_power);
	void do_setMatrice(uint32_t *mat);
};

#endif
