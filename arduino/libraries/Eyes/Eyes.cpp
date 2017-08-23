#include "Arduino.h"
#include "Eyes.h"

#define MAX_POWER 40


uint8_t vide[] = {
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t eyesstraight[] = {
	0,0,7,7,7,0,0,0,0,7,7,7,0,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,7,0,2,0,7,0,0,7,0,2,0,7,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t eyesright[] = {
	0,0,7,7,7,0,0,0,0,7,7,7,0,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,7,2,0,0,7,0,0,7,2,0,0,7,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t eyesleft[] = {
	0,0,7,7,7,0,0,0,0,7,7,7,0,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,7,0,0,2,7,0,0,7,0,0,2,7,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,0,7,7,7,0,0,0,0,7,7,7,0,0
};


uint8_t eyestop[] = {
	0,0,7,7,7,0,0,0,0,7,7,7,0,0,
	0,7,0,2,0,7,0,0,7,0,2,0,7,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t eyesbottom[] = {
	0,0,7,7,7,0,0,0,0,7,7,7,0,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,7,0,0,0,7,0,0,7,0,0,0,7,0,
	0,7,0,2,0,7,0,0,7,0,2,0,7,0,
	0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t exclamations[] = {
	0,0,1,0,1,0,1,0,1,0,1,0,0,0,
	0,0,1,0,1,0,1,0,1,0,1,0,0,0,
	0,0,1,0,1,0,1,0,1,0,1,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,1,0,1,0,1,0,1,0,1,0,0,0
};

uint8_t interogations[] = {
	0,4,4,4,0,4,4,4,0,4,4,4,0,0,
	0,4,0,4,0,4,0,4,0,4,0,4,0,0,
	0,0,4,4,0,0,4,4,0,0,4,4,0,0,
	0,0,4,0,0,0,4,0,0,0,4,0,0,0,
	0,0,2,0,0,0,2,0,0,0,2,0,0,0
};

uint8_t stop[] = {
	1,1,1,0,1,1,1,0,1,1,1,0,1,1,
	1,0,0,0,0,1,0,0,1,0,1,0,1,1,
	1,1,1,0,0,1,0,0,1,0,1,0,1,0,
	0,0,1,0,0,1,0,0,1,0,1,0,1,0,
	1,1,1,0,0,1,0,0,1,1,1,0,1,0
};

uint8_t hello[] = {
	7,0,7,0,2,2,0,1,0,1,0,5,5,5,
	7,0,7,0,2,0,0,1,0,1,0,5,0,5,
	7,7,7,0,2,2,0,1,0,1,0,5,0,5,
	7,0,7,0,2,0,0,1,0,1,0,5,0,5,
	7,0,7,0,2,2,0,1,1,1,1,5,5,5

};

uint8_t error[] = {
	1,1,3,3,3,1,1,1,3,3,3,1,1,1,
	1,0,3,0,3,1,0,1,3,0,3,1,0,1,
	1,1,3,3,3,1,1,1,3,0,3,1,1,1,
	1,0,3,3,0,1,1,0,3,0,3,1,1,0,
	1,1,3,0,3,1,0,1,3,3,3,1,0,1
};


uint8_t *tabeyes[] = {
	vide,
	eyesstraight,
	eyesright,
	eyesleft,
	eyestop,
	eyesbottom,
	exclamations,
	interogations,
	stop,
	hello,
	error
};

Eyes::Eyes(int pin) : mode(STILL)
{
	pixels = new Adafruit_NeoPixel(NUMPIXELS, pin, NEO_GRB + NEO_KHZ800);
}

Eyes::~Eyes()
{
	delete pixels;
}


void Eyes::begin()
{
	pixels->begin();
}

int Eyes::display_void()
{
	for (int i = 0 ; i < NUMPIXELS ; i++) {
		pixels->setPixelColor(i, pixels->Color(0, 0, 0));
		pixels->show();
	}

	return EYESVIDE;
}

int Eyes::mat_id_to_pixel_id(int i)
{
	int ligne = (i / LINEWIDTH);

	if (ligne % 2 == 0)
		return ligne * LINEWIDTH + (i % LINEWIDTH);
	else
		return ligne * LINEWIDTH + (LINEWIDTH - (i % LINEWIDTH)) - 1;
}

uint32_t Eyes::scale_color(uint32_t color, int max_power)
{
	uint32_t channel;
	uint32_t mask;
	int shift;

	for (mask = 0xff, shift = 0 ; shift <= 16 ; mask <<= 8, shift += 8) {
		channel = (color & mask) >> shift;
		channel = (uint32_t)((double)channel
				     * (double)max_power / (double)0xff);
		color &= ~mask;
		color |= channel << shift;
	}

	return color;
}

int Eyes::setMatrice(uint8_t *mat)
{
	int id = 0;

	for(int i=0; i<NUMPIXELS; i++) {
		id = mat_id_to_pixel_id(i);
		uint32_t color = pixels->Color(
					((mat[i] & 1) == 1 ? MAX_POWER : 0),
					((mat[i] & 2) == 2 ? MAX_POWER : 0),
					((mat[i] & 4) == 4 ? MAX_POWER : 0));
		pixels->setPixelColor(id, color);
	}

	pixels->show();
}

void Eyes::setMatrice(uint32_t *mat)
{
	mode = STILL;
	do_setMatrice(mat);
}

void Eyes::do_setMatrice(uint32_t *mat)
{
	for (int i = 0 ; i < NUMPIXELS ; ++i) {
		int id = mat_id_to_pixel_id(i);
		pixels->setPixelColor(id, scale_color(mat[i], MAX_POWER));
	}

	pixels->show();
}

int Eyes::setMatrice(int id)
{
	mode = STILL;
	setMatrice(tabeyes[id]);

	return id;
}

void Eyes::setAnimation(const robairmain::EyesAnim &anim)
{
	if (anim.frames_length == 0)
		return;

	animation = robairmain::EyesAnim(anim);
	mode = ANIMATION;
	do_setMatrice(anim.frames[0].mat);
	anim_cur_frame = 0;
	anim_last_change = millis();
}

void Eyes::animation_step()
{
	if (mode != ANIMATION)
		return;

	unsigned long cur_dur = millis() - anim_last_change;

	if (cur_dur >= animation.frames[anim_cur_frame].duration) {
		anim_last_change = millis();
		anim_cur_frame++;
		if (anim_cur_frame >= animation.frames_length)
			anim_cur_frame = 0;

		do_setMatrice(animation.frames[anim_cur_frame].mat);
	}
}


int Eyes::display_stop()
{
	setMatrice(stop);
	return EYESSTOP;
}
