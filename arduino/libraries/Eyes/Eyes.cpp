#include "Arduino.h"
#include "Eyes.h"

#include <avr/pgmspace.h>

#define MAX_POWER 40

const uint8_t PROGMEM tabeyes[][70] = {
	{ // vide
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0
	},
	{ // straight
		0,0,7,7,7,0,0,0,0,7,7,7,0,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,7,0,2,0,7,0,0,7,0,2,0,7,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,0,7,7,7,0,0,0,0,7,7,7,0,0
	},
	{ // right
		0,0,7,7,7,0,0,0,0,7,7,7,0,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,7,2,0,0,7,0,0,7,2,0,0,7,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,0,7,7,7,0,0,0,0,7,7,7,0,0
	},
	{ // left
		0,0,7,7,7,0,0,0,0,7,7,7,0,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,7,0,0,2,7,0,0,7,0,0,2,7,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,0,7,7,7,0,0,0,0,7,7,7,0,0
	},
	{ // top
		0,0,7,7,7,0,0,0,0,7,7,7,0,0,
		0,7,0,2,0,7,0,0,7,0,2,0,7,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,0,7,7,7,0,0,0,0,7,7,7,0,0
	},
	{ // bottom
		0,0,7,7,7,0,0,0,0,7,7,7,0,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,7,0,0,0,7,0,0,7,0,0,0,7,0,
		0,7,0,2,0,7,0,0,7,0,2,0,7,0,
		0,0,7,7,7,0,0,0,0,7,7,7,0,0
	},
	{ // exclamations
		0,0,1,0,1,0,1,0,1,0,1,0,0,0,
		0,0,1,0,1,0,1,0,1,0,1,0,0,0,
		0,0,1,0,1,0,1,0,1,0,1,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,1,0,1,0,1,0,1,0,1,0,0,0
	},
	{ // interrogations
		0,4,4,4,0,4,4,4,0,4,4,4,0,0,
		0,4,0,4,0,4,0,4,0,4,0,4,0,0,
		0,0,4,4,0,0,4,4,0,0,4,4,0,0,
		0,0,4,0,0,0,4,0,0,0,4,0,0,0,
		0,0,2,0,0,0,2,0,0,0,2,0,0,0
	},
	{ // stop
		1,1,1,0,1,1,1,0,1,1,1,0,1,1,
		1,0,0,0,0,1,0,0,1,0,1,0,1,1,
		1,1,1,0,0,1,0,0,1,0,1,0,1,0,
		0,0,1,0,0,1,0,0,1,0,1,0,1,0,
		1,1,1,0,0,1,0,0,1,1,1,0,1,0
	},
	{ // hello
		7,0,7,0,2,2,0,1,0,1,0,5,5,5,
		7,0,7,0,2,0,0,1,0,1,0,5,0,5,
		7,7,7,0,2,2,0,1,0,1,0,5,0,5,
		7,0,7,0,2,0,0,1,0,1,0,5,0,5,
		7,0,7,0,2,2,0,1,1,1,1,5,5,5

	},
	{ // error
		1,1,3,3,3,1,1,1,3,3,3,1,1,1,
		1,0,3,0,3,1,0,1,3,0,3,1,0,1,
		1,1,3,3,3,1,1,1,3,0,3,1,1,1,
		1,0,3,3,0,1,1,0,3,0,3,1,1,0,
		1,1,3,0,3,1,0,1,3,3,3,1,0,1
	}
};

Eyes::Eyes(int pin) : mode(STILL), anim_dl(false)
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

int Eyes::setMatrice(const uint8_t *mat)
{
	int id = 0;

	for(int i=0; i<NUMPIXELS; i++) {
		id = mat_id_to_pixel_id(i);
		uint8_t led = pgm_read_byte(mat + i);
		uint32_t color = pixels->Color(
					((led & 1) == 1 ? MAX_POWER : 0),
					((led & 2) == 2 ? MAX_POWER : 0),
					((led & 4) == 4 ? MAX_POWER : 0));
		pixels->setPixelColor(id, color);
	}

	pixels->show();
}

void Eyes::setMatrice(const uint32_t *mat)
{
	mode = STILL;
	do_setMatrice(mat);
}

void Eyes::do_setMatrice(const uint32_t *mat)
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

bool Eyes::setAnimation(const robairmain::EyesAnim &anim)
{
	if (anim.frame_count > MAX_ANIM_FRAMES
	    || anim.frame_count == 0
	    || anim.index >= MAX_ANIM_FRAMES)
		return false;

	if (!anim_dl) {
		start_animation_download(anim);
	} else if (anim.id != anim_dl_id || anim.frame_count != anim_dl_len) {
		if (anim_dl_start + ANIM_DL_TIMEOUT < millis())
			start_animation_download(anim);
		else
			return false;
	}

	animation[anim.index] = anim;

	for (unsigned int i = 0 ; i < anim_dl_len ; ++i) {
		if (animation[i].frame_count == 0)
			return false;
	}

	// All frames were received
	anim_dl = false;
	start_animation();
	return true;
}

void Eyes::start_animation_download(const robairmain::EyesAnim &anim)
{
	stop_animation();

	anim_dl = true;
	anim_dl_id = anim.id;
	anim_dl_len = anim.frame_count;
	anim_dl_start = millis();

	for (unsigned int i = 0 ; i < anim_dl_len ; ++i)
		animation[i].frame_count = 0;
}

void Eyes::start_animation()
{
	mode = ANIMATION;
	do_setMatrice(animation[0].frame.mat);
	anim_cur_frame = 0;
	anim_last_change = millis();
}

void Eyes::stop_animation()
{
	mode = STILL;
}

void Eyes::animation_step()
{
	if (mode != ANIMATION)
		return;

	unsigned long cur_dur = millis() - anim_last_change;

	if (cur_dur >= animation[anim_cur_frame].duration) {
		anim_last_change = millis();
		anim_cur_frame++;
		if (anim_cur_frame >= animation[anim_cur_frame].frame_count)
			anim_cur_frame = 0;

		do_setMatrice(animation[anim_cur_frame].frame.mat);
	}
}


int Eyes::display_stop()
{
	setMatrice(EYESSTOP);
	return EYESSTOP;
}
