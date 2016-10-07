#ifndef EYES_H
#define EYES_H

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>


class Eyes
{
	public:
		Eyes();
		Eyes(int lines, int column);
		~Eyes();
		void print();
		void show();
		void begin();
		void display_void();
		void display_panic();
		void display_wifi();
		void display_obstacle();
		void display_collision();
		void gaze_direction(byte value);	
		
	public:

		Adafruit_NeoPixel**matrix;

		int _nbLine;
		int _nbColumn;	
};

#endif
