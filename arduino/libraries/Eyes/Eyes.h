#ifndef EYES_H
#define EYES_H

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            7

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      70

#define LINEWIDTH 14


#define EYESVIDE 0
#define EYESSTRAIGHT 1
#define EYESRIGHT 2
#define EYESLEFT 3
#define EYESTOP 4
#define EYESBOTTOM 5
#define EYESEXCLAMATIONS 6
#define EYESINTEROGATIONS 7
#define EYESSTOP 8
#define EYESHELLO 9
#define EYESERROR 10

class Eyes
{
public:
    Eyes(int pin);
    ~Eyes();
    void begin();
    int display_void();
    int display_stop();
    int gaze_direction(byte value);
    int setMatrice(char * mat);
    int setMatrice(int id);

public:
    Adafruit_NeoPixel * pixels;

};

#endif
