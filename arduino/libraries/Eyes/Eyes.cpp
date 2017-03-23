#include "Arduino.h"
#include "Eyes.h"


uint8_t vide[]={
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t eyesstraight[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t eyesright[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,2,0,0,7,0,0,7,2,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t eyesleft[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,2,7,0,0,7,0,0,2,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};


uint8_t eyestop[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t eyesbottom[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

uint8_t exclamations[]={
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0
};

uint8_t interogations[]={
    0,4,4,4,0,4,4,4,0,4,4,4,0,0,
    0,4,0,4,0,4,0,4,0,4,0,4,0,0,
    0,0,4,4,0,0,4,4,0,0,4,4,0,0,
    0,0,4,0,0,0,4,0,0,0,4,0,0,0,
    0,0,2,0,0,0,2,0,0,0,2,0,0,0
};

uint8_t stop[]={
    1,1,1,0,1,1,1,0,1,1,1,0,1,1,
    1,0,0,0,0,1,0,0,1,0,1,0,1,1,
    1,1,1,0,0,1,0,0,1,0,1,0,1,0,
    0,0,1,0,0,1,0,0,1,0,1,0,1,0,
    1,1,1,0,0,1,0,0,1,1,1,0,1,0
};

uint8_t hello[]={
    7,0,7,0,2,2,0,1,0,1,0,5,5,5,
    7,0,7,0,2,0,0,1,0,1,0,5,0,5,
    7,7,7,0,2,2,0,1,0,1,0,5,0,5,
    7,0,7,0,2,0,0,1,0,1,0,5,0,5,
    7,0,7,0,2,2,0,1,1,1,1,5,5,5

};



uint8_t error[]={
    1,1,3,3,3,1,1,1,3,3,3,1,1,1,
    1,0,3,0,3,1,0,1,3,0,3,1,0,1,
    1,1,3,3,3,1,1,1,3,0,3,1,1,1,
    1,0,3,3,0,1,1,0,3,0,3,1,1,0,
    1,1,3,0,3,1,0,1,3,3,3,1,0,1
};


uint8_t * tabeyes[] = {
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

Eyes::Eyes(int pin){
    pixels = new Adafruit_NeoPixel(NUMPIXELS, pin, NEO_GRB + NEO_KHZ800);
}

Eyes::~Eyes(){
    delete pixels;
}


void Eyes::begin()
{
    pixels->begin();
}

int Eyes::display_void()
{
    for(int i=0; i < NUMPIXELS; i++)
    {
        pixels->setPixelColor(i, pixels->Color(0,0,0));
        pixels->show();
    }
    return EYESVIDE;
}

int Eyes::setMatrice(uint8_t * mat)
{

    int ligne = 0;
    int id = 0;
    int mir = 0;
    for(int i=0; i<NUMPIXELS; i++) {
        ligne = (i /LINEWIDTH);
        if(ligne % 2 == 0)
        {
            id=ligne*LINEWIDTH+((i%LINEWIDTH));
        }
        else
        {
            id = ligne*LINEWIDTH+(LINEWIDTH-(i%LINEWIDTH))-1;
        }
        int puiss=40;
        pixels->setPixelColor(id, pixels->Color(((mat[i] & 1) == 1 ? puiss : 0),((mat[i] & 2) == 2 ? puiss : 0),((mat[i] & 4) == 4 ? puiss : 0)));
    }

    pixels->show();

}

int Eyes::setMatrice(int id)
{
    setMatrice(tabeyes[id]);

    return id;
}


int Eyes::display_stop()
{
    setMatrice(stop);
    return EYESSTOP;
}


int Eyes::gaze_direction(byte value){
    switch (value) {
    case EYESSTRAIGHT:
        setMatrice(eyesleft);
        break;
    case EYESLEFT:
        setMatrice(eyesleft);
        break;
    case EYESRIGHT:
        setMatrice(eyesright);
        break;

    case EYESTOP:
        setMatrice(eyestop);
        break;

    case EYESBOTTOM:
        setMatrice(eyesbottom);
        break;
    }
    return value;
}
