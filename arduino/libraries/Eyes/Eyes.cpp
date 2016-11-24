#include "Arduino.h"
#include "Eyes.h"


char vide[]={
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

char eyesstraight[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

char eyesright[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,2,7,0,0,7,0,0,2,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

char eyesleft[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,2,0,0,7,0,0,7,2,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};


char eyestop[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

char eyesbottom[]={
    0,0,7,7,7,0,0,0,0,7,7,7,0,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,0,0,7,0,0,7,0,0,0,7,0,
    0,7,0,2,0,7,0,0,7,0,2,0,7,0,
    0,0,7,7,7,0,0,0,0,7,7,7,0,0
};

char exclamations[]={
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,1,0,1,0,1,0,1,0,1,0,0,0
};

char interogations[]={
    0,4,4,4,0,4,4,4,0,4,4,4,0,0,
    0,4,0,4,0,4,0,4,0,4,0,4,0,0,
    0,0,4,4,0,0,4,4,0,0,4,4,0,0,
    0,0,4,0,0,0,4,0,0,0,4,0,0,0,
    0,0,2,0,0,0,2,0,0,0,2,0,0,0
};

char stop[]={
    1,1,1,0,1,1,1,0,1,1,1,0,1,1,
    1,0,0,0,0,1,0,0,1,0,1,0,1,1,
    1,1,1,0,0,1,0,0,1,0,1,0,1,0,
    0,0,1,0,0,1,0,0,1,0,1,0,1,0,
    1,1,1,0,0,1,0,0,1,1,1,0,1,0
};

char hello[]={
    7,0,7,0,2,2,0,1,0,1,0,5,5,5,
    7,0,7,0,2,0,0,1,0,1,0,5,0,5,
    7,7,7,0,2,2,0,1,0,1,0,5,0,5,
    7,0,7,0,2,0,0,1,0,1,0,5,0,5,
    7,0,7,0,2,2,0,1,1,1,1,5,5,5

};



char error[]={
    1,1,3,3,3,1,1,1,3,3,3,1,1,1,
    1,0,3,0,3,1,0,1,3,0,3,1,0,1,
    1,1,3,3,3,1,1,1,3,0,3,1,1,1,
    1,0,3,3,0,1,1,0,3,0,3,1,1,0,
    1,1,3,0,3,1,0,1,3,3,3,1,0,1
};


char * tabeyes[] = {
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

int Eyes::setMatrice(char * mat)
{

    int ligne = 0;
    int id = 0;
    int mir = 0;
    for(int i=0; i<NUMPIXELS; i++) {
        ligne = 4- i /LINEWIDTH;
        if(ligne % 2 == 0)
        {
            id=i;
        }
        else
        {
            id = ligne*LINEWIDTH+(LINEWIDTH-(i%LINEWIDTH))-1;
        }
        int puiss=100;
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
