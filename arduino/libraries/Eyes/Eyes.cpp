#include "Arduino.h"
#include "Eyes.h"

static byte leftleft[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, //
    0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, //
    0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0 //
}; 

static byte left[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, //
    0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, //
    0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0 //
}; 

static byte middle[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, //
    0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, //
    0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0 //
}; 

static byte right[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, //
    0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, //
    0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0 //
}; 


static byte rightright[] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, //
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, //
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 //
}; 

static byte panic[] =
{
    1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, //
    0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, //
    0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, //
    0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, //
    1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1 //
};


static byte wifi[] =
{
    0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, //
    0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, //
    0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, //
    0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, //
    0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0 //
};

static byte collision[] =
{
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, //
    0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, //
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, //
    0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, //
    0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0 //
};

static byte obstacle[] =
{
    0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, //
    0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, //
    0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, //
    0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, //
    0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0 //
};


Eyes::Eyes(){
	_nbLine = 0;
	_nbColumn = 0;
}

Eyes::Eyes(int lines, int column){
	_nbLine = lines;
	_nbColumn = column;

	matrix = new Adafruit_NeoPixel*[_nbColumn];
	//PIN 1,2,3,4,5,etc.
	for(int i=0; i < _nbLine; i++)
	{
		matrix[i] = new Adafruit_NeoPixel(_nbColumn, i+2, NEO_GRB + NEO_KHZ800);	
	}
}

void Eyes::print()
{

}

void Eyes::show()
{
	// matrix[0]->setPixelColor(0, matrix[0]->Color(0,100,100));
	for(int i=0; i < _nbLine; i++)
	{
		for(int j=0; j<_nbColumn; j++)
		{
			//matrix[i]->setPixelColor(j, matrix[i]->Color(100,0,0));
			//delay(100);
			matrix[i]->setPixelColor(j, matrix[i]->Color(0,0,101));
		}
		matrix[i]->show();
	}
}

void Eyes::begin()
{
	for(int i=0; i < _nbLine; i++)
	{
		matrix[i]->begin();
	}

}

void Eyes::display_void()
{
        for(int i=0; i < _nbLine; i++)
        {
                for(int j=0; j<_nbColumn; j++)
                {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(0,0,0));
                }
                matrix[i]->show();
        }
}

void Eyes::display_panic()
{
        for(int i=0; i < _nbLine; i++)
        {
                for(int j=_nbColumn-1; j>=0; j--)
                {
                   if(panic[_nbColumn*i+j] == 1) {
		        matrix[i]->setPixelColor(j, matrix[i]->Color(255,0,0));
		   } else {
		        matrix[i]->setPixelColor(j, matrix[i]->Color(0,0,0));
		   }	
                }
                matrix[i]->show();
        }
}

void Eyes::display_wifi()
{
        for(int i=0; i < _nbLine; i++)
        {
                for(int j=_nbColumn; j>=0; j--)
                {
                   if(wifi[_nbColumn*i+j] == 1) {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(255,0,0));
                   } else {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(0,0,0));  
                   }
                }
                matrix[i]->show();
        }
}

void Eyes::display_collision()
{
        for(int i=0; i < _nbLine; i++)
        {
                for(int j=0; j<_nbColumn; j++)
                {
                   if(obstacle[_nbColumn*i+j] == 1) {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(255,0,0));
                   } else {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(0,0,0));
                   }
                }
                matrix[i]->show();
        }
}

void Eyes::display_obstacle()
{
        for(int i=0; i < _nbLine; i++)
        {
                for(int j=0; j<_nbColumn; j++)
                {
                   if(obstacle[_nbColumn*i+j] == 1) {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(255,0,0));
                   } else {
                        matrix[i]->setPixelColor(j, matrix[i]->Color(0,0,0));
                   }
                }
                matrix[i]->show();
        }
}


Eyes::~Eyes(){
	for(int i=0; i < _nbLine; i++)
	{
		delete[] matrix[i];
	}	
	delete[] matrix;
}

void Eyes::gaze_direction(byte value){
	
	for(int i=0; i < _nbLine; i++)
	{
		for(int j=0; j<_nbColumn; j++)
		{
			if(value > 0 && value < 55) {
				if(leftleft[_nbColumn*i+j] == 1)
				{
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,0,100));
				}
    				else
				{
                                    if(leftleft[_nbColumn*i+j] == 2) {
				//	matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,100,100));                      
				    }
                                    else 
                                     {
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(0,0,0));
                                     }
				}
				matrix[i]->show();
			}
			if(value >= 56 && value < 100) {
				if(left[_nbColumn*i+j] == 1)
				{
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,0,100));
				}
				else
				{
				    if(leftleft[_nbColumn*i+j] == 2) {
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,100,100));                      
				    }
                                    else 
                                     {
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(0,0,0));
                                     }
				}
				matrix[i]->show();
			}
			if(value >= 101 && value < 155) {
				if(middle[_nbColumn*i+j] == 1)
				{
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,0,100));
				}
				else
				{
			            if(leftleft[_nbColumn*i+j] == 2) {
				//	matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,100,100));                      
				    }
                                    else 
                                     {
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(0,0,0));
                                     }
				}
				matrix[i]->show();
			}
			if(value >= 156 && value < 200) {
				if(right[_nbColumn*i+j] == 1)
				{
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,0,100));
				}
				else
				{
				    if(leftleft[_nbColumn*i+j] == 2) {
				//	matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,100,100));                      
				    }
                                    else 
                                     {
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(0,0,0));
                                     }
				}
				matrix[i]->show();
			}
			if(value >= 201 && value < 256) {
				if(rightright[_nbColumn*i+j] == 1)
				{
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,0,100));
				}
				else
				{
				    if(leftleft[_nbColumn*i+j] == 2) {
				//	matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(100,100,100));                      
				    }
                                    else 
                                     {
					matrix[i]->setPixelColor(_nbColumn - j - 1, matrix[i]->Color(0,0,0));
                                     }
				}
				matrix[i]->show();
			}		
		}
	}		
}


