#include "Arduino.h"
#include "Papierlogik.h"



Papierlogik::Papierlogik(){
	_alpha = 0.98;
	_last_sensorValue = 0.0;
}

Papierlogik::Papierlogik(float alpha){
	_alpha = alpha;
	_last_sensorValue = 0.0;
}


Papierlogik::~Papierlogik(){
}

void Papierlogik::init(float sensor_value){
	_last_sensorValue  = sensor_value;
}
	

bool Papierlogik::detect_contact(float sensor_value, float thershold){
	_last_sensorValue = _last_sensorValue*_alpha  + (1.0-_alpha)*sensor_value;
	float diff  = _last_sensorValue - sensor_value;

	if (diff < thershold ){ return 1;}
	
	return 0;
}


