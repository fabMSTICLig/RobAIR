#ifndef PAPIERLOGIK_H
#define PAPIERLOGIK_H

#include "Arduino.h"


class Papierlogik
{
	public:
		Papierlogik();
		Papierlogik(float alpha);

		void init(float sensor_value);
		bool detect_contact(float sensor_value, float thershold);
		
		~Papierlogik();
		
	private:

		float _alpha;
		float _last_sensorValue;
};	

#endif