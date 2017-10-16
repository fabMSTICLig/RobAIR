#ifndef PAPIERLOGIK_H
#define PAPIERLOGIK_H

#include "Arduino.h"

class Papierlogik
{
public:
	Papierlogik(void);
	Papierlogik(float alpha);

	void init(float sensor_value);
	bool detect_contact(float sensor_value, float threshold);

	~Papierlogik(void);

private:
	float _alpha;
	float _last_sensorValue;
};	

#endif
