#include <math.h>

#define PIN_X A1
#define PIN_Y A0

int rest_dx, rest_dy;

void setup()
{
	Serial.begin(9600);

	rest_dx = 512 - analogRead(PIN_X);
	rest_dy = 512 - analogRead(PIN_Y);
}

void analogGamepad(float dx, float dy, float *linear, float *angular)
{
	float norm = sqrt(dx * dx + dy * dy);

	if (norm < 0.01) {
		*linear = 0.0;
		*angular = 0.0;
		return;
	}

	if (norm > 1) {
		dx = dx / norm;
		dy = dy / norm;
		norm = 1;
	}

	float theta = atan(dy / dx);
	if (dx < 0)
		theta = theta + PI;
	else if (dy < 0)
		theta = theta + 2 * PI;

	*linear = sin(theta) * norm;
	*angular = -1. * cos(theta) * norm * 0.5 / 0.2;
}

void loop()
{
	float dx = map(analogRead(PIN_X) + rest_dx, 0, 1023, -100, 100) / 100.0,
	       dy = map(analogRead(PIN_Y) + rest_dy, 0, 1023, -100, 100) / 100.0;

	union { uint8_t i[4]; float f; } linear, angular;

	analogGamepad(dx, dy, &linear.f, &angular.f);

	Serial.write("\xff\xff\xff\xff");
	Serial.write(linear.i, 4);
	Serial.write(angular.i, 4);
	delay(100);
}
