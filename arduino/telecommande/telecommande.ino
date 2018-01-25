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

void analogGamepad(double dx, double dy, double *linear, double *angular)
{
	double norm = sqrt(dx * dx + dy * dy);

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

	double theta = atan(dy / dx);
	if (dx < 0)
		theta = theta + PI;
	else if (dy < 0)
		theta = theta + 2 * PI;

	*linear = sin(theta) * norm;
	*angular = -1. * cos(theta) * norm * 0.5 / 0.2;
}

void loop()
{
	double dx = map(analogRead(PIN_X) + rest_dx, 0, 1023, -100, 100) / 100.0,
	       dy = map(analogRead(PIN_Y) + rest_dy, 0, 1023, -100, 100) / 100.0;

	double linear, angular;

	analogGamepad(dx, dy, &linear, &angular);

	Serial.print("P");
	Serial.print(linear);
	Serial.print(" ");
	Serial.println(angular);
	delay(100);
}
