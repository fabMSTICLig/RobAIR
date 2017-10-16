#include "md49.h"

#define CMD        (byte)0x00  // MD49 command address of 0
#define GET_SPEED1       0x21  // returns the current requested speed of motor 1
#define GET_SPEED2       0x22  // returns the current requested speed of motor 2
#define GET_ENC1         0x23  // motor 1 encoder count, 4 bytes returned high byte first (signed)
#define GET_ENC2         0x24  // motor 2 encoder count, 4 bytes returned high byte first (signed)
#define GET_ENCS         0x25  // returns 8 bytes -  encoder1 count, encoder2 count
#define GET_VOLTS        0x26  // returns the input battery voltage level
#define GET_CURR1        0x27  // returns the current drawn by motor 1
#define GET_CURR2        0X28  // returns the current drawn by motor 2
#define GET_ACCEL        0x2A  // returns the current acceleration level
#define GET_MODE         0x2B  // returns the current selected mode
#define GET_VI           0x2C  // returns battery volts, motor1 current and then motor2 current (3 bytes)
#define GET_ERROR        0x2D  // returns a byte within which the bits indicate errors on the MD49
#define SET_SPEED1       0x31
#define SET_SPEED2       0x32
#define SET_ACCEL        0x33
#define SET_MODE         0X34
#define RESET_ENCODERS   0x35
#define DISABLE_REGUL    0x36
#define ENABLE_REGUL     0x37
#define DISABLE_TIMEOUT  0x38
#define ENABLE_TIMEOUT   0x39

MD49::MD49(HardwareSerial & serial) : m_serial(serial), m_mode(MD49_MODE0)
{
}

void MD49::sendCmd(byte cmd, byte val)
{
	m_serial.write((byte)CMD);
	m_serial.write(cmd);
	m_serial.write(val);
}


byte MD49::getByte()
{
	m_timeout = millis() + 500;

	while (m_serial.available() <= 0 && millis() < m_timeout)
		delay(1);

	if (millis() > m_timeout)
		return -1;
	return m_serial.read();
}

int MD49::getInt()
{
	m_timeout = millis() + 500;

	while (m_serial.available() < 4 && millis() < m_timeout)
		delay(1);

	if (millis() > m_timeout)
		return -1;

	int ret = 0;
	ret = m_serial.read() << 24;
	ret += m_serial.read() << 16;
	ret += m_serial.read() << 8;
	ret += m_serial.read();
	return ret;
}

void MD49::init(int baud)
{
	m_serial.begin(baud);
}

int MD49::getSpeed(byte num)
{
	sendCmd(num == 1 ? GET_SPEED1 : GET_SPEED2);
	return getByte();
}

int MD49::getTurn()
{
	return getSpeed(2);
}

int MD49::getCurrent(byte num)
{
	sendCmd(num == 1 ? GET_CURR1 : GET_CURR2);
	return getByte();

}

int MD49::getEncoder(byte num)
{
	sendCmd(num == 1 ? GET_ENC1 : GET_ENC2);
	return getInt();
}

void MD49::getEncoders(int *encs)
{
	sendCmd(GET_ENCS);
	encs[0] = getInt();
	encs[1] = getInt();
}

int MD49::getVolt()
{
	sendCmd(GET_VOLTS);
	return getByte();
}

int MD49::getAccel()
{
	sendCmd(GET_ACCEL);
	return getByte();

}

MD49_MODE_t MD49::getMode()
{
	sendCmd(GET_MODE);
	m_mode = (MD49_MODE_t)getByte();
	return m_mode;
}

byte MD49::getError()
{
	sendCmd(GET_ERROR);
	return getByte();
}

boolean MD49::checkspeed(int val)
{
	if (val < 0 && (m_mode == MD49_MODE0 || m_mode == MD49_MODE2))
		return false;
	if (val > 128 && (m_mode == MD49_MODE1 || m_mode == MD49_MODE3))
		return false;

	return true;
}

void MD49::setSpeed1(int speed)
{
	if (checkspeed(speed))
		sendCmd(SET_SPEED1, speed);
}

void MD49::setSpeed2(int speed)
{
	if (checkspeed(speed) && (m_mode == MD49_MODE1 || m_mode == MD49_MODE0))
		sendCmd(SET_SPEED2, speed);
}

void MD49::setTurn(int turn)
{
	setSpeed2(turn);
}

void MD49::setAccel(byte acc)
{
	sendCmd(SET_ACCEL, acc);

}

void MD49::setMode(MD49_MODE_t mode)
{
	m_mode = mode;
	sendCmd(SET_MODE, (byte)mode);

}

void MD49::resetEncoder()
{
	sendCmd(RESET_ENCODERS);
}

void MD49::setRegulator(boolean on)
{
	sendCmd(on ? ENABLE_REGUL : DISABLE_REGUL);
}

void MD49::setTimeout(boolean on)
{
	sendCmd(on ? ENABLE_TIMEOUT : DISABLE_TIMEOUT);
}


void MD49::stop()
{
	getMode();

	if (m_mode == MD49_MODE1 || m_mode == MD49_MODE0) {
		setSpeed1(128);
		setSpeed2(128);
	}

	if (m_mode == MD49_MODE2 || m_mode == MD49_MODE3) {
		setSpeed1(0);
		setTurn(0);
	}
}
