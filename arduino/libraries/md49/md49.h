#ifndef MD49_H
#define MD49_H

#include <Arduino.h>

typedef enum MD49_MODE_t {
	MD49_MODE0 = 0,
	MD49_MODE1,
	MD49_MODE2,
	MD49_MODE3
};

class MD49 {
private:
	HardwareSerial &m_serial;
	MD49_MODE_t m_mode;

	unsigned long m_timeout;
	void sendCmd(byte cmd, byte val = 0);
	byte getByte(void);
	int getInt(void);
	boolean checkspeed(int val);

public:
	MD49(HardwareSerial &serial);
	void init(int baud);
	int getSpeed(byte num);
	int getTurn(void);
	int getCurrent(byte num);
	int getEncoder(byte num);
	void getEncoders(int *encs);
	int getVolt(void);
	int getAccel(void);
	MD49_MODE_t getMode(void);
	byte getError(void);

	void setSpeed1(int speed);
	void setSpeed2(int speed);
	void setTurn(int turn);
	void setAccel(byte acc);
	void setMode(MD49_MODE_t mode);
	void resetEncoder(void);
	void setRegulator(boolean on);
	void setTimeout(boolean on);
	void stop(void);
};

#endif
