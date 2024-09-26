/*
Quanser QUBE Servo 2 Header

Created 2016 by Quanser Inc.
www.quanser.com
*/

#ifndef QUBEServo2_h
#define QUBEServo2_h

#include <Arduino.h>

class Display
{
public:
	Display(); //Build the display class
	void buildString(float theta, float alpha, float currentSense, int moduleID, int moduleStatus); //Create the data string
	String dData;  // string that will be printed to the Arduino Serial Monitor
	boolean dDataReady;  // true when there is a string ready to be printed
	int dDataIndex;  // used to print the string one character at a time
private:
	void reset(int moduleID, int moduleStatus); //reset the data string
};
#endif
