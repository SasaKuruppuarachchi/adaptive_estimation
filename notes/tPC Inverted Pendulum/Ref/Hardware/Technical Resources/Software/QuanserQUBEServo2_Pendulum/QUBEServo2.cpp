/*
Quanser QUBE Servo 2 Core Library

Created 2016 by Quanser Inc.
www.quanser.com
*/

#include <Arduino.h>
#include "QUBEServo2.h"

//Status Bit Masks
const byte stallErrorMask = B00000100;
const byte stallDetectedMask = B00000010;
const byte amplifierFaultMask = B00000001;

//Serial Variables
static String dData;  // string that will be printed to the Arduino Serial Monitor
static boolean dDataReady;  // true when there is a string ready to be printed
static int dDataIndex;  // used to print the string one character at a time

//Create the display class
Display::Display()
{
	this->dData = "";
	this->dDataReady = false;
	this->dDataIndex = 0;

	// reserve 256 bytes for the string that will be printed to the Arduino Serial Monitor
	dData.reserve(256);
}


void Display::buildString(float theta, float alpha, float currentSense, int moduleID, int moduleStatus)
{

	if (this->dDataReady == false) {
		// assemble the string to be printed to the Arduino Serial Monitor
		// (Note that the String() conversion function is time consuming.  If more data
		// needs to displayed than in this example, it may be necessary to assemble the
		// displayData string over multiple sample periods.)

		reset(moduleID, moduleStatus);

		this->dData += "\r\nArm angle (deg): ";
		float thetaDeg = theta * (180.0 / M_PI);
		this->dData += String(round(thetaDeg));

		this->dData += "\r\nPendulum angle (deg): ";
		float alphaDeg = alpha * (180.0 / M_PI);
		this->dData += String(round(alphaDeg));

		this->dData += "\r\nCurrent (mA): ";
		float currentSenseAmps = (3.3 / (4.0 * 4095.0)) * ((currentSense / 2.0) - 4095.0);
		this->dData += String(currentSenseAmps * 1000);
		this->dData += "\r\n\n";

		this->dDataReady = true;  // the string is ready to be printed
		this->dDataIndex = 0;
	}
}

void Display::reset(int moduleID, int moduleStatus)
{
	this->dData = "";  // clear the string

	if (moduleID == -1) {
		this->dData += "Module ID: No module detected";
	}
	else {
		this->dData += "Module ID: ";
		this->dData += String(moduleID);
	}

  if (moduleStatus == 0) {
    this->dData += "\r\nStatus: Good";  // '\r' is carriage return and '\n' is new line
  }
  else if (moduleStatus & amplifierFaultMask) {
		this->dData += "\r\nStatus: Amplifier Fault";
	}
  else if (moduleStatus & stallErrorMask) {
    this->dData += "\r\nStatus: Stall Error";
  }
  else if (moduleStatus & stallDetectedMask) {
    this->dData += "\r\nStatus: Stall Detected";
  }
	else {
		this->dData += "\r\nStatus: ";
		this->dData += String(moduleStatus);
	}
}
