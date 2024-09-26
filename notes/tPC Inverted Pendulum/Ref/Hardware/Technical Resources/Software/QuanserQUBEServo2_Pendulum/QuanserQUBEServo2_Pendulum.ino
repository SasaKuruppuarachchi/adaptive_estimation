/*
Quanser QUBE Servo Control

An example using the Arduino UNO board to communicate with the Quanser QUBE Servo 2
through the microcontroller interface panel.  When the pendulum is positioned within
30 degrees of upright, the controller will balance the pendulum. Make sure the 
pendulum is resting initially.

Select 115200 baud on the Arduino Serial Monitor.
Tested with Arduino IDE 1.6.8 (arduino.cc) and Arduino IDE 1.7.8 (arduino.org).

Created 2016 by Quanser Inc.
www.quanser.com
*/

// include the QUBE Servo 2 library
#include "QUBEServo2.h"

// include the SPI library and the math library
#include <SPI.h>
#include <math.h>

bool startup = true;  // true the first time the sketch is run after the Arduino power is cycled or the reset pushbutton is pressed

unsigned long previousMicros = 0;  // used to store the last time the SPI data was written
const long sampleTime = 1000;  // set the sample time (the interval between SPI transactions) to 1000us = 1ms

// set pin 10 as the slave select for the Quanser QUBE
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
const int slaveSelectPin = 10;

// initialize the SPI data to be written
byte mode = 1;                      // normal mode = 1
byte writeMask = B00011111;         // Bxxxxxx11 to enable the motor, Bxxx111xx to enable the LEDs, Bx11xxxxx to enable writes to the encoders
byte LEDRedMSB = 0;                 // red LED command MSB
byte LEDRedLSB = 0;                 // red LED command LSB
byte LEDGreenMSB = 0;               // green LED command MSB
byte LEDGreenLSB = 0;               // green LED command LSB
byte LEDBlueMSB = 0;                // blue LED command MSB
byte LEDBlueLSB = 0;                // blue LED command LSB
byte encoder0ByteSet[3] = {0,0,0};  // encoder0 is set to this value only when writes are enabled with writeMask
byte encoder1ByteSet[3] = {0,0,0};  // encoder1 is set to this value only when writes are enabled with writeMask
byte motorMSB = 0x80;               // motor command MSB must be B1xxxxxxx to enable the amplifier
byte motorLSB = 0;                  // motor command LSB

// initialize the SPI data to be read
byte moduleIDMSB = 0;               // module ID MSB (module ID for the QUBE Servo is 777 decimal)
byte moduleIDLSB = 0;               // module ID LSB
byte encoder0Byte[3] = {0,0,0};     // arm encoder counts
byte encoder1Byte[3] = {0,0,0};     // pendulum encoder counts
byte tach0Byte[3] = {0,0,0};        // arm tachometer
byte moduleStatus = 0;              // module status (the QUBE Servo sends status = 0 when there are no errors)
byte currentSenseMSB = 0;           // motor current sense MSB 
byte currentSenseLSB = 0;           // motor current sense LSB

// global variables for LED intensity (999 is maximum intensity, 0 is off)
int LEDRed = 0;
int LEDGreen = 0;
int LEDBlue = 0;

// global variables for the pendulum balance control
float theta_n_k1 = 0;
float theta_dot_k1 = 0;
float alpha_n_k1 = 0;
float alpha_dot_k1 = 0;

//Setup serial builder
Display displayData;

void setup() {
  // set the slaveSelectPin as an output
  pinMode (slaveSelectPin, OUTPUT);
  
  // initialize SPI
  SPI.begin();
  
  // initialize serial communication at 115200 baud
  // (Note that 115200 baud must be selected from the drop-down list on the Arduino
  // Serial Monitor for the data to be displayed properly.)
  Serial.begin(115200);
}

void loop() {
  
  // after the Arduino power is cycled or the reset pushbutton is pressed, call the resetQUBEServo function
  if (startup) {
    resetQUBEServo();
    startup = false;
  }
  
  // if the difference between the current time and the last time an SPI transaction
  // occurred is greater than the sample time, start a new SPI transaction
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= sampleTime) {
    previousMicros = previousMicros + sampleTime;  
    
    // initialize the SPI bus using the defined speed, data order and data mode
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    // take the slave select pin low to select the device
    digitalWrite(slaveSelectPin, LOW);
    
    // send and receive the data via SPI (except for the motor command, which is sent after the pendulum control code) 
    moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
    moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
    encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
    encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
    encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
    encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
    encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
    encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
    tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
    tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
    tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
    moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
    currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
    currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
    SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
    
    // combine the received bytes to assemble the sensor values

    /*Module ID*/
    int moduleID = (moduleIDMSB << 8) | moduleIDLSB;
    
    /*Motor Encoder Counts*/
    long encoder0 = ((long)encoder0Byte[2] << 16) | (long)(encoder0Byte[1] << 8) | encoder0Byte[0];
    if (encoder0 & 0x00800000) {
      encoder0 = encoder0 | 0xFF000000;
    }
    // convert the arm encoder counts to angle theta in radians
    float theta = (float)encoder0 * (-2.0 * M_PI / 2048);

    /*Pendulum Encoder Counts*/
    long encoder1 = ((long)encoder1Byte[2] << 16) | (long)(encoder1Byte[1] << 8) | encoder1Byte[0];
    if (encoder1 & 0x00800000) {
      encoder1 = encoder1 | 0xFF000000;
    }
    // wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
    encoder1 = encoder1 % 2048;
    if (encoder1 < 0) {
      encoder1 = 2048 + encoder1;
    }    
    // convert the pendulum encoder counts to angle alpha in radians
    float alpha = (float)encoder1 * (2.0 * M_PI / 2048) - M_PI;

    /*Current Sense Value*/
    float currentSense = (currentSenseMSB << 8) | currentSenseLSB;

    /*Start Pendulum Control Code*/
    
    // if the pendulum is within +/-30 degrees of upright, enable balance control
    if ( fabs(alpha) <= (30.0 * M_PI / 180.0) ) {
      // turn on the green LEDs, maximum intensity
      LEDRed = 0;
      LEDGreen = 999;
      LEDBlue = 0;
      
      // transfer function = 50s/(s+50)
      // z-transform at 1ms = (50z - 50)/(z-0.9512)
      float theta_n = -theta;
      float theta_dot = (50.0 * theta_n) - (50.0 * theta_n_k1) + (0.9512 * theta_dot_k1);
      theta_n_k1 = theta_n;
      theta_dot_k1 = theta_dot;
      
      // transfer function = 50s/(s+50)
      // z-transform at 1ms = (50z - 50)/(z-0.9512)
      float alpha_n = -alpha;
      float alpha_dot = (50.0 * alpha_n) - (50.0 * alpha_n_k1) + (0.9512 * alpha_dot_k1);
      alpha_n_k1 = alpha_n;
      alpha_dot_k1 = alpha_dot;
      
      // multiply by proportional and derivative gains
      float kp_theta = 2.0;
      float kd_theta = -2.0;
      float kp_alpha = -30.0;
      float kd_alpha = 2.5;
      float motorVoltage = (theta * kp_theta) + (theta_dot * kd_theta) + (alpha * kp_alpha) + (alpha_dot * kd_alpha);
      
      // set the saturation limit to +/- 15V
      if (motorVoltage > 15.0) {
        motorVoltage = 15.0;
      }
      else if (motorVoltage < -15.0) {
        motorVoltage = -15.0;
      }
      
      // invert for positive CCW
      motorVoltage = -motorVoltage;
      
      // convert the analog value to the PWM duty cycle that will produce the same average voltage
      float motorPWM = motorVoltage * (625.0 / 15.0);
      
      int motor = (int)motorPWM;  // convert float to int (2 bytes)
      motor = motor | 0x8000;  // motor command MSB must be B1xxxxxxx to enable the amplifier
      motorMSB = (byte)(motor >> 8);
      motorLSB = (byte)(motor & 0x00FF);      
    }
    else {
      // turn on the red and green LEDs for orange
      LEDRed = 999;
      LEDGreen = 500;
      LEDBlue = 0;
      
      // turn off the motor
      motorMSB = 0x80;
      motorLSB = 0;
    }

    /*End of Pendulum Code*/

    // convert the LED intensities to MSB and LSB
    LEDRedMSB = (byte)(LEDRed >> 8);
    LEDRedLSB = (byte)(LEDRed & 0x00FF);
    LEDGreenMSB = (byte)(LEDGreen >> 8);
    LEDGreenLSB = (byte)(LEDGreen & 0x00FF);
    LEDBlueMSB = (byte)(LEDBlue >> 8);
    LEDBlueLSB = (byte)(LEDBlue & 0x00FF);
    
    // send the motor data via SPI
    SPI.transfer(motorMSB);
    SPI.transfer(motorLSB);
    
    // take the slave select pin high to de-select the device
    digitalWrite(slaveSelectPin, HIGH);
    SPI.endTransaction();
    
    displayData.buildString(theta, alpha, currentSense, moduleID, moduleStatus);
  }
  // print data to the Arduino Serial Monitor in between SPI transactions
  // (Note that the Serial.print() function is time consuming.  Printing the entire
  // string at once would exceed the sample time required to balance the pendulum.)
  else {
    // only print if there's a string ready to be printed, and there's enough time before the next SPI transaction
    if ( (displayData.dDataReady) && (currentMicros - previousMicros <= (sampleTime - 100)) ) {
      // if there is room available in the serial buffer, print one character
      if(Serial.availableForWrite() > 0) {
        Serial.print(displayData.dData[displayData.dDataIndex]);
        displayData.dDataIndex = displayData.dDataIndex + 1;
        // if the entire string has been printed, clear the flag so a new string can be obtained
        if(displayData.dDataIndex == displayData.dData.length()) {
          displayData.dDataReady = false;
        }
      }
    }
  }
}

// This function is used to clear the stall error and reset the encoder values to 0.
// The motor and LEDs are turned off when this function is called.
void resetQUBEServo() {
  
  // enable the motor and LEDs, and enable writes to the encoders
  writeMask = B01111111;
  
  // turn off the LEDs
  LEDRedMSB = 0;
  LEDRedLSB = 0;
  LEDGreenMSB = 0;
  LEDGreenLSB = 0;
  LEDBlueMSB = 0;
  LEDBlueLSB = 0;
  
  // reset the encoder values to 0
  encoder0ByteSet[2] = 0;
  encoder0ByteSet[1] = 0;
  encoder0ByteSet[0] = 0;
  encoder1ByteSet[2] = 0;
  encoder1ByteSet[1] = 0;
  encoder1ByteSet[0] = 0;
  
  // turn off the motor, and clear the stall error by disabling the amplifier
  motorMSB = 0;  // motor command MSB is B0xxxxxxx to disable the amplifier
  motorLSB = 0;
  
  // initialize the SPI bus using the defined speed, data order and data mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, LOW);   // take the slave select pin low to select the device
  
  // send and receive the data via SPI
  moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
  moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
  encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
  currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
  SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
  SPI.transfer(motorMSB);                              // send the motor MSB
  SPI.transfer(motorLSB);                              // send the motor LSB
  
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();
  
  writeMask = B00011111;  // enable the motor and LEDs, disable writes to the encoders
  motorMSB = 0x80;  // enable the amplifier
}


