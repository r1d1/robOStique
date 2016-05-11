/*
 *  robOStique.cpp
 *  
 *
 *  Created by R1M@ster on 28/06/2012.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 * Modified on 22/02/2013
 */

#include "robOStique.h"

using namespace robOStique;

/* ----------------- Ultrasonic Sensor ----------------- */
/*
UltrasonicSensor::UltrasonicSensor(){}

UltrasonicSensor::UltrasonicSensor(const int pinToSet, float maxSensingDistance = 400.0)
{
	pin_ = pinToSet;
	maxDist_ = maxSensingDistance;
}
			
UltrasonicSensor::UltrasonicSensor(const int pinToSet)
{
	pin_ = pinToSet;
}

float UltrasonicSensor::microsecondsToCentimeters(float microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  float value = microseconds / 29.0 / 2.0;
  value =  (value > maxDist_) ? maxDist_ : value;
  return value;
}

// Measure from Ultrasound Sensor and return Proximity computed against 4.00 meters
float UltrasonicSensor::senseUS_prox()
{
  float duration, prox;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin_, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin_, INPUT);
  duration = pulseIn(pin_, HIGH);
//  Serial.println(duration);
  // convert the time into a distance ratio
  prox = 1.0 - microsecondsToCentimeters(duration) / maxDist_;
 // Serial.println(cm);
  return prox;
//  return duration;
}

// Measure distance from Ultrasonic Sensor and return the distance in centimeters
float UltrasonicSensor::senseUS_dist()
{
  float duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin_, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin_, INPUT);
  duration = pulseIn(pin_, HIGH);
//  Serial.println(duration);
  // convert the time into a distance ratio
  cm = microsecondsToCentimeters(duration);
 // Serial.println(cm);
  return cm;
//  return duration;
}

bool UltrasonicSensor::isNear_prox(float border)
{
	if(this->senseUS_prox() >  border){ return true; }
	return false;
}

bool UltrasonicSensor::isNear_dist(float border)
{
	if(this->senseUS_dist() < border){ return true; }
	return false;
}
*/
/* ----------------- Motor Control ----------------- */

MotorControl::MotorControl(){}

MotorControl::MotorControl(const int pinSpd, const int pinI1, const int pinI2)
{
	pinSpeed_ = pinSpd;
	pinI1_ = pinI1;
	pinI2_ = pinI2;
}

MotorControl::~MotorControl(){}
	
void MotorControl::rotate(float speed)
{
	float signe = 1.0;
	// si speed > 0, signe = 0, si speed < 0, signe = -2*speed / (2*speed) = -1, si speed = 0, problème.
	//signe = (speed == 0) ? 0 : (speed - fabs(speed)) / (2*fabs(speed));
	
	if(speed < 0)
	{
		analogWrite(pinSpeed_, speed);
		digitalWrite(this->pinI1_, LOW);
		digitalWrite(this->pinI2_, HIGH);
	}
	else
	{
		analogWrite(pinSpeed_, speed);
		digitalWrite(this->pinI1_, HIGH);
		digitalWrite(this->pinI2_, LOW);
	}
	delay(200);
}

float MotorControl::signrotate(float speed)
{
	float signe = 1.0;
	// si speed > 0, signe = 0, si speed < 0, signe = -2*speed / (2*speed) = -1, si speed = 0, problème.
	//signe = (speed == 0) ? 0 : (speed - fabs(speed)) / (2*fabs(speed));
	if(speed < 0){ signe = -1; }

	return signe;
}
/*
void MotorControl::runMotor(const char side, int wheelspeed)
{
  if( (side != 'L') && (side != 'R')){ return; }
  if(side == 'L')
  {
     // Left Wheel Forward
    analogWrite(speedpinB, (wheelspeed < 150) ? wheelspeed : 150); 

    digitalWrite(pinI3, HIGH);
    digitalWrite(pinI4, LOW); 
  }
  if(side == 'R')
  {
     // Right Wheel Forward
    analogWrite(speedpinA, (wheelspeed < 150) ? wheelspeed : 150); 

    digitalWrite(pinI1, HIGH);
    digitalWrite(pinI2, LOW); 
  }
}*/

/* ----------------- SevenBitDisplay ----------------- */

SevenBitDisplay::SevenBitDisplay(){}

SevenBitDisplay::SevenBitDisplay(int b1, int b2, int b3, int b4, int b5, int b6, int b7, int b8)
{
	pin1 = b1;
	pin2 = b2;
	pin3 = b3;
	pin4 = b4;
	pin5 = b5;
	pin6 = b6;
	pin7 = b7;
	pin8 = b8;
}

void SevenBitDisplay::display(byte symbol)
{
  digitalWrite(pin8, symbol & B00000001);
  digitalWrite(pin7, symbol & B00000010);
  digitalWrite(pin6, symbol & B00000100);
  digitalWrite(pin5, symbol & B00001000);
  digitalWrite(pin4, symbol & B00010000);
  digitalWrite(pin3, symbol & B00100000);
  digitalWrite(pin2, symbol & B01000000);
  digitalWrite(pin1, symbol & B10000000);
}

SevenBitDisplay::~SevenBitDisplay(){}

/*
void SevenBitDisplay::display(int s1, int s2,int s3, int s4,int s5, int s6, int s7, int s8)
{
  digitalWrite(pin1, s1);
  digitalWrite(pin2, s2);

  digitalWrite(pin3, s3);
  digitalWrite(pin4, s4);
  digitalWrite(pin5, s5);
  digitalWrite(pin6, s6);
  digitalWrite(pin7, s7);
  digitalWrite(pin8, s8);
}*/
