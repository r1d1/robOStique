/*
 *  Sensors.cpp
 *  
 *	Sensors from robOStique framework for Arduino
 *  Created by R1M@ster on 27/06/14.
 *  Copyright 2014 __MyCompanyName__. All rights reserved.
 *
 */

#include "Sensors.h"

using namespace robOStique;
// ===== Sensors Classes =====

// ---- Distance Sensing ----

// UltrasonicSensor
UltrasonicSensor::UltrasonicSensor(){}

UltrasonicSensor::UltrasonicSensor(const int pinToSet, float maxSensingDistance = 400.0)
{
	pin_ = pinToSet;
	maxDist_ = maxSensingDistance;
}
			
UltrasonicSensor::UltrasonicSensor(const int pinToSet){ pin_ = pinToSet; }

float UltrasonicSensor::microsecondsToCentimeters(float microseconds)
{
	// The speed of sound is 340 m/s or 29 microseconds per centimeter.
	// The ping travels out and back, so to find the distance of the
	// object we take half of the distance travelled.
	float value = microseconds / 29.0 / 2.0;
	value =  (value > maxDist_) ? maxDist_ : value;
	return value;
}

// Measure from Ultrasound Sensor and return linear Proximity ratio computed over maxDist_ meters
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

	// convert the time into a linear proximity ratio
	prox = 1.0 - microsecondsToCentimeters(duration) / maxDist_;

  return prox;
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

	// convert the time into a distance
	cm = microsecondsToCentimeters(duration);

  return cm;
}

// Check for obstacle in neighboorhood according to proximity
bool UltrasonicSensor::isNear_prox(float border)
{
	if(this->senseUS_prox() >  border){ return true; }
	return false;
}

// Check for obstacle in neighboorhood according to distance
bool UltrasonicSensor::isNear_dist(float border)
{
	if(this->senseUS_dist() < border){ return true; }
	return false;
}
