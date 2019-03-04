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
// PING or HCSR04
// Main difference is the nb of pins and how to control them
// but interface should be the same
UltrasonicSensor::UltrasonicSensor(){}

UltrasonicSensor::UltrasonicSensor(const int echo_pin, const int trigger_pin, float max_sensing_distance = 400.0)
{
	echo_pin_ = echo_pin;
	trigger_pin_ = trigger_pin;
	max_dist_ = max_sensing_distance;
}
			
//UltrasonicSensor::UltrasonicSensor(const int pinToSet){ pin_ = pinToSet; }

UltrasonicSensor::~UltrasonicSensor(){}

float UltrasonicSensor::microsecondsToCentimeters(float microseconds)
{
	// The speed of sound is 340 m/s or 29 microseconds per centimeter.
	// The ping travels out and back, so to find the distance of the
	// object we take half of the distance travelled.
	float value = microseconds / 29.0 / 2.0;
	value =  (value > max_dist_) ? max_dist_ : value;
	return value;
}

// Measure from Ultrasound Sensor and return linear Proximity ratio computed over maxDist_ meters
float UltrasonicSensor::senseUS_prox()
{
	float duration, prox;
	// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	pinMode(trigger_pin_, OUTPUT);
	digitalWrite(trigger_pin_, LOW);
	delayMicroseconds(2);
	digitalWrite(trigger_pin_, HIGH);
	delayMicroseconds(5);
	digitalWrite(trigger_pin_, LOW);

	// The same pin is used to read the signal from the PING))): a HIGH
	// pulse whose duration is the time (in microseconds) from the sending
	// of the ping to the reception of its echo off of an object.
	pinMode(echo_pin_, INPUT);
	duration = pulseIn(echo_pin_, HIGH);

	// convert the time into a linear proximity ratio
	prox = 1.0 - microsecondsToCentimeters(duration) / max_dist_;

  return prox;
}

// Measure distance from Ultrasonic Sensor and return the distance in centimeters
float UltrasonicSensor::senseUS_dist()
{
	float duration, cm;
	// The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
	// Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	pinMode(trigger_pin_, OUTPUT);
	digitalWrite(trigger_pin_, LOW);
	delayMicroseconds(2);
	digitalWrite(trigger_pin_, HIGH);
	delayMicroseconds(5);
	digitalWrite(trigger_pin_, LOW);

	// The same pin is used to read the signal from the PING))): a HIGH
	// pulse whose duration is the time (in microseconds) from the sending
	// of the ping to the reception of its echo off of an object.
	pinMode(echo_pin_, INPUT);
	duration = pulseIn(echo_pin_, HIGH);

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
