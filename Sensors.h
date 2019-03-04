/*
 *  Sensors.h
 *  
 *	Sensors from robOStique framework for Arduino
 *
 *  Created by R1D1 on 27/06/14.
 *  Copyright 2014 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef SENSOR_H
#define SENSOR_H
#include "Arduino.h"

namespace robOStique
{

// ===== Sensors Classes =====

// ---- Distance Sensing ----
class UltrasonicSensor
{
	public :
		UltrasonicSensor();
		UltrasonicSensor(const int echo_pin, const int trigger_pin, float maxSensingDistance);
		~UltrasonicSensor();

		float microsecondsToCentimeters(float microseconds);
		float senseUS_prox();
		float senseUS_dist();
		bool isNear_prox(float border);
		bool isNear_dist(float border);
		
	private :
		int echo_pin_;
		int trigger_pin_;
		int max_dist_;
};

}

#endif
