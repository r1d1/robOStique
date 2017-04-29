/*
 *  Sensors.h
 *  
 *	Sensors from robOStique framework for Arduino
 *
 *  Created by R1M@ster on 27/06/14.
 *  Copyright 2014 __MyCompanyName__. All rights reserved.
 *
 */

#include "Arduino.h"

namespace robOStique
{

// ===== Sensors Classes =====

// ---- Distance Sensing ----
class UltrasonicSensor
{
	public :
		UltrasonicSensor();
		UltrasonicSensor(const int pinToSet);
		UltrasonicSensor(const int pinToSet, float maxSensingDistance);
		~UltrasonicSensor();

		float microsecondsToCentimeters(float microseconds);
		float senseUS_prox();
		float senseUS_dist();
		bool isNear_prox(float border);
		bool isNear_dist(float border);
		
	private :
		int pin_;
		int maxDist_;
};

}
