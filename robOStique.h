/*
 *  robOStique.h
 *  
 *
 *  Created by R1D1 on 28/06/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ROBOSTIQUE_H
#define ROBOSTIQUE_H
 
#include "Arduino.h"

#include "Sensors.h"

namespace robOStique
{

/*class UltrasonicSensor
{
	public :
		UltrasonicSensor();
//		UltrasonicSensor(const int pinToSet);
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
*/

class MotorControl
{
	public :
		MotorControl();
		MotorControl(const int pinSpd, const int pinI1, const int pinI2);
		~MotorControl();
	
		// Rotate a wheel depending on a speed and its sign
		void rotate(float speed);
		float signrotate(float speed);
		// Forward and Max speed = 150, (L)eft or (R)ight argument to pass.
		//void runMotor(const char side, int wheelspeed);

	private :
		int pinSpeed_;
		int pinI1_;
		int pinI2_;
};

class SevenBitDisplay
{
	public :
		SevenBitDisplay();
		SevenBitDisplay(int b1, int b2, int b3, int b4, int b5, int b6, int b7, int b8);
		~SevenBitDisplay();
	
		void display(byte symbole);
		//void displayDec(int symbole);
		int* displayDec(byte symbole, bool floatval=false);

	private :
		int pins[8];
		int pin1;
		int pin2;
		int pin3;
		int pin4;
		int pin5;
		int pin6;
		int pin7;
		int pin8;
};

}

#endif
