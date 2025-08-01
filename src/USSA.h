#pragma once
#include <cmath>

// An object-oriented implementation of the 1976 United States Standard Atmosphere based on:
// Yager, R. J. Calculating Atmospheric Conditions (Temperature, Pressure, Air Density, and 
// Speed of Sound) Using C++; ARL - TN - 543; U.S.Army Research Laboratory : Aberdeen Proving Ground, MD, 2013.

class USSA {
	// Properties
	public:
		const double TABLE4[8][4] = {//<===============================TRANSITION POINTS
			00000 , -0.0065 , 288.150 , 1.01325000000000E+5 ,// FOR PRESSURE &
			11000 , 0.0000 , 216.650 , 2.26320639734629E+4 ,// TEMPERATURE VS
			20000 , 0.0010 , 216.650 , 5.47488866967777E+3 ,// GEOPOTENTIAL
			32000 , 0.0028 , 228.650 , 8.68018684755228E+2 ,// ALTITUDE CURVES
			47000 , 0.0000 , 270.650 , 1.10906305554966E+2 ,// [table 4]
			51000 , -0.0028 , 270.650 , 6.69388731186873E+1 ,// (3RD COLUMN IS
			71000 , -0.0020 , 214.650 , 3.95642042804073E+0 ,// TEMPERATURE,
			84852 , 0.0000 , 186.946 , 3.73383589976215E-1 // 4TH, PRESSURE)
		}; //~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~15MAR2013~~~~~~
	// Methods
	public:
		double Temperature(double z);
		double Pressure(double z);
		double Density(double T, double P);
		double SpeedofSound(double T);
		double Gravity(double z);
};

