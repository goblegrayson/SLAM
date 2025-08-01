#include "USSA.h"

double USSA::Temperature(//<==================================TEMPERATURE (K)
	double z) {//<--------ALTITUDE (m) (T IS VALID FOR -5,000 m < z < 86,000 m)
	double H = z * 6356766 / (z + 6356766);//..............................[equation 18]
	int b;/*<-*/for (b = 0; b < 7; ++b)if (H < TABLE4[b + 1][0])break;
	return TABLE4[b][2] + TABLE4[b][1] * (H - TABLE4[b][0]);//...........[equation 23]
}//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~15MAR2013~~~~~~

double USSA::Pressure(//<=======================================PRESSURE (Pa)
	double z) {//<--------ALTITUDE (m) (P IS VALID FOR -5,000 m < z < 86,000 m)
	double H = z * 6356766 / (z + 6356766);//..............................[equation 18]
	int b;/*<-*/for (b = 0; b < 7; ++b)if (H < TABLE4[b + 1][0])break;
	double C = -.0341631947363104;//................C = -G0*M0/RSTAR [pages 8,9,3]
	double Hb = TABLE4[b][0], Lb = TABLE4[b][1], Tb = TABLE4[b][2], Pb = TABLE4[b][3];
	return Pb * (fabs(Lb) > 1E-12 ? pow(1 + Lb / Tb * (H - Hb), C / Lb) : exp(C * (H - Hb) / Tb));
}//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~15MAR2013~~~~~~

double USSA::Density(//<=====================================DENSITY (kg/m^3)
	double T,//<-------------TEMPERATURE (K) (CALCULATE T USING Temperature())
	double P) {//<-----------------PRESSURE (Pa) (CALCULATE P USING Pressure())
	return P * .00348367635597379 / T;//...............................[equation 42]
}//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~15MAR2013~~~~~~

double USSA::SpeedofSound(//<============================SPEED OF SOUND (m/s)
	double T) {//<------------TEMPERATURE (K) (CALCULATE T USING Temperature())
	return sqrt(401.87430086589 * T);//..............................[equation 50]
}//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~15MAR2013~~~~~~

double USSA::Gravity(//<==================ACCELERATION DUE TO GRAVITY (m/s^2)
	double z) {//<--------ALTITUDE (m) (g IS VALID FOR -5,000 m < z < 86,000 m)
	return 9.80665 * pow(1 + z / 6356766, -2);//..........................[equation 17]
}//~~~~YAGENAUT@GMAIL.COM~~~~~~~~~~~~~~~~~~~~~~~~~LAST~UPDATED~15MAR2013~~~~~~