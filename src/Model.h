#pragma once
#include <cmath>
#include "USSA.h"


struct State {
	// Reference Parameters
	double ReferenceWingArea_ft2 = 196.1;
	double ReferenceChord_ft = 9.55;
	double ReferenceSpan_ft = 21.94;
	double SpeedOfSound_SeaLevel_fps = 661.47 * 1.688;
	double StaticPressure_SeaLevel_psf = 2116.23 ;
	// Mass Properties
	double AircraftWeight_lbs = 16300;
	double CGx_pct = 7;
	double Ixx_slft = 3549;
	double Iyy_slft = 58611;
	double Izz_slft = 59669;
	double Ixy_slft = 0;
	double Ixz_slft = 0;
	double Iyz_slft = 0;
	// Environment
	double AltitudeMeanSeaLevel_ft = 0;
	double MachNumber = 0;
	double TrueAirspeed_fps = 0;
	double TrueAirspeed_kt = 0;
	double EquivilantAirspeed_fps = 0;
	double EquivilantAirspeed_kt = 0;
	double AccelGravity_fts2 = 32.2;
	double SpeedOfSound_kt = 0;
	double SpeedOfSound_fps = 0;
	double AirTemperature_r = 0; // Rankine
	double AirDensity_slugft3 = 0;
	double StaticPressure_psf = 0;
	double DynamicPressure_psf = 0;
	double TotalPressure_psf = 0;
	// Aero Angles
	double AngleOfAttack_deg = 0;
	double AngleOfSideslip_deg = 0;
	// Lon Aero
	double CL = 0.2;
	double CD = 0.055;
	double CL_alpha = 2;
	double CD_alpha = 0.38;
	double CM_alpha = -1.3;
	double CL_alpha_dot = 0;
	double CM_alpha_dot = -2;
	double CL_q = 0;
	double CM_q = -4.8;
	double CL_m = -0.2;
	double CD_m = 0;
	double CM_m = -0.01;
	double CL_Delta_Elevator = 0.52;
	double CM_Delta_Elevator = -0.1;
	// Lat-Dir Aero - Note the distinction between CL (Lift Coefficient) and Cl (Rolling Moment Coefficient)
	double CY_b = -1;
	double Cl_b = -0.09;
	double CN_b = 0.24;
	double Cl_p = -0.27;
	double CN_p = -0.09;
	// double Cyr = -2 * (lv / b) * Cyb;
	double Cl_r = 0.15;
	double CN_r = -0.65;
	double Cl_alpha = 0.017;
	double CN_alpha = 0.0025;
	double CY_delta_r = 0.05;
	double Cl_delta_r = 0.008;
	double CN_delta_r = -0.04;
	// Rates
	double U_dot_fps2 = 0;
	double V_dot_fps2 = 0;
	double W_dot_fps2 = 0;
	double P_dot_dps2 = 0;
	double Q_dot_dps2 = 0;
	double R_dot_dps2 = 0;
	double Phi_dot_dps = 0;
	double Theta_dot_dps = 0;
	double Psi_dot_dps = 0;
	// Integrated States
	double U_fps = 0;
	double V_fps = 0;
	double W_fps = 0;
	double P_dps = 0;
	double Q_dps = 0;
	double R_dps = 0;
	double Phi_d = 0;
	double Theta_d = 0;
	double Psi_d = 0;
	// Simulation
	double Time_sec;
};


class Model{
	// Properties
	public:
		double dt = 0.01;
		USSA atmosphere;

	// Methods
	public:
		// Model Methods
		Model();
		State GetDefaultInitialState();
		State Step(State state);
		// Component Models
		State Atmosphere(State state);
		State AeroAngles(State state);
		State LonAero(State state);
		State LatDirAero(State state);
		State Propulsion(State state);
		State EquationsOfMotion(State state);
		State Integration(State state);
};

