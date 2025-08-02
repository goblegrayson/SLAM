#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include "USSA.h"


struct State {
	// Reference Parameters
	double ReferenceWingArea_ft2 = 196.1;
	double ReferenceChord_ft = 9.55;
	double ReferenceSpan_ft = 21.94;
	double SpeedOfSound_SeaLevel_fps = 661.47 * 1.688;
	double StaticPressure_SeaLevel_psf = 2116.23 ;
	double ThrustLimit_lbs = 15600;
	double StabLimit_deg = 10; // Just a guess
	double AilerontLimit_deg = 10; // Just a guess
	// Controls
	double PitchStick_norm = 0; // 1 full is nose up, -1 is full now down
	double RollStick_norm = 0; // 1 full is right wing down, -1 is full left wing down
	double StabPosition_deg = 0;
	double AileronPosition_deg = 0;
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
	double Alpha_deg = 0;
	double Alpha_dot_dps = 0;
	double Beta_deg = 0;
	double Beta_dot_dps = 0;
	double Gamma_deg = 0;
	// Lon Aero
	double CL0 = 0.2;
	double CD0 = 0.055;
	double CL_alpha = 2;
	double CD_alpha = 0.38;
	double CM_alpha = -1.3;
	double CL_alpha_dot = 0;
	double CM_alpha_dot = -2;
	double CL_q = 0;
	double CM_q = -4.8;
	double CL_m = -0.2; // This is probably unreasonable for low speeds
	double CD_m = 0;
	double CM_m = -0.01;
	double CL_delta_stab = 0.52;
	double CM_delta_stab = -0.1;
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
	// Propulsion
	double Throttle_norm = 0;
	double Thrust_lb = 0;
	// Forces
	double Lon_FX_lbs = 0;
	double Lon_FY_lbs = 0;
	double Lon_FZ_lbs = 0;
	double LatDir_FX_lbs = 0;
	double LatDir_FY_lbs = 0;
	double LatDir_FZ_lbs = 0;
	double Propulsion_FX_lbs = 0;
	double Propulsion_FY_lbs = 0;
	double Propulsion_FZ_lbs = 0;
	double FX_lbs = 0;
	double FY_lbs = 0;
	double FZ_lbs = 0;
	// Moments 
	double Lon_MX_lbs = 0;
	double Lon_MY_lbs = 0;
	double Lon_MZ_lbs = 0;
	double LatDir_MX_lbs = 0;
	double LatDir_MY_lbs = 0;
	double LatDir_MZ_lbs = 0;
	double Propulsion_MX_lbs = 0;
	double Propulsion_MY_lbs = 0;
	double Propulsion_MZ_lbs = 0;
	double MX_lbs = 0;
	double MY_lbs = 0;
	double MZ_lbs = 0;
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
	double Altitude_dot_fps = 0;
	// Integrated States
	double U_fps = 0;
	double V_fps = 0;
	double W_fps = 0;
	double P_dps = 0;
	double Q_dps = 0;
	double R_dps = 0;
	double Phi_deg = 0;
	double Theta_deg = 0;
	double Psi_deg = 0;
	// Simulation
	double Time_sec = 0;
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
		State SetAltitude(State state, double altitude_msl_ft);
		State SetMach(State state, double mach);
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

