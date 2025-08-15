#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include "USSA.h"
#include <algorithm>
#include <vector>


// Reference Constants
constexpr double ReferenceWingArea_ft2 = 196.1;
constexpr double ReferenceChord_ft = 9.55;
constexpr double ReferenceSpan_ft = 21.94;
constexpr double SpeedOfSound_SeaLevel_fps = 661.47 * 1.688;
constexpr double StaticPressure_SeaLevel_psf = 2116.23;
constexpr double ThrustLimit_lbs = 15600;
constexpr double AccelGravity_fts2 = 32.174;

// Conversion Constants
constexpr double FT_TO_M = 1.0 / 3.281;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180 / M_PI;
constexpr double K_TO_R = 1.8;
constexpr double PA_TO_PSF = 0.020885;
constexpr double SLUGFT3_TO_KGM3 = 515.4;
constexpr double KT_TO_FTS = 1.688;

struct State {
	// Sim Settings
	bool Integrate = false;
	// Controls
	double StabCommand_deg = 0;
	double AileronCommand_deg = 0;
	double RudderCommand_deg = 0;
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
	double CL0 = 0.05; // Ballpark guess
	double CD0 = 0.0172; // https://historicalfighters.com/f-104g-general-technical-data/
	double CM0 = 0.15; // Ballpark guess - positive because of very low camber and big t-tail
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
	double CL_delta_stab = 0.52;
	// Baking in some correction here for physics that this simple model is missing in order to get reasonable trim alphas
	double CM_delta_stab = -1.13 * 2; 
	// Lat-Dir Aero - Note the distinction between CL (Lift Coefficient) and Cl (Rolling Moment Coefficient)
	double CY_b = -1;
	double Cl_b = -0.09;
	double CN_b = 0.24;
	double Cl_p = -0.27;
	double CN_p = -0.09;
	// double Cyr = -2 * (lv / b) * Cyb;
	double Cl_r = 0.15;
	double CN_r = -0.65;
	double Cl_delta_ail = 0.017;
	double CN_delta_ail = 0.0025;
	double CY_rud = 0.05;
	double Cl_delta_rud = 0.008;
	double CN_delta_rud = -0.04;
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
	double Lon_MX_ftlbs = 0;
	double Lon_MY_ftlbs = 0;
	double Lon_MZ_ftlbs = 0;
	double LatDir_MX_ftlbs = 0;
	double LatDir_MY_ftlbs = 0;
	double LatDir_MZ_ftlbs = 0;
	double Propulsion_MX_ftlbs = 0;
	double Propulsion_MY_ftlbs = 0;
	double Propulsion_MZ_ftlbs = 0;
	double MX_ftlbs = 0;
	double MY_ftlbs = 0;
	double MZ_ftlbs = 0;
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

struct Input {
	std::vector<double> time;
	std::vector<double> value;
};

struct Inputs {
	Input StabCommand_deg;
	Input AileronCommand_deg;
	Input RudderCommand_deg;
};

class Model{
	// Properties
	public:
		double dt = 0.005;
		USSA atmosphere;
		Inputs inputs;

	// Methods
	public:
		// Model Methods
		Model();
		State GetDefaultInitialState();
		void SetAltitude(State& state, double altitude_msl_ft);
		void SetMach(State& state, double mach);
		void SetAlpha(State& state, double alpha_deg);
		void Step(State& state);
		// Component Models
		void Atmosphere(State& state);
		void AeroAngles(State& state);
		void LonAero(State& state);
		void LatDirAero(State& state);
		void Propulsion(State& state);
		void EquationsOfMotion(State& state);
		void Integration(State& state);
};

