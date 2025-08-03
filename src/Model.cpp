#include "Model.h"


// Constructor
Model::Model() {
	
};


// State Methods
State Model::GetDefaultInitialState() {
	return State();
};

State Model::SetAltitude(State state, double altitude_msl_ft) {
	// Set the altitude
	state.AltitudeMeanSeaLevel_ft = altitude_msl_ft;
	// Update the atmospherics
	state = Atmosphere(state);
	// Output
	return state;
};

State Model::SetMach(State state, double mach) {
	// Update true airspeed
	double AltitudeMeanSeaLevel_m = state.AltitudeMeanSeaLevel_ft / 3.281;
	double AirTemperature_k = atmosphere.Temperature(AltitudeMeanSeaLevel_m);
	double SpeedOfSound_fps = atmosphere.SpeedofSound(AirTemperature_k) * 3.281; // m/s to ft/s
	state.TrueAirspeed_fps = SpeedOfSound_fps * mach;
	// Propegate states
	double alpha_rad = state.Alpha_deg * M_PI / 180.0;
	double beta_rad = state.Beta_deg * M_PI / 180.0;
	state.U_fps = state.TrueAirspeed_fps * cos(alpha_rad) * cos(beta_rad);
	state.V_fps = state.TrueAirspeed_fps * sin(beta_rad);
	state.W_fps = state.TrueAirspeed_fps * sin(alpha_rad) * cos(beta_rad);
	// Update the atmospherics
	state = Atmosphere(state);
	state = AeroAngles(state);
	// Output
	return state;
};

State Model::SetAlpha(State state, double alpha_deg) {
	// Set the new angle of attack
	state.Alpha_deg = alpha_deg;
	// Preserve gamma
	state.Theta_deg = alpha_deg + state.Gamma_deg;
	// Convert angles to radians
	double alpha_rad = alpha_deg * M_PI / 180.0;
	double beta_rad = state.Beta_deg * M_PI / 180.0;
	// Maintain current true airspeed
	double V = state.TrueAirspeed_fps;
	// Update body-axis velocities based on new alpha
	state.U_fps = V * cos(alpha_rad) * cos(beta_rad);
	state.V_fps = V * sin(beta_rad);
	state.W_fps = V * sin(alpha_rad) * cos(beta_rad);
	// Update any downstream dependencies
	state = Atmosphere(state);
	state = AeroAngles(state);
	// Return updated state
	return state;
}


State Model::Step(State state) {
	// Atmosphere
	state = Atmosphere(state);
	// Aerodynamic Angles
	state = AeroAngles(state);
	// Longitudinal Aero
	state = LonAero(state);
	// Lateral-Directional Aero
	state = LatDirAero(state);
	// Propulsion
	state = Propulsion(state);
	// Equations Of Motion
	state = EquationsOfMotion(state);
	// Integration
	if (state.Integrate) {
		state = Integration(state);
	}
	// Output
	return state;
};


State Model::Atmosphere(State state) {
	// Update Atmospherics - Careful with Units
	double AltitudeMeanSeaLevel_m = state.AltitudeMeanSeaLevel_ft / 3.281;
	// Temperature
	double AirTemperature_k = atmosphere.Temperature(AltitudeMeanSeaLevel_m); 
	state.AirTemperature_r = AirTemperature_k * 1.8; // Kelvin to Rankine 
	// Pressure
	double StaticPressure_pa = atmosphere.Pressure(AltitudeMeanSeaLevel_m);
	state.StaticPressure_psf = StaticPressure_pa * 0.020885;
	// Density
	double AirDensity_kgm3 = atmosphere.Density(AirTemperature_k, StaticPressure_pa);
	state.AirDensity_slugft3 = AirDensity_kgm3 / 515.4;
	// Speed of Sound
	double SpeedOfSound_ms = atmosphere.SpeedofSound(AirTemperature_k);
	state.SpeedOfSound_kt = SpeedOfSound_ms * 1.944;
	state.SpeedOfSound_fps = state.SpeedOfSound_kt * 1.688;
	// True Airspeed
	state.TrueAirspeed_fps = std::sqrt(std::pow(state.U_fps, 2) + std::pow(state.V_fps, 2) + std::pow(state.W_fps, 2));
	state.TrueAirspeed_kt = state.TrueAirspeed_fps / 1.688;
	// Mach Number
	state.MachNumber = state.TrueAirspeed_fps / state.SpeedOfSound_fps;
	// Dynamic Pressure
	state.DynamicPressure_psf = 0.5 * state.AirDensity_slugft3 * std::pow(state.TrueAirspeed_fps, 2);
	// Total Pressure
	state.TotalPressure_psf = state.DynamicPressure_psf + state.StaticPressure_psf;
	// Equivilant Airspeed
	state.EquivilantAirspeed_fps = state.SpeedOfSound_SeaLevel_fps * state.MachNumber * std::sqrt(state.StaticPressure_psf / state.StaticPressure_SeaLevel_psf);
	state.EquivilantAirspeed_kt = state.EquivilantAirspeed_fps / 1.688;
	// Output
	return state;
};


State Model::AeroAngles(State state) {
	// Aero angles
	state.Alpha_deg = atan2(state.W_fps, state.U_fps) * 180.0 / M_PI;
	state.Beta_deg = asin(state.V_fps / state.TrueAirspeed_fps) * 180.0 / M_PI;
	state.Gamma_deg = state.Theta_deg - state.Alpha_deg;
	// Rates
	double V = std::sqrt(state.U_fps * state.U_fps + state.V_fps * state.V_fps + state.W_fps * state.W_fps);
	double V_horizontal = std::sqrt(state.U_fps * state.U_fps + state.W_fps * state.W_fps);
	if (std::abs(state.TrueAirspeed_fps) < 1e-10) {
		state.Alpha_dot_dps = 0;
		state.Beta_dot_dps = 0;
	}
	else if (std::abs(V_horizontal) < 1e-10)  {
		state.Alpha_dot_dps = (state.W_dot_fps2 * state.U_fps - state.U_dot_fps2 * state.W_fps) / (state.U_fps * state.U_fps + state.W_fps * state.W_fps);
		state.Beta_dot_dps = 0;
	}
	else {
		state.Alpha_dot_dps = (state.W_dot_fps2 * state.U_fps - state.U_dot_fps2 * state.W_fps) / (state.U_fps * state.U_fps + state.W_fps * state.W_fps);
		double numerator = state.V_dot_fps2 * V_horizontal - state.V_fps * (state.U_fps * state.U_dot_fps2 + state.W_fps * state.W_dot_fps2) / V_horizontal;
		double denominator = V_horizontal * V_horizontal;
		state.Beta_dot_dps = numerator / denominator;
	}
	// Output
	return state;
};


State Model::LonAero(State state) {
	// Angles in rads/sec
	double alpha_rad = state.Alpha_deg * M_PI / 180.0;
	double q_rad = state.Q_dps * M_PI / 180.0;
	double alpha_dot_rps = state.Alpha_dot_dps * M_PI / 180.0;
	double beta_dot_rps = state.Beta_dot_dps * M_PI / 180.0;
	// Stab position
	double stab_rad = state.StabCommand_deg * M_PI / 180.0;
	// CL
	double CL_Total = state.CL0;
	CL_Total += state.CL_alpha * alpha_rad;
	CL_Total += state.CL_alpha_dot * alpha_dot_rps * state.ReferenceChord_ft / (2.0 * state.TrueAirspeed_fps);
	CL_Total += state.CL_q * q_rad * state.ReferenceChord_ft / (2 * state.TrueAirspeed_fps);
	CL_Total += state.CL_m * state.MachNumber;
	CL_Total += state.CL_delta_stab * stab_rad;
	// CD
	double CD_Total = state.CD0;
	CD_Total += state.CD_alpha * alpha_rad;
	CD_Total += state.CD_m * state.MachNumber;
	// CM
	double CM_Total = state.CM0;
	CM_Total += state.CM_alpha * alpha_rad;
	CM_Total += state.CM_alpha_dot * alpha_dot_rps * state.ReferenceChord_ft / (2.0 * state.TrueAirspeed_fps);
	CM_Total += state.CM_q * q_rad * state.ReferenceChord_ft / (2 * state.TrueAirspeed_fps);
	CM_Total += state.CM_m * state.MachNumber * std::min(std::max((state.MachNumber - 0.8), 0.0), 1.0); // Fade this below transonic
	CM_Total += state.CM_delta_stab * stab_rad;
	// Sum of Forces
	state.Lon_FX_lbs = (CL_Total * sin(alpha_rad) - CD_Total * cos(alpha_rad)) * state.DynamicPressure_psf * state.ReferenceWingArea_ft2;
	state.Lon_FY_lbs = 0;
	state.Lon_FZ_lbs = (-CL_Total * cos(alpha_rad) - CD_Total * sin(alpha_rad)) * state.DynamicPressure_psf * state.ReferenceWingArea_ft2;;
	// Sum of Moments
	state.Lon_MX_ftlbs = 0;
	state.Lon_MY_ftlbs = CM_Total * state.DynamicPressure_psf * state.ReferenceWingArea_ft2 * state.ReferenceChord_ft;
	state.Lon_MZ_ftlbs = 0;
	// Output
	return state;
};


State Model::LatDirAero(State state) {
	// Convert angles and rates to radians
	double beta_rad = state.Beta_deg * M_PI / 180.0;
	double alpha_rad = state.Alpha_deg * M_PI / 180.0;
	double p_rad = state.P_dps * M_PI / 180.0;
	double r_rad = state.R_dps * M_PI / 180.0;
	double aileron_rad = state.AileronCommand_deg * M_PI / 180.0;
	double rudder_rad = state.RudderCommand_deg * M_PI / 180.0;

	// Span and airspeed from state
	double b_ft = state.ReferenceSpan_ft;
	double V_fps = state.TrueAirspeed_fps;

	// CY (Side force coefficient)
	double CY_Total = state.CY_b * beta_rad;
	CY_Total += state.CY_rud * rudder_rad;

	// Cl (Rolling moment coefficient)
	double Cl_Total = state.Cl_b * beta_rad;
	Cl_Total += state.Cl_p * p_rad * b_ft / (2.0 * V_fps);
	Cl_Total += state.Cl_r * r_rad * b_ft / (2.0 * V_fps);
	Cl_Total += state.Cl_delta_ail * aileron_rad;
	Cl_Total += state.Cl_delta_rud * rudder_rad;

	// Cn (Yawing moment coefficient)
	double Cn_Total = state.CN_b * beta_rad;
	Cn_Total += state.CN_p * p_rad * b_ft / (2.0 * V_fps);
	Cn_Total += state.CN_r * r_rad * b_ft / (2.0 * V_fps);
	Cn_Total += state.CN_delta_ail * aileron_rad;
	Cn_Total += state.CN_delta_rud * rudder_rad;

	// Aerodynamic Forces (body-Y is lateral)
	state.LatDir_FX_lbs = 0;
	state.LatDir_FY_lbs = CY_Total * state.DynamicPressure_psf * state.ReferenceWingArea_ft2;
	state.LatDir_FZ_lbs = 0;

	// Aerodynamic Moments
	state.LatDir_MX_ftlbs = Cl_Total * state.DynamicPressure_psf * state.ReferenceWingArea_ft2 * b_ft;
	state.LatDir_MY_ftlbs = 0;
	state.LatDir_MZ_ftlbs = Cn_Total * state.DynamicPressure_psf * state.ReferenceWingArea_ft2 * b_ft;

	// Output
	return state;
};


State Model::Propulsion(State state) {
	// Thrust
	state.Thrust_lb = state.Throttle_norm * state.ThrustLimit_lbs;
	// Sum of Forces
	state.Propulsion_FX_lbs = state.Thrust_lb ;
	state.Propulsion_FY_lbs = 0;
	state.Propulsion_FZ_lbs = 0;
	// Sum of Moments
	state.Propulsion_MX_ftlbs = 0;
	state.Propulsion_MY_ftlbs = 0;
	state.Propulsion_MZ_ftlbs = 0;
	// Output
	return state;
};


State Model::EquationsOfMotion(State state) {
	// Convert degrees to radians
	double phi_rad = state.Phi_deg * M_PI / 180.0;
	double theta_rad = state.Theta_deg * M_PI / 180.0;
	double psi_rad = state.Psi_deg * M_PI / 180.0;
	double p_rad = state.P_dps * M_PI / 180.0;
	double q_rad = state.Q_dps * M_PI / 180.0;
	double r_rad = state.R_dps * M_PI / 180.0;
	// Trig values
	double cos_phi = cos(phi_rad);
	double sin_phi = sin(phi_rad);
	double cos_theta = cos(theta_rad);
	double sin_theta = sin(theta_rad);
	double tan_theta = tan(theta_rad);
	// Gravity
	double g = state.AccelGravity_fts2;
	double mass_sl = state.AircraftWeight_lbs / g;
	double Gravity_FX_lbs = -state.AircraftWeight_lbs * sin(theta_rad);
	double Gravity_FY_lbs = state.AircraftWeight_lbs * cos(theta_rad) * sin(phi_rad);
	double Gravity_FZ_lbs = state.AircraftWeight_lbs * cos(theta_rad) * cos(phi_rad);
	// Sum of Forces
	state.FX_lbs = state.Lon_FX_lbs + state.LatDir_FX_lbs + state.Propulsion_FX_lbs + Gravity_FX_lbs;
	state.FY_lbs = state.Lon_FY_lbs + state.LatDir_FY_lbs + state.Propulsion_FY_lbs + Gravity_FY_lbs;
	state.FZ_lbs = state.Lon_FZ_lbs + state.LatDir_FZ_lbs + state.Propulsion_FZ_lbs + Gravity_FZ_lbs;
	// Sum of Moments
	state.MX_ftlbs = state.Lon_MX_ftlbs + state.LatDir_MX_ftlbs + state.Propulsion_MX_ftlbs;
	state.MY_ftlbs = state.Lon_MY_ftlbs + state.LatDir_MY_ftlbs + state.Propulsion_MY_ftlbs;
	state.MZ_ftlbs = state.Lon_MZ_ftlbs + state.LatDir_MZ_ftlbs + state.Propulsion_MZ_ftlbs;
	// Translation Derivatives
	state.U_dot_fps2 = state.FX_lbs / mass_sl + r_rad * state.V_fps - q_rad * state.W_fps;
	state.V_dot_fps2 = state.FY_lbs / mass_sl - r_rad * state.U_fps + p_rad * state.W_fps;
	state.W_dot_fps2 = state.FZ_lbs / mass_sl + q_rad * state.U_fps - p_rad * state.V_fps;
	// Rotation Derivatives assuming body frame aligned with principal axes.
	// This is likely an oversimplification due to the T-tail impact on Ixz, but close enough for now.
	// Future improvement could be solving the full form of the equations using a solver
	double p_dot_rps2 = (state.MX_ftlbs + (state.Iyy_slft - state.Izz_slft) * q_rad * r_rad) / state.Ixx_slft;
	double q_dot_rps2 = (state.MY_ftlbs + (state.Izz_slft - state.Ixx_slft) * p_rad * r_rad) / state.Iyy_slft;
	double r_dot_rps2 = (state.MZ_ftlbs + (state.Ixx_slft - state.Iyy_slft) * p_rad * q_rad) / state.Izz_slft;
	state.P_dot_dps2 = p_dot_rps2 * 180.0 / M_PI;
	state.Q_dot_dps2 = q_dot_rps2 * 180.0 / M_PI;
	state.R_dot_dps2 = r_dot_rps2 * 180.0 / M_PI;
	// Output
	return state;
};


State Model::Integration(State state) {
	// Trig values
	double phi_rad = state.Phi_deg * M_PI / 180.0;
	double theta_rad = state.Theta_deg * M_PI / 180.0;
	double cos_phi = cos(phi_rad);
	double sin_phi = sin(phi_rad);
	double sin_theta = sin(theta_rad);
	double cos_theta = cos(theta_rad);
	double tan_theta = tan(theta_rad);
	// Integrate Translation
	state.U_fps = state.U_fps + state.U_dot_fps2 * dt;
	state.V_fps = state.V_fps + state.V_dot_fps2 * dt;
	state.W_fps = state.W_fps + state.W_dot_fps2 * dt;
	state.Altitude_dot_fps = sin_theta * state.U_fps - sin_phi * cos_theta * state.V_fps - cos_phi * cos_theta * state.W_fps;
	state.AltitudeMeanSeaLevel_ft = state.AltitudeMeanSeaLevel_ft + state.Altitude_dot_fps * dt;
	// Integrate Rotation
	state.P_dps = state.P_dps + state.P_dot_dps2 * dt;
	state.Q_dps = state.Q_dps + state.Q_dot_dps2 * dt;
	state.R_dps = state.R_dps + state.R_dot_dps2 * dt;
	// Euler angle derivatives
	state.Phi_dot_dps = state.P_dps + sin_phi * tan_theta * state.Q_dps + cos_phi * tan_theta * state.R_dps;
	state.Theta_dot_dps = cos_phi * state.Q_dps - sin_phi * state.R_dps;
	state.Psi_dot_dps = (sin_phi / cos_theta) * state.Q_dps + (cos_phi / cos_theta) * state.R_dps;
	// Integrate Euler angles
	state.Phi_deg = state.Phi_deg + state.Phi_dot_dps * dt;
	state.Theta_deg = state.Theta_deg + state.Theta_dot_dps * dt;
	state.Psi_deg = state.Psi_deg + state.Psi_dot_dps * dt;
	// Increment time
	state.Time_sec = state.Time_sec + dt;
	// Output
	return state;
};
