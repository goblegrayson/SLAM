#include "Model.h"


// Constructor
Model::Model() {
	
};


// State Methods
State Model::GetDefaultInitialState() {
	return State();
};


State Model::Step(State state) {
	// Atmosphere
	state = Atmosphere(state);
	// Longitudinal Aero
	state = LonAero(state);
	// Lateral-Directional Aero
	state = LatDirAero(state);
	// Propulsion
	state = Propulsion(state);
	// Integration Model
	state = Integration(state);
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
	state.TrueAirspeed_kt = state.MachNumber * state.SpeedOfSound_kt;
	state.TrueAirspeed_fps = state.TrueAirspeed_kt * 1.688;
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
	// For the moment, just fix alpha and beta so we get forces and moments
	state.AngleOfAttack_deg = 1;
	state.AngleOfSideslip_deg = 1;
	// Output
	return state;
};


State Model::LonAero(State state) {
	// Output
	return state;
};


State Model::LatDirAero(State state) {
	// Output
	return state;
};


State Model::Propulsion(State state) {
	// Output
	return state;
};


State Model::EquationsOfMotion(State state) {
	// Calculate Rotation


	// Calculate Translation


	// Output
	return state;
};


State Model::Integration(State state) {
	// Integrate Rotation
	

	// Integrate Translation
	

	// Increment time
	state.Time_sec = state.Time_sec + dt;
	// Output
	return state;
};
