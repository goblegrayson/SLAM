#pragma once
#include <cmath>

struct State {
	// Environment
	float PressureAltitude_ft;
	float CalibratedAirspeed_kt;

	// Orientation

	// Simulation
	float Time_sec;
};

class Model{
	// Properties
	public:
		float dt = 0.01;

	// Methods
	public:
		Model();
		State GetDefaultInitialState();
		State Step(State state);
		State Integration(State state);
};

