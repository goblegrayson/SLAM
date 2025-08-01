#include "Model.h"


// Constructor
Model::Model() {
	// TODO
};


// State Methods
State Model::GetDefaultInitialState() {
	return State();
};

State Model::Step(State state) {
	// Integration Model
	state = Integration(state);
	// Output
	return state;
};

State Model::Integration(State state) {
	// Increment time
	state.Time_sec = state.Time_sec + dt;
	return state;
};
