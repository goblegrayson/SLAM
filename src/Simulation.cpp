#include "Simulation.h"


// Constructor
Simulation::Simulation() {
	// Initialize model state
	initial_state = model.GetDefaultInitialState();
}


// Run Methods
void Simulation::run(float maxTime) {
	// Init
	int n_frames = int(std::ceil(maxTime / model.dt)) + 1;
	State state = initial_state;
	state_history.resize(n_frames);
	state_history[0] = initial_state;
	// Main Loop
	for (int i_frame = 1; i_frame < n_frames; i_frame++) {
		// Run time step
		state = model.Step(state);
		// Store state in state_history vector
		state_history[i_frame] = state;
		// Print time
		std::cout << state_history[i_frame].Time_sec << std::endl;
	};
};


// Output methods
void Simulation::toCSV(const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // Write header row (column names)
    file << "AltitudeMeanSeaLevel_ft,MachNumber,Time_sec\n";

    // Write data rows
	for (int i_frame = 0; i_frame < state_history.size() - 1; i_frame++) {
		State state = state_history[i_frame];
        file << state.AltitudeMeanSeaLevel_ft << ","
            << state.MachNumber << ","
            << state.Time_sec << "\n";
    }

    file.close();
    std::cout << "State history written to " << filename << std::endl;
}