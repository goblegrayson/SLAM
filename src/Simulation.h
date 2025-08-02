#pragma once
#include "Model.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

// Simulation Class
class Simulation{
	// Properties	
	public:
		Model model;
		State initial_state;
		std::vector<State> state_history;

	// Methods
	public:
		Simulation();
		void printTimestep();
		void run(double maxTime);
		void toCSV(const std::string& filename);
};

