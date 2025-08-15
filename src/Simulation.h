#pragma once
#include "Model.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>


// Simulation Class
class Simulation{
	// Properties	
	public:
		Model model;
		State initial_state;
		std::vector<State> state_history;
		std::vector<double> input_time;


	// Methods
	public:
		Simulation();
		void printTimestep();
		void run(double maxTime);
		void toCSV(const std::string& filename);
		State TrimResiduals(const State& base, const State& inputs);
		double CalcCost(const State& state);
		bool SolveTrim(const State& initialGuess, State& optimalTrim);
		double TestCost(double alpha, double pitch, double throttle);
};

