// SLAM.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "main.h"

int main(int argc, char* argv[]) {
    // Model initial conditions
    std::string manuever_type = argv[1];
    double AltitudeMeanSeaLevel_ft = atof(argv[2]);
    double MachNumber = atof(argv[3]);
    // Initial trim guess
    double Alpha_deg = 5;
    double Throttle_norm = 1;
    double StabCommand_deg = 0;
    // Initialize Model
    Simulation Sim;
    Sim.initial_state = Sim.model.SetAltitude(Sim.initial_state, AltitudeMeanSeaLevel_ft);
    Sim.initial_state = Sim.model.SetMach(Sim.initial_state, MachNumber);
    Sim.initial_state = Sim.model.SetAlpha(Sim.initial_state, Alpha_deg);
    Sim.initial_state.Throttle_norm = Throttle_norm;
    Sim.initial_state.StabCommand_deg = StabCommand_deg;
    // Basic 3-Dof Lon trim
    State guess = Sim.initial_state;
    bool isTrimmed = Sim.SolveTrim(Sim.initial_state, guess);
    Sim.initial_state = guess;
    // Print Trim States
    std::cout << std::fixed << std::setprecision(3)
        << "Alpha_deg: " << std::setw(8) << Sim.initial_state.Alpha_deg
        << "  StabCommand_deg: " << std::setw(6) << Sim.initial_state.StabCommand_deg
        << "  Throttle_norm: " << std::setw(6) << Sim.initial_state.Throttle_norm
        << "  Qdot: " << std::setw(8) << Sim.initial_state.Q_dot_dps2
        << "  Udot: " << std::setw(8) << Sim.initial_state.U_dot_fps2
        << "  Wdot: " << std::setw(8) << Sim.initial_state.W_dot_fps2
        << std::endl;
    // Pick a manuever type and set up inputs (if any)
    double max_time = 60;
    if (manuever_type == "AileronDoublet") {
        // Aileron Doublet
        max_time = 30;
        Sim.model.inputs.AileronCommand_deg.time.push_back(1.0);
        Sim.model.inputs.AileronCommand_deg.value.push_back(5.0);
        Sim.model.inputs.AileronCommand_deg.time.push_back(2.0);
        Sim.model.inputs.AileronCommand_deg.value.push_back(-5.0);
        Sim.model.inputs.AileronCommand_deg.time.push_back(3.0);
        Sim.model.inputs.AileronCommand_deg.value.push_back(0.0);
    }
    else if (manuever_type == "RudderDoublet") {
        // Rudder Doublet
        max_time = 30;
        Sim.model.inputs.RudderCommand_deg.time.push_back(1.0);
        Sim.model.inputs.RudderCommand_deg.value.push_back(10.0);
        Sim.model.inputs.RudderCommand_deg.time.push_back(2.0);
        Sim.model.inputs.RudderCommand_deg.value.push_back(-10.0);
        Sim.model.inputs.RudderCommand_deg.time.push_back(3.0);
        Sim.model.inputs.RudderCommand_deg.value.push_back(0.0);
    }
    else if (manuever_type == "StabDoublet") {
        // Stab Doublet
        max_time = 30;
        double init_stab = Sim.initial_state.StabCommand_deg;
        Sim.model.inputs.StabCommand_deg.time.push_back(1.0);
        Sim.model.inputs.StabCommand_deg.value.push_back(init_stab + 2.0);
        Sim.model.inputs.StabCommand_deg.time.push_back(2.0);
        Sim.model.inputs.StabCommand_deg.value.push_back(init_stab -2.0);
        Sim.model.inputs.StabCommand_deg.time.push_back(3.0);
        Sim.model.inputs.StabCommand_deg.value.push_back(init_stab);
    }
    else {
        // Default to this if invalid name is provided
        if (manuever_type != "LonTrim") {
            std::cout << "Unknown Manuever! Assuming LonTrim." << std::endl;
            manuever_type = "LonTrim";
        }
        // Longitudinal Trim

    }
    // Run Model
    Sim.run(max_time);
    // Output State History
    Sim.toCSV("output_files/" + manuever_type + ".csv");
    return 0;
};


// Cost function sweeps
// Use this main to plot trim cost vs sweeps of alpha and stab command
//int main(int argc, char* argv[]) {
//    // Model inputs
//    double AltitudeMeanSeaLevel_ft = 55000;
//    double MachNumber = 1.8;
//    double Alpha_deg = 0.25;
//    // Initialize Model
//    Simulation Sim;
//    Sim.initial_state = Sim.model.SetAltitude(Sim.initial_state, AltitudeMeanSeaLevel_ft);
//    Sim.initial_state = Sim.model.SetMach(Sim.initial_state, MachNumber);
//    if (argc == 4) {
//        double Alpha_deg = atof(argv[1]);
//        double StabCommand_deg = atof(argv[2]);
//        double Throttle_norm = atof(argv[3]);
//        std::cout << Sim.TestCost(Alpha_deg, StabCommand_deg, Throttle_norm) << std::endl;
//        return 0;
//    }
//    std::cerr << "Usage: <exe> alpha stab throttle" << std::endl;
//    return 1;
//}











// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
