// SLAM.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "main.h"

int main() {
    // Model inputs
    double max_time = 60;
    double altitude_msl_ft = 55000;
    double mach = 1.8;
    //double altitude_msl_ft = 0;
    //double mach = 0.3;
    double alpha_deg = 5;
    double throttle_norm = 1;
    double stab_position_deg = 0;
    //double mach = 0.5;
    // Initialize Model
    Simulation Sim;
    Sim.initial_state = Sim.model.SetAltitude(Sim.initial_state, altitude_msl_ft);
    Sim.initial_state = Sim.model.SetMach(Sim.initial_state, mach);
    Sim.initial_state = Sim.model.SetAlpha(Sim.initial_state, alpha_deg);
    Sim.initial_state.Throttle_norm = throttle_norm;
    Sim.initial_state.StabPosition_deg = stab_position_deg;
    // Basic 3-Dof Lon trim
    State guess = Sim.initial_state;
    bool isTrimmed = Sim.SolveTrim(Sim.initial_state, guess);
    Sim.initial_state = guess;
    // Print Trim States
    std::cout << std::fixed << std::setprecision(3)
        << "Alpha_deg: " << std::setw(8) << Sim.initial_state.Alpha_deg
        << "  StabPosition_deg: " << std::setw(6) << Sim.initial_state.StabPosition_deg
        << "  Throttle_norm: " << std::setw(6) << Sim.initial_state.Throttle_norm
        << "  Qdot: " << std::setw(8) << Sim.initial_state.Q_dot_dps2
        << "  Udot: " << std::setw(8) << Sim.initial_state.U_dot_fps2
        << "  Wdot: " << std::setw(8) << Sim.initial_state.W_dot_fps2
        << std::endl;
    // Run Model
    Sim.run(max_time);
    // Output State History
    Sim.toCSV("StateHistory.csv");
    return 0;
};

//int main(int argc, char* argv[]) {
//    // Model inputs
//    double max_time = 10;
//    double altitude_msl_ft = 55000;
//    double mach = 1.8;
//    double alpha_deg = 0.25;
//    //double mach = 0.5;
//    // Initialize Model
//    Simulation Sim;
//    Sim.initial_state = Sim.model.SetAltitude(Sim.initial_state, altitude_msl_ft);
//    Sim.initial_state = Sim.model.SetMach(Sim.initial_state, mach);
//    if (argc == 4) {
//        double alpha = atof(argv[1]);
//        double pitch = atof(argv[2]);
//        double throttle = atof(argv[3]);
//        std::cout << Sim.TestCost(alpha, pitch, throttle) << std::endl;
//        return 0;
//    }
//
//    std::cerr << "Usage: <exe> alpha pitch throttle" << std::endl;
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
