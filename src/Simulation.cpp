#include "Simulation.h"


// Constructor
Simulation::Simulation() {
	// Initialize model state
	initial_state = model.GetDefaultInitialState();
}


// Run Methods
void Simulation::run(double max_time) {
	// Init
	int n_frames = int(std::ceil(max_time / model.dt)) + 1;
	State state = initial_state;
    // Ensure integration is on 
    state.Integrate = true;
    // Set up state history
	state_history.resize(n_frames);
	state_history[0] = state;
    // Input counters
    int i_stab = 0;
    int i_ail = 0;
    int i_rud = 0;
	// Main Loop
	for (int i_frame = 1; i_frame < n_frames; i_frame++) {
        // Set Stab input
        if (!model.inputs.StabCommand_deg.time.empty() && abs(state.Time_sec - model.inputs.StabCommand_deg.time[i_stab]) < model.dt) {
            state.StabCommand_deg = model.inputs.StabCommand_deg.value[i_stab];
            i_stab = std::min(i_stab + 1, int(model.inputs.StabCommand_deg.time.size()) - 1);
        }
        // Set Aileron input
        if (!model.inputs.AileronCommand_deg.time.empty() && abs(state.Time_sec - model.inputs.AileronCommand_deg.time[i_ail]) < model.dt) {
            state.AileronCommand_deg = model.inputs.AileronCommand_deg.value[i_ail];
            i_ail = std::min(i_ail + 1, int(model.inputs.AileronCommand_deg.time.size()) - 1);
        }
        // Set Rudder input
        if (!model.inputs.RudderCommand_deg.time.empty() && abs(state.Time_sec - model.inputs.RudderCommand_deg.time[i_rud]) < model.dt) {
            state.RudderCommand_deg = model.inputs.RudderCommand_deg.value[i_rud];
            i_rud = std::min(i_rud + 1, int(model.inputs.RudderCommand_deg.time.size()) - 1);
        }
		// Run time step
		state = model.Step(state);
		// Store state in state_history vector
		state_history[i_frame] = state;
		// Print time
		//std::cout << state_history[i_frame].Time_sec << std::endl;
	};
};


// Output methods
void Simulation::toCSV(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // =============================
    // WRITE HEADER ROW
    // =============================
    file <<
        // Simulation
        "Time_sec,"

        // Reference Parameters
        "ReferenceWingArea_ft2,ReferenceChord_ft,ReferenceSpan_ft,"
        "SpeedOfSound_SeaLevel_fps,StaticPressure_SeaLevel_psf,ThrustLimit_lbs,"

        // Controls
        "StabCommand_deg,AileronCommand_deg,RudderCommand_deg,Throttle_norm,"

        // Mass Properties
        "AircraftWeight_lbs,CGx_pct,"
        "Ixx_slft,Iyy_slft,Izz_slft,Ixy_slft,Ixz_slft,Iyz_slft,"

        // Environment
        "AltitudeMeanSeaLevel_ft,MachNumber,"
        "TrueAirspeed_fps,TrueAirspeed_kt,EquivilantAirspeed_fps,EquivilantAirspeed_kt,"
        "AccelGravity_fts2,SpeedOfSound_kt,SpeedOfSound_fps,"
        "AirTemperature_r,AirDensity_slugft3,"
        "StaticPressure_psf,DynamicPressure_psf,TotalPressure_psf,"

        // Aero Angles
        "Alpha_deg,Alpha_dot_dps,Beta_deg,Beta_dot_dps,Gamma_deg,"

        // Longitudinal Aero Coefficients
        "CL0,CD0,CM0,CL_alpha,CD_alpha,CM_alpha,"
        "CL_alpha_dot,CM_alpha_dot,CL_q,CM_q,"
        "CL_m,CD_m,CM_m,CL_delta_stab,CM_delta_stab,"

        // Lateral-Directional Aero Coefficients
        "CY_b,Cl_b,CN_b,Cl_p,CN_p,Cl_r,CN_r,"
        "Cl_delta_ail,CN_delta_ail,CY_rud,Cl_delta_rud,CN_delta_rud,"

        // Propulsion
        "Thrust_lb,"

        // Forces & Moments
        "Lon_FX_lbs,Lon_FY_lbs,Lon_FZ_lbs,"
        "LatDir_FX_lbs,LatDir_FY_lbs,LatDir_FZ_lbs,"
        "Propulsion_FX_lbs,Propulsion_FY_lbs,Propulsion_FZ_lbs,"
        "FX_lbs,FY_lbs,FZ_lbs,"
        "Lon_MX_ftlbs,Lon_MY_ftlbs,Lon_MZ_ftlbs,"
        "LatDir_MX_ftlbs,LatDir_MY_ftlbs,LatDir_MZ_ftlbs,"
        "Propulsion_MX_ftlbs,Propulsion_MY_ftlbs,Propulsion_MZ_ftlbs,"
        "MX_ftlbs,MY_ftlbs,MZ_ftlbs,"

        // Derivatives (Rates)
        "U_dot_fps2,V_dot_fps2,W_dot_fps2,"
        "P_dot_dps2,Q_dot_dps2,R_dot_dps2,"
        "Phi_dot_dps,Theta_dot_dps,Psi_dot_dps,"

        // Integrated States
        "U_fps,V_fps,W_fps,P_dps,Q_dps,R_dps,"
        "Phi_deg,Theta_deg,Psi_deg"
        << "\n";

    // =============================
    // WRITE DATA ROWS
    // =============================
    for (const auto& s : state_history) {
        file <<
            s.Time_sec << ","

            // Reference Parameters
            << s.ReferenceWingArea_ft2 << "," << s.ReferenceChord_ft << "," << s.ReferenceSpan_ft << ","
            << s.SpeedOfSound_SeaLevel_fps << "," << s.StaticPressure_SeaLevel_psf << "," << s.ThrustLimit_lbs << ","

            // Controls
            << s.StabCommand_deg << "," << s.AileronCommand_deg << "," << s.RudderCommand_deg << "," << s.Throttle_norm << ","

            // Mass Properties
            << s.AircraftWeight_lbs << "," << s.CGx_pct << ","
            << s.Ixx_slft << "," << s.Iyy_slft << "," << s.Izz_slft << ","
            << s.Ixy_slft << "," << s.Ixz_slft << "," << s.Iyz_slft << ","

            // Environment
            << s.AltitudeMeanSeaLevel_ft << "," << s.MachNumber << ","
            << s.TrueAirspeed_fps << "," << s.TrueAirspeed_kt << "," << s.EquivilantAirspeed_fps << "," << s.EquivilantAirspeed_kt << ","
            << s.AccelGravity_fts2 << "," << s.SpeedOfSound_kt << "," << s.SpeedOfSound_fps << ","
            << s.AirTemperature_r << "," << s.AirDensity_slugft3 << ","
            << s.StaticPressure_psf << "," << s.DynamicPressure_psf << "," << s.TotalPressure_psf << ","

            // Aero Angles
            << s.Alpha_deg << "," << s.Alpha_dot_dps << ","
            << s.Beta_deg << "," << s.Beta_dot_dps << "," << s.Gamma_deg << ","

            // Longitudinal Aero Coefficients
            << s.CL0 << "," << s.CD0 << "," << s.CM0 << ","
            << s.CL_alpha << "," << s.CD_alpha << "," << s.CM_alpha << ","
            << s.CL_alpha_dot << "," << s.CM_alpha_dot << "," << s.CL_q << "," << s.CM_q << ","
            << s.CL_m << "," << s.CD_m << "," << s.CM_m << ","
            << s.CL_delta_stab << "," << s.CM_delta_stab << ","

            // Lateral-Directional Aero Coefficients
            << s.CY_b << "," << s.Cl_b << "," << s.CN_b << ","
            << s.Cl_p << "," << s.CN_p << "," << s.Cl_r << "," << s.CN_r << ","
            << s.Cl_delta_ail << "," << s.CN_delta_ail << ","
            << s.CY_rud << "," << s.Cl_delta_rud << "," << s.CN_delta_rud << ","

            // Propulsion
            << s.Thrust_lb << ","

            // Forces & Moments
            << s.Lon_FX_lbs << "," << s.Lon_FY_lbs << "," << s.Lon_FZ_lbs << ","
            << s.LatDir_FX_lbs << "," << s.LatDir_FY_lbs << "," << s.LatDir_FZ_lbs << ","
            << s.Propulsion_FX_lbs << "," << s.Propulsion_FY_lbs << "," << s.Propulsion_FZ_lbs << ","
            << s.FX_lbs << "," << s.FY_lbs << "," << s.FZ_lbs << ","
            << s.Lon_MX_ftlbs << "," << s.Lon_MY_ftlbs << "," << s.Lon_MZ_ftlbs << ","
            << s.LatDir_MX_ftlbs << "," << s.LatDir_MY_ftlbs << "," << s.LatDir_MZ_ftlbs << ","
            << s.Propulsion_MX_ftlbs << "," << s.Propulsion_MY_ftlbs << "," << s.Propulsion_MZ_ftlbs << ","
            << s.MX_ftlbs << "," << s.MY_ftlbs << "," << s.MZ_ftlbs << ","

            // Derivatives
            << s.U_dot_fps2 << "," << s.V_dot_fps2 << "," << s.W_dot_fps2 << ","
            << s.P_dot_dps2 << "," << s.Q_dot_dps2 << "," << s.R_dot_dps2 << ","
            << s.Phi_dot_dps << "," << s.Theta_dot_dps << "," << s.Psi_dot_dps << ","

            // Integrated States
            << s.U_fps << "," << s.V_fps << "," << s.W_fps << ","
            << s.P_dps << "," << s.Q_dps << "," << s.R_dps << ","
            << s.Phi_deg << "," << s.Theta_deg << "," << s.Psi_deg

            << "\n";
    }

    file.close();
    std::cout << "State history (" << state_history.size()
              << " frames) written to: " << filename << std::endl;
} // Thanks Claude :)

// Trim methods
State Simulation::TrimResiduals(const State& base, const State& inputs) {
    // Set inputs
    State state = base;
    state = model.SetAlpha(state, inputs.Alpha_deg);
    state.StabCommand_deg = inputs.StabCommand_deg;
    state.Throttle_norm = inputs.Throttle_norm;
    // Run Step
    state = model.Step(state);
    return state;
}


// Objective: sum of squares
double Simulation::CalcCost(const State& state) {
    double cost = std::pow(state.Q_dot_dps2, 2) ;
    cost += std::pow(state.W_dot_fps2, 2);
    cost += std::pow(state.U_dot_fps2, 2);
    return cost;
}


bool Simulation::SolveTrim(const State& initialState, State& trimGuess) {
    // Trim Settings
    int n_states = 3;
    const double tol = 1e-3;
    const int maxIters = 1e6;
    std::vector<double> stepSize = { 1e-3, 1e-4, 1e-3 };
    const double fdEpsilon = 1e-3;
    const double maxGrad = 1e5;
    bool verbose = false;
    // Trim routine
    std::cout << "Trimming:" << std::endl;
    for (int iter = 0; iter < maxIters; ++iter) {
        // Evaluate current cost
        State state = TrimResiduals(initialState, trimGuess);
        double cost = CalcCost(state);
        // Print Outputs
        if (verbose) {
            std::cout << "Step " << iter << " Cost: " << cost << std::endl;
            std::cout << std::fixed << std::setprecision(3)
                << "Alpha_deg: " << std::setw(8) << state.Alpha_deg
                << "  StabCommand_deg: " << std::setw(6) << state.StabCommand_deg
                << "  Throttle_norm: " << std::setw(6) << state.Throttle_norm
                << "  Qdot: " << std::setw(8) << state.Q_dot_dps2
                << "  Udot: " << std::setw(8) << state.U_dot_fps2
                << "  Wdot: " << std::setw(8) << state.W_dot_fps2
                << std::endl;
        }
        // Check convergence
        if (cost < tol) {
            trimGuess = state;
            std::cout << "Success: " << iter << " Iterations" << std::endl;
            return true;
        }
        // Reduce step size discreetly for simplicity as we approach solution
        if (cost < 1e-2) {
            stepSize[0] = 1e-5;
            stepSize[1] = 1e-4;
            stepSize[2] = 1e-4;
        }
        // Compute finite-difference gradients
        State grad = {0, 0, 0};
        for (int i = 0; i < n_states; ++i) {
            State perturbed = trimGuess;
            double* param = (i == 0) ? &perturbed.Alpha_deg
                : (i == 1) ? &perturbed.StabCommand_deg
                : &perturbed.Throttle_norm;

            double original = *param;
            *param = original + fdEpsilon;

            State perturbedState = TrimResiduals(initialState, perturbed);
            double newCost = CalcCost(perturbedState);

            double gradVal = (newCost - cost) / fdEpsilon;
            if (std::abs(gradVal) > maxGrad) {
                gradVal = std::copysign(maxGrad, gradVal);
            }
            //std::cout << "  Gradient[" << i << "] = " << gradVal << std::endl;
            if (i == 0) grad.Alpha_deg = gradVal;
            else if (i == 1) grad.StabCommand_deg = gradVal;
            else grad.Throttle_norm = gradVal;
            *param = original; // restore
        }
        // Gradient descent step
        trimGuess.Alpha_deg -= stepSize[0] * grad.Alpha_deg;
        trimGuess.StabCommand_deg -= stepSize[1] * grad.StabCommand_deg;
        trimGuess.Throttle_norm -= stepSize[2] * grad.Throttle_norm;
        // Clamp inputs
        if (trimGuess.Alpha_deg < -45)    trimGuess.Alpha_deg = -45;
        if (trimGuess.Alpha_deg > 45)    trimGuess.Alpha_deg = 45;
        if (trimGuess.StabCommand_deg < -30) trimGuess.StabCommand_deg = -30;
        if (trimGuess.StabCommand_deg > 30.0)  trimGuess.StabCommand_deg = 30.0;
        if (trimGuess.Throttle_norm < 0.0)    trimGuess.Throttle_norm = 0.0;
        if (trimGuess.Throttle_norm > 1.0)    trimGuess.Throttle_norm = 1.0;
    }
    std::cout << "Failed" << std::endl;
    return false; // Did not converge
}


// At the bottom of Simulation.cpp
double Simulation::TestCost(double Alpha_deg, double StabCommand_deg, double Throttle_norm) {
    State guess = initial_state;
    guess.Alpha_deg = Alpha_deg;
    guess.StabCommand_deg = StabCommand_deg;
    guess.Throttle_norm = Throttle_norm;
    State result = TrimResiduals(initial_state, guess);
    return CalcCost(result);
}