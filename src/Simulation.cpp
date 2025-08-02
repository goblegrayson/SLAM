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
	state_history.resize(n_frames);
	state_history[0] = initial_state;
	// Main Loop
	for (int i_frame = 1; i_frame < n_frames; i_frame++) {
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

    // ========================================================================
    // WRITE HEADER ROW - Organized by category for readability
    // ========================================================================
    file <<
        // Simulation
        "Time_sec,"

        // Reference Parameters
        "ReferenceWingArea_ft2,ReferenceChord_ft,ReferenceSpan_ft,"
        "SpeedOfSound_SeaLevel_fps,StaticPressure_SeaLevel_psf,"
        "ThrustLimit_lbs,StabLimit_deg,AilerontLimit_deg,"

        // Controls
        "PitchStick_norm,RollStick_norm,Throttle_norm,"

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
        "Alpha_deg,Beta_deg,Gamma_deg,"

        // Longitudinal Aerodynamics
        "CL0,CD0,CL_alpha,CD_alpha,CM_alpha,"
        "CL_alpha_dot,CM_alpha_dot,CL_q,CM_q,"
        "CL_m,CD_m,CM_m,CL_delta_stab,CM_delta_stab,"

        // Lateral-Directional Aerodynamics
        "CY_b,Cl_b,CN_b,Cl_p,CN_p,Cl_r,CN_r,"
        "Cl_alpha,CN_alpha,CY_delta_r,Cl_delta_r,CN_delta_r,"

        // Propulsion
        "Throttle_norm,Thrust_lb,"

        // Forces & Moments
        "FX_lbs,FY_lbs,FZ_lbs,MX_lbs,MY_lbs,MZ_lbs,"

        // Derivatives (Rates)
        "U_dot_fps2,V_dot_fps2,W_dot_fps2,"
        "P_dot_dps2,Q_dot_dps2,R_dot_dps2,"
        "Phi_dot_dps,Theta_dot_dps,Psi_dot_dps,"

        // Integrated States
        "U_fps,V_fps,W_fps,P_dps,Q_dps,R_dps,"
        "Phi_deg,Theta_deg,Psi_deg"
        << "\n";

    // ========================================================================
    // WRITE DATA ROWS
    // ========================================================================
    for (size_t i = 0; i < state_history.size(); ++i) {
        const State& s = state_history[i];  // Use reference to avoid copying

        file <<
            // Simulation
            s.Time_sec << ","

            // Reference Parameters
            << s.ReferenceWingArea_ft2 << "," << s.ReferenceChord_ft << "," << s.ReferenceSpan_ft << ","
            << s.SpeedOfSound_SeaLevel_fps << "," << s.StaticPressure_SeaLevel_psf << ","
            << s.ThrustLimit_lbs << "," << s.StabLimit_deg << "," << s.AilerontLimit_deg << ","

            // Controls
            << s.PitchStick_norm << "," << s.RollStick_norm << "," << s.Throttle_norm << ","

            // Mass Properties
            << s.AircraftWeight_lbs << "," << s.CGx_pct << ","
            << s.Ixx_slft << "," << s.Iyy_slft << "," << s.Izz_slft << ","
            << s.Ixy_slft << "," << s.Ixz_slft << "," << s.Iyz_slft << ","

            // Environment
            << s.AltitudeMeanSeaLevel_ft << "," << s.MachNumber << ","
            << s.TrueAirspeed_fps << "," << s.TrueAirspeed_kt << ","
            << s.EquivilantAirspeed_fps << "," << s.EquivilantAirspeed_kt << ","
            << s.AccelGravity_fts2 << "," << s.SpeedOfSound_kt << "," << s.SpeedOfSound_fps << ","
            << s.AirTemperature_r << "," << s.AirDensity_slugft3 << ","
            << s.StaticPressure_psf << "," << s.DynamicPressure_psf << "," << s.TotalPressure_psf << ","

            // Aero Angles
            << s.Alpha_deg << "," << s.Beta_deg << "," << s.Gamma_deg << ","

            // Longitudinal Aerodynamics
            << s.CL0 << "," << s.CD0 << "," << s.CL_alpha << "," << s.CD_alpha << "," << s.CM_alpha << ","
            << s.CL_alpha_dot << "," << s.CM_alpha_dot << "," << s.CL_q << "," << s.CM_q << ","
            << s.CL_m << "," << s.CD_m << "," << s.CM_m << ","
            << s.CL_delta_stab << "," << s.CM_delta_stab << ","

            // Lateral-Directional Aerodynamics
            << s.CY_b << "," << s.Cl_b << "," << s.CN_b << ","
            << s.Cl_p << "," << s.CN_p << "," << s.Cl_r << "," << s.CN_r << ","
            << s.Cl_alpha << "," << s.CN_alpha << ","
            << s.CY_delta_r << "," << s.Cl_delta_r << "," << s.CN_delta_r << ","

            // Propulsion
            << s.Throttle_norm << "," << s.Thrust_lb << ","

            // Forces & Moments
            << s.FX_lbs << "," << s.FY_lbs << "," << s.FZ_lbs << ","
            << s.MX_lbs << "," << s.MY_lbs << "," << s.MZ_lbs << ","

            // Derivatives (Rates)
            << s.U_dot_fps2 << "," << s.V_dot_fps2 << "," << s.W_dot_fps2 << ","
            << s.P_dot_dps2 << "," << s.Q_dot_dps2 << "," << s.R_dot_dps2 << ","
            << s.Phi_dot_dps << "," << s.Theta_dot_dps << "," << s.Psi_dot_dps << ","

            // Integrated States (no comma after last item)
            << s.U_fps << "," << s.V_fps << "," << s.W_fps << ","
            << s.P_dps << "," << s.Q_dps << "," << s.R_dps << ","
            << s.Phi_deg << "," << s.Theta_deg << "," << s.Psi_deg
            << "\n";
    }

    file.close();
    std::cout << "Complete state history (" << state_history.size()
        << " records) written to " << filename << std::endl;
    // Thanks Claude :)
}