# Simple Linear Aircraft Model (SLAM)
SLAM is a demonstrative 6 degree-of-freedom airplane flight simulation based loosely on the Lockheed F-104 Starfighter and implemented in object-oriented C++. Simulation results are output in CSV format and plotted using the included Python utility scripts.

## Model Structure
SLAM is implemented in C++ using an object-oriented philosophy. It consists of the following object structure:
- Simulation Object 
  - Manages simulation level operations like initializing, trimming, and running time histories.
  - Trim is accomplished using gradient descent to find a 3-state longitudinal trim solution for angle-of-attack, stabilator, and throttle.
  - The trim cost function can also be exposed to plot explanatory trim cost surfaces using `plot_cost.py`
- Model Object
  - Encapsulates all physical modeling including atmospherics, longitudinal and lateral-directional aerodynamics, propulsion, and equations of motion.
  - Allows for time-scheduled inputs for stabilator, aileron, and rudder.
- USSA Object
  - Allows for easy lookup of atmospherics from the 1976 Standard Atmosphere tables.

## Physical and Aerodynamic Modeling Philosophy
Atmospherics in SLAM are calculated using an implementation of the 1976 Standard Atmosphere (Yager, 2013). Aerodynamic coefficients are linear and invariant (hence the "L" in SLAM) and were scavenged from class notes, textbooks, and internet sources. 
Equations of motion are simplified assuming the aircraft body axes align with the principal axes of the aircraft and are integrated using Euler integration.

## Usage
Once built, SLAM can be called from the command line using the following signature:
SLAM.exe \[Maneuver Type\] \[Altitude Above MSL (ft)\] \[Mach Number\]

Maneuver type should be one of the following current options:
- LonTrim
  - Basic 3-DOF trim and 60 second shakeout time history
- StabDoublet
  - Basic 3-DOF trim and 30 second time history with stabilator doublet
- AileronDoublet
  - Basic 3-DOF trim and 30 second time history with aileron doublet
- RudderDoublet
  - Basic 3-DOF trim and 30 second time history with rudder doublet

## Plotting
Plotting can be accomplished in the included `plot_maneuvers.py` using Pandas, and Matplotlib. This Python script currently runs all maneuvers types in SLAM via command line and plots exploratory plots for each maneuver. 

## Potential Future Work:
- Additional maneuvers, such as thrust doublets or coordinated turns.
- PID control could be implemented to track states such as speed, gamma, or bank angle.
- Currently, the simulation relies on Euler angles to track orientation, which are subject to gimbal-lock. Quaternion states could be implemented to avoid invalid results in unusual attitudes. 
- Full, cross-coupled equations of motion could be implemented using a solver.
- Polymorphism of the model class could allow the Simulation class to operate using different models.

## Example Results
![Basic Longitudinal Trim](https://github.com/goblegrayson/SLAM/blob/main/output_files/LonTrim_Plot.png)
![Stabilator Doublet](https://github.com/goblegrayson/SLAM/blob/main/output_files/StabDoublet_Plot.png)
![Aileron Doublet](https://github.com/goblegrayson/SLAM/blob/main/output_files/AileronDoublet_Plot.png)
![Rudder Doublet](https://github.com/goblegrayson/SLAM/blob/main/output_files/RudderDoublet_Plot.png)
