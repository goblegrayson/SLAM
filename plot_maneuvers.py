import os

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import subprocess
from pathlib import Path

# User Settings
AltitudeMeanSeaLevel_ft = 55000
MachNumber = 1.8
maneuver_types = ['LonTrim', 'StabDoublet', 'AileronDoublet', 'RudderDoublet']
pretty_types = ['Longitudinal Trim', 'Stabilator Doublet', 'Aileron Doublet', 'Rudder Doublet']

# Run simulations via SLAM.exe
base_path = Path(__file__).absolute().parent
state_paths = [base_path.joinpath('output_files', maneuver_type + '.csv') for maneuver_type in maneuver_types]
exe_path = r"D:\Data\SLAM\x64\Debug\SLAM.exe"
[subprocess.run([exe_path, maneuver_type, str(AltitudeMeanSeaLevel_ft), str(MachNumber)], check=True) for i, maneuver_type in enumerate(maneuver_types) if not state_paths[i].exists()]

# Set up up plots
plot_specs = [[
    ('AltitudeMeanSeaLevel_ft', 'Altitude (ft)', [50000, 60000]),
    ('MachNumber', 'Mach Number', [1.79, 1.81]),
    ('StabCommand_deg', 'Stabilator Command (deg)', [-10, 10]),
    ('Alpha_deg', 'Angle of Attack (deg)', [0, 30]),
    ('Theta_deg', 'Pitch Angle (deg)', [0, 30]),
    ('Gamma_deg', 'Flight Path Angle (deg)', [-1, 1])
    ], [
    ('AltitudeMeanSeaLevel_ft', 'Altitude (ft)', [50000, 60000]),
    ('MachNumber', 'Mach Number', [1.79, 1.81]),
    ('StabCommand_deg', 'Stabilator Command (deg)', [-10, 10]),
    ('Alpha_deg', 'Angle of Attack (deg)', [0, 30]),
    ('Theta_deg', 'Pitch Angle (deg)', [0, 30]),
    ('Gamma_deg', 'Flight Path Angle (deg)', [-1, 1])
    ], [
    ('Gamma_deg', 'Flight Path Angle (deg)', [-1, 1]),
    ('AileronCommand_deg', 'Sym. Aileron Command (deg, +RWD)', [-10, 10]),
    ('P_dps', 'Roll Rate (deg/s)', [-15, 15]),
    ('Beta_deg', 'Angle of Sideslip (deg)', [-5, 5]),
    ('Phi_deg', 'Bank Angle (deg)', [-15, 15]),
    ('Psi_deg', 'Heading Angle (deg)', [-5, 5])
    ], [
    ('Gamma_deg', 'Flight Path Angle (deg)', [-1, 1]),
    ('RudderCommand_deg', 'Rudder Command (deg)', [-15, 15]),
    ('R_dps', 'Yaw Rate (deg/s)', [-5, 5]),
    ('Beta_deg', 'Angle of Sideslip (deg)', [-5, 5]),
    ('Phi_deg', 'Bank Angle (deg)', [-20, 20]),
    ('Psi_deg', 'Heading Angle (deg)', [-5, 5])
    ]
]

# Make plots
for i_maneuver, plot_spec in enumerate(plot_specs):
    maneuver_type = maneuver_types[i_maneuver]
    LonTrim = pd.read_csv(state_paths[i_maneuver])
    LonTrim = LonTrim.replace('\(ind\)', '', regex=True).astype(float).fillna(0)
    fig, axs = plt.subplots(6, 1, sharex=True, dpi=150, figsize=(10, 12))
    fig.suptitle(f'SLAM: {pretty_types[i_maneuver]} Maneuver at {AltitudeMeanSeaLevel_ft:.0f} ft, Mach {MachNumber}', fontsize=12)
    for ax, (col, ylabel, ylim) in zip(axs, plot_spec):
        ax.plot(LonTrim['Time_sec'], LonTrim[col], label=col, color='tab:blue', linewidth=1.5)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_ylim(ylim)
        ax.grid(True, which='both', linestyle='--', linewidth=0.6)
        ax.tick_params(axis='both', which='major', labelsize=8)
        ax.minorticks_on()

    axs[-1].set_xlabel('Time (s)', fontsize=8)
    plt.tight_layout()
    plt.savefig(base_path.joinpath('output_files', f'{maneuver_type}_Plot.png'), bbox_inches='tight')

# Show plots
plt.show()


