import numpy as np
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Grid settings
alpha_vals = np.linspace(-5, 5, 100)           # Alpha in degrees
pitch_vals = np.linspace(-1, 1, 10)            # PitchStick_norm
throttle = 1                                    # Fixed throttle

# Output grid
Alpha_grid, Pitch_grid = np.meshgrid(alpha_vals, pitch_vals)
Cost_grid = np.zeros_like(Alpha_grid)

# Path to compiled simulation executable
exe_path = r"D:\Data\SLAM\x64\Debug\SLAM.exe"

# Compute cost values
for i in range(Alpha_grid.shape[0]):
    for j in range(Alpha_grid.shape[1]):
        alpha = Alpha_grid[i, j]
        pitch = Pitch_grid[i, j]
        cmd = [exe_path, str(alpha), str(pitch), str(throttle)]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            cost = float(result.stdout.strip())
        except Exception as e:
            print(f"Error at Alpha={alpha}, Pitch={pitch}: {e}")
            cost = np.nan
        Cost_grid[i, j] = cost

# Plotting
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Surface plot
surf = ax.plot_surface(Alpha_grid, Pitch_grid, Cost_grid,
                       cmap='viridis', edgecolor='none', linewidth=0, antialiased=True)

# Labels
ax.set_xlabel("Alpha_deg")
ax.set_ylabel("PitchStick_norm")
ax.set_zlabel("Trim Cost")
ax.set_title("Trim Cost Surface (Throttle = {:.2f})".format(throttle))

# Color bar
fig.colorbar(surf, shrink=0.5, aspect=10)

plt.tight_layout()
plt.show()

print('')