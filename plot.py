import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


# Load dataframe
file_path = 'StateHistory.csv'
df = pd.read_csv(file_path)
df = df.replace('\(ind\)', '', regex=True)
df = df.astype(float)
df = df.fillna(0)

fig, ax = plt.subplots(5, 1, figsize=(12, 6), tight_layout=True)
df.plot(ax=ax[0], x='Time_sec', y='AltitudeMeanSeaLevel_ft', kind='line', subplots=True)
df.plot(ax=ax[1], x='Time_sec', y='MachNumber', kind='line', subplots=True)
df.plot(ax=ax[2], x='Time_sec', y='Alpha_deg', kind='line', subplots=True)
df.plot(ax=ax[3], x='Time_sec', y='Theta_deg', kind='line', subplots=True)
df.plot(ax=ax[4], x='Time_sec', y='Gamma_deg', kind='line', subplots=True)

# fig, ax = plt.subplots(5, 1, figsize=(12, 6), tight_layout=True)
# df.plot(ax=ax[0], x='Time_sec', y='AltitudeMeanSeaLevel_ft', kind='line', subplots=True)
# df.plot(ax=ax[1], x='Time_sec', y='TrueAirspeed_kt', kind='line', subplots=True)
# df.plot(ax=ax[2], x='Time_sec', y='U_fps', kind='line', subplots=True)
# df.plot(ax=ax[3], x='Time_sec', y='V_fps', kind='line', subplots=True)
# df.plot(ax=ax[4], x='Time_sec', y='W_fps', kind='line', subplots=True)

# fig, ax = plt.subplots(6, 1, figsize=(12, 6), tight_layout=True)
# df.plot(ax=ax[0], x='Time_sec', y='AltitudeMeanSeaLevel_ft', kind='line', subplots=True)
# df.plot(ax=ax[1], x='Time_sec', y='TrueAirspeed_kt', kind='line', subplots=True)
# df.plot(ax=ax[2], x='Time_sec', y='Alpha_deg', kind='line', subplots=True)
# df.plot(ax=ax[3], x='Time_sec', y='FX_lbs', kind='line', subplots=True)
# df.plot(ax=ax[4], x='Time_sec', y='FZ_lbs', kind='line', subplots=True)
# df.plot(ax=ax[5], x='Time_sec', y='MY_lbs', kind='line', subplots=True)



plt.show()


