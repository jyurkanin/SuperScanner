import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


df = pd.read_csv('log.csv')
temp = df.values

print("Shape", temp.shape)


f, t, Sxx = signal.spectrogram(temp[:,0], 88200, detrend=False)

plt.pcolormesh(t, f, Sxx)
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [sec]')
plt.show()




f, t, Sxx = signal.spectrogram(temp[:,1], 88200, detrend=False)

plt.pcolormesh(t, f, Sxx)
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [sec]')
plt.show()
