import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


df = pd.read_csv('log_file.csv')
temp = df.values


plt.plot(temp[:,0])
plt.show()


f, t, Sxx = signal.spectrogram(temp[:,0], 44100, detrend=False, nfft=1024)

plt.pcolormesh(t, f, Sxx)
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [sec]')
plt.show()

