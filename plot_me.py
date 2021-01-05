import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('log_file.csv')
plt.plot(df['gain'])
#plt.plot(df['O'])
plt.show()
