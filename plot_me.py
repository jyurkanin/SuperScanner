import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('log_file.csv')
temp = df.values

plt.plot(temp[100:] - temp[:-100])
plt.show()
