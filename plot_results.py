import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data1 = pd.read_csv('data/Fase3-1.csv')
data2 = pd.read_csv('data/Fase3-2.csv')
data = pd.read_csv('data/Fase3.csv')

plt.plot(data['Position_Robot'], data['Speed_Robot'])
plt.plot(data2['Position_Robot'], data2['Speed_Robot'])
plt.plot(data1['Position_Robot'], data1['Speed_Robot'])

plt.legend(['Inercia + Fricción', 'Fricción', 'Nada'], loc="upper right")
plt.ylabel('Robot Speed')
plt.xlabel('Robot Position (meters)')
plt.title('Análisis')
plt.show()

data = pd.read_csv('data/Fase4.csv')

plt.plot(data['Position_Robot'], data['Speed_Robot'])
plt.plot(data['Position_Robot'], data['Speed_Wheel']/10)
plt.plot(data['Position_Robot'], data['Force_Wheel']/50)
plt.plot(data['Position_Robot'], [2]*len(data['Speed_Wheel']))
print(np.mean(data['Speed_Robot']))

plt.ylabel('Robot Speed')
plt.xlabel('Robot Position (meters)')
plt.title('Análisis')
plt.show()