import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv('data/Fase4.csv')
data1 = pd.read_csv('data/Fase3-1.csv')
data2 = pd.read_csv('data/Fase3-2.csv')
data3 = pd.read_csv('data/Fase3.csv')

plt.plot(data3['Position_Robot'], data3['Speed_Robot'])
plt.plot(data2['Position_Robot'], data2['Speed_Robot'])
plt.plot(data1['Position_Robot'], data1['Speed_Robot'])

plt.legend(['Inercia + Fricción', 'Fricción', 'Nada'], loc="upper right")
plt.ylabel('Robot Speed')
plt.xlabel('Robot Position (meters)')
plt.title('Análisis Fase 3')
plt.show()

# Calculo de errores entre la Fase 4 y el objetivo de 2 m/s
# print("Ej 4: ",np.sqrt(np.mean((data['Speed_Robot'] - [2]*len(data['Speed_Wheel'])) ** 2)), np.std(data['Speed_Robot']))
# print("Ej 3: ",np.sqrt(np.mean((data3['Speed_Robot'] - [2]*len(data3['Speed_Wheel'])) ** 2)), np.std(data3['Speed_Robot']))

plt.plot(data['Position_Robot'], data['Speed_Robot'])
plt.plot(data['Position_Robot'], [2]*len(data['Speed_Wheel']))

plt.legend(['Fase 4', 'Objetivo 2 m/s'], loc="upper right")
plt.ylabel('Robot Speed')
plt.xlabel('Robot Position (meters)')
plt.title('Análisis Fase 4')
plt.show()

plt.plot(data['Position_Robot'], data['Speed_Robot'])
plt.plot(data3['Position_Robot'], data3['Speed_Robot'])
plt.plot(data['Position_Robot'], [2]*len(data['Speed_Wheel']))

plt.legend(['Fase 4', 'Fase 3', 'Objetivo 2 m/s'], loc="upper right")
plt.ylabel('Robot Speed')
plt.xlabel('Robot Position (meters)')
plt.title('Análisis Fase 3 y 4')
plt.show()
