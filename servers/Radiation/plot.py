import matplotlib.pyplot as plt
import requests
import numpy as np

g = requests.get("http://localhost:8000/plot")
data = g.json()['data']

x = []
y = []
y1 = []
x1 = []
for i in data:
    x.append(float(i[1]))
    y.append(float(i[-2]))
    if float(i[-2]) > float(0.068):
        y1.append(float(i[-2]))
        x1.append(float(i[1]))

x = np.array(x)
y = np.array(y)

plt.plot(x, y, 'b', label="data")
plt.plot(x1, y1, 'r', label="alarm")

plt.xlabel('Time(UTC)')
plt.ylabel('Dosage(cGy/day')
plt.legend(loc='upper left')
plt.title('Dose Mars')
plt.show()
