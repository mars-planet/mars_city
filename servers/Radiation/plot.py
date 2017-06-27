import matplotlib.pyplot as plt
import requests

g = requests.get("http://localhost:8000/plot")
data = g.json()['data']
alarm_data = g.json()['alarm_data']

x = []
y = []
x1 = []
y1 = []

for i in data:
    x.append(float(i[1]))
    y.append(float(i[-2]))


if alarm_data:
    for i in alarm_data:
        x1.append(float(i[1]))
        y1.append(float(i[-2]))
    plt.plot(x1, y1, 'r', label="alarm")


plt.plot(x, y, 'b', label="data")
plt.xlabel('Time')
plt.ylabel('Dosage')
plt.legend(loc='upper left')
plt.show()
