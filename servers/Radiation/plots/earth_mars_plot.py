from pymongo import MongoClient
import numpy as np
import matplotlib.pyplot as plt

cli = MongoClient()
db = cli.ascrapper
test = db.test

x = []
y = []
x1 = []
y1 = []

for i in test.find():
	ear = i['data']
	break
for i in ear:
	x.append(i[0])
	y.append(i[-1])

for i in test.find().sort('_id', -1).limit(1):
	mars = i['data']

for i in mars:
	x1.append(i[0])
	y1.append(i[-1])


 
x = np.array(x)
y = np.array(y)
x1 = np.array(x1)
y1 = np.array(y1)

plt.plot(x, y, 'b', label="earth")
plt.plot(x1, y1, 'r', label="mars")

plt.xlabel('Time(jul day)')
plt.ylabel('Dosage(cGy/day)')
plt.legend(loc='upper left')
plt.title('Dose Mars')
plt.show()



