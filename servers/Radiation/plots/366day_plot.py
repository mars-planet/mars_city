from pymongo import MongoClient
import numpy as np
import matplotlib.pyplot as plt

cli = MongoClient()
db = cli.test
pre = db.prediccs

x = []
y = []

for i in pre.find().sort('_id', -1).limit(1):
	data = i['data']
	break

for i in data:
	x.append(float(i[1]))
	y.append(float(i[5]))

x = np.array(x)
y = np.array(y)

plt.plot(x, y, 'b', label="data")


f_data = [2016.38257074,2016.64677418,2016.64644884,2016.64679701]
for i in f_data:
	plt.axvline(x=i,color='k', linestyle='--')




plt.xlabel('Time(UTC)')
plt.ylabel('Dosage(cGy/day')
plt.legend(loc='upper left')
plt.title('Dose Mars')
plt.show()

