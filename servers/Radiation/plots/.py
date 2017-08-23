from pymongo import MongoClient
import numpy as np
import matplotlib.pyplot as plt

cli = MongoClient()
db = cli.ascrapper
pre = db.prediccs

x = []
y = []


for i in pre.find():
	if '2016.' in  i['data'][-1][1]:
		data = i['data']
		print i
		for j in data:
			x.append(float(j[1]))
			y.append(float(j[5]))
		break

x = np.array(x)
y = np.array(y)

plt.plot(x, y, 'b', label="data")
'''
f_data = [2016.20901826,2016.38270548,2016.6409532,2016.64657534,2016.64694635,2016.64694863,2016.64697489,2016.64981735,2016.65111301,2016.68752854,2016.99810502,2016.99880137]

for i in f_data:
	plt.axvline(x=i,color='k', linestyle='--')
'''

plt.xlabel('Time(UTC)')
plt.ylabel('Dosage(cGy/day')
plt.legend(loc='upper left')
plt.title('Dose Mars')
plt.show()



