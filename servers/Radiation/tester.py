from pymongo import MongoClient
from time import sleep
from datetime import datetime
from StartClass import start

present_hour = datetime.now().hour
hour = present_hour
StartObj = start('test_db')

client = MongoClient()
db = client.test_db
fos = db.forspef
pre = db.prediccs
cac = db.cactus

ffos = open("Data/forspef.txt", 'r+')
fpre = open("Data/prediccs.txt", 'r+')
fcac = open("Data/cactus.txt", 'r+')

fos_data = []
pre_data = []
cac_data = []

for lines in ffos:
    fos_data.append(lines)

for lines in fpre:
    pre_data.append(lines)

for lines in fcac:
    cac_data.append(lines)

flag = 0

while True:
    present_hour = datetime.now().hour
    if True:
        fos.insert(eval(fos_data[flag]))
        pre.insert(eval(pre_data[flag]))
        cac.insert(eval(cac_data[flag]))
        flag = flag + 1
        StartObj.alarm()
        variable = "alarm triggered"
        if StartObj.alarm_triggered and not StartObj.SEP:
            StartObj.prediccs_alarm()
        if StartObj.SEP:
            StartObj.all_clear()
        print "**************End of loop********************"
        sleep(10)
