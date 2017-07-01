import json
import falcon
import os
from StartClass import start
from datetime import datetime
from pymongo import MongoClient

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

os.chdir("scrapper/scrapper/spiders/")


class test(object):

    StartObj = start('test_db')

    def __init__(self):
        self.flag = 0

    def on_get(self, req, resp):
        var1 = ''
        var2 = 0
        var3 = 0
        fos.insert(eval(fos_data[self.flag]))
        pre.insert(eval(pre_data[self.flag]))
        cac.insert(eval(cac_data[self.flag]))
        self.flag = self.flag + 1
        var1 = self.StartObj.alarm()
        if self.StartObj.alarm_triggered and not self.StartObj.SEP:
            var2 = self.StartObj.prediccs_alarm()
        if self.StartObj.SEP:
            var3 = self.StartObj.all_clear()
        data = []
        data.append(var1)
        data.append(var2)
        data.append(var3)
        data = {'data': data, 'time': str(datetime.now()), 'thresholds': {
            'SEP probability threshold': 0.04,
            'Thin spacesuit shielding threshold': 0.09,
            'Storm shelter shielding threshold': 0.001}}
        resp.body = json.dumps(data)
        print self.StartObj.stack
        resp.status = falcon.HTTP_200
        resp.content_type = 'application/json'


class plots(test):

    def on_get(self, req, resp):

        print super(plots, self).StartObj.stack
        data = self.StartObj.plots()
        resp.body = json.dumps(data)
        resp.status = falcon.HTTP_200
        resp.content_type = 'application/json'


app = application = falcon.API()
testapi = [test(), plots()]

app.add_route('/test', testapi[0])
app.add_route('/plot', testapi[1])
