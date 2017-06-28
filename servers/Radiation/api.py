import json
import falcon
import os
from StartClass import start
from datetime import datetime

os.chdir("scrapper/scrapper/spiders/")


class test(object):

    StartObj = start('scrapper')

    def on_get(self, req, resp):
        var1 = ''
        var2 = 0
        var3 = 0
        os.system('scrapy crawl forspef')
        os.system('scrapy crawl prediccs')
        os.system('scrapy crawl cactus')
        var1 = self.StartObj.alarm()
        if self.StartObj.alarm_triggered and not self.StartObj.SEP:
            var2 = self.StartObj.prediccs_alarm()
        if self.StartObj.SEP:
            var3 = self.StartObj.all_clear()
        data = {'time of arrival': var1,
                'prediccs-alarm': var2, 'all-clear': var3}
        data = {'data': data, 'time': str(datetime.now()), 'thresholds': {
            'SEP probability threshold': 0.04,
            'Thin spacesuit shielding threshold': 0.068,
            'Storm shelter shielding threshold': 0.068}}
        resp.body = json.dumps(data)
        resp.status = falcon.HTTP_200
        resp.content_type = 'application/json'


class plots(object):

    def __init__(self):
        self.StartObj = start('scrapper')

    def on_get(self, req, resp):

        data = self.StartObj.plots()
        resp.body = json.dumps(data)
        resp.status = falcon.HTTP_200
        resp.content_type = 'application/json'


app = application = falcon.API()
api = [test(), plots()]

app.add_route('/test', api[0])
app.add_route('/plot', api[1])
