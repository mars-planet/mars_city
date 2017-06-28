import os
from pymongo import MongoClient
from datetime import datetime
from StartClass import start

os.chdir("scrapper/scrapper/spiders/")
present_hour = datetime.now().hour
hour = present_hour
StartObj = start('scrapper')

while True:
    present_hour = datetime.now().hour
    if hour == present_hour:
        os.system('scrapy crawl forspef')
        os.system('scrapy crawl prediccs')
        os.system('scrapy crawl cactus')
        hour = hour + 1
        StartObj.alarm()
        if StartObj.alarm_triggered and not StartObj.SEP:
            StartObj.prediccs_alarm()
        if StartObj.SEP:
            StartObj.all_clear()
