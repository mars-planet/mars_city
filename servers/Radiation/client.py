import requests
from datetime import datetime
from time import sleep


present_hour = datetime.now().hour
hour = present_hour

while True:
    present_hour = datetime.now().hour
    if hour == present_hour:
        req = requests.get("http://localhost:8000/test")
        data = req.json()
        if 'data' in data:
            data_list = data['data']
        print data
        hour = hour + 1
