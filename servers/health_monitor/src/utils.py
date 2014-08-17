from datetime import datetime, date, time
import json


def datetimehandler(obj):
    if(isinstance(obj, datetime)
            or isinstance(obj, date)
            or isinstance(obj, time)):
        return obj.isoformat()
    else:
        return json.JSONEncoder().default(obj)
