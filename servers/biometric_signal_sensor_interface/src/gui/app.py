from __future__ import absolute_import, division, print_function
from flask import Flask, flash, redirect, render_template, url_for
import datetime
import PyTango
import json
import time
import os

'''
Python Flask GUI Dashboard for the Biometric Signal Sensor's system
'''
__author__ = 'abhijith'

app = Flask(__name__)
app.secret_key = os.urandom(24)
app.config.from_object(__name__)
DEBUG = True

biometric_monitor = PyTango.DeviceProxy("C3/biometric_monitor/1")

def get_AF_anomaly():
	af_anomaly_json = json.loads(biometric_monitor.af_to_gui())
	_af_anomaly = []
	for key, value in af_anomaly_json.items():
		_record = []
		_record.append(time.strftime('%Y-%m-%d %H:%M:%S',
			time.localtime(float(key)/256)))
		_record.append(time.strftime('%Y-%m-%d %H:%M:%S',
			time.localtime(float(value[0])/256)))
		_record.append(value[1])
		_record.append(value[2])
		_record.append(value[3])
		_record.append(value[4])
		# For other processing
		_record.append(float(key)/256)
		_af_anomaly.append(_record)
	return _af_anomaly

def get_VT_anomaly():
	vt_anomaly_json = json.loads(biometric_monitor.vt_to_gui())
	_vt_anomaly = []
	for key, value in vt_anomaly_json.items():
		_record = []
		_record.append(time.strftime('%Y-%m-%d %H:%M:%S',
			time.localtime(float(key)/256)))
		_record.append(time.strftime('%Y-%m-%d %H:%M:%S',
			time.localtime(float(value[0])/256)))
		_record.append(value[1])
		_record.append(value[2])
		# For other processing
		_record.append(float(key)/256)
		_vt_anomaly.append(_record)
	return _vt_anomaly


@app.route("/")
def home():
	username = biometric_monitor.username
	recordID = biometric_monitor.recordID
	_af_anomaly = get_AF_anomaly()[::-1]
	today = datetime.datetime.today()
	week_ago = today - datetime.timedelta(days=7)
	today = ((today - datetime.datetime(1970,1,1)).total_seconds())
	week_ago = ((week_ago - datetime.datetime(1970,1,1)).total_seconds())
	
	safe = True
	try:
		if week_ago > _af_anomaly[0][6]:
			safe = True
		else:
			safe = False
	except:
		pass

	return render_template('home.html', username=username, recordID=recordID,
		safe=safe)

@app.route("/anomaly")
def anomaly():
	_af_anomaly = get_AF_anomaly()[::-1]
	_vt_anomaly = get_VT_anomaly()[::-1]
	return render_template('anomaly.html', AF=_af_anomaly, VT=_vt_anomaly)

@app.route("/raw_data")
def raw_data():
	_af_anomaly = []
	_vt_anomaly = []
	return render_template('raw_data.html', AF=_af_anomaly, VT=_vt_anomaly)


if __name__ == "__main__":
    app.run(threaded=True, debug=DEBUG)