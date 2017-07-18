from __future__ import absolute_import, division, print_function
from flask import Flask, flash, redirect, render_template, url_for
import datetime
import PyTango
import json
import time
import os
import pandas as pd
import numpy as np
import plotly
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
	name = biometric_monitor.username
	recordID = biometric_monitor.recordID
	user_info = json.loads(biometric_monitor.userinfo)
	user_info = user_info['objects'][0]
	details = {}
	# ------ User details
	details['email'] = user_info['email']
	details['id'] = user_info['id']
	details['dob'] = user_info['profile']['date_of_birth']
	details['gender'] = user_info['profile']['gender']
	details['username'] = user_info['username']
	details['hid'] = user_info['profile']['preferences']['deviceId']
	#------- User health details
	details['hr_rest_val'] = user_info['profile']['fitness'][0]['value']
	details['hr_rest_desc'] = user_info['profile']['fitness'][0]['zone_description']
	details['bmi_val'] = user_info['profile']['fitness'][1]['value']
	details['height'] = user_info['profile']['fitness'][2]['value']
	details['weight'] = user_info['profile']['fitness'][3]['value']
	details['weight_desc'] = user_info['profile']['fitness'][3]['zone_description']
	details['hr_max_val'] = user_info['profile']['fitness'][4]['value']
	details['hr_max_desc'] = user_info['profile']['fitness'][4]['zone_description']
	details['sleep_val'] = user_info['profile']['fitness'][5]['value']
	details['sleep_desc'] = user_info['profile']['fitness'][5]['zone_description']
	details['hr_rec_val'] = user_info['profile']['fitness'][7]['value']
	details['hr_rec_desc'] = user_info['profile']['fitness'][7]['zone_description']
	details['vo2_max_val'] = user_info['profile']['fitness'][8]['value']
	details['vo2_max_desc'] = user_info['profile']['fitness'][8]['zone_description']
	details['resp_val'] = user_info['profile']['fitness'][9]['value']

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

	return render_template('home.html', name=name, recordID=recordID,
		details=details)

@app.route("/anomaly")
def anomaly():
	_af_anomaly = get_AF_anomaly()[::-1]
	_vt_anomaly = get_VT_anomaly()[::-1]
	return render_template('anomaly.html', AF=_af_anomaly, VT=_vt_anomaly)

@app.route("/raw_data")
def raw_data():
	_af_anomaly = []
	_vt_anomaly = []
	rng = pd.date_range('1/1/2011', periods=7500, freq='H')
	ts = pd.Series(np.random.randn(len(rng)), index=rng)

	graphs = [
	    dict(
            data=[
                dict(
                    x=ts.index,  # Can use the pandas data structures directly
                    y=ts
				),
            ],
            layout=dict(
                title='ECG'
            )
		),
		dict(
            data=[
                dict(
                    x=ts.index,  # Can use the pandas data structures directly
                    y=ts
				),
            ],
            layout=dict(
                title='Heart Rate'
            )
		)
	]


	# Add "ids" to each of the graphs to pass up to the client
	# for templating
	ids = ['graph-{}'.format(i) for i, _ in enumerate(graphs)]

	# Convert the figures to JSON
	# PlotlyJSONEncoder appropriately converts pandas, datetime, etc
	# objects to their JSON equivalents
	graphJSON = json.dumps(graphs, cls=plotly.utils.PlotlyJSONEncoder)
	return render_template('raw_data.html', AF=_af_anomaly, VT=_vt_anomaly,
		safe=safe, ids=ids, graphJSON=graphJSON)


if __name__ == "__main__":
    app.run(threaded=True, debug=DEBUG)