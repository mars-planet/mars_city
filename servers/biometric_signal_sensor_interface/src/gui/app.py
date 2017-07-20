from __future__ import absolute_import, division, print_function
from flask import Flask, flash, redirect, render_template, url_for
from threading import Thread
import ConfigParser
import datetime
import requests
import PyTango
import json
import time
import os
import pandas as pd
import numpy as np
import plotly
import sys
sys.path.insert(0, '../hexoskin_helper')
import resource_helper as resource

'''
Python Flask GUI Dashboard for the Biometric Signal Sensor's system
'''
__author__ = 'abhijith'

app = Flask(__name__)
app.secret_key = os.urandom(24)
app.config.from_object(__name__)
DEBUG = True

requests.packages.urllib3.disable_warnings()
config = ConfigParser.ConfigParser()
config.read("../health_monitor/config.cfg")

def config_helper(section):
    '''
    Returns a dictonary of the configuration stored in
    ../biometric_monitor/config.cfg
        @param section: configuration section from the config file that,
                        has to be read
    '''
    dict_config = {}
    options = config.options(section)
    for option in options:
        try:
            dict_config[option] = config.get(section, option)
        except:
            print("exception on %s!" % option)
            dict_config[option] = None
    return dict_config


biometric_monitor = PyTango.DeviceProxy(
	config_helper("BiometricMonitor")['name'])


def get_APC_anomaly():
	apc_anomaly_json = json.loads(biometric_monitor.apc_to_gui())
	_apc_anomaly = []
	for key, value in apc_anomaly_json.items():
		_record = []
		_record.append(time.strftime('%Y-%m-%d %H:%M:%S',
			time.localtime(float(key)/256)))
		_record.append(value[1])
		_record.append(value[2])
		_record.append(value[3])
		# For other processing
		_record.append(float(key)/256)
		_apc_anomaly.append(_record)
	_apc_anomaly =  sorted(_apc_anomaly, key=lambda x: (x[1]))
	return _apc_anomaly

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
	_af_anomaly =  sorted(_af_anomaly, key=lambda x: (x[1]))
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
	_vt_anomaly =  sorted(_vt_anomaly, key=lambda x: (x[1]))
	return _vt_anomaly

def get_initial_data(user_info):
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

	return details

@app.route("/")
def home():
	name = biometric_monitor.username
	recordID = biometric_monitor.recordID
	user_info = json.loads(biometric_monitor.userinfo)
	user_info = user_info['objects'][0]

	details = get_initial_data(user_info)
	
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
		safe=safe, details=details)

@app.route("/anomaly")
def anomaly():
	_af_anomaly = get_AF_anomaly()[::-1]
	_vt_anomaly = get_VT_anomaly()[::-1]
	_apc_anomaly = get_APC_anomaly()[::-1]

	x_af = []
	y_af_nec = []
	y_af_dr = []
	for d in _af_anomaly:
		y_af_nec.append(d[3])
		y_af_dr.append(d[4])
		x_af.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d[6])))

	x_vt = []
	y_vt = []
	for d in _vt_anomaly:
		y_vt.append(d[3])
		x_vt.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d[4])))

	x_apc = []
	y_apc = []
	for d in _apc_anomaly:
		y_apc.append(d[3])
		x_apc.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d[4])))

	graphs = [
	    dict(
            data=[
                dict(
                    x=x_af,  # Can use the pandas data structures directly
                    y=y_af_nec,
                    type='scatter',
                    name="Num of NEC"
				),
				dict(
                    x=x_af,  # Can use the pandas data structures directly
                    y=y_af_dr,
                    type='scatter',
                    name="Data Reliability"
				)
            ]
		),
		dict(
            data=[
                dict(
                    x=x_vt,  # Can use the pandas data structures directly
                    y=y_vt
				),
            ]
		),
		dict(
            data=[
                dict(
                    x=x_apc,  # Can use the pandas data structures directly
                    y=y_apc
				),
            ]
		)
	]


	# Add "ids" to each of the graphs to pass up to the client
	# for templating
	ids = ['graph-{}'.format(i) for i, _ in enumerate(graphs)]

	# Convert the figures to JSON
	# PlotlyJSONEncoder appropriately converts pandas, datetime, etc
	# objects to their JSON equivalents
	graphJSON = json.dumps(graphs, cls=plotly.utils.PlotlyJSONEncoder)


	return render_template('anomaly.html', AF=_af_anomaly[:15],
		VT=_vt_anomaly[:15], APC=_apc_anomaly[:15], ids=ids,
		graphJSON=graphJSON)

@app.route("/raw_data")
def raw_data():
	_af_anomaly = []
	_vt_anomaly = []
	rng = pd.date_range('1/1/2011', periods=10, freq='H')
	ts = pd.Series(np.random.randn(len(rng)), index=rng)

	graphs = [
	    dict(
            data=[
                dict(
                    x=ts.index,  # Can use the pandas data structures directly
                    y=ts,
                    type='line'
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
		ids=ids, graphJSON=graphJSON)


if __name__ == "__main__":
    app.run(threaded=True, debug=DEBUG)