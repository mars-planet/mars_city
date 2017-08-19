from __future__ import absolute_import, division, print_function
from flask import Flask, render_template
import ConfigParser
import datetime
import requests
import PyTango
import json
import time
import os
import plotly

'''
Python Flask GUI Dashboard for the Biometric Signal Sensor's system
'''
__author__ = 'abhijith'

app = Flask(__name__)
app.secret_key = os.urandom(24)
app.config.from_object(__name__)
DEBUG = True

requests.packages.urllib3.disable_warnings()

# Config file
config = ConfigParser.ConfigParser()
config.read("../health_monitor/config.cfg")


def config_helper(section):
    '''
    Helper function to read the config file passed
        @param section :    Section name of the desired value in the config
                            file
        @return :           Dictionary containing the values from the config
                            file
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


# Biometriv Tango Device Client
biometric_monitor = PyTango.DeviceProxy(
    config_helper("BiometricMonitor")['name'])


def get_AF_anomaly():
    '''
    Calls the Tango device server command to access the Database for the
    APC-PVC anomalies
        @return :           A list containing rows from the database
    '''
    af_anomaly_json = json.loads(biometric_monitor.af_to_gui())
    _af_anomaly = []
    for key, value in af_anomaly_json.items():
        _record = []
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(key) / 256)))
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(value[0]) / 256)))
        _record.append(value[1])
        _record.append(value[2])
        _record.append(value[3])
        _record.append(value[4])
        # For other processing
        _record.append(float(key) / 256)
        _af_anomaly.append(_record)
    _af_anomaly = sorted(_af_anomaly, key=lambda x: (x[0]))
    return _af_anomaly


def get_VT_anomaly():
    '''
    Calls the Tango device server command to access the Database for the
    APC-PVC anomalies
        @return :           A list containing rows from the database
    '''
    vt_anomaly_json = json.loads(biometric_monitor.vt_to_gui())
    _vt_anomaly = []
    for key, value in vt_anomaly_json.items():
        _record = []
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(key) / 256)))
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(value[0]) / 256)))
        _record.append(value[1])
        _record.append(value[2])
        # For other processing
        _record.append(float(key) / 256)
        _vt_anomaly.append(_record)
    _vt_anomaly = sorted(_vt_anomaly, key=lambda x: (x[0]))
    return _vt_anomaly


def get_APC_anomaly():
    '''
    Calls the Tango device server command to access the Database for the
    APC-PVC anomalies
        @return :           A list containing rows from the database
    '''
    apc_anomaly_json = json.loads(biometric_monitor.apc_to_gui())
    _apc_anomaly = []
    for key, value in apc_anomaly_json.items():
        _record = []
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(key) / 256)))
        _record.append(value[0])
        _record.append(value[1])
        _record.append(value[2])
        # For other processing
        _record.append(float(key) / 256)
        _apc_anomaly.append(_record)
    _apc_anomaly = sorted(_apc_anomaly, key=lambda x: (x[0]))
    return _apc_anomaly


def get_Resp_anomaly():
    '''
    Calls the Tango device server command to access the Database for the
    Respiration anomalies
        @return :           A list containing rows from the database
    '''
    resp_anomaly_json = json.loads(biometric_monitor.resp_to_gui())
    _resp_anomaly = []
    for key, value in resp_anomaly_json.items():
        _record = []
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(key) / 256)))
        _record.append(value[0])
        _record.append(value[1])
        _record.append(value[2])
        # For other processing
        _record.append(float(key) / 256)
        _resp_anomaly.append(_record)
    _resp_anomaly = sorted(_resp_anomaly, key=lambda x: (x[0]))
    return _resp_anomaly

def get_Sleep_anomaly():
    '''
    Calls the Tango device server command to access the Database for the
    Slee[] anomalies
        @return :           A list containing rows from the database
    '''
    sleep_anomaly_json = json.loads(biometric_monitor.sleep_to_gui())
    _sleep_anomaly = []
    for key, value in sleep_anomaly_json.items():
        _record = []
        _record.append(time.strftime('%Y-%m-%d %H:%M:%S',
                                     time.localtime(float(key) / 256)))
        _record.append(value[0])
        # For other processing
        _record.append(float(key) / 256)
        _sleep_anomaly.append(_record)
    _sleep_anomaly = sorted(_sleep_anomaly, key=lambda x: (x[0]))
    return _sleep_anomaly

def get_initial_data(user_info):
    '''
    Helper function to cleanly retrieve the user_info JSON object as a
    dictionary
        @param user_info :  JSON object of the user info
        @return :           Dictionary containing the necessary items from
                            the JSON
    '''
    details = {}
    # ------ User details
    details['email'] = user_info['email']
    details['id'] = user_info['id']
    details['dob'] = user_info['profile']['date_of_birth']
    details['gender'] = user_info['profile']['gender']
    details['username'] = user_info['username']
    details['hid'] = user_info['profile']['preferences']['deviceId']
    # ------- User health details
    details['hr_rest_val'] = user_info['profile']['fitness'][0]['value']
    temp_ = user_info['profile']['fitness'][0]['zone_description']
    details['hr_rest_desc'] = temp_
    details['bmi_val'] = user_info['profile']['fitness'][1]['value']
    details['height'] = user_info['profile']['fitness'][2]['value']
    details['weight'] = user_info['profile']['fitness'][3]['value']
    temp_ = user_info['profile']['fitness'][3]['zone_description']
    details['weight_desc'] = temp_
    details['hr_max_val'] = user_info['profile']['fitness'][4]['value']
    temp_ = user_info['profile']['fitness'][4]['zone_description']
    details['hr_max_desc'] = temp_
    details['sleep_val'] = user_info['profile']['fitness'][5]['value']
    temp_ = user_info['profile']['fitness'][5]['zone_description']
    details['sleep_desc'] = temp_
    details['hr_rec_val'] = user_info['profile']['fitness'][7]['value']
    temp_ = user_info['profile']['fitness'][7]['zone_description']
    details['hr_rec_desc'] = temp_
    details['vo2_max_val'] = user_info['profile']['fitness'][8]['value']
    temp_ = user_info['profile']['fitness'][8]['zone_description']
    details['vo2_max_desc'] = temp_
    details['resp_val'] = user_info['profile']['fitness'][9]['value']

    return details


@app.route("/")
def home():
    '''
    Route function called for localhost:5000/
    '''
    name = biometric_monitor.username
    recordID = biometric_monitor.recordID
    user_info = json.loads(biometric_monitor.userinfo)
    user_info = user_info['objects'][0]

    details = get_initial_data(user_info)

    _af_anomaly = get_AF_anomaly()[::-1]
    today = datetime.datetime.today()
    week_ago = today - datetime.timedelta(days=7)
    today = ((today - datetime.datetime(1970, 1, 1)).total_seconds())
    week_ago = ((week_ago - datetime.datetime(1970, 1, 1)).total_seconds())

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
    '''
    Route function called for localhost:5000/anomaly
    '''
    _af_anomaly = get_AF_anomaly()[::-1]
    _vt_anomaly = get_VT_anomaly()[::-1]
    _apc_anomaly = get_APC_anomaly()[::-1]
    _resp_anomaly = get_Resp_anomaly()[::-1]
    _sleep_anomaly = get_Sleep_anomaly()[::-1]

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

    x_resp = []
    y_resp = []
    for d in _resp_anomaly:
        y_resp.append(d[1])
        x_resp.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d[4])))

    x_sleep = []
    y_sleep = []
    for d in _sleep_anomaly:
        y_sleep.append(d[1])
        x_sleep.append(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(d[2])))

    # For the Plot.ly/javascript graphs
    graphs = [
        dict(
            data=[
                dict(
                    x=x_af,
                    y=y_af_nec,
                    type='scatter',
                    name="Num of NEC"
                ),
                dict(
                    x=x_af,
                    y=y_af_dr,
                    type='scatter',
                    name="Data Reliability"
                )
            ]
        ),
        dict(
            data=[
                dict(
                    x=x_vt,
                    y=y_vt
                ),
            ]
        ),
        dict(
            data=[
                dict(
                    x=x_apc,
                    y=y_apc
                ),
            ]
        ),
        dict(
            data=[
                dict(
                    x=x_resp,
                    y=y_resp
                ),
            ]
        ),
        dict(
            data=[
                dict(
                    x=x_sleep,
                    y=y_sleep
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
                           VT=_vt_anomaly[:15], APC=_apc_anomaly[:15],
                           RESP=_resp_anomaly, SLEEP=_sleep_anomaly,
                           ids=ids, graphJSON=graphJSON)


@app.route("/raw_data")
def raw_data():
    '''
    Route function called for localhost:5000/raw_data
    '''
    # biometric_monitor.delete_from_db()
    return render_template('raw_data.html')


@app.route('/raw_data/fetch', methods=['GET'])
def fetch_realtime_data():
    '''
    Helper function called from the GUI for real-time update of the
    plots.
        @return :   All the real-time data
    '''
    to_gui = {}
    _af_anomaly = get_AF_anomaly()[::-1]
    _vt_anomaly = get_VT_anomaly()[::-1]
    _apc_anomaly = get_APC_anomaly()[::-1]
    _resp_anomaly = get_Resp_anomaly()[::-1]
    rt_data = biometric_monitor.rt_to_gui()

    to_gui['1'] = rt_data
    try:
        to_gui['2'] = time.strftime(
            '%Y-%m-%d %H:%M:%S', time.localtime(_af_anomaly[0][6]))
    except:
        to_gui['2'] = 0

    try:
        to_gui['3'] = time.strftime(
            '%Y-%m-%d %H:%M:%S', time.localtime(_vt_anomaly[0][4]))
    except:
        to_gui['3'] = 0

    try:
        to_gui['4'] = time.strftime(
            '%Y-%m-%d %H:%M:%S', time.localtime(_apc_anomaly[0][4]))
    except:
        to_gui['4'] = 0

    try:
        to_gui['5'] = time.strftime(
            '%Y-%m-%d %H:%M:%S', time.localtime(_resp_anomaly[0][4]))
    except:
        to_gui['5'] = 0

    biometric_monitor.delete_from_db()

    return json.dumps(to_gui)


if __name__ == "__main__":
    '''
    Main Function
    '''
    biometric_monitor = PyTango.DeviceProxy(
        config_helper("BiometricMonitor")['name'])

    biometric_monitor.poll_command("rt_to_gui", 50000)
    biometric_monitor.poll_command("af_to_gui", 50000)
    biometric_monitor.poll_command("vt_to_gui", 50000)
    biometric_monitor.poll_command("apc_to_gui", 50000)
    biometric_monitor.poll_command("resp_to_gui", 50000)

    app.run(threaded=True, debug=DEBUG)
