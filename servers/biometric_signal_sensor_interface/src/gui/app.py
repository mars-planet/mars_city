from __future__ import absolute_import, division, print_function
from flask import Flask, flash, redirect, render_template, url_for
import PyTango
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

@app.route("/")
def home():
	username = biometric_monitor.username
	recordID = biometric_monitor.recordID
	return render_template('home.html', username=username, recordID=recordID)


if __name__ == "__main__":
    app.run(threaded=True, debug=DEBUG)