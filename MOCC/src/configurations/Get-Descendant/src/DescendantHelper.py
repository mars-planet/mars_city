#!/usr/bin/env python

from __future__ import division, print_function
import json

META_CHARACTER_SEPARATOR = ":"

def getDescendants(device_id, dev_names):
	devices = []
	for dev in dev_names:
		if dev == device_id:
			pass
		else:
			devices.append(dev)

	descendants = []
	for dev in devices:
		if device_id in dev:
			descendants.append(dev)

	return	descendants