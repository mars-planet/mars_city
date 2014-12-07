from __future__ import print_function

import sys
import json
import PyTango

def get_data(device):
    dev = PyTango.DeviceProxy(device)
    return dict((attr, dev[attr].value) for attr in dev.get_attribute_list())

def get_json_data(device):
    return json.dumps(get_data(device))

if __name__ == '__main__':
    dev = sys.argv[1]

    print('data:', get_data(dev))
    print('json:', get_json_data(dev))
