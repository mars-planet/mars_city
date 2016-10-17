from __future__ import print_function

import sys
import json
import numpy
import PyTango


class NumpyAwareJSONEncoder(json.JSONEncoder):

    def default(self, obj):
        if isinstance(obj, numpy.ndarray) and obj.ndim == 1:
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def get_data(device):
    dev = PyTango.DeviceProxy(device)
    return dict((attr, dev[attr].value) for attr in dev.get_attribute_list())


def get_json_data(device):
    return json.dumps(get_data(device), cls=NumpyAwareJSONEncoder)

if __name__ == '__main__':
    dev = sys.argv[1]

    print('data:', get_data(dev))
    print('json:', get_json_data(dev))
