from __future__ import print_function

import os
import sys
from collections import namedtuple
from datetime import datetime
from time import sleep

from PyTango import DeviceProxy

sys.path.append("../../")


from src.preprocessing import read_data, extract_hr_acc

if __name__ == '__main__':

    proxy = DeviceProxy("C3/hr_monitor/1")

    dirname = os.path.dirname(__file__)

    filename = os.path.join(dirname, 'dataset.dat')
    print(filename)
    data = extract_hr_acc(read_data(filename))

    DP = namedtuple("DP", ["timestamp", "hr", "acc_x", "acc_y", "acc_z"])
    i = 0
    percentiles = int(len(data) / 1000)
    prow, ptimestamp = None, None
    for index, row in data.iterrows():
        timestamp = float(index.to_datetime().strftime('%s.%f'))
        if i % percentiles == 0:
            print("%s%%" % (i / percentiles))
        if prow is not None:
            datapoint = DP(timestamp=ptimestamp, hr=prow['hr'],
                           acc_x=prow['acc_x'], acc_y=prow['acc_y'],
                           acc_z=prow['acc_z'])
            print("%s, %s" % (index.to_datetime(), datapoint.hr))
            proxy.register_datapoint(datapoint)
            sleep(timestamp - ptimestamp)
        prow, ptimestamp = row, timestamp
        i += 1
