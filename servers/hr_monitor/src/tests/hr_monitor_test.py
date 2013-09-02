from __future__ import print_function

import os
from collections import namedtuple

from PyTango import DeviceProxy

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
    for index, row in data.iterrows():
        if i % percentiles == 0:
            print("%s%%" % (i / percentiles))
        timestamp = float(index.to_datetime().strftime('%s.%f'))
        datapoint = DP(timestamp=timestamp, hr=row['hr'],
                       acc_x=row['acc_x'], acc_y=row['acc_y'],
                       acc_z=row['acc_z'])
        print(datapoint)
        proxy.register_datapoint(datapoint)
        i += 1
