# Usage: python3 logger.py c3/device/name
# This will create a file called c3-device-name.log,
# read data in loop, and save them as JSON.
# If the file exists it will overwrite it.

import sys
import PyTango

import server

dev = sys.argv[1]
fname = dev.replace('/', '-') + '.log'

with open(fname, 'w') as f:
    try:
        while True:
            data = server.get_json_data(dev)
            f.write(data + '\n')
    except KeyboardInterrupt:
        print('Logging terminated.')
