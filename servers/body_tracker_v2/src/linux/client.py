# Borrowed from Shridhar for boilerplate code. Need to edit for multi-threading.

import json
import time
import logging

import PyTango


logger = logging.getLogger()
logging.basicConfig(
        format='%(asctime)s - %(name)s - %(levelname)s - %(module)s - '
        '%(pathname)s : %(lineno)d - %(message)s',
        level=logging.INFO)

device_name = 'C3/body_tracker_v2/eras1'

td = PyTango.DeviceProxy(device_name)
attr_name = 'distance'
global sample

def write_json(filename,sample):
    with open(filename+".json", 'w') as fp:
        json.dump(sample, fp)

# Set up a listener
def printer(event_data):
    try:
        global sample
        sample = event_data
    except Exception:
        logger.exception('Exception while handling event, event_data: {}'
                         .format(event_data))
poll_event_id = td.subscribe_event(
    attr_name, PyTango.EventType.CHANGE_EVENT, printer)

write_json("dump",sample)

# This is how you would unsubscribe from the events.
#td.unsubscribe_event(poll_event_id)
