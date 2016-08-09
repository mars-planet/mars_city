import json
import time
import logging

import PyTango


logger = logging.getLogger()
logging.basicConfig(
        format='%(asctime)s - %(name)s - %(levelname)s - %(module)s - '
        '%(pathname)s : %(lineno)d - %(message)s',
        level=logging.INFO)

device_name = 'C3/unity/eras1'

td = PyTango.DeviceProxy(device_name)
attr_name = 'skelton'
global sample

def write_json(filename,sample):
    with open(filename+".json", 'w') as fp:
        json.dump(sample, fp)

# Set up a listener
def printer(event_data):
    try:
        #print event_data # A PyTango.EventData instance
        global sample
        sample = event_data
    except Exception:
        # Not handling exceptions seems to break the event update loop. Or I was
        # doing something else stupid, must still investigate :)
        logger.exception('Exception while handling event, event_data: {}'
                         .format(event_data))
poll_event_id = td.subscribe_event(
    attr_name, PyTango.EventType.CHANGE_EVENT, printer)

write_json("dump",sample)

# This is how you would unsubscribe from the events.
#td.unsubscribe_event(poll_event_id)



