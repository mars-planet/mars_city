import time
#from RiftDevice import *
from pyOVR import RiftDevice

rift = RiftDevice()

while True:
    print(rift.get_orientation())
    time.sleep(0.016) # sleep for a 60 fps refreshcycle
