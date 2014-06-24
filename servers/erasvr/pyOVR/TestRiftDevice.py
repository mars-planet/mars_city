import time
from RiftDevice import *

rift = RiftDevice()

while True:
    print(rift.getOrientation())
    time.sleep(0.016) # sleep for a 60 fps refreshcycle
