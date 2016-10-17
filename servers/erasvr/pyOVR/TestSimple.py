import time
from pyOVR import *

ovr_Initialize()
hmd = ovrHmd_Create(0)
hmdDesc = ovrHmdDesc()
ovrHmd_GetDesc(hmd, byref(hmdDesc))
print(hmdDesc.ProductName)
ovrHmd_StartSensor( hmd, ovrSensorCap_Orientation | ovrSensorCap_YawCorrection, 0)

while True:
  ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())
  pose = ss.Predicted.Pose
  print("{:+.2f} {:+.2f} {:+.2f} {:+.2f}".format(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z))
  time.sleep(0.016)
