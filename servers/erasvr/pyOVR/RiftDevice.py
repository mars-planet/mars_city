from ctypes import byref
from pyOVR import ovr_Initialize, ovrHmd_Create, ovrHmdDesc, ovrHmd_GetDesc, ovrHmd_StartSensor, ovrSensorCap_Orientation, ovrHmd_GetSensorState, ovrSensorCap_YawCorrection, ovr_GetTimeInSeconds, ovrHmd_Destroy, ovr_Shutdown

class RiftDevice(object):
    def __init__(self):
        ovr_Initialize()
        self.hmd = ovrHmd_Create(0)
        self.hmdDesc = ovrHmdDesc()
        ovrHmd_GetDesc(self.hmd, byref(self.hmdDesc))
        ovrHmd_StartSensor( self.hmd, ovrSensorCap_Orientation | ovrSensorCap_YawCorrection, 0)
        
    def __del__(self):
        ovrHmd_Destroy(self.hmd)
        ovr_Shutdown()
        print("LibOVR shutdown complete.")

    def get_orientation(self):
        self.ss = ovrHmd_GetSensorState(self.hmd, ovr_GetTimeInSeconds())
        pose = self.ss.Predicted.Pose
        return pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z
