
from ctypes import byref
from .pyOVR import ovr_Initialize, ovrHmd_Create, ovrHmdDesc, ovrHmd_GetDesc, ovrHmd_StartSensor, ovrSensorCap_Orientation, ovrHmd_GetSensorState, ovrSensorCap_YawCorrection, ovr_GetTimeInSeconds, ovrHmd_Destroy, ovr_Shutdown, ovrFovPort, ovrEyeRenderDesc, ovrHmd_GetRenderDesc, ovrMatrix4f, ovrMatrix4f_Projection

class RiftDevice(object):
    def __init__(self):
        # init oculus sdk
        ovr_Initialize()
        self.hmd = ovrHmd_Create(0)
        self.hmdDesc = ovrHmdDesc()
        ovrHmd_GetDesc(self.hmd, byref(self.hmdDesc))
        ovrHmd_StartSensor( self.hmd, ovrSensorCap_Orientation | ovrSensorCap_YawCorrection, 0)
        
        # init ovrFovPort struct for left/right eye
        self.eyeFov = [ovrFovPort(), ovrFovPort()]
        self.eyeFov[0] = self.hmdDesc.DefaultEyeFov[0]
        self.eyeFov[1] = self.hmdDesc.DefaultEyeFov[1]
        
        # init eyeRenderDesc struct for left/right eye
        self.eyeRenderDesc = [ovrEyeRenderDesc(), ovrEyeRenderDesc()]
        self.eyeRenderDesc[0] = ovrHmd_GetRenderDesc(self.hmd, pyOVR.ovrEye_Left, self.eyeFov[0])
        self.eyeRenderDesc[1] = ovrHmd_GetRenderDesc(self.hmd, pyOVR.ovrEye_Right, self.eyeFov[1])
        
        # setup projection matrices
        self.proj = [ovrMatrix4f(), ovrMatrix4f()]
        self.proj[0] = ovrMatrix4f_Projection(self.eyeRenderDesc[0].Fov, 0.01, 10000.0, True)
        self.proj[1] = ovrMatrix4f_Projection(self.eyeRenderDesc[1].Fov, 0.01, 10000.0, True)
        
    def __del__(self):
        ovrHmd_Destroy(self.hmd)
        ovr_Shutdown()
        print("LibOVR shutdown complete.")
    
    def get_projection_matrix(self, index):
        # create bge compatible proj matrix
        table = [ [ 0 for i in range(4) ] for j in range(4) ]
        for i in range(4):
            for j in range(4):
                table[i][j]= self.proj[index].M[i][j]
        print(table)
        return table

    def get_orientation(self):
        self.ss = ovrHmd_GetSensorState(self.hmd, ovr_GetTimeInSeconds())
        pose = self.ss.Predicted.Pose
        return pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z

    def say_hello(self):
        print("Rift says hi!")

