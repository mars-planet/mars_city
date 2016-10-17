from ctypes import *
from oculusvr import *

class RiftDevice(object):
    def __init__(self):
        # init oculus sdk
        print('Rift Init started')
        Hmd.initialize()
        self.hmd = Hmd()
        self.hmdDesc = cast(self.hmd.hmd,POINTER(ovrHmdDesc)).contents
        print(self.hmdDesc.ProductName)
        self.frame = 0
        self.hmd.configure_tracking() 

        self.eyeOffsets = [ ovrVector3f(), ovrVector3f() ]
        for eye in range(0, 2):
            self.eyeOffsets[eye] = 0.0,0.0,0.0

        print('Rift Initialized')

        
    def __del__(self):
        self.hmd.destroy()
        self.hmd = None
        Hmd.shutdown()
        print("LibOVR shutdown complete.")
    
    def get_orientation(self):
        self.frame += 1
        poses = self.hmd.get_eye_poses(self.frame, self.eyeOffsets)
        #self.hmd.begin_frame(self.frame)
        #for i in range(0, 2):
        #    eye = self.hmdDesc.EyeRenderOrder[i]
        eye = self.hmdDesc.EyeRenderOrder[0]
        rot = poses[0].Orientation
        return rot.w, rot.x, rot.y, rot.z

    def say_hello(self):
        print("Rift says hi!")

