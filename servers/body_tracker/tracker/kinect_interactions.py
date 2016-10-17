import ctypes
from pykinect import nui

_KINECT_GEST_REC_DLL = ctypes.WinDLL("KinectGestureRecognizer.dll")

HAND_STATUS_OPEN = 0
HAND_STATUS_CLOSED = 1


class KinectGestureRecognizer:
    _kgr = None

    def __init__(self, kinect):
        _KINECT_GEST_REC_DLL.new_KinectGestureRecognizer.argtypes = [ctypes.POINTER(nui._NuiInstance),
                                                                     ctypes.c_voidp,
                                                                     ctypes.c_voidp,
                                                                     ctypes.c_voidp]
        _KINECT_GEST_REC_DLL.new_KinectGestureRecognizer.restype = ctypes.c_void_p

        _KINECT_GEST_REC_DLL.c_ProcessDepth.argtypes = [ctypes.c_void_p,
                                                        ctypes.POINTER(nui.ImageFrame)]
        _KINECT_GEST_REC_DLL.c_ProcessDepth.restype = None

        _KINECT_GEST_REC_DLL.c_ProcessSkeleton.argtypes = [ctypes.c_void_p,
                                                           ctypes.POINTER(nui.SkeletonFrame)]
        _KINECT_GEST_REC_DLL.c_ProcessSkeleton.restype = None

        _KINECT_GEST_REC_DLL.c_startRecognition.argtypes = [ctypes.c_void_p]
        _KINECT_GEST_REC_DLL.c_startRecognition.restype = None

        _KINECT_GEST_REC_DLL.c_getRightHandStatus.argtypes = [ctypes.c_void_p]
        _KINECT_GEST_REC_DLL.c_getRightHandStatus.restype = ctypes.c_int

        _KINECT_GEST_REC_DLL.c_getLeftHandStatus.argtypes = [ctypes.c_void_p]
        _KINECT_GEST_REC_DLL.c_getLeftHandStatus.restype = ctypes.c_int

        self._kgr = _KINECT_GEST_REC_DLL.new_KinectGestureRecognizer(ctypes.byref(kinect._nui),
                                                                     kinect.depth_stream._stream,
                                                                     kinect._depth_event,
                                                                     kinect._skeleton_event)

    def process_depth(self, depth_frame):
        _KINECT_GEST_REC_DLL.c_ProcessDepth(self._kgr, ctypes.byref(depth_frame))

    def process_skeleton(self, skeleton_frame):
        _KINECT_GEST_REC_DLL.c_ProcessSkeleton(self._kgr, ctypes.byref(skeleton_frame))

    def start_recognition(self):
        _KINECT_GEST_REC_DLL.c_startRecognition(self._kgr)

    def get_right_hand_status(self):
        return _KINECT_GEST_REC_DLL.c_getRightHandStatus(self._kgr)

    def get_left_hand_status(self):
        return _KINECT_GEST_REC_DLL.c_getLeftHandStatus(self._kgr)
