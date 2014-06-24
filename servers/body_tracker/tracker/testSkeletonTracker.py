from ctypes import *

open("SkeletonTracker.so", "rb")
lib = CDLL("SkeletonTracker.so")

skeletonTracker = lib.SkeletonTracker_new()
lib.getUserSkeletonHeadX.restype = c_float
lib.getUserSkeletonHeadY.restype = c_float
lib.getUserSkeletonHeadZ.restype = c_float

lib.getUserSkeletonNeckX.restype = c_float
lib.getUserSkeletonNeckY.restype = c_float
lib.getUserSkeletonNeckZ.restype = c_float

lib.getUserSkeletonL_ShX.restype = c_float
lib.getUserSkeletonL_ShY.restype = c_float
lib.getUserSkeletonL_ShZ.restype = c_float

lib.getUserSkeletonR_ShX.restype = c_float
lib.getUserSkeletonR_ShY.restype = c_float
lib.getUserSkeletonR_ShZ.restype = c_float

lib.getUserSkeletonL_ElbowX.restype = c_float
lib.getUserSkeletonL_ElbowY.restype = c_float
lib.getUserSkeletonL_ElbowZ.restype = c_float

lib.getUserSkeletonR_ElbowX.restype = c_float
lib.getUserSkeletonR_ElbowY.restype = c_float
lib.getUserSkeletonR_ElbowZ.restype = c_float

lib.getUserSkeletonL_HandX.restype = c_float
lib.getUserSkeletonL_HandY.restype = c_float
lib.getUserSkeletonL_HandZ.restype = c_float

lib.getUserSkeletonR_HandX.restype = c_float
lib.getUserSkeletonR_HandY.restype = c_float
lib.getUserSkeletonR_HandZ.restype = c_float

lib.getUserSkeletonTorsoX.restype = c_float
lib.getUserSkeletonTorsoY.restype = c_float
lib.getUserSkeletonTorsoZ.restype = c_float

lib.getUserSkeletonL_HipX.restype = c_float
lib.getUserSkeletonL_HipY.restype = c_float
lib.getUserSkeletonL_HipZ.restype = c_float

lib.getUserSkeletonR_HipX.restype = c_float
lib.getUserSkeletonR_HipY.restype = c_float
lib.getUserSkeletonR_HipZ.restype = c_float

lib.getUserSkeletonL_KneeX.restype = c_float
lib.getUserSkeletonL_KneeY.restype = c_float
lib.getUserSkeletonL_KneeZ.restype = c_float

lib.getUserSkeletonR_KneeX.restype = c_float
lib.getUserSkeletonR_KneeY.restype = c_float
lib.getUserSkeletonR_KneeZ.restype = c_float

lib.getUserSkeletonL_FootX.restype = c_float
lib.getUserSkeletonL_FootY.restype = c_float
lib.getUserSkeletonL_FootZ.restype = c_float

lib.getUserSkeletonR_FootX.restype = c_float
lib.getUserSkeletonR_FootY.restype = c_float
lib.getUserSkeletonR_FootZ.restype = c_float

while(True):
    lib.loop(skeletonTracker)
    if lib.getUsersCount(skeletonTracker) > 0:
        headX = lib.getUserSkeletonHeadX(skeletonTrack, 0)
        headY = lib.getUserSkeletonHeadY(skeletonTrack, 0)
        headZ = lib.getUserSkeletonHeadZ(skeletonTrack, 0)
        print str(headX) + " " + str(headY) + " " + str(headZ)
