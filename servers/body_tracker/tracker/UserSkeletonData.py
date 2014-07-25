from stracker import SkeletonTracker, lib
from collections import namedtuple


# UserSkeletonData contains information about the joints of user
class UserSkeletonData(object):
    st = SkeletonTracker()

    def __init__(self, userid = 0):
        self._userid = userid
        self._Coords = namedtuple('coords', 'x y z')

    @property
    def head(self):
        '''returns namedtuple containing x, y, z coordinates of head'''
        headX = lib.getUserSkeletonHeadX(self.st.addr, self._userid)
        headY = lib.getUserSkeletonHeadY(self.st.addr, self._userid)
        headZ = lib.getUserSkeletonHeadZ(self.st.addr, self._userid)
        return self._Coords(headX, headY, headZ)

    @property
    def neck(self):
        '''returns namedtuple containing x, y, z coordinates of neck'''
        neckX = lib.getUserSkeletonNeckX(self.st.addr, self._userid)
        neckY = lib.getUserSkeletonNeckY(self.st.addr, self._userid)
        neckZ = lib.getUserSkeletonNeckZ(self.st.addr, self._userid)
        return self._Coords(neckX, neckY, neckZ)

    @property
    def left_shoulder(self):
        '''returns namedtuple containing x, y, z coordinates of
        left shoulder'''
        left_shoulderX = lib.getUserSkeletonL_ShX(self.st.addr, self._userid)
        left_shoulderY = lib.getUserSkeletonL_ShY(self.st.addr, self._userid)
        left_shoulderZ = lib.getUserSkeletonL_ShZ(self.st.addr, self._userid)
        return self._Coords(left_shoulderX, left_shoulderY, left_shoulderZ)

    @property
    def right_shoulder(self):
        '''returns namedtuple containing x, y, z coordinates of
        right shoulder'''
        right_shoulderX = lib.getUserSkeletonR_ShX(self.st.addr, self._userid)
        right_shoulderY = lib.getUserSkeletonR_ShY(self.st.addr, self._userid)
        right_shoulderZ = lib.getUserSkeletonR_ShZ(self.st.addr, self._userid)
        return self._Coords(right_shoulderX, right_shoulderY, right_shoulderZ)

    @property
    def left_elbow(self):
        '''returns namedtuple containing x, y, z coordinates of left elbow'''
        left_elbowX = lib.getUserSkeletonL_ElbowX(self.st.addr, self._userid)
        left_elbowY = lib.getUserSkeletonL_ElbowY(self.st.addr, self._userid)
        left_elbowZ = lib.getUserSkeletonL_ElbowZ(self.st.addr, self._userid)
        return self._Coords(left_elbowX, left_elbowY, left_elbowZ)

    @property
    def right_elbow(self):
        '''returns namedtuple containing x, y, z coordinates of right elbow'''
        right_elbowX = lib.getUserSkeletonR_ElbowX(self.st.addr, self._userid)
        right_elbowY = lib.getUserSkeletonR_ElbowY(self.st.addr, self._userid)
        right_elbowZ = lib.getUserSkeletonR_ElbowZ(self.st.addr, self._userid)
        return self._Coords(right_elbowX, right_elbowY, right_elbowZ)

    @property
    def left_hand(self):
        '''returns namedtuple containing x, y, z coordinates of left hand'''
        left_handX = lib.getUserSkeletonL_HandX(self.st.addr, self._userid)
        left_handY = lib.getUserSkeletonL_HandY(self.st.addr, self._userid)
        left_handZ = lib.getUserSkeletonL_HandZ(self.st.addr, self._userid)
        return self._Coords(left_handX, left_handY, left_handZ)

    @property
    def right_hand(self):
        '''returns namedtuple containing x, y, z coordinates of right hand'''
        right_handX = lib.getUserSkeletonR_HandX(self.st.addr, self._userid)
        right_handY = lib.getUserSkeletonR_HandY(self.st.addr, self._userid)
        right_handZ = lib.getUserSkeletonR_HandZ(self.st.addr, self._userid)
        return self._Coords(right_handX, right_handY, right_handZ)

    @property
    def torso(self):
        '''returns namedtuple containing x, y, z coordinates of torso'''
        torsoX = lib.getUserSkeletonTorsoX(self.st.addr, self._userid)
        torsoY = lib.getUserSkeletonTorsoY(self.st.addr, self._userid)
        torsoZ = lib.getUserSkeletonTorsoZ(self.st.addr, self._userid)
        return self._Coords(torsoX, torsoY, torsoZ)

    @property
    def left_hip(self):
        '''returns namedtuple containing x, y, z coordinates of left hip'''
        left_hipX = lib.getUserSkeletonL_HipX(self.st.addr, self._userid)
        left_hipY = lib.getUserSkeletonL_HipY(self.st.addr, self._userid)
        left_hipZ = lib.getUserSkeletonL_HipZ(self.st.addr, self._userid)
        return self._Coords(left_hipX, left_hipY, left_hipZ)

    @property
    def right_hip(self):
        '''returns namedtuple containing x, y, z coordinates of right hip'''
        right_hipX = lib.getUserSkeletonR_HipX(self.st.addr, self._userid)
        right_hipY = lib.getUserSkeletonR_HipY(self.st.addr, self._userid)
        right_hipZ = lib.getUserSkeletonR_HipZ(self.st.addr, self._userid)
        return self._Coords(right_hipX, right_hipY, right_hipZ)

    @property
    def left_knee(self):
        '''returns namedtuple containing x, y, z coordinates of left knee'''
        left_kneeX = lib.getUserSkeletonL_KneeX(self.st.addr, self._userid)
        left_kneeY = lib.getUserSkeletonL_KneeY(self.st.addr, self._userid)
        left_kneeZ = lib.getUserSkeletonL_KneeZ(self.st.addr, self._userid)
        return self._Coords(left_kneeX, left_kneeY, left_kneeZ)

    @property
    def right_knee(self):
        '''returns namedtuple containing x, y, z coordinates of right knee'''
        right_kneeX = lib.getUserSkeletonR_KneeX(self.st.addr, self._userid)
        right_kneeY = lib.getUserSkeletonR_KneeY(self.st.addr, self._userid)
        right_kneeZ = lib.getUserSkeletonR_KneeZ(self.st.addr, self._userid)
        return self._Coords(right_kneeX, right_kneeY, right_kneeZ)

    @property
    def left_foot(self):
        '''returns namedtuple containg x, y, z coordinates of left foot'''
        left_footX = lib.getUserSkeletonL_FootX(self.st.addr, self._userid)
        left_footY = lib.getUserSkeletonL_FootY(self.st.addr, self._userid)
        left_footZ = lib.getUserSkeletonL_FootZ(self.st.addr, self._userid)
        return self._Coords(left_footX, left_footY, left_footZ)

    @property
    def right_foot(self):
        '''returns namedtuple containing x, y, z coordinates of right foot'''
        right_footX = lib.getUserSkeletonR_FootX(self.st.addr, self._userid)
        right_footY = lib.getUserSkeletonR_FootY(self.st.addr, self._userid)
        right_footZ = lib.getUserSkeletonR_FootZ(self.st.addr, self._userid)
        return self._Coords(right_footX, right_footY, right_footZ)

    def get_jointInfo(self):
        '''returns all skeleton joints information.
           +------------------------------------------------------------------+
           | Index--> Joint      | Index--> Joint     | Index--> Joint        |
           -------------------------------------------------------------------+
           | 0    --> head       | 1    --> neck      | 2    -->left shoulder |
           +------------------------------------------------------------------+
           | 3    --> left elbow | 4    --> left hand | 5    -->right shoulder|
           +------------------------------------------------------------------+
           | 6    --> right elbow| 7    --> right hand| 8    -->torso         |
           +------------------------------------------------------------------+
           | 9    --> left hip   | 10   --> left knee | 11   -->left foot     |
           +------------------------------------------------------------------+
           | 12   --> right hip  | 13   --> right knee| 14   -->right foot    |
           +------------------------------------------------------------------+
        '''
        return (self.head, self.neck, self.left_shoulder, self.left_elbow,
                self.left_hand, self.right_shoulder, self.right_elbow,
                self.right_hand, self.torso, self.left_hip, self.left_knee,
                self.left_foot, self.right_hip, self.right_knee,
                self.right_foot)
