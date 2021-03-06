# This module create a class which contains joints information in Vector form
# relative to a stable point, (stable point here)

import PyTango
#from cvisual import Vector
from mathutils import Vector



# Joints information in form of coordinate vectors
#def get_absoluteJoints():
#    '''returns all absolute skeleton joints(in world space coordinates)
#       information.
#       +------------------------------------------------------------------+
#       | Index--> Joint      | Index--> Joint     | Index--> Joint        |
#       +------------------------------------------------------------------+
#       | 0    --> head       | 1    --> neck      | 2    -->left shoulder |
#       +------------------------------------------------------------------+
#       | 3    --> left elbow | 4    --> left hand | 5    -->right shoulder|
#       +------------------------------------------------------------------+
#       | 6    --> right elbow| 7    --> right hand| 8    -->torso         |
#       +------------------------------------------------------------------+
#       | 9    --> left hip   | 10   --> left knee | 11   -->left foot     |
#       +------------------------------------------------------------------+
#       | 12   --> right hip  | 13   --> right knee| 14   -->right foot    |
#       +------------------------------------------------------------------+
#    '''
#    head = Vector(self.dev['skeleton_head'].value)
#    neck = Vector(self.dev['skeleton_neck'].value)
#    left_shoulder = Vector(self.dev['skeleton_left_shoulder'].value)
#    left_elbow = Vector(self.dev['skeleton_left_elbow'].value)
#    left_hand = Vector(self.dev['skeleton_left_hand'].value)
#    right_shoulder = Vector(self.dev['skeleton_right_shoulder'].value)
#    right_elbow = Vector(self.dev['skeleton_right_elbow'].value)
#    right_hand = Vector(self.dev['skeleton_right_hand'].value)
#    torso = Vector(self.dev['skeleton_torso'].value)
#    left_hip = Vector(self.dev['skeleton_left_hip'].value)
#    left_knee = Vector(self.dev['skeleton_left_knee'].value)
#    left_foot = Vector(self.dev['skeleton_left_foot'].value)
#    right_hip = Vector(self.dev['skeleton_right_hip'].value)
#    right_knee = Vector(self.dev['skeleton_right_knee'].value)
#    right_foot = Vector(self.dev['skeleton_right_foot'].value)
#    return (head, neck, left_shoulder, left_elbow, left_hand, right_shoulder,
#            right_elbow, right_hand, torso, left_hip, left_knee, left_foot,
#            right_hip, right_knee, right_foot)


class RelativeJointInfo:

    def __init__(self,device_name):
        # get the tracker device
        self.dev = PyTango.DeviceProxy(device_name)

    @property
    def stablepoint(self):
        #define method to get stable point
        #it should be in Vector form
        torsoX, torsoZ, torsoY = self.dev['skeleton_torso'].value
        #torsoY = -torsoY
        torsoX = -torsoX
        return Vector((torsoX, torsoY, torsoZ))

    @property
    def head(self):
        '''returns head Vector relative to stable point
        '''
        headX, headZ, headY = self.dev['skeleton_head'].value
#        headY = -headY
        headX = -headX
        head = Vector((headX, headY, headZ))
        return head - self.stablepoint

    @property
    def neck(self):
        '''returns neck Vector relative to stable point
        '''
        neckX, neckZ, neckY = self.dev['skeleton_neck'].value
#        neckY = -neckY
        neckX = -neckX
        neck = Vector((neckX, neckY, neckZ))
        return neck - self.stablepoint

    @property
    def left_shoulder(self):
        '''returns left shoulder Vector relative to stable point
        '''
        l_shX, l_shZ, l_shY = self.dev['skeleton_left_shoulder'].value
#        l_shY = -l_shY
        l_shX = -l_shX
        left_shoulder = Vector((l_shX, l_shY, l_shZ))
        return left_shoulder - self.stablepoint

    @property
    def right_shoulder(self):
        '''returns right shoulder Vector relative to stable point
        '''
        r_shX, r_shZ, r_shY = self.dev['skeleton_right_shoulder'].value
#        r_shY = -r_shY
        r_shX = -r_shX
        right_shoulder = Vector((r_shX, r_shY, r_shZ))
        return right_shoulder - self.stablepoint

    @property
    def left_elbow(self):
        '''returns left elbow Vector relative to stable point
        '''
        l_elX, l_elZ, l_elY = self.dev['skeleton_left_elbow'].value
#        l_elY = -l_elY
        l_elX = -l_elX
        left_elbow = Vector((l_elX, l_elY, l_elZ))
        return left_elbow - self.stablepoint

    @property
    def right_elbow(self):
        '''returns right elbow Vector relative to stable point
        '''
        r_elX, r_elZ, r_elY = self.dev['skeleton_right_elbow'].value
#        r_elY = -r_elY
        r_elX = -r_elX
        right_elbow = Vector((r_elX, r_elY, r_elZ))
        return right_elbow - self.stablepoint

    @property
    def left_hand(self):
        '''returns left hand Vector relative to stable point
        '''
        left_handX, left_handZ, left_handY = self.dev['skeleton_left_hand'].value
#        left_handY = -left_handY
        left_handX = -left_handX
        left_hand = Vector((left_handX, left_handY, left_handZ))
        return left_hand - self.stablepoint

    @property
    def right_hand(self):
        '''returns right hand Vector relative to stable point
        '''
        right_handX, right_handZ, right_handY = self.dev['skeleton_right_hand'].value
#        right_handY = -right_handY
        right_handX = -right_handX
        right_hand = Vector((right_handX, right_handY, right_handZ))
        return right_hand - self.stablepoint

    @property
    def torso(self):
        '''returns torso Vector relative to stable point
        '''
        torsoX, torsoZ, torsoY = self.dev['skeleton_torso'].value
#        torsoY = -torsoY
        torsoX = -torsoX
        torso = Vector((torsoX, torsoY, torsoZ))
        return torso - self.stablepoint

    @property
    def left_hip(self):
        '''returns left hip Vector relative to stable point
        '''
        left_hipX, left_hipZ, left_hipY = self.dev['skeleton_left_hip'].value
#        left_hipY = -left_hipY
        left_hipX = -left_hipX
        left_hip = Vector((left_hipX, left_hipY, left_hipZ))
        return left_hip - self.stablepoint

    @property
    def right_hip(self):
        '''returns right hip Vector relative to stable point
        '''
        right_hipX, right_hipZ, right_hipY = self.dev['skeleton_right_hip'].value
#        right_hipY = -right_hipY
        right_hipX = -right_hipX
        right_hip = Vector((right_hipX, right_hipY, right_hipZ))
        return right_hip - self.stablepoint

    @property
    def left_knee(self):
        '''returns left knee Vector relative to stable point
        '''
        l_kneeX, l_kneeZ, l_kneeY = self.dev['skeleton_left_knee'].value
#        l_kneeY = -l_kneeY
        l_kneeX = -l_kneeX
        left_knee = Vector((l_kneeX, l_kneeY, l_kneeZ))
        return left_knee - self.stablepoint

    @property
    def right_knee(self):
        '''returns right knee Vector relative to stable point
        '''
        r_kneeX, r_kneeZ, r_kneeY = self.dev['skeleton_right_knee'].value
#        r_kneeY = -r_kneeY
        r_kneeX = -r_kneeX
        right_knee = Vector((r_kneeX, r_kneeY, r_kneeZ))
        return right_knee - self.stablepoint

    @property
    def left_foot(self):
        '''returns left foot Vector relative to stable point
        '''
        l_footX, l_footZ, l_footY = self.dev['skeleton_left_foot'].value
#        l_footY = -l_footY
        l_footX = -l_footX
        left_foot = Vector((l_footX, l_footY, l_footZ))
        return left_foot - self.stablepoint

    @property
    def right_foot(self):
        '''returns right foot Vector relative to stable point
        '''
        r_footX, r_footZ, r_footY = self.dev['skeleton_right_foot'].value
#        r_footY = -r_footY
        r_footX = -r_footX
        right_foot = Vector((r_footX, r_footY, r_footZ))
        return right_foot - self.stablepoint

    def get_RelativeJointInfo(self):
        '''returns all skeleton joints information relative to a defined stable
           point.
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
