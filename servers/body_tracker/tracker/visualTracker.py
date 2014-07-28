import PyTango
from visual import *
from collections import namedtuple


# get the tango device
dev = PyTango.DeviceProxy('e3/MAC/tracker')

# Coordinate structure
Coords = namedtuple('coords', 'x y z')

def getJoints():
    '''returns joints information read from tango bus
    '''
    head = vector(dev['skeleton_head'].value)
    neck = vector(dev['skeleton_neck'].value)
    left_shoulder = vector(dev['skeleton_left_shoulder'].value)
    left_elbow = vector(dev['skeleton_left_elbow'].value)
    left_hand = vector(dev['skeleton_left_hand'].value)
    right_shoulder = vector(dev['skeleton_right_shoulder'].value)
    right_elbow = vector(dev['skeleton_right_elbow'].value)
    right_hand = vector(dev['skeleton_right_hand'].value)
    torso = vector(dev['skeleton_torso'].value)
    left_hip = vector(dev['skeleton_left_hip'].value)
    left_knee = vector(dev['skeleton_left_knee'].value)
    left_foot = vector(dev['skeleton_left_foot'].value)
    right_hip = vector(dev['skeleton_right_hip'].value)
    right_knee = vector(dev['skeleton_right_knee'].value)
    right_foot = vector(dev['skeleton_right_foot'].value)
    return (head, neck, left_shoulder, left_elbow, left_hand, right_shoulder,
            right_elbow, right_hand, torso, left_hip, left_knee, left_foot,
            right_hip, right_knee, right_foot)

def scaledJoints():
    '''returns joints scaled by scale factor sf'''
    sf = 1.5
    sj = [Coords(joint.x/sf, joint.y/sf, joint.z/sf) for joint in getJoints()]
    return sj


class SkeletonFrame(object):
    '''Skeleton frame drawn by visual python in given frame
    '''
    def __init__(self, frame):
        '''Creates a skeleton frame in given frame'''
        self.frame = frame
        self.joints = [sphere(frame = frame, radius = 50, color =
                              color.yellow) for i in range(0, 15)]
        self.joints[0].radius = 80
        self.bones = [cylinder(frame = frame, radius = 25, color =
                                color.green) for bone in bones]

    def draw(self):
        '''draw and updates the skeleton joints and bones in Skeleton frame.
           Returns true if recent stream from tango bus contains skeleton
           information.
        '''
        updated = False
        # Update joints
        for joint, skjoint in zip(self.joints, scaledJoints()):
            joint.pos = vector(skjoint.x, skjoint.y, skjoint.z)

        # Update bones
        for bone, bone_id in zip(self.bones, bones):
            pv1, pv2 = [self.joints[id].pos for id in bone_id]
            bone.pos = pv1
            bone.axis = pv2 - pv1
        updated = True

        return updated


# bones contains list of pairs of bone id's which are end points of a bone
# a bone is represented as a cylinder linking two joints.
# Bone ID's --> Joint
# 0  --> head          1  --> neck           2  --> left shoulder
# 3  --> left elbow    4  --> left hand      5  --> right shoulder
# 6  --> right elbow   7  --> right hand     8  --> torso
# 9  --> left hip      10 --> left knee      11 --> left foot
# 12 --> right hip     13 --> right knee     14 --> right foot
bones = [(0, 1), (2, 5), (1, 8), (2, 3), (3, 4), (5, 6), (6, 7), (8, 12),
         (8, 9), (9, 10), (12, 13), (13, 14), (10, 11)]

if __name__ == '__main__':
    skeletonFrame = SkeletonFrame(frame(visible=False))
    while True:
        rate(30)
        skeletonFrame.frame.visible = skeletonFrame.draw()
