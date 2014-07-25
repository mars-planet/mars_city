from visual import *
from collections import namedtuple
from UserSkeletonData import UserSkeletonData


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
           Returns true if recent frame from sensor contains skeleton
           information.
        '''
        updated = False
        # loop for skeleton in the frame obtained from depth sensor frame
        st.loop()
        if st.usersCount() > 0:

            # Update joints
            for joint, skjoint in zip(self.joints, scaled_jointsInfo()):
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

# Set environment in order to get joints data from kinect sensor
# user0 contains skeleton information of the user
# st is SkeletonTracker object
user0 = UserSkeletonData()
st = user0.st

Coords = namedtuple('coords', 'x y z')

def scaled_jointsInfo():
    '''returns joints information scaled by scaled factor sf'''
    sf = 1.5
    scaled_joints = []
    for joint in user0.get_jointInfo():
        sj = Coords(joint.x/sf, joint.y/sf, joint.z/sf)
        scaled_joints.append(sj)
    return scaled_joints

if __name__ == '__main__':
    skeletonFrame = SkeletonFrame(frame(visible=False))
    while True:
        rate(30)
        skeletonFrame.frame.visible = skeletonFrame.draw()
