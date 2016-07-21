def save_body_coodrinates(self, joints, jointPoints):
    # Torso
    torso = []
    torso.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck))
    torso.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder))
    torso.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                                      PyKinectV2.JointType_SpineMid))
    torso.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase))
    torso.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                                      PyKinectV2.JointType_ShoulderRight))
    torso.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                                      PyKinectV2.JointType_ShoulderLeft))
    torso.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight))
    torso.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft))

    # Right Arm
    right_arm = []
    right_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight,
                                          PyKinectV2.JointType_ElbowRight))
    right_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight,
                                          PyKinectV2.JointType_WristRight))
    right_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_WristRight,
                                          PyKinectV2.JointType_HandRight))
    right_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_HandRight,
                                          PyKinectV2.JointType_HandTipRight))
    right_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_WristRight,
                                          PyKinectV2.JointType_ThumbRight))

    # Left Arm
    left_arm = []
    left_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft,
                                         PyKinectV2.JointType_ElbowLeft))
    left_arm.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft,
                             PyKinectV2.JointType_WristLeft))
    left_arm.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft))
    left_arm.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_HandLeft,
                                         PyKinectV2.JointType_HandTipLeft))
    left_arm.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_WristLeft,
                             PyKinectV2.JointType_ThumbLeft))

    # Right Leg
    right_leg = []
    right_leg.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight))
    right_leg.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_KneeRight,
                                          PyKinectV2.JointType_AnkleRight))
    right_leg.append(self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight,
                                          PyKinectV2.JointType_FootRight))

    # Left Leg
    left_leg = []
    left_leg.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft))
    left_leg.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft))
    left_leg.append(
        self.get_coordinates(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft))

    skeleton = {}
    skeleton = {"torso": torso, "right arm": right_arm, "left arm": left_arm, "right leg": right_leg,
                "left leg": left_leg}
    skeletonnd = numpy.asarray(skeleton)
    return skeletonnd