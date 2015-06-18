from PyTango import AttrQuality, AttrWriteType, DispLevel, DevState, AttrDataFormat, ArgType
from PyTango.server import Device, DeviceMeta, attribute, command
from PyTango.server import class_property, device_property

from pykinect.nui import JointId

class PyTracker(Device):
    __metaclass__ = DeviceMeta

    POLLING = 30

    joints = [
        'skeleton_head',
        'skeleton_neck',
        'skeleton_left_shoulder',
        'skeleton_right_shoulder',
        'skeleton_left_elbow',
        'skeleton_right_elbow',
        'skeleton_left_hand',
        'skeleton_right_hand',
        'skeleton_torso',
        'skeleton_left_hip',
        'skeleton_right_hip',
        'skeleton_left_knee',
        'skeleton_right_knee',
        'skeleton_left_foot',
        'skeleton_right_foot'
    ]

    attr_init_params = dict(
        dtype=('float32',),
        unit='m',
        max_dim_x=3,
        polling_period=POLLING
    )

    for joint in joints:
        exec "%s = attribute(**attr_init_params)" % joint

    tracker = None

    def set_tracker(self, tracker):
        self.tracker = tracker

    def acquire_skeleton_lock(self):
        if (self.tracker is None or
              self.tracker.get_skeleton() is None or
                not self.tracker.get_skeleton_lock().acquire(False)):
            return False
        else:
            return True

    def release_skeleton_lock(self):
        self.tracker.get_skeleton_lock().release()

    def joint_distance(self, joint_1, joint_2):
        dx = joint_1.x - joint_2.x
        dy = joint_1.y - joint_2.y
        dz = joint_1.z - joint_2.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def estimate_height(self):
        sk = self.tracker.get_skeleton()

        if sk is None:
            return -1

        done = False

        self.tracker.get_skeleton_lock().acquire()

        upper_body_height = 0
        right_leg = 0
        left_leg = 0
        try:
            upper_body_height += self.joint_distance(sk[JointId.Head],
                                                     sk[JointId.ShoulderCenter])
            upper_body_height += self.joint_distance(sk[JointId.Spine],
                                                     sk[JointId.ShoulderCenter])
            upper_body_height += self.joint_distance(sk[JointId.Spine],
                                                     sk[JointId.HipCenter])

            # hip_center | avg(hip_left, hip_right)
            avg_hip = sk[JointId.HipLeft]
            avg_hip.x = (sk[JointId.HipLeft].x + sk[JointId.HipRight].x) / 2.0
            avg_hip.y = (sk[JointId.HipLeft].y + sk[JointId.HipRight].y) / 2.0
            avg_hip.z = (sk[JointId.HipLeft].z + sk[JointId.HipRight].z) / 2.0
            upper_body_height += self.joint_distance(avg_hip,
                                                     sk[JointId.HipCenter])

            # left leg
            left_leg += self.joint_distance(sk[JointId.HipLeft],
                                            sk[JointId.KneeLeft])
            left_leg += self.joint_distance(sk[JointId.AnkleLeft],
                                            sk[JointId.KneeLeft])
            left_leg += self.joint_distance(sk[JointId.AnkleLeft],
                                            sk[JointId.FootLeft])

            # right leg
            right_leg += self.joint_distance(sk[JointId.HipRight],
                                             sk[JointId.KneeRight])
            right_leg += self.joint_distance(sk[JointId.AnkleRight],
                                             sk[JointId.KneeRight])
            right_leg += self.joint_distance(sk[JointId.AnkleRight],
                                             sk[JointId.FootRight])

            done = True

        finally:
            self.tracker.get_skeleton_lock().release()

        if done:
            return upper_body_height + ((left_leg + right_leg) / 2.0)
        else:
            return -1

    def read_skeleton_head(self):
        # sync access to skeleton
        if not self.acquire_skeleton_lock():
            return self._skeleton_head

        try:
            skeleton = self.tracker.get_skeleton()

            self._skeleton_head = (skeleton[JointId.Head].x,
                                  skeleton[JointId.Head].y,
                                  skeleton[JointId.Head].z)
            self._skeleton_neck = (skeleton[JointId.ShoulderCenter].x,
                                  (
                                    skeleton[JointId.ShoulderLeft].y * 0.3 +
                                    skeleton[JointId.ShoulderRight].y * 0.3 +
                                    skeleton[JointId.ShoulderCenter].y * 0.4
                                  ),
                                  skeleton[JointId.ShoulderCenter].z)
            self._skeleton_left_shoulder = (skeleton[JointId.ShoulderLeft].x,
                                           skeleton[JointId.ShoulderLeft].y,
                                           skeleton[JointId.ShoulderLeft].z)
            self._skeleton_right_shoulder = (skeleton[JointId.ShoulderRight].x,
                                            skeleton[JointId.ShoulderRight].y,
                                            skeleton[JointId.ShoulderRight].z)
            self._skeleton_left_elbow = (skeleton[JointId.ElbowLeft].x,
                                        skeleton[JointId.ElbowLeft].y,
                                        skeleton[JointId.ElbowLeft].z)
            self._skeleton_right_elbow = (skeleton[JointId.ElbowRight].x,
                                         skeleton[JointId.ElbowRight].y,
                                         skeleton[JointId.ElbowRight].z)
            self._skeleton_left_hand = (skeleton[JointId.HandLeft].x,
                                       skeleton[JointId.HandLeft].y,
                                       skeleton[JointId.HandLeft].z)
            self._skeleton_right_hand = (skeleton[JointId.HandRight].x,
                                        skeleton[JointId.HandRight].y,
                                        skeleton[JointId.HandRight].z)
            self._skeleton_torso = (skeleton[JointId.Spine].x,
                                   skeleton[JointId.Spine].y,
                                   skeleton[JointId.Spine].z)
            self._skeleton_left_hip = (skeleton[JointId.HipLeft].x,
                                      skeleton[JointId.HipLeft].y,
                                      skeleton[JointId.HipLeft].z)
            self._skeleton_right_hip = (skeleton[JointId.HipRight].x,
                                       skeleton[JointId.HipRight].y,
                                       skeleton[JointId.HipRight].z)
            self._skeleton_left_knee = (skeleton[JointId.KneeLeft].x,
                                       skeleton[JointId.KneeLeft].y,
                                       skeleton[JointId.KneeLeft].z)
            self._skeleton_right_knee = (skeleton[JointId.KneeRight].x,
                                        skeleton[JointId.KneeRight].y,
                                        skeleton[JointId.KneeRight].z)
            self._skeleton_left_foot = (skeleton[JointId.FootLeft].x,
                                       skeleton[JointId.FootLeft].y,
                                       skeleton[JointId.FootLeft].z)
            self._skeleton_right_foot = (skeleton[JointId.FootRight].x,
                                        skeleton[JointId.FootRight].y,
                                        skeleton[JointId.FootRight].z)

            # TODO: user step estimation (and add command)
        finally:
            self.release_skeleton_lock()

        return self._skeleton_head

    def read_skeleton_neck(self):
        return self._skeleton_neck

    def read_skeleton_left_shoulder(self):
        return self._skeleton_left_shoulder

    def read_skeleton_right_shoulder(self):
        return self._skeleton_right_shoulder

    def read_skeleton_left_elbow(self):
        return self._skeleton_left_elbow

    def read_skeleton_right_elbow(self):
        return self._skeleton_right_elbow

    def read_skeleton_left_hand(self):
        return self._skeleton_left_hand

    def read_skeleton_right_hand(self):
        return self._skeleton_right_hand

    def read_skeleton_torso(self):
        return self._skeleton_torso

    def read_skeleton_left_hip(self):
        return self._skeleton_left_hip

    def read_skeleton_right_hip(self):
        return self._skeleton_right_hip

    def read_skeleton_left_knee(self):
        return self._skeleton_left_knee

    def read_skeleton_right_knee(self):
        return self._skeleton_right_knee

    def read_skeleton_left_foot(self):
        return self._skeleton_left_foot

    def read_skeleton_right_foot(self):
        return self._skeleton_right_foot

    @command(dtype_out=float)
    def get_height(self):
        return self.estimate_height()

    def init_device(self):
        Device.init_device(self)
        self.info_stream('In Python init_device method')
        self.set_state(DevState.ON)
        self._skeleton_head = (0, 0, 0)
        self._skeleton_neck = (0, 0, 0)
        self._skeleton_left_shoulder = (0, 0, 0)
        self._skeleton_right_shoulder = (0, 0, 0)
        self._skeleton_left_elbow = (0, 0, 0)
        self._skeleton_right_elbow = (0, 0, 0)
        self._skeleton_left_hand = (0, 0, 0)
        self._skeleton_right_hand = (0, 0, 0)
        self._skeleton_torso = (0, 0, 0)
        self._skeleton_left_hip = (0, 0, 0)
        self._skeleton_right_hip = (0, 0, 0)
        self._skeleton_left_knee = (0, 0, 0)
        self._skeleton_right_knee = (0, 0, 0)
        self._skeleton_left_foot = (0, 0, 0)
        self._skeleton_right_foot = (0, 0, 0)
