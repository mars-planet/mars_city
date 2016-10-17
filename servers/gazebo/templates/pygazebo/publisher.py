import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2


@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/trevor/joint_cmd',
                          'gazebo.msgs.JointCmd'))

    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.name = 'trevor::front_gripper::palm_riser'
    message.axis = 0
    message.force = 10.0

    count = 0
    while True:
        print("Publishing message #", count)
        yield From(publisher.publish(message))
        yield From(trollius.sleep(1.0))
        count += 1

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
