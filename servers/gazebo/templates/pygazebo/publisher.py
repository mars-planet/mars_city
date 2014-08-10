import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2

@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/model/joint_cmd',
                          'gazebo.msgs.JointCmd'))

    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.axis = 0
    message.force = 1.0
    while True:
        yield From(publisher.publish(message))
        yield From(trollius.sleep(1.0))

loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
