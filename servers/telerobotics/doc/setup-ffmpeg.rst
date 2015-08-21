==========================================================================
Instructions for setting up a Real-Time Streaming Server
==========================================================================

:Author: Siddhant Shrivastava

Change Record
=============

21\ :sup:`st`\  June, 2015 - Document created

26\ :sup:`th`\  June, 2015 - Added single camera streaming Instructions

10\ :sup:`th`\  July, 2015 - Added `Explanations`_ section

15\ :sup:`th`\  July, 2015 - Added FFmpeg command explanation for single camera

19\ :sup:`th`\  July, 2015 - Added `Debugging`_ section

20\ :sup:`th`\  August, 2015 - Added details and links.


FFmpeg Setup
=================

`Download from source <http://ffmpeg.org/download.html>`_ Generally, ``FFmpeg`` comes with ``v4l2-utils`` support built-in. This is important for controlling video devices on Linux-based Operating Systems.

Steps to get the setup working
================================

1. Configure a ffmpeg server on the remote machine.

2. For this, copy the ``ffserver.conf`` file from ``eras/servers/telerobotics/streams/single_camera``
   into the ``/etc/`` directory of the remote machine.

3. Start the server via the ffmpeg command specified in webcam.sh

4. The server reads from ``/dev/video0`` and serves it on the listening port.

5. The client machine has the Blender scene and script running.

Testing and Explanation
=============

Single Camera
-------

    ``  ffmpeg -v verbose -r 30 -s 640x480 -f video4linux2 -i /dev/video0 http://localhost:8190/webcam.ffm ``

- This command ensures a **single camera stream** explained by the various options of FFmpeg. The `-r` option specifies a framerate of 30fps (frames per second from the input device) and `-s` option specifies an input resolution of 640x480 from the camera.
- The input device is ``/dev/video0`` which is a ``video4linux2`` supported device.
- The output device is the **Feed** which is identified by the ``ffserver.conf`` configuration.

Debugging
=============

In case of dropped frames or memory-related issues, it is quite possible that the problem is hardware related. In this case, the ``v4l2-ctl`` tool is quite useful. Some important commands while troubleshooting such issues are -

- ``v4l2-ctl --list-devices``
- ``v4l2-ctl --verbose <any_command>``
- ``v4l2-ctl --list-ctrls <device name>``

Knowing the capability of the camera in hand goes a long way in identifying the usage profile of the camera as a video device. Cameras have a fixed set of frame rates supported by the hardware. These must be kept in mind before streaming at an intended frame rate.

``ffmpeg`` has its own levels of verbosity which can be changed while streaming. ``ffprobe`` can be used for further performance analysis. ``ffplay`` is a good tool to check the output quality of a stream.
