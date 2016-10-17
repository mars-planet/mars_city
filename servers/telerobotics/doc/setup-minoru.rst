==========================================================================
Setting up and Callibrating the Minoru 3D Camera
==========================================================================

:Author: Siddhant Shrivastava

This manual has been adopted partially `from here. <https://code.google.com/p/sentience/wiki/MinoruWebcam>`_

Change Record
=============

20\ :sup:`th`\  July, 2015 - Document Created

20\ :sup:`th`\  August, 2015 - Draft Open for review


The Minoru camera is recognized as a ``v4l2`` or ``Video4Linux2`` device.

The Minoru is UVC compilant, and therefore very easy to use on a GNU/Linux operating system. Plug in the camera, then open a command shell and type:

``ls /dev/video*``

This should display two extra video devices.

After setting up ``v4l2stereo`` from the ``streams/Minoru3D`` directory, run the following command -

``v4l2stereo -0 /dev/video1 -1 /dev/video0 --features``

So, once you have established that the cameras are working the first thing to do is calibrate them using the --calibrate option. This uses the OpenCV stereo camera calibration routines in order to obtain the optical parameters. First, print out a calibration pattern, which consists of a checkerboard pattern, and mount it on a rigid backing such as cardboard or wood. Then type:

``v4l2stereo --dev0 /dev/video1 --dev1 /dev/video0 --calibrate "6 9 24"``

The first number of the calibrate option is the number of squares across, the second is the number of squares down, and the third is the size of each square in millimetres. The order of the dimensions should correspond to how the calibration pattern is presented to the cameras. The video below shows the procedure.

Optionally the number of calibration images which are gathered can be set. By default this is 20, but higher numbers should give a more accurate result.

``v4l2stereo --dev0 /dev/video1 --dev1 /dev/video0 --calibrate "6 9 24" --calibrationimages 60``

Once camera calibration is complete the parameters are automatically saved to a file called calibration.txt. Normally when running v4l2stereo the program will search for this file in the current directory, but optionally you can also specify it as follows:

``v4l2stereo -0 /dev/video1 -1 /dev/video0 --calibrationfile calibration.txt``

To test the image rectification:

``v4l2stereo -0 /dev/video1 -1 /dev/video0``

If the rectification is good you should notice that when the left and right images are placed side by side the rows of both images correspond. If there is any vertical displacement it is possible to manually alter this, either by editing the vshift parameter within calibration.txt or by using the --offsety option.

