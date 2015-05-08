=========================
Execution of body tracker
=========================

:Author: Vito Gentile

Change Record
=============

3\ :sup:`rd`\  May, 2015 - Document created.

Compilation and execution of C# tracker
=======================================

In order to execute the tracker, before of all you have to compile it.
To do this open with Visual Studio the *MSKinectMultipeTrackerGUI.sln* file,
that is under the *MSKinectMultipeTrackerGUI* folder.

Than you can compile and run the C# project. This has to be done only the first
time, and after any change; if you don't need to compile the project, you can simply
execute the *.exe* file under ``MSKinectMultipeTrackerGUI/bin/Release`` or ``MSKinectMultipeTrackerGUI/bin/Debug``.

GUI usage
=========

The GUI is done to allow multiple Kinect sensors working simultaneously.
In particular, the maximum number of Kinect sensors that can be used is 4.

Each column of the GUI is dedicated to a specific Kinect, that is labeled with a letter
(A, B, C or D). You can identify which is the letter of each physical device
by using the button *Identify*, that allow you to move the motorized tilt of
each Kinect.

You can also adjust the tilt manually, using the trackbar.

When you identify which is the letter of Kinect, you can start the tracking
by selecting a Tango device name for that sensor. Names available are
ERAS-1, ERAS-2, ERAS-3 and ERAS4. When you select the name, you cannot change it
and the tracking will start.

For example, if you select ERAS-2 for the Kinect A, then data from the Kinect A
will be published in the Tango bus with the device name eras-2. In other words,
data from Kinect A will be continously written in ``C:\\Joints\\eras-2\\joints.json``,
and the python script ``tracker`` will be executed with parameter ``eras-2``,
to read that JSON file and publish its content on the Tango bus.
