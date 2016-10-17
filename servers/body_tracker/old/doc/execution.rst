=========================
Execution of body tracker
=========================

:Author: Vito Gentile

Change Record
=============

3\ :sup:`rd`  May, 2015 - Document created.

Compilation and execution of C# tracker
=======================================

In order to execute the tracker, first of all you have to compile it.
To do this open with Visual Studio the ``MSKinectMultipeTrackerGUI.sln`` file,
that is under the ``MSKinectMultipeTrackerGUI`` folder.

Then you can compile and run the C# project. This has to be done only the first
time, and after any change; if you don't need to compile the project, you can simply
execute the *.exe* file under ``MSKinectMultipeTrackerGUI/bin/Release`` or ``MSKinectMultipeTrackerGUI/bin/Debug``.

GUI usage
=========

The GUI is done to allow multiple Kinect sensors to work simultaneously.
In particular, the maximum number of Kinect sensors that can be used is 4.

When you plug all the Kinects in the Windows machine, a letter will be assigned
to each of them. These letters simply represent the random order used by
Windows to list them in the MS Kinect API. But when you open the GUI,
you cannot know which is the letter assigned to each phisycal device.

After you setup the Windows machine, you will probably put one Kinect
in front of each Motivity station, that will be equipped with another
machine that uses Oculu Rift and Blender to implement Virtual Reality
(and the name of this machine will be ERAS-X).

Now, it is possible that the Kinect in front of the Motivity that uses ERAS-2
machine is, for instance, the Kinect A. So in this situation you will need
to associate the Kinect A to ERAS-2, so that skeletal data will be published
on Tango by a device server named ``c3/MAC/eras-2``.

Each column of the GUI is dedicated to a specific Kinect, that is labeled with a letter
(A, B, C or D). You can identify which is the letter of each physical device
by using the button *Identify*, that allow you to move the motorized tilt of
each Kinect.

You can also adjust the tilt manually, using the trackbar.

When you identify the letter of the Kinect, you can start the tracking
by selecting a Tango device name for that sensor. Names available are
ERAS-1, ERAS-2, ERAS-3 and ERAS-4. When you select the name, you cannot change it
and the tracking will start.

For example, if you select ERAS-2 for the Kinect A, then data from the Kinect A
will be published in the Tango bus with the device name eras-2. In other words,
data from Kinect A will be continuously written in ``C:\\Joints\\eras-2\\joints.json``,
and the python script ``tracker`` will be executed with parameter ``eras-2``,
to read that JSON file and publish its content on the Tango bus.
