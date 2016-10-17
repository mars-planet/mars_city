==========================================================================
Instructions for setting up a MS Windows machine for running body tracking
==========================================================================

:Author: Vito Gentile

Change Record
=============

13\ :sup:`th`  Dec, 2014 - Document created.

3\ :sup:`rd`  May, 2015 - Improved formatting (minor fix).

4\ :sup:`th`  Jun, 2015 - Updated setup instructions for Python, Tango and PyKinect.

18\ :sup:`th`  Jun, 2015 - Fix typos.

2\ :sup:`nd`  Aug, 2015 - Added setup instructions for VPython.


Version of Microsoft Windows
============================

The following instructions are based and were tested on Microsoft Windows 7 64-bit.
After having installed the operating system, setup all drivers needed to use
the machine in the right way.

Before starting with all mandatory software to run body tracking, may be useful to
install all the following software:

* Recommended:
   * A modern browser (e.g. Firefox or Chrome)
   * 7zip
   * Geany (or any other programming-oriented text editor)
   * Daemon Tools Lite (useful for mounting .iso images, during some installation phases)
   * VirtualBox
   * TortoiseHG

* Suggested:
   * Adobe Flash Player
   * Adobe Reader (or another PDF reader)

Installing Python
=================

To support PyKinect, you must install *Python 32-bit 2.7*.
To install this version of Python, use `this link <https://www.python.org/ftp/python/2.7/python-2.7.msi>`_

It is recommended to install this version of Python in ``C:\Python27_32bit\``.

To be able to excute Python from a command line, you must add the installation
folder path to the ``Path`` variabe in Windows. You will also need to set
the ``PYTHONPATH`` variable. To this end do the following:

* Right-click *My Computer* and select *Properties*
* Click the *Advanced System Settings* link in the left column
* The System Properties window will open. Here click on the *Advanced* tab, then click the *Environment Variables* button near the bottom of that tab
* In the *Environment Variables* window, highlight the ``Path`` variable in the *System variables* section and click the *Edit* button
* Append the following at the end of the value: ``C:\Python27_32bit\``, then click on *Ok*
* In the *Environment Variables* window, check if there is a ``PYTHONPATH`` variable. If yes, highlight it and change its value in ``C:\Python27_32bit\``; otherwise click on the *New* button, and create this environment variable

Installing Visual Studio and Kinect SDK
=======================================

In order to use Kinect on Windows, you have to install Microsoft Visual Studio.
Tests were done using *Visual Studio 2013*.

The installation process will be quite long, and it will probably require some reboots.
After that, you have to install (in this order):

* `Kinect SDK 1.8 <https://www.microsoft.com/en-us/download/details.aspx?id=40278>`_
* `Kinect Developer Kit 1.8 <https://www.microsoft.com/en-us/download/details.aspx?id=40276>`_

Finally, simply plug-in the Kinect and let Windows Update to install its drivers.

Installing PTVS and PyKinect
============================

Python Tools for Visual Studio (PTVS) "is a free, open source plugin that
turns Visual Studio into a Python IDE". It can be useful if you want to
develop in Python with Visual Studio, and it also provides some facilities
for Kinect developers.

Installing and use PTVS on Visual Studio
----------------------------------------

In order to install PTVS, go to http://pytools.codeplex.com/releases and
download the most recent version of PTVS that fits with your Visual Studio
version. Tests were done using Visual Studio 2013 and *PTVS 2.1*, and the
following documentation refers to these versions.

After having installed PTVS, open Visual Studio, and go to
:guilabel:`File -> New -> Project`. Then, under
:guilabel:`Template -> Python -> Samples`, select :guilabel:`PyGame using PyKinect`.
This will create a new Python project, with a Python script structured to
be used with PyGame and PyKinect.

Project configuration
---------------------

In Visual Studio 2013, go to :guilabel:`Tools -> Options`. Then, under
:guilabel:`Python Tools -> Environment Options`, select the
:guilabel:`Python 32-bit 2.7` environment. If it is not available, select
:guilabel:`Add Environment`, name it :guilabel:`Python 32-bit 2.7` and add
the following fields:

* :guilabel:`Path`: ``C:\Python27_32bit\python.exe``
* :guilabel:`Windows Path`: ``C:\Python27_32bit\pythonw.exe``
* :guilabel:`Library Path`: ``C:\Python27_32bit\lib``
* :guilabel:`Architecture`: ``x86``
* :guilabel:`Language Version`: ``2.7``
* :guilabel:`Path Environment Variable`: ``PYTHONPATH``

Now open the Solution Explorer under the project name, right click on
:guilabel:`Python Environments` and select :guilabel:`Add/Remove Python Environments...`.
Then make sure that only the :guilabel:`Python 32-bit 2.7` environment
is checked.

Installing PyGame
-----------------

Go to http://www.lfd.uci.edu/~gohlke/pythonlibs/#pygame for downloading
and installing PyGame for Python 32-bit 2.7. You can do it with ``pip``,
but also by following the instructions shown in Visual Studio after project
creation. These instruction are summarized as follows, and can be generally
used for installing any additional Python package.

* In the Solution Explorer, right click on `Python 32-bit 2.7`
  (under `Python Environments`) and then select
  `Install Python Packages...`
* If you want to install a Python package without explicitly download it:
    * Select :guilabel:`pip`, type the package name and then select :guilabel:`OK`
* If you want to install a downloaded .whl package (e.g. obtained from
  http://www.lfd.uci.edu/~gohlke/pythonlibs/):

    * Make sure to have the package ``wheel`` installed. If not, install
      it as described above
    * Select :guilabel:`pip`, type the full path to the file (wrapped by
      double quotes) and then select :guilabel:`OK`

Using the above instructions you will be able to install PyGame, by typing
the double-quoted full path of the PyGame package downloaded from
http://www.lfd.uci.edu/~gohlke/pythonlibs/#pygame. Make sure to select the
last 32-bit version for Pythion 2.7 (the file name should be something like
``pygame‑X.X.XXX‑cp27‑none‑win32.whl``.

Tests were done with PyGame 1.9.2a0 32-bit for Python 2.7.

Installing PyKinect
-------------------

By following the above instructions for installing a Python package from
Visual Studio, or simply using ``pip`` on a command line terminal,
install the package ``pykinect``.

Installing additional Python packages
=====================================

Before continuing, you need also to install the following Python packages:

* *numpy*: required to install PyTango; it can be installed with ``pip``
  or using the above instructions for installing a Python package from
  Visual Studio
* *PyTango*: download the last 32-bit version for Python 2.7, available
  from https://pypi.python.org/pypi/PyTango/
* *pgu*: download from https://code.google.com/p/pgu/ and install it with
  ``pip`` (follow the above instructions, as if the package you download
  is a .whl file)
* *VPython*: download the automatic installer from http://vpython.org/contents/download_windows.html
  (chose the Win-32 version, not the Win-64 one!)

.. note::

      As a source to fine a lot of Python libraries, packed as Windows
      installers or as .whl files, you can refer to http://www.lfd.uci.edu/~gohlke/pythonlibs/

Installing Tango
================

Go to http://www.tango-controls.org/downloads/source/ and select the binary
distribution for Windows 64 bits. Download and install it.

After the installation, you will be able to access to a lot of utility and tools to get
information about Tango and the device servers (e.g. *Jive*). To use them, you must install
*Java for Windows*; you can get it from https://java.com/download/

Configure Tango Host
--------------------

To be able to get all Tango information, you need to specify the address of the Tango host.
Assuming that it is 198.168.1.100:10000, open the command line and type:

    ``set TANGO_HOST=192.168.1.100:10000``

Using a virtual machine manager
===============================

Installing a virtual machine manager like *VirtualBox* can be very useful in
order to install Ubuntu or another Linux distribution on the same Windows machine.

If you want to do this, you are probably interested in setting up a shared folder between
host and guest operating systems.
To do this in VirtualBox, see: http://my-wd-local.wikidot.com/otherapp:configure-virtualbox-shared-folders-in-a-windows-ho
