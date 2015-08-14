==========================================================================
Machine Configuration for Telerobotics-Bodytracking Interface
==========================================================================

:Author: Siddhant Shrivastava

.. contents::
   :local:
   :depth: 3


Change Record
=============

13\ :sup:`th`  August, 2015 - Document Created

14\ :sup:`th`  August, 2015 - Replication Details added

15\ :sup:`th`  August, 2015 - First Draft open for review

Machine Setup
============================

Successfully tested on a **single** laptop with the following configuration -

- **Ubuntu 14.04 amd64** *host OS*
- **Windows 7 Ultimate 64-bit** *Virtual Machine*
- **VMware Workstation 11**
- *Hardware Specifications* - 8GB RAM, intel Core i7 x64
- *Networking Specifications*: ``NAT-Only`` mode between host OS and VM


Windows Configuration
============================

The following instructions are based and were tested on Microsoft Windows 7 64-bit.
After having installed the operating system in VMware, setup all drivers needed to use
the machine in the right way.

Installing Python
------------------------------

To support PyKinect, you must install *Python 32-bit 2.7.10*.
To install this version of Python, `use this link. <https://www.python.org/downloads/release/python-2710/>`_

It is recommended to install this version of Python in ``C:\Python27_32bit\``.

To be able to excute Python from a command line, you must add the installation
folder path to the ``Path`` variabe in Windows. In order to do this, follow these steps -

* Open **My Computer**
* Right-click and select *Properties*
* Choose *Advanced system settings* from the options on the left panel
* A menu should appear. Click on the *Environment Variables* button
* Add the Python path to the Path variable

You will also need to set the ``PYTHONPATH`` variable likewise. Create a new environment variable with the name **PYTHONPATH**
and the value of ``C:\Python27_32bit\``

In order to use Python 2.7 on Windows, Visual C++ compiler (*VCforPython27*)must also be installed. It can be downloaded `from this link. <www.microsoft.com/en-in/download/details.aspx?id=44266>`_

Setting up Kinect Tools
---------------------------------------------
In order to use Kinect on Windows, Kinect Developer Kit and the SDK need to be installed

The installation process will be quite long, and it will probably require some reboots.
After that, you have to install (in this order):

* `Kinect SDK`: <https://www.microsoft.com/en-us/download/details.aspx?id=40278>
* `Kinect Developer Kit`: <https://www.microsoft.com/en-us/download/details.aspx?id=40276>

Installing PyKinect
-------------------

By following the above instructions for installing a Python package from
Visual Studio, or simply using ``pip`` on a command line terminal,
install the package ``pykinect``.

Installing additional Python packages
-------------------------------------------------------
Before continuing, you need also to install the following Python packages:

* *numpy*: required to install PyTango; it can be installed with ``pip install numpy``
* *PyTango*: download the last 32-bit version for Python 2.7, available
  from https://pypi.python.org/pypi/PyTango/
* *pgu*: download from https://code.google.com/p/pgu/ and install it with
  ``pip`` (follow the above instructions, as if the package you download
  is a .whl file)
* *VPython*: download the automatic installer for Visual Python `from here <http://vpython.org/contents/download_windows.html>`_
  (chose the x86 version, not the x64 one!)

.. note::

      As a source to find a lot of Python libraries, packed as Windows
      installers or as ``.whl`` files, you can `refer here <http://www.lfd.uci.edu/~gohlke/pythonlibs/>`_

Installing Tango
---------------------------------------------
Go to http://www.tango-controls.org/downloads/source/ and select the binary
distribution for Windows x64. Download and install it.

After the installation, you will be able to access to a lot of utility and tools to get
information about Tango and the device servers (e.g. *Jive*). To use them, you must install
*Java for Windows*; you can get it from https://java.com/download/

Configure Tango Host
--------------------

To be able to get all Tango informations, you need to specify the address of the Tango host.
Assuming that it is 198.168.1.100:10000, open the command line and type:

    ``set TANGO_HOST=192.168.1.100:10000``

Installing Java
---------------------------------------------
Dowload the latest Java runtime from the Oracle website. The specific page is `located here. <http://www.oracle.com/technetwork/java/javase/downloads/jre8-downloads-2133155.html>`_

Ubuntu configuration
====================

Install Ubuntu 14.04.2 on the computer.

VMware Setup
---------------------
Install **VMware Workstation 11**. Follow the Ubuntu instructions as shown `on this page. <https://www.vmware.com/go/downloadworkstation>`_
Set up Windows on this VMware setup and follow the instructions for Windows.

ROS Setup
----------------
Install **ROS Indigo** on Ubuntu using the following instructions -

* Setup the Sources list

  ``sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'``

* Set up keys

  ``sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116``

* Installation

  ``sudo apt-get update``

  ``sudo apt-get install ros-indigo-desktop-full``

  ``sudo apt-get install synaptic``

* Initialize ROS

  ``sudo rosdep init``

  ``rosdep update``

  ``echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc``

  ``source ~/.bashrc``

* From the Dash, open ``Synaptic Package Manager``
* Search for ``husky indigo`` and install all the packages prefixed by ``ros-indigo-husky-``

Tango Setup
--------------------
Follow the instructions in the `ERAS documentation to set up Tango.<http://eras.readthedocs.org/en/latest/doc/setup.html>`_


This should set up everything on the Ubuntu side.

Networking Setup
============

Tango should be appropriately configured on both sides. In the working setup, the Tango Master is configured to be the Ubuntu machine.

Set the **Networking Mode** of the Virtual Machine to be ``NAT-Only``. Observe the output of ``ifconfig`` in the host OS. The Virtual Machine should have created two additional interfaces - ``vmnet1`` and ``vmnet8``. Use the interface whose subnet matches the interface in the output of ``ipconfig`` in the Windows Virtual machine.

Configure Tango Host to the IP Address corresponding to the ``vmnet`` interface which matches the subnet information. Try running ``jive`` on both the Operating Systems to check for consistency.

Replication instructions
===========================

Once the setup and configuration is complete (as discussed in the previous sections), run the following commands on the host OS -

First configure the Tango Database server to use the Bodytracking device with the following attributes -

* **Device Name** - ``eras-1``
* **Device Class** - ``PyTracker``
* **Canonical Name** - ``c3/mac/eras-1``

On the **Bodytracking (Windows) machine** -
------------------------------------------------------

* Open a Command Prompt window and execute -

  ``python tracker.py eras-1 --sim <json_file_location>``

* Open another Command Prompt window and execute -

  ``python visualTracker.py eras-1``

This should bring up the skeleton model which is updated in real-time.

On the **Telerobotics (Ubuntu) machine** -
------------------------------------------------------

``cd`` to the ``src`` directory of the ``Telerobotics`` ERAS server.

Open **three** terminals -

* In the first terminal, run

  ``roslaunch husky-gazebo husky_empty_world.launch``

* In the second terminal, run

  ``roslaunch husky_viz view_robot.launch``

* In the third terminal, run

  ``python telerobotics-bodytracking.py``
