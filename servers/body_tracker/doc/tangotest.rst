===========================================
Testing Tango integration on a Windows host
===========================================

:Author: Vito Gentile

Change Record
=============

3\ :sup:`rd`  May, 2015 - Document created.

18\ :sup:`rd`  Jun, 2015 - Fix typos.

Purpose
=======
This document explains the procedure to test Tango-based software
on Windows. The basic idea is to provide clear instructions to setup all the necessary
components to test, on a single machine, if data sent on Tango bus can be 
read from other clients.

In order to test Tango integration, VirtualBox needs to be installed on
the Windows machine. This allows to create a virtual machine with an installation
of Ubuntu. In this way, a Tango server can be installed in the virtual machine,
and will be possible to emulate Tango communications between Windows and other
clients, by testing if data from Windows can be read also on Ubuntu.

Reference Documents
===================

- [1] -- `Instructions for setting up a MS Windows machine for running body tracking`: <https://eras.readthedocs.org/en/latest/servers/body_tracker_ms/doc/setup.html>

Version of Microsoft Windows
============================

The following instructions are based and were tested on Microsoft Windows 7 64-bit.
After having installed the operating system, setup all drivers needed to use
the machine in the right way.

Furthermore, it is assumed that [1] has been read and used before of all.

Version of Ubuntu
=================

The following instruction are based and were tested on Ubuntu 14.10 and
Ubuntu MATE 14.10.

If you would like to use any other Linux distribution, consider the possibility
of unexpected installation errors.

Setup of an Ubuntu virtual machine on VirtualBox
================================================

Setting up a virtual machine on VirtualBox is quite easy, and doesn't need
further explanation.

After the Ubuntu installation, install the Guest Additions as follows:
 * Start the Ubuntu virtual machine
 * Click on the *Devices* menu and choose *Install Guest Additions...*
 * This mounts the VBox Guest Additions ISO image. Run this as root:

    ``# ./media/cdrom0/VBoxLinuxAdditions.run``

 * Shut down the virtual machine.

Then, after shutting down the virtual machine, go to *Settings -> Network*
and select *Bridged networking*. This allows to assign an IP to the
virtual machine (Ubuntu), that will be different from the IP of the host
system (Windows).

Set a static local IP address for the virtual machine
=====================================================

Setting up a static IP address to the virtual machine can facilitate the
communication between Windows client and the Tango server (that will be
installed on Ubuntu).

To do this with Ubuntu in a semi-graphical way (preferred, due to automatic
management of the Network Manager plugin), see:
http://www.sudo-juice.com/how-to-a-set-static-ip-in-ubuntu/

It is also possible to configure a static IP manually.
For more details, see: https://www.howtoforge.com/debian-static-ip-address

Installation of Tango on Ubuntu (guest)
=======================================

The next step is to install Tango on the Ubuntu virtual machine. Please
refer to the `Tango setup <https://eras.readthedocs.org/en/latest/doc/setup.html>`
page of this documentation.

Set environment variable on Windows
===================================

In the `Instructions for setting up a MS Windows machine for running body
tracking <https://eras.readthedocs.org/en/latest/servers/body_tracker_ms/doc/setup.html>`,
you were asked to set up an address for the Tango server. What we will do here
is to perform our tests with a simple Tango server installed in the virtual
machine.

Assuming the the IP address of the virtual machine is 192.168.0.111, open
a terminal on Windows and execute the following command:

    ``set TANGO_HOST=192.168.0.111:10000``

Perform tests
=============

Now everything is ready for testing. You can publish your data on the Tango
bus (from Windows), and then use **jive** from the virtual machine to see
if your data have been correctly published.
