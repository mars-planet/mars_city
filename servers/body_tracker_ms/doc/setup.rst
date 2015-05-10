==========================================================================
Instructions for setting up a MS Windows machine for running body tracking
==========================================================================

:Author: Vito Gentile

Change Record
=============

13\ :sup:`th`  December, 2014 - Document created.

3\ :sup:`rd`  May, 2015 - Improved formatting (minor fix).


Version of Microsoft Windows
============================

The following instructions are based and were tested on Microsoft Windows 7 64-bit.
After having installed the operating system, setup all drivers needed to use
the machine in the right way.

Before starting with all mandatory software to run body tracking, may be useful to
install all the following software:

* A modern browser (e.g. Firefox or Chrome)
* Adobe Flash Player
* Adobe Reader (or another PDF reader)
* 7zip
* Geany (or a better programming-oriented text editor)
* Daemon Tools Lite
* VMWare or VirtualBox (or any other good virtual machine manager)
* TortoiseHG

.. note::

      All the installers needed for setting up the machine are available in the ERAS-0
      server, under the folder `windows-installers`.

Installing Python
=================

All the tests were done using *Python 2.6* (the system may work also with next versions,
but it must be tested to this end).
To install this version of Python, go to https://www.python.org/ftp/python/2.6.6/python-2.6.6.amd64.msi

**Warning**: It is strogly recommended to install Python 2.6 in ``C:\Python26\``

After installing it, you need also the following libraries:

* *NumPy*
* *PyTango*
* *VPython*

.. note::

      A lot of python libraries, packed as Windows installers, are available
      at http://www.lfd.uci.edu/~gohlke/pythonlibs/

To be able to excute Python from a command line, you must add the installation folder path to the
``Path`` variabe in Windows. To this end do the following:

* Right-click *My Computer* and select *Properties*
* Click the *Advanced System Settings* link in the left column
* The System Properties window will open. Here click on the *Advanced* tab, then click the *Environment Variables* button near the bottom of that tab
* In the *Environment Variables* window, highlight the ``Path`` variable in the *System variables* section and click the *Edit* button
* Append the following at the end of the value: ``C:\Python26\``


Installing Visual Studio and Kinect SDK
=======================================

In order to use Kinect on Windows, you have to install Microsoft Visual Studio.
Tests were done using *Visual Studio 2012 Ultimate Edition x86*.

The installation process will be quite long, and it will probably require some reboots.
After that, you have to install (in this order):

* Kinect SDK 1.8
* Kinect Develper Kit 1.8

Finally, simply plug-in the Kinect and let Windows Update to install its drivers.

Installing Tango
================

Go to http://www.tango-controls.org/download and select the binary distribution for
Windows 64 bits. Download and istall it.

After the installation, you will be able to access to a lot of utility and tools to get
information about Tango and the device servers (e.g. *Jive*). To use them, you must install
*Java for Windows*; you can get it from https://java.com/download/

Configure Tango Host
--------------------

To be able to get all Tango informations, you need to specify the address of the Tango host.
Assuming that it is 198.168.1.100:10000, open the command line and type:

    set TANGO_HOST=192.168.1.100:10000
    
Installing and configuring XAMPP
================================

The last software we need is *XAMPP*. Install it in ``C:\xampp``.
After having the installation, run it and click on the *Config* button in the
*Apache* row. Then select *Apache (http.conf)*, and be sure that the *DocumentRoot* section
is like the follwing::

    #
    # DocumentRoot: The directory out of which you will serve your
    # documents. By default, all requests are taken from this directory, but
    # symbolic links and aliases may be used to point to other locations.
    #
    DocumentRoot "C:/xampp/htdocs"
    <Directory "C:/xampp/htdocs">
        #
        # Possible values for the Options directive are "None", "All",
        # or any combination of:
        #   Indexes Includes FollowSymLinks SymLinksifOwnerMatch ExecCGI MultiViews
        #
        # Note that "MultiViews" must be named *explicitly* --- "Options All"
        # doesn't give it to you.
        #
        # The Options directive is both complicated and important.  Please see
        # http://httpd.apache.org/docs/2.4/mod/core.html#options
        # for more information.
        #
        Options Indexes FollowSymLinks Includes ExecCGI

        #
        # AllowOverride controls what directives may be placed in .htaccess files.
        # It can be "All", "None", or any combination of the keywords:
        #   AllowOverride FileInfo AuthConfig Limit
        #
        AllowOverride All

        #
        # Controls who can get stuff from this server.
        #
        Require all granted
    </Directory>

Now, go to ``C:/xampp/htdocs``, and here create a folder, naming it ``Joints``.
Then create four subfolders of the latter, and name them ``eras-1``, ``eras-2``,
``eras-3`` and ``eras-4``.

Finally, come back to XAMPP and click on the *Start* button in the
*Apache* row.

Using a virtual machine manager
===============================

Can be very useful to use a virtual machine manager like *VMWare* or *VirtualBox*, in
order to install Ubuntu or another Linux distribution on the same Windows machine.
If you want to do this, you are probably interested in setting up a shared folder between
host and guest operating systems.
To do this in VMWare, see http://askubuntu.com/questions/29284/how-do-i-mount-shared-folders-in-ubuntu-using-vmware-tools/41386#41386
To do this in VirtualBox, see http://my-wd-local.wikidot.com/otherapp:configure-virtualbox-shared-folders-in-a-windows-ho

