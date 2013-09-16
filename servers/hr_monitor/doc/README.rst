======================================
Heart Rate Monitor - Setup and Running
======================================

:Author: Mario Tambos


1. Change Record
================

2013.09.11 - Document created.

2. Prerequisites
================

* Python 2.7
* Tango Controls v7.2.2
* MySql Server
* Python modules:
   * numpy
   * scipy
   * pandas
   * matplotlib
   * PyTango v7.2.2
   * sqlalchemy
   * wxpython
   * wxmplot

2.1 Installing Prerequisites in Ubuntu System
---------------------------------------------

2.1.1 Python 2.7
~~~~~~~~~~~~~~~~

Python 2.7 comes pre-installed, but just in case you can install it with:

::

   sudo apt-get install python2.7

2.1.2 Tango Controls v7.2.2, PyTango and MySql
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install these three components following the `Tango Setup`_ guide.

Besides that, the HR Monitor needs the MySql-Python connector and
a local MySql instance with a database named hr_monitor;
it also needs access with user 'root' and blank password.

To fullfill these requirements, open a Terminal and type:

::

   sudo apt-get install python-mysqldb
   mysql -u root
   > create database hr_monitor;
   > show databases;

The first line will open the MySql console, the second will create the database
and the third will show you the existing database, to confirm everythong is OK.

.. _`Tango Setup`: https://eras.readthedocs.org/en/latest/doc/setup.html

2.1.3 Numpy, Scipy, Pandas and Matplotlib
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Since these three modules rely on C libraries, it's recommended to install
them using apt-get instead of easy_install/pip.
They should have been installed during the PyTango installation, but if not:

::

   sudo apt-get install python-numpy python-scipy python-matplotlib
   sudo pip install pandas

2.1.3 SqlAlchemy
~~~~~~~~~~~~~~~~

You can install it from PyPi with:

::

   sudo apt-get install python-pip
   sudo pip install SQLAlchemy

2.1.3 wxPython and wxmplot
~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install wxPython following the `wxPython Installation`_ guide.

.. _`wxPython Installation`: http://wiki.wxpython.org/InstallingOnUbuntuOrDebian


To install wxmplot just open a Terminal and write:

::

   sudo easy_install -U wxmplot


3. Running the hr_monitor Tango Server
======================================

First of all, you need to register both the Aouda and HR Monitor Tango Servers.
To do it, just follow `this guide`_. In both cases the class name is 'PyDsExp',
without quotation marks.

.. _`this guide`: https://eras.readthedocs.org/en/latest/doc/setup.html#adding-a-new-server-in-tango

Second, you'll have to run the Aouda Tango Server.
This server will simmulate the Aouda Suit, making data available
for the HR Monitor to consume. To do this, just open a Terminal and type:

::

   cd /path/to/hr_monitor/src
   python aouda [instance name]

Next you need to configure Aouda Server's Tango Device Name in the Monitor
configuration file (hr_monitor.cfg);
the variable you need to modify is "aouda_address".
Once done, you can start the HR Monitor Server itself with:

::

   python hr_monitor [instance name]

The simmulation has data available for only 45 minutes. After that the Aouda
Server will shut down, so you'll need to start it again.

Now if you want to see the alarm levels, you can do it by starting
the GUI prototype.
First you need to configure the HR Monitor's Tango Device Name
in the GUI configuration file (gui/hr_monitor_gui.cfg);
the variable you need to modify is "monitor_address".
Once done, just type the following in a Terminal:

::

   export TANGO_HOST=[IP:Port of the Tango central server]
   cd /path/to/hr_monitor/src/gui
   python app.py

To start collecting data from the HR Monitor Server,
just press the "Collect data" button.




