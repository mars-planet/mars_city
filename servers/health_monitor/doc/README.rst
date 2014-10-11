.. sectnum:: :start: 1

===================================================
Health Monitor Software User and Maintenance Manual
===================================================

:Author: Mario Tambos

.. contents:: :local:

Change Record
=============

2014.05.16 - Document created.
2014.08.20 - Actual manual written.

Introduction
============

Purpose
-------

This document describes the installation, use and maintenance of the Health
Monitor.

Applicable Documents
--------------------

- [1] -- `C3 Prototype document v.4`_
- [3] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [4] -- `ERAS 2014 GSoC Strategic Plan`_
- [5] -- `Software Architecture Document for the Health Monitor`_
- [6] -- `TANGO distributed control system`_
- [7] -- `PyTANGO - Python bindings for TANGO`_
- [8] -- `Tango Setup`_
- [9] -- `wxPython Installation`_
- [10] -- `Adding a new Server in Tango`_

.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`PAMAP2 Physical Activity Monitoring`: <http://archive.ics.uci.edu/ml/datasets/PAMAP2+Physical+Activity+Monitoring>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`ERAS 2013 GSoC Strategic Plan`: <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202014>
.. _`Software Architecture Document for the Health Monitor`: <https://eras.readthedocs.org/en/latest/servers/health_monitor/doc/sad.html>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`PyTANGO - Python bindings for TANGO`: <http://www.tango-controls.org/static/PyTango/latest/doc/html/index.html>
.. _`Tango Setup`: https://eras.readthedocs.org/en/latest/doc/setup.html
.. _`wxPython Installation`: http://wiki.wxpython.org/InstallingOnUbuntuOrDebian
.. _`Adding a new Server in Tango`: https://eras.readthedocs.org/en/latest/doc/setup.html#adding-a-new-server-in-tango

Glossary
--------

.. glossary::

    ``AD``
        Anomaly Detection

    ``API``
        Application Programming Interface

    ``AS``
        Aouda Device Server

    ``ERAS``
        European Mars Analog Station

    ``GUI``
        Graphic User Interface

    ``HM``
        Health Monitor Device Server

    ``HMGUI``
        Health Monitor Graphical User Interface

    ``IMS``
        Italian Mars Society

    ``TBC``
        To Be Confirmed

    ``TBD``
        To Be Defined

Overview
========

Hardware Architecture
---------------------

The different hardware components that need to be taken into account are shown
in the Deployment Diagram below. As done at the moment all software components
can be run in a single computer, they can however also be run each in a
different machine.

One key assumption is that one instance of the :term:`AS`
will monitor one single Suit. In other words, one instance of the :term:`AS` is
needed for each crew member during EVA.

Software Architecture
---------------------

The components involved can be divided in five categories:
#. The Central Tango Daemon: It keeps track of the existing Tango Device
   Servers. For details refer to [7] and [8].
   In the context of deployment, the computer that runs the Central Tango
   Daemon is called "Central Tango Server".
#. The Aouda Device Server (:term:`AS`): This componen can either run on a
   Raspberry Pi and read the sensors provided by an e-Health shield, or run on
   any machine and simulate the sensors.
   In the context of deployment, the computers that run the :term:`AS` are
   called "Raspberri Py i" and the :term:`AS` themselvs are called
   "Aouda Daemon i", where i is an integer.
#. The Health Monitor Device Server (:term:`HM`): Collects data from the
   :term:`AS` and performs anomaly detection on their sensor readings.
   In the context of deployment, the computer that runs the :term:`HM` is
   called "Health Monitor Server" and the :term:`HM` itself is called
   "Health Monitor Daemon".
#. The Health Monitor Graphic User Interface (:term:`HMGUI`): allows the user
   to oversee the crew's status. It collects data from the :term:`AS` and
   the :term:`HM`.
   In the context of deployment, the computer that runs the :term:`HMGUI` is
   called "Health Monitor Workstation" and the :term:`HMGUI` itself is called
   "Health Monitor GUI".

Deployment Diagram
------------------

.. image:: images/Deployment.png

Installation Guide
==================

Installing the Central Tango Daemon on the Central Tango Server
---------------------------------------------------------------
You can install this component following the `Tango Setup`_ guide.

Installing the Health Monitor Daemon on the Health Monitor Server
-----------------------------------------------------------------
You can install this component following the `Tango Setup`_ guide.


Prerequisites
~~~~~~~~~~~~~

* Python 2.7
* Python modules:
   + inflect >= 0.2.4
   + numpy >= 1.8.1
   + pandas >= 0.14.0
   + pip >= 1.5.4
   + PyTango >= 8.1.2
   + scipy >= 0.14.0
   + SQLAlchemy >= 0.9.4

Python 2.7, numpy, pip and scipy
++++++++++++++++++++++++++++++++

Python 2.7 comes pre-installed, but just in case you can install it, 
together with numpy, pip and scipy, with:

::

   sudo apt-get install python2.7 python-pip python-numpy python-scipy

inflect, pandas, PyTango and SQLAlchemy
+++++++++++++++++++++++++++++++++++++++

inflect, pandas and SQLAlchemy can be installed the normal way:

::

   sudo pip install inflect pandas SQLAlchemy 

PyTango, however, needs an additional parameter:

::

   sudo pip install PyTango --egg

Health Monitor Daemon
~~~~~~~~~~~~~~~~~~~~~

First you need to download the latest version of the software from :term:`TBD`.
The file contains, bar the prerequisites, all needed to run the :term:`HM`,
the :term:`AS`, including the simmulated data, and the :term:`HMGUI`.
Once decompressed you need to (all paths are relative to the archive's root):

#. Configure each :term:`HRM` instance
   Now you need to configure each :term:`HRM` instance's connection string.
   To do it open each instance's configuration file (**src/hr_monitor.cfg**)
   and modify the *conn_str* variable as needed. A sample connection string
   is provided with the configuration file.
#. Register both the Aouda and HR Monitor Tango Servers:
   To do it, just follow `Adding a new Server in Tango`_.
   In both cases the class name is 'PyDsExp', without quotation marks.
#. Configure Aouda Server's Tango Device Name in each :term:`HRM`'s
   configuration file (**src/hr_monitor.cfg**); the variable you need
   to modify is *aouda_address*.
#. Configure the :term:`HRM`'s Tango Device Name in the GUI configuration
   file (**src/gui/hr_monitor_gui.cfg**); the variable you need to modify is
   *monitor_address*.

Once all this is done, all is in place to start running the programs.

Installing the Health Monitor Daemon on the Health Monitor Server
-----------------------------------------------------------------
You can install this component following the `Tango Setup`_ guide.

wxPython and wxmplot
++++++++++++++++++++

You can install wxPython following the `wxPython Installation`_ guide.

To install wxmplot just open a Terminal and write:

::

   sudo easy_install -U wxmplot


Prerequisites
~~~~~~~~~~~~~

* Python 2.7
* Python modules:
   * numpy
   * scipy
   * pandas
   * matplotlib
   * PyTango v8.1.2
   * sqlalchemy
   * wxpython
   * wxmplot



User Manual
===========

TBD

Maintenance Manual
==================

TBD

Troubleshooting
===============

TBD
