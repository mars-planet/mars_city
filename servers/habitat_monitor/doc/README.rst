.. sectnum:: :start: 1

===================================================
Habitat Monitor Software User and Maintenance Manual
===================================================

:Author: Ambar Mehrotra

.. contents:: :local:

Introduction
============

Purpose
-------

This document describes the installation, use and maintenance of the Health
Monitor.

Applicable Documents
--------------------

- [1] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [2] -- `Software Architecture Document for the Habitat Monitor`_
- [3] -- `TANGO distributed control system`_
- [4] -- `PyTANGO - Python bindings for TANGO`_
- [5] -- `Tango Setup`_
- [6] -- `Qt Installation`_
- [7] -- `PyQt Installation`_
- [8] -- `Adding a new Server in Tango`_

.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`Software Architecture Document for the Habitat Monitor`: <http://eras.readthedocs.org/en/latest/servers/habitat_monitor/doc/sad.html>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`PyTANGO - Python bindings for TANGO`: <http://www.tango-controls.org/static/PyTango/latest/doc/html/index.html>
.. _`Tango Setup`: https://eras.readthedocs.org/en/latest/doc/setup.html
.. _`Qt Installation` : http://www.wikihow.com/Install-Qt-SDK-on-Ubuntu-Linux
.. _`PyQt Installation`: http://www.saltycrane.com/blog/2008/01/how-to-install-pyqt4-on-ubuntu-linux/
.. _`Adding a new Server in Tango`: https://eras.readthedocs.org/en/latest/doc/setup.html#adding-a-new-server-in-tango

Glossary
--------

.. glossary::

    ``API``
        Application Programming Interface

    ``AS``
        Aouda Device Server

    ``ERAS``
        European Mars Analog Station

    ``GUI``
        Graphic User Interface

    ``HM``
        Habitat Monitor Device Server

    ``IMS``
        Italian Mars Society

    ``TBC``
        To Be Confirmed

    ``TBD``
        To Be Defined

Installation Guide
==================

The first step is to download the component to install (Health Monitor Daemon,
Aouda Daemon or Health Monitor GUI) in  the machine that is going to run it.
The components can be installed all in the same computer, all in different
computers or any combination thereof.

Installing the Central Tango Daemon on the Central Tango Server
---------------------------------------------------------------
You can install this component following the `Tango Setup`_ guide. Tango's
libraries must be installed in all computers.

Installing the Health Monitor Daemon
------------------------------------

Prerequisites
~~~~~~~~~~~~~

* Python 2.7
* Python modules:
   + python-qt4
   + numpy >= 1.8.1
   + pandas >= 0.14.0
   + pip >= 1.5.4
   + pyqtgraph
   + PyTango >= 8.1.5
* libboost-python-dev >= 1.54
* MongoDB

Python 2.7, numpy, pip and scipy
++++++++++++++++++++++++++++++++

Python 2.7 comes pre-installed, but just in case you can install it, 
together with numpy, pip and scipy, with:

::

   sudo apt-get install -y libboost-python-dev python2.7 python-pip python-numpy python-scipy


PyTango
+++++++

   sudo pip install PyTango --egg

MongoDB
+++++++

   Ubuntu Installation:
   ~~~~~~~~~~~~~~~~~~~~

   1. Import the public key used by the package management system.
   The Ubuntu package management tools (i.e. dpkg and apt) ensure package consistency and authenticity
   by requiring that distributors sign packages with GPG keys.
   Issue the following command to import the MongoDB public GPG Key:
   ::

        sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 7F0CEB10

   2. Create a list file for MongoDB.
   Create the /etc/apt/sources.list.d/mongodb-org-3.0.list list file using the following command:
   ::

       echo "deb http://repo.mongodb.org/apt/ubuntu "$(lsb_release -sc)"/mongodb-org/3.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.0.list

   3. Reload local package database.
   ::

       sudo apt-get update

   4. Install the latest stable version of MongoDB.
   ::

       sudo apt-get install -y mongodb-org

    Directly from binaries:
    ~~~~~~~~~~~~~~~~~~~~~~~
    Follow the following tutorial - http://docs.mongodb.org/manual/tutorial/install-mongodb-on-linux/


pyqtgraph:
++++++++++

    Download the appropriate installer for your Operating System and install from the following link:
        http://www.pyqtgraph.org/


