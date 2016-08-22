=========================================================
Software Architecture Document for the Heads Up Display
=========================================================

:Author: Aakash Rajpal


Change Record
=============
20th August 2016- Document Created


Introduction
============

Purpose
-------

The purpose of this project is to implement a Heads Up Display for the V-ERAS environment which displays data from the Habitat Monitor Client and use the gesture recognition provided by leap motion controller to make the Heads Up Display interactive.

Scope
-----

Describes the scope of this requirements specification.

Applicable Documents
--------------------
- [1] -- `Leap Motion Controller Installation for Python 3.x`_
- [2] -- `How to Pytango`_
- [3] -- `PyQt4 Reference Guide`_

.. _`How to use Tango Controls`: https://community.leapmotion.com/t/leap-motion-sdk-with-python-3-5-in-linux-tutorial/5249
.. _`How to PyTango`: http://www.tango-controls.org/resources/howto/how-pytango/
.. _`PyQt4 Reference Guide`: http://pyqt.sourceforge.net/Docs/PyQt4/

Reference Documents
-------------------
`Habitat Monitoring Client User Manual <https://eras.readthedocs.io/en/latest/servers/habitat_monitor/doc/README.html>`_

`Habitat Monitoring Client Software Architecture <https://eras.readthedocs.io/en/latest/servers/habitat_monitor/doc/sad.html>`_

`ERAS VR Software Architecture Document <https://eras.readthedocs.io/en/latest/servers/erasvr/doc/sad.html>`_


Glossary
--------

``HUD``
    Heads Up Display

``ERAS``
    European MaRs Analogue Station for Advanced Technologies Integration

``IMS``
    Italian Mars Society

``V-ERAS``
    Virtual European Mars Analog Station

``VR``
    Virtual Reality

``bpy``
    Blender Python API


Overview
--------

This document will guide you through the requirements of the project.


Architectural Requirements 
==========================

This section describes the requirements which are important for developing the software architecture.

Non-functional requirements
---------------------------

#. Implementation Constraints
       #. Language
            The application should be written in python.
       #. Operating System
            The application should be run on Ubuntu distributions.
       #. Software
            PyQt Library
            PyTango Library
            Tango server(pyTango),
            Python 2.x,
            Pep8,
            Blender 2.7x,
            Leap Linux 2.2.3SDK

#. Supportability
      #. Ease of Installation
           System requires installation of PyQt and Tango
           server.

Use Case View (functional requirements)
---------------------------------------

The goal of this project is to build an interactive HUD which receives data from the Habitat Monitoring client as requested from the user and displays the data on the V-ERAS environment.


