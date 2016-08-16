========================================================================
Software Architecture Document for the Unity Plugin for Kinect Version 2
========================================================================

:Author: Shridhar Mishra


Change Record
=============

- 12 July 2016
- 15 August 2016

   .. literalinclude:: ../../servers/servername/NEWS


Introduction
============

Purpose
-------

The purpose of this project is to provide support for kinect to on Unity game engine.
In this project Kinect V2 has been used along with PyKinect 2 to get data from the Kinect device.
The data that is being retrieved from kinect include
- Hd RGB image of the user.
- Skeleton joint coordinates.

.. image:: fourplusone.jpg

Scope
-----

Describes the scope of this requirements specification.

Applicable Documents
--------------------



Reference Documents
-------------------

Glossary
--------

``C3``
    Command, Control, Communication

``ERAS``
    European MaRs Analogue Station for Advanced Technologies Integration

``IMS``
    Italian Mars Society

``V-ERAS``
    Virtual European Mars Analog Station

``VR``
    Virtual Reality


Overview
--------

This document will guide you through the requirements of the project.

Architectural Requirements 
==========================

- Kinect 2
- The user must have a Windows 8 or higher.
- PyTango and tango on Windows machine.
- Linux Machine with Tango and Pytango.

Non-functional requirements
---------------------------

TBA.

Use Case View (functional requirements)
---------------------------------------

This project can bu used to further develop advanced applications with Unity game engine as it provides the postion of
skeleton which can be used to plot the person on a virtual environment.


Interface Requirements
======================




Performance Requirements
========================

- System with USB 3.0 is a must with windows 8 or above.
- Discrete Graphics memory is a Plus.
- CPU wih with 2.5GHZ or more.





