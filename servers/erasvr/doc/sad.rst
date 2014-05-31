=========================================================
Software Architecture Document for the ERAS VR Simulation
=========================================================

:Author: Alexander Demets


Change Record
=============

23\ :sup:`th`\  May, 2014 - Document created.


Introduction
============

Purpose
-------

Implementation of an interactive VR simulation of the Eras Mars Station.

A user will be represented by a virtual avatar, he fully controlls and with which he can interact with the environment. Using an Oculus Rift device, the user will get an immersive represantation of the ERAS Mars station, in which he can walk, look around and interact with.

Blender will be used for the development and its Blender Game Engine for the real time simulation.

Goal of the project is the implementation of full Oculus Rift support for the Blender Game Engine using the official Oculus SDK and a simulation in the BGE to demonstrate the the ERAS station.


Scope
-----

Describes the scope of this requirements specification.

Applicable Documents
--------------------

- [1] -- 'Oculus Developer Platform'_
- [2] -- 'Oculus SDK Overview v0.3.2'_
- [3] -- 'Blender Python API'_

.. _'Oculus Developer Platform': https://developer.oculusvr.com
.. _'Oculus SDK Overview v0.3.2': http://static.oculusvr.com/sdk-downloads/documents/Oculus_SDK_Overview_0.3.2_Preview2.pdf
.. _'Blender Python API': http://www.blender.org/documentation/blender_python_api_2_70_5/

Reference Documents
-------------------
- [1] -- `C3 Prototype document v.4`_
- [2] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [3] -- `ERAS 2014 GSoC Strategic Plan`_

.. _`C3 Prototype document v.4`: http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148
.. _`Software Engineering Practices Guidelines for the ERAS Project`: https://eras.readthedocs.org/en/latest/doc/guidelines.html
.. _`ERAS 2014 GSoC Strategic Plan`: https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202014

Glossary
--------

Overview
--------

Make an overview in which you describe the rest of this document the and which chapter is primarily of interest for which reader.


Architectural Requirements 
==========================

This section describes the requirements which are important for developing the software architecture.

Non-functional requirements
---------------------------

Hardware:
	#. Fast GPU: For VR a steady framerate of at least 60 frames/second is needed for acceptable simulation
	#. Oculus Rift device: a virtual reality headset is needed for proper headtracking and full immersion
Software:
	#. A working installation of Blender for development
	#. Ubuntu >13.10



Use Case View (functional requirements)
---------------------------------------

#. First person character controller with adjustable height and various input methods (keyboard, joystick, external hardware)
#. Oculus SDK integration into Blender for:
	#. Sensor Fusion head-tracking
	#. Barrel distortion rendering
#. Integration with hand tracking project

Interface Requirements
======================

This section describes how the software interfaces with other software products
or users for input or output. Examples of such interfaces include library
routines, token streams, shared memory, data streams, and so forth.

User Interfaces
---------------

Describes how this product interfaces with the user.

GUI (Graphical User Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Describes the graphical user interface if present. This section should include
a set of screen dumps or mockups to illustrate user interface features.
If the system is menu-driven, a description of all menus and their components
should be provided.

CLI (Command Line Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Describes the command-line interface if present. For each command, a
description of all arguments and example values and invocations should be
provided.

API (Application Programming Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Describes the application programming interface, if present. Foreach public
interface function, the name, arguments, return values, examples of invocation,
and interactions with other functions should be provided. If this package is a
library, the functions that the library provides should be described here
together with the parameters.

Hardware Interfaces
-------------------

Oculus Rift VR headset, for documentation see [1] & [2]

Software Interfaces
-------------------

A high level description (from a software point of view) of the software
interface if one exists. This section can refer to an ICD (Interface Control
Document) that will contain the detail description of this interface.

Communication Interfaces
------------------------

Describe any communication interfaces that will be required.


Performance Requirements
========================

Specifies speed and memory requirements.

Logical View 
============
Describe the architecturally significant logical structure of the system. Think of decomposition in terms of layers and subsystems. Also describe the way in which, in view of the decomposition, Use Cases are technically translated into Use Case Realizations

Layers
------
The ERAS software applicationg belong to the heterogeneous Distributed Control System (DCS) domain  which can be represented as a layered architecture. This is a very common design pattern used when developing systems that consist of many components across multiple levels of abstraction as in ERAS case. Normally, you should be developing components that belong to the Application layer

.. image:: layered.jpg

Subsystems
----------
Describe the decomposition of the system in subsystems and show their relation.

Use Case Realizations
---------------------
Give examples of the way in which the Use Case Specifications are technically translated into Use Case Realizations, for example, by providing a sequence-diagram.

Implementation View
===================
This section describes the technical implementation of the logical view.

Deployment View
===============
Describe the physical network and hardware configurations on which the software will be deployed. This includes at least the various physical nodes (computers, CPUs), the interaction between (sub)systems and the connections between these nodes (bus, LAN, point-to-point, messaging, etc.). Use a deployment diagram.


Development and Test Factors
============================

Hardware Limitations
--------------------

Describe any hardware limitations if any exist.

Software validation and verification
------------------------------------

Give a detail requirements plan for the how the software will be tested and
verified.

Planning
--------

Describe the planning of the whole process mentioning major milestones and
deliverables at these milestones.




Notes
=====

.. notes can be handled automatically by Sphinx


Appendix A: Use Case template
=============================

Use Cases drive the whole software process and bind together all the phases
from requirements capture to final delivery of the system and maintenance.
They are a very effective way of communicating with customers and among team
members. Before every discussion always provide the partners with a set of
relevant Use Cases.

During meetings, they stimulate focused discussions and help identifying
important details. It is important to keep in mind that Use Cases have to
describe WHAT the system has to do in response to certain external stimuli
and NOT HOW it will do it. The HOW is part of the architecture and of the
design.

What follows is the empty template:

Use Case: <Name>
================
<Short description>

Actors
------
<List of Actors>

Priority
--------
<Low, Normal, Critical>

Preconditions
-------------
<List of preconditions that must be fulfilled>

Basic Course
------------
<Step-by-step description of the basic course>

Alternate Course
----------------
<Step-by-step description of the alternate course>

Exception Course
----------------
<Step-by-step description of the exception course>

Postconditions
--------------
<List of postconditions (if apply)>

Notes
-----

