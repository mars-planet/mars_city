=========================================================
Utilisation of the EUROPA Scheduler/Planner.
=========================================================

:Author: Shridhar Mishra


Change Record
=============

.. If the changelog is saved on an external file (e.g. in servers/sname/NEWS),
   it can be included here by using (dedent to make it work):

- Created on 27 May 2015.

Introduction
============

Purpose
-------
The main aim of this project will be making a  Astronaut’s Digital Assistant which will take into account all the constraints and rules that has been defined and plot a plan of action. It will also schedule all the tasks for the astronaut such that job of the astronaut becomes easy.

Scope
-----

TBA

Applicable Documents
--------------------

- [1] -- `C3 Prototype document v.4`_
- [2] -- `EUROPA`_
- [3] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [4] -- `TANGO distributed control system`_
- [5] -- `Py4J`_


.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`EUROPA`: <code.google.com/p/europa-pso/>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`Py4J`: <http://py4j.sourceforge.net/>

Reference Documents
-------------------

Glossary
--------

.. To create a glossary use the following code (dedent it to make it work):

  .. glossary::
  

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


.. Use the main :ref:`glossary` for general terms, and :term:`Term` to link
   to the glossary entries.


Overview
--------

.. Make an overview in which you describe the rest of this document the and which chapter is primarily of interest for which reader.


Architectural Requirements 
==========================

Linux box connected to Tango bus, which in turn would be connected to all the sensors and other devices.

.. This section describes the requirements which are important for developing the software architecture.

Non-functional requirements
---------------------------

- High speed secure network.

Use Case View (functional requirements)
---------------------------------------

It will be used for scheduling and planning of TREVOR using Pyeuropa.



Hardware Interfaces
-------------------

- Linux box with ubuntu 14.10.
- Temperature sensors  for astronauts.
- Other relevant sensors monitor health stats of astronaut.


Software Interfaces
-------------------
- Europa
- PyEuropa.
- Pytango.

Communication Interfaces
------------------------

All the necessary data for the planning and estimation will be collected from a tango bus.
All the devices shall be connected to a single tango bus, hence data can be collected and used effectively.

Performance Requirements
========================

The band width should be high enough to support real time data accusation for processing of the data and coming up with a plan.

Layers
------
TBA

Subsystems
----------
- Linux box running core Europa.
- Sensors collecting real time data.  
- Tango bus up and running.

Planning
--------

- Setup working copy of Europa on all the systems.
 
TBA

Notes
=====

.. notes can be handled automatically by Sphinx

