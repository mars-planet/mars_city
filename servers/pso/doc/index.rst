===========================================
Planning & Scheduling for the Trevor Rover
===========================================

:Author: Mathew Kallada


Change Record
=============

| 2014.06.24 - Document updated.
| 2014.05.21 - Document created.

Introduction
============

Purpose
-------

In many cases, to perform simple tasks rovers should have the capability to 
operate on their own without needing an external operator.
This document outlines the integration of NASA's EUROPA ([2]) with the
 Italian Mars Society's Trevor Rover.


Applicable Documents
--------------------

- [1] -- `C3 Prototype document v.4`_
- [2] -- `EUROPA`_
- [3] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [4] -- `ERAS 2013 GSoC Strategic Plan`_
- [5] -- `Marscape Scenario`_
- [6] -- `TANGO distributed control system`_
- [7] -- `Py4J`_

.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`EUROPA`: <code.google.com/p/europa-pso/>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`ERAS 2013 GSoC Strategic Plan`: <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202013>
.. _`Marscape Scenario`: <http://code.google.com/p/europa-pso/wiki/ExampleRover>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`Py4J`: <http://py4j.sourceforge.net/>

Glossary
--------

.. glossary::


    ``P&S``
        Planning and Scheduling

    ``API``
        Application Programming Interface

    ``ERAS``
        European Mars Analog Station

    ``IMS``
        Italian Mars Society

Overview
--------

Design Considerations
=====================

Hardware Requirements
----------------------------
We will be using the Minoru3D Webcam and the RaspberryPi. In case of fast 
moving objects, we will need to optimize the speed of the Minoru+RPi.

Software Requirements
----------------------------

EUROPA-PSO
--------
To give the Trevor rover autonomous capabilities we will be using EUROPA
which has an API in Java and in C++. To gain full access to EUROPA from Python
we will create a gateway to the Java API using Py4J.

Communication Channel Between Vision and Web Interface
--------
Redis is used to share information between the vision side to the planning side and web interface side.

Hardware Interfaces
--------
We will be using the Minoru 3D webcam and the RaspberryPi for computer vision
processing and AI planning and scheduling.

User Interfaces
--------
The WALDO interface which currently supports deleting/viewing details of plans created with this module such as the `one previously generated here <https://bitbucket.org/italianmarssociety/eras/src/036539d82d656047de631a79251a7cc9ccc4bff0/servers/pso/src/log?at=default>`.

pyEUROPA Sample Code
===============

    from pyEUROPA.psengine import makePSEngine, stopPSEngine

    # Launch & connect to EUROPA
    europa = makePSEngine()

    # Now we can interact with EUROPA like we normally would (just from Python)
    europa.start()
    europa.executeScript("nddl","some_file.nddl",True)

    # Shuts down all PSEngine instances
    stopPSEngine()

Please see the README in the pyEUROPA python package for more information.
This can be found in [servers/pso/pyEUROPA] of the ERAS repository.

Development and Progression
----------------------------

Standards Compliance
--------
The guidelines defined in [3] should be followed.


Planning
--------

A high level schedule is shown below.

- Milestone I: Create the pyEUROPA module

[Midterm Evaluation]

- Milestone II: Integrate with Trevor & Waldo Interface
- Milestone III: Finalize Integration with RaspberryPi
