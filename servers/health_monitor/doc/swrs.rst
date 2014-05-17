
.. sectnum:: :start: 2

==========================================================
Software Requirements Specification for the Health Monitor
==========================================================

:Author: Mario Tambos

.. contents::
   :local:
   :depth: 2

Change Record
=============

2014.05.16 - Document created.

Introduction
============

Scope
-----

This document describes the top level requirements for the Health Monitor
module, which in turn is part of the Crew Mission Assistant system.


Reference Documents
-------------------

- [1] -- `C3 Prototype document v.4`_
- [2] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [3] -- `ERAS 2014 GSoC Strategic Plan`_

.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`ERAS 2014 GSoC Strategic Plan`: <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202014>


Glossary
--------

.. glossary::

    ``ERAS``
        European Mars Analog Station

    ``GUI``
        Graphic User Interface

    ``IMS``
        Italian Mars Society

    ``EVA``
        Extra-Vehicular Activity

    ``TBD``
        To Be Defined

    ``TBC``
        To Be confirmed


General Description
===================

Problem Statement
-----------------

Crew monitoring is an integral part of any manned mission. Since automated
diagnosis tools are as yet not advanced enough, there is the problem
of providing a human overseer with enough information to allow her to spot
possible health problems as soon as possible.

Taking this into account this project is divided in three parts:

#. Investigation of consumer-level biomedic devices available in the market.
#. The implementation of a service to gather health metrics of all the crew.
   A priori this project will focus on the crew performing :term:`EVA`, since,
   through their suits, their health information is readily available.
#. The implementation of a :term:`GUI`, where the collected data shall be
   summarized and presented to an overseer for evaluation.


Functional Description
----------------------

The goal of this project is to build a service that allows the central
monitoring of the crew's health. It will do this by collecting, processing and
presenting data from the crew space suits.
A data collector, implemented as a TANGO server, should gather the data from
all available space suits and store it in a database.
A GUI should then request the latest data from the database, summarize it and
present it to an overseer in a way that allows him/her to detect problems at
a glance. The GUI should also be able to be viewed from inside a
VR-environment. At the moment a possible solution to this is to stream a
computer screen/window to a surface texture in Blender, as shown in this
`video <https://www.youtube.com/watch?v=8IUU_XeXvSM>`_.

Additionally, it will be investigated what biometrics devices could be used
in VR-simulations to monitor the crew participating in it. For selected
devices a Tango server will be developed, from which then the collector will
also gather data.

Constraints
-----------

* As mentioned before, it should be possible to view the developed GUI from
  inside a virtual environment.

* The framework selected for the GUI development should be multiplatform. It
  should also have as few prerequisites as possible.

* Biometric devices:
    * Their cost should not exceed us$200.
    * They should be easy to integrate with TANGO.
    * Tey should not be cumbersome.
    * If possible each device should integrate several functions.

* The data collector should gather data from any device deemed relevant. Any
  relevant device not available should be simulated.

Interface Requirements
======================

Software Interfaces
-------------------

Communication Interfaces
~~~~~~~~~~~~~~~~~~~~~~~~

The data collector module will be implemented as a Python TANGO server,
which will expose methods to request raw as well as summarized data.
The GUI should use this interface to take information from the data collector,
avoiding direct acces to the data storage.


Development and Test Factors
============================

Standards Compliance
--------------------

The guidelines defined in [2] should be followed.

Planning
--------

The schedule is as defined in [3], with deliverables as follows:

* A TANGO server that implements the data collector.
* A GUI that presents summarized and detailed data of the crew's biometrics.
* A document describing the biometric devices selected for the project.
* A space suit simulator that integrates the real devices.
* Testing
    * Test environment to help diagnose the server's work.
    * A set of integration tests between the collector and the GUI.
    * A set of interface tests for the GUI.
* Documentation.
    * User requirements (this document).
    * Design Study document.
    * User Manual.


Use-Cases
=========


Request for current average Health
----------------------------------
The Client request the Server the biometric data of the last T seconds.

.. image:: images/UCClientRequestBiometricData.png

Actors
~~~~~~
Client: a TANGO client that makes the request.
Server: the Health Monitor TANGO server.

Priority
~~~~~~~~
High

Preconditions
~~~~~~~~~~~~~
The Server is running and its DevState is ON.

Basic Course
~~~~~~~~~~~~
#. The Client calls the appropriate method on the Server, passing T as
   argument.
#. The Server searchs the database for the appropriate records.
#. The Server returns the records found.

Alternate Course
~~~~~~~~~~~~~~~~
None

Exception Course
~~~~~~~~~~~~~~~~
#. The Client calls the appropriate method on the Server, passing T as
   argument.
#. The Server searchs the database for the appropriate records.
#. No data is available.
#. The Server returns an error.

Postconditions
~~~~~~~~~~~~~~
None

Sequence diagram
~~~~~~~~~~~~~~~~
.. image:: images/SeqClientRequestBiometricData.png


Server requests new data
------------------------
The Server reads the data of each available Aouda suit from the Framework
Software Bus, ands stores it in the database.

.. image:: images/UCServerRequestsNewData.png

Actors
~~~~~~
Server: the Health Monitor TANGO server.
Aouda Server: Tango server that provides the Aouda Suit simmulated data.

Priority
~~~~~~~~
High

Preconditions
~~~~~~~~~~~~~
The Server is running and its DevState is ON.

Basic Course
~~~~~~~~~~~~
#. The Server request new data from each available Aouda Server.
#. The Aouda Servers returns the data available.
#. The Server stores the data of each suit.

Alternate Course
~~~~~~~~~~~~~~~~
None

Exception Course
~~~~~~~~~~~~~~~~
None

Postconditions
~~~~~~~~~~~~~~
None

Sequence diagram
~~~~~~~~~~~~~~~~
.. image:: images/SeqServerRequestsNewData.png


The GUI shows overview of crew's biometrics
-------------------------------------------
The GUI gets all data from the previos T seconds, summarizes it and displays
it.

.. image:: images/UCGuiShowsOverview.png

Actors
~~~~~~
GUI: a GUI with an embedded TANGO client.
Server: the Health Monitor TANGO server.

Priority
~~~~~~~~
High

Preconditions
~~~~~~~~~~~~~
The Server is running and its DevState is ON.

Basic Course
~~~~~~~~~~~~
#. The GUI calls the appropriate method on the Server, passing T as
   argument.
#. The Server searchs the database for the appropriate records.
#. The Server returns the records found.
#. For each available suit *s*:
   #. The GUI calls the appropriate method on itself,
      in order to summarize the biometrics of *s*.
   #. The GUI calls the appropriate method on itself,
      in order to display the summarized biometrics of *s*.

Alternate Course
~~~~~~~~~~~~~~~~
None

Exception Course
~~~~~~~~~~~~~~~~
None

Postconditions
~~~~~~~~~~~~~~
None

Sequence diagram
~~~~~~~~~~~~~~~~
.. image:: images/SeqGuiShowsOverview.png


A User requests a crewmember's detailed biometrics
--------------------------------------------------
A user requests the detailed biometrics for a given crewmember and the GUI
complies.

.. image:: images/UCUserRequestsCrewmemberDetails.png

Actors
~~~~~~
User: a user of the GUI.
GUI: a GUI with an embedded TANGO client.

Priority
~~~~~~~~
High

Preconditions
~~~~~~~~~~~~~
The Server is running and its DevState is ON.

Basic Course
~~~~~~~~~~~~
#. The User clicks on the icon of crewmember *c*.
#. The GUI hides the summarized view for *c*.
#. The GUI shows the detailed  view for *c*.

Alternate Course
~~~~~~~~~~~~~~~~
None

Exception Course
~~~~~~~~~~~~~~~~
None

Postconditions
~~~~~~~~~~~~~~
None

Sequence diagram
~~~~~~~~~~~~~~~~
.. image:: images/SeqUserRequestsCrewmemberDetails.png
