===================================================================
Software Requirements Specification for Radiation Forecasting Server
===================================================================

:Author: Nived Narayanan


Change Record
=============

2017.06.21 - Document created


Introduction
============

Purpose
-------

This document describes the software requirements specification for
the Radiation Forecasting Server.

Scope
-----

Describes the scope of this requirements specification.


Reference Documents
-------------------

- [1] -- `PREDICCS. <http://prediccs.sr.unh.edu/index.html#what>`_
- [2] -- `CACTUS. <http://www.sidc.oma.be/cactus/>`_
- [3] -- `FORSPEF. <http://tromos.space.noa.gr/forspef/main/>`_
- [4] -- `Software Engineering Practices Guidelines. <https://eras.readthedocs.org/en/latest/doc/guidelines.html>`_


Glossary
--------

.. glossary::

    ``ERAS``
        European Mars Analog Station

    ``IMS``
        Italian Mars Society

    ``FORSPEF``
        FORecasting Solar Particle Events and Flares TOOL

    ``SEP``
        Solar Energy Particles

*Overview*
----------

.. Provides a brief overview of the package defined as a result of the
.. requirements elicitation process.


General Description
===================

Problem Statement
-----------------

Radiation is the transmission of energy in the form of waves or sub-atomic particles.In
space missions a major concern is the particle radiation.Energetic particle radiation passing
through human body could damage the cells or DNA causing an increased risk for cancer.
Especially when out of Earth's magnetic field protection, astronauts are exposed to ionizing
radiation with doses in the range from 50 to 2,000 mSv(milli Sievert). The evidence of cancer
risk from ionizing radiation is extensive for radiation doses that are above about 50 mSv.
The module forecasts the radiation events and helps train crew members to tackle
the issue of radiation effectively in simulated environments(Mars city project).

Functional Description
----------------------

The goal of the module is to implement the radiation forecast using a 
deterministic model using the data
provided by sources like FORSPEF,PREDICCS and CACTUS.The model will give a 
heads up for SEP events and also issues an all-clear signal when the event has 
settled down.

User objectives
---------------

As of now the module has more significance in the MARS CITY project which aims
in providing a simulation of the Martian environment for preparing the crew.

Constraints
-----------

The module is constrained by the continued availability of functional data
from the respective satellites.

Functional Description
----------------------

The package takes in the continuous data stream from sources and returns 
the time of arrival as the output. In case if the event has already been
recorded by the sources the package alarms when the event has subsided.

Environment
-----------

The package currently is intended to run in simulation environments 
of the marscity project. The package can be used in the V-ERAS project
to alert throgh occulus or other VR gadgets. 

User objectives
---------------

Crew
~~~~
The user is the crew in simulated environments and the package is 
expected to give information of SEP events for preparing the crew
for the same.

Interface Requirements
==========================

.. This section describes how the software interfaces with other software products
.. or users for input or output. Examples of such interfaces include library
.. routines, token streams, shared memory, data streams, and so forth.

*User Interfaces*
-----------------------

GUI (Graphical User Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

At present a web based GUI is used for representing the streaming data 
and the alarms.In future the package could be used in VR systems.

API (Application Programming Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The falcon REST API provides the user with data as well as alarms
after analysing data from the sources.

**Alarms end point**::
    
    request         : GET
    URL             : http://localhost:8000/test 
    example output  : {u'thresholds': {u'SEP probability threshold': 0.25, u'Thin spacesuit shielding threshold': 0.068, u'Storm shelter shielding threshold': 0.068}, u'data': {u'all-clear': None, u'time of arrival': None, u'prediccs-alarm': 0}, u'time': u'2017-08-19 11:07:09.483405'}

**Data end point**::
    
    request         : GET
    URL             : http://localhost:8000/test


**Flask API for web based GUI**::
    
    request         : GET
    URL             : http://0.0.0.0:9999/
    output          : graph generated with plotly.js library and alarm data.
   
    Alarm  data description

    time of arrival

        None            :-  when the SEP probability threshold(provided by forspef) is below threshold

        Time in seconds :-  when the probability is above the threshold value

    prediccs-alarm
    
        None       :- When the radiation dosage is below the threshold 

        Warning!!! :- When the radiation dosage is below the threshold 

    
    all-clear   
    
        None       :- When there is no  event occuring

        all-clear  :- When the event has passed 


