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

- [1] -- `GOES Data warehouse. <http://www.swpc.noaa.gov/ftpmenu/warehouse.html>`_
- [2] -- `SXI Solar X-ray imager. <http://www.swpc.noaa.gov/sxi/index.html>`_
- [3] -- `Software Engineering Practices Guidelines. <https://eras.readthedocs.org/en/latest/doc/guidelines.html>`_
- [4] -- `ERAS 2013 GSoC Strategic Plan. <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202013>`_
- [5] -- `Solar Dynamics Observatory Data. <http://sdo.gsfc.nasa.gov/data/>`_
- [6] -- `Example DSD. <http://www.swpc.noaa.gov/ftpdir/warehouse/2012/2012_DSD.txt>`_


Glossary
--------

.. glossary::

    ``ERAS``
        European Mars Analog Station

    ``IMS``
        Italian Mars Society

    ``GOES``
        Geostationary Operational Environmental Satellite

    ``NGDC``
	National Geophysical Data Center

    ``SDO``
        Solar Dynamics Observatory

    ``DSD``
        Daily Solar Data

    ``SESC``
	   Space Environment Services Center

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

The goal of the module is to implement the radiation forecast either using a 
deterministic approach or a machine learning(Neural Network) using the data
provided by sources like FORSPEF,PREDICCS and CACTUS.The model will give a 
heads up for SEP events and also issues an all-clear signal when the event has 
settled down.

User objectives
---------------

As of now the module has more significance in the MARS CITY project which aims
in providing a simulation of the Martian environment for preparing the crew.

Constraints
-----------

Although, the archive data for the training is readily available in [1].
The module is constrained by the continued availability of functional data
from the respective satellites.


*Functional Requirements*
====================================

.. This section lists the functional requirements in ranked order. Functional
.. requirements describe the possible effects of a software system, in other
.. words, what the system must accomplish. Other kinds of requirements (such as
.. interface requirements, performance requirements, or reliability requirements)
.. describe how the system accomplishes its functional requirements.
.. Each functional requirement should be specified in a format similar to the
.. following.:

.. Requirement
.. -----------

.. Description
.. ~~~~~~~~~~~

.. Criticality
.. ~~~~~~~~~~~

.. * High | Normal | Low

.. Dependency
.. ~~~~~~~~~~
.. Indicate if this requirement is dependant on another.


Interface Requirements
==========================

.. This section describes how the software interfaces with other software products
.. or users for input or output. Examples of such interfaces include library
.. routines, token streams, shared memory, data streams, and so forth.

*User Interfaces*
-----------------------

.. Describes how this product interfaces with the user.

Diagnostics
~~~~~~~~~~~

A validation set of data will be maintained for the diagnostic requirements.


Software Interfaces
-------------------

Communication Interfaces
~~~~~~~~~~~~~~~~~~~~~~~~

The module is to be implementhed as a Python Tango server, which issues
appropriate warnings in case of forecasted Solar storm.


Development and Test Factors
============================

Standards Compliance
--------------------

The Software Engineering Practices Guidelines for the ERAS Project in [3] to be followed.


Planning
--------

The planned steps for the design and implementation of the model :

1. Variable selection
2. Data collection
3. Data preprocessing
4. Training and validation sets
5. Neural network paradigms
6. Evaluation criteria
7. Neural network training
8. Implementation

This procedure is not a single-pass one, and may require the revisiting of
previous steps especially between training and variable selection. Although,
the implementation step is listed as last one, it is being given careful
consideration prior to collecting data.



Use-Cases
=========

Use Case: Data collection and integration
-----------------------------------------

The main focus is Data colection and preprocessing.

Actors
~~~~~~
Raw data, local database

Priority
~~~~~~~~
High

Preconditions
~~~~~~~~~~~~~
The raw data (txt files) must be downloaded on lacal machine.

Basic Course
~~~~~~~~~~~~

The raw data from the warehouse in [1] is to be parsed and the data to be
stored on local database (preferably using Mysql ). The data collected from
the txt files will be integrated in database using the date as key. An example
of the :term:`DSD` file is in [5]. Using this:

The following feature sets will be extarcted

1. Radio flux
2. :term:`SESC` Sunspot number
3. Sunspot area
4. New regions
5. X-ray background flux
6. C-forecast
7. M-forecast
8. X-forecast

The database will then be seperated into training and validation sets to be used
for the neural network training.


Alternate Course
~~~~~~~~~~~~~~~~

Although, it was initially thought of using image data from :term:`SDO` in [4]. But,
it is presently generating about 1.5TB of data daily and even the downsampled images
would require immense processing power and bandwidth (SDO is receiving about 700Mb every
36 secs). Such processing power is not currently available for this implementation.Still,
attempts wil be made to find any source of processed data access points or APIs which may
provide us the preprocessed data.


Postconditions
~~~~~~~~~~~~~~
The database split into training and validation sets.




Use Case: Neural network training
---------------------------------
The focus will be to train the neural network to classify the Solar flares.

Actors
~~~~~~
Neural network, local database

Priority
~~~~~~~~
High

Preconditions
~~~~~~~~~~~~~
The training database must be available.

Basic Course
~~~~~~~~~~~~

Using the training database, four different Neural networks will be trained
where each neural net will be trained to classify the features into a different
class ( classes to be trained for X, M, C, A&B ). Each of these neural nets will
be trained for one class only. The extracted feature set in the database will be
used to identify the class as the output.

The following Neural Network paradigms will be considered :

1. number of hidden layers
2. number of hidden neurons
3. transfer functions

Additional factors considered for the training :

1. number of training iterations
2. learning rate
3. momentum

After the training, the validation set will be used to verify the performance
for the neural network.


Alternate Course
~~~~~~~~~~~~~~~~

As an alternate course, a neural network consisting of multiple outputs to
classify the features into the respective classes can be trained. Based on
the input feature set, the output will be the corresponding class. The
performance of both the implementations can be analysed to identify the
most suitable solution.

Postconditions
~~~~~~~~~~~~~~
A trained neural network implementation.




Use Case: Solar storm warning
-----------------------------

Actors
~~~~~~
Trained neural network as server,
client that responds to warning

Priority
~~~~~~~~
Normal

Preconditions
~~~~~~~~~~~~~
The neural network has access to input data feed.

Basic Course
~~~~~~~~~~~~

The input features would be fed to the trained neural network. As the 
network has already been trained offline, the implemented neural network
should be able to provide fast response. In case of a warning, the 
relevant warning will be issued specifying the type of forecast.

Alternate Course
~~~~~~~~~~~~~~~~
None

Postconditions
~~~~~~~~~~~~~~
None
