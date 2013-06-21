================================================================
Software Requirements Specification for the Neuro Headset Server
================================================================

:Author: Ezio Melotti


Change Record
=============

.. If the changelog is saved on an external file (e.g. in servers/sname/NEWS),
   it can be included here by using (dedent to make it work):

   .. literalinclude:: ../../servers/servername/NEWS


Introduction
============

Purpose
-------

This document describes the software requirements specification for the neuro
headset server.

Scope
-----

Describes the scope of this requirements specification.

Applicable Documents
--------------------

Reference Documents
-------------------

Glossary
--------

.. To create a glossary use the following code (dedent it to make it work):

  .. glossary::

     ``Term``
        This is a sample term

.. Use the main :ref:`glossary` for general terms, and :term:`Term` to link
   to the glossary entries.

Overview
--------

The package will initially provide low-level access to the data sent by the
neuro headset.  The package will also provide training software and ways to
define an higher-level interface used to control several different devices.


General Description
===================

Problem Statement
-----------------

In a manned mission on Mars, astronauts need to control a number of devices
(e.g. a Mars rover) but often have limited mobility.  Ideally, the best
approach consists in an hand-free input control, such as voice command or
brain waves.  These input methods would allow astronauts to operate devices
without needing specific hardware (e.g. a joystick), and even in situations
of limited mobility (e.g. while wearing a space suit).
While these input methods clearly have advantages, they might not be as
accurate as traditional input methods.

Functional Description
----------------------

The package will offer an interface between Tango and the EPOC neuro headset.
Developers can use then use the data provided by the package to control
different kind of devices.

Environment
-----------

The neuro headset can be used in the habitat or even during EVAs while
wearing a space suit, but will requires a nearby computer that will receive
and process the signal.

User objectives
---------------

User1
~~~~~

Describe all the users and their expectations for this package

Constraints
-----------

Describe any constraints that are placed on this software.


Functional Requirements
=======================

This section lists the functional requirements in ranked order. Functional
requirements describe the possible effects of a software system, in other
words, what the system must accomplish. Other kinds of requirements (such as
interface requirements, performance requirements, or reliability requirements)
describe how the system accomplishes its functional requirements.
Each functional requirement should be specified in a format similar to the
following:

Requirement
-----------

Description
~~~~~~~~~~~

Criticality
~~~~~~~~~~~

* High | Normal | Low

Dependency
~~~~~~~~~~

Indicate if this requirement is dependant on another.


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


CLI (Command Line Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


API (Application Programming Interface)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Tango server will provide a low-level API to access the raw data for
the neuro headset.  This low level API, in combination with a training
software, will be used to create higher-level APIs, possibly as new servers.
The actual APIs are still to be determined.

Diagnostics
~~~~~~~~~~~

Describes how to obtain debugging information or other diagnostic data.

Hardware Interfaces
-------------------

A high level description (from a software point of view) of the hardware
interface if one exists. This section can refer to an ICD (Interface Control
Document) that will contain the detail description of this interface.

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


Development and Test Factors
============================

Standards Compliance
--------------------

Mention to what standards this software must adhere to.

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


Use-Case Models
===============

If UML Use-Case notation is used in capturing the requirements, these models
can be inserted and described in this section. Also providing references in
paragraphs 5, 6 and 7 where applicable.


Notes
=====

.. notes can be handled automatically by Sphinx


Appendix A: Use Case template
=============================

Use Case: Controlling a rover with the neuro headset
====================================================

The user wants to control a rover using the neuro headset.


Actors
------
User, rover.

Priority
--------
Normal

Preconditions
-------------

The user should be wearing a charged neuro headset and be within wireless
range of a computer that will receive and process the data, making them
available on Tango.  The user might be required to do a training before
being able to use the neuro headset successfully.

Basic Course
------------

1. If the user didn't do the training yet, he should do it in order to
   associate specific thoughts to specific movements.
2. After the training, he should be able to just think at the movements the
   rover should do.
3. The server will process the inputs sent by the headset and convert them
   to higher level signals, accoding to the data collected during the
   training.
4. The higher level signals can be accessed by other servers (e.g. the rover
   server) and used to determine what actions should be taken.

Alternate Course
----------------

None

Exception Course
----------------

If any of the preconditions are not met, the user should make sure to
address the problems before continuing with the basic course.

Postconditions
--------------

At the end of the session the user should turn off and remove the neuro
headset, and recharge it if needed.

Notes
-----

None
