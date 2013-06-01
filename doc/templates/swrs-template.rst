=========================================================
Software Requirements Specification for the <XXX package>
=========================================================

:Author: Name Surname


Change Record
=============

.. If the changelog is saved on an external file (e.g. in servers/sname/NEWS),
   it can be included here by using (dedent to make it work):

   .. literalinclude:: ../../servers/servername/NEWS


Introduction
============

Purpose
-------

Describes the purpose of the document, and the intended audience.

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

Provides a brief overview of the package defined as a result of the
requirements elicitation process.


General Description
===================

Problem Statement
-----------------

This section describes the essential problem(s) currently confronted by the
user community. In other words, this section should discuss what purpose this
software package fulfills.

Functional Description
----------------------

Describes the general functionality of the software, which will be discussed
in more detail below.

Environment
-----------

Describes the environment in which this software will function.

User objectives
---------------

User1
~~~~~

Describe all the users and there expectations for this package

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

Use Cases drive the whole software process and bind together all the phases from requirements capture to final delivery of the system and maintenance. They are a very effective way of communicating with customers and among team members. Before every discussion always provide the partners with a set of relevant Use Cases.

During meetings, they stimulate focused discussions and help identifying important details. It is important to keep in mind that Use Cases have to describe WHAT the system has to do in response to certain external stimuli and NOT HOW it will do it. The HOW is part of the architecture and of the design.

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

