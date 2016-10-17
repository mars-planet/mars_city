==================================================
Software Design Study for the Neuro Headset Server
==================================================

:Author: Ezio Melotti


Change Record
=============

Introduction
============

Purpose
-------

This document describes the software design study of the neuro
headset server.

Scope
-----

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


Design Considerations
=====================

This section describes many of the issues, which need to be addressed or
resolved before attempting to devise a complete design solution.

Assumptions and dependencies
----------------------------

Describe any assumptions or dependencies regarding the software and its use.
These may concern such issues as:

* Related software or hardware
* Operating systems
* End-user characteristics
* Possible and/or probable changes in functionality

General Constraints
-------------------

Describe any global limitations or constraints that have a significant impact
on the design of the system's software (and describe the associated impact).
Such constraints may be imposed by any of the following (the list is not
exhaustive):

* Hardware or software environment
* End-user environment
* Availability or volatility of resources
* Standards compliance
* Interoperability requirements
* Interface/protocol requirements
* Data repository and distribution requirements
* Security requirements (or other such regulations)
* Memory and other capacity limitations
* Performance requirements
* Network communications
* Verification and validation requirements (testing)
* Other means of addressing quality goals
* Other requirements described in the requirements specification

Objectives
----------

The objective of the server is to provide within the TANGO infrastructure
all the data available from the neuro headset.  The server (or possibly a
different server) will also process the data and provide an higher-level
result.

These data can be in turn used by other servers.


Software Architecture
=====================

The server will provide two different API at two different levels.
At the lowest level will have the raw data read by all the EEG sensors,
including the signal intensity and signal quality.
These data will be processed and the result will be provided on an
higher-level API.

In addition to these two APIs, a third API that provides the data from the
accelerometers and possibly battery level will be provided.

These APIs might be provided by the same server or by different servers,
but this hasn't be defined yet.


Software Design
===============

The class diagrams containing all the classes can be put here.

Unit n
------

Most components described in the System Architecture section will require
a more detailed discussion. Other lower-level components and subcomponents
may need to be described as well. Each subsection of this section will refer
to or contain a detailed description of a system software component. The
discussion provided should cover the following software component attributes:

Classification
--------------

The kind of component, such as a subsystem, module, class, package, function,
file, etc.

Definition
----------

The kind of component, such as a subsystem, module, class, package, function,
file, etc.

Responsibilities
----------------

The primary responsibilities and/or behavior of this component. What does
this component accomplish? What roles does it play? What kinds of services
does it provide to its clients? For some components, this may need to refer
back to the requirements specification.

Constraints
-----------

Any relevant assumptions, limitations, or constraints for this component.
This should include constraints on timing, storage, or component state,
and might include rules for interacting with this component (encompassing
preconditions, postconditions, invariants, other constraints on input or
output values and local or global values, data formats and data access,
synchronization, exceptions, etc.)

Composition
-----------

A description of the use and meaning of the subcomponents that are a part
of this component.

Uses/Interactions
-----------------

A description of this components collaborations with other components.
What other components is this entity used by? What other components does
this entity use (this would include any side-effects this entity might
have on other parts of the system)? This concerns the method of interaction
as well as the interaction itself. Object-oriented designs should include
a description of any known or anticipated subclasses, super-classes, and
meta-classes.

Unit n+1
--------
