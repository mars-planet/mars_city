===========================================
Software Design Study for the <Application>
===========================================

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

Software Requirement Specification for the <Application>

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

Describe any goals, guidelines, principles, or priorities, which dominate
or embody the design of the system's software. Such goals might be:

* Emphasis on speed versus memory use
* Keeping it simple and easy to maintain, extend
* working, looking, or "feeling" like an existing product

For each such goal or guideline, unless it is implicitly obvious, describe
the reason for its desirability. Feel free to state and describe each goal
in its own sub subsection if you wish.


Software Architecture
=====================

This section should provide a high-level overview of how the functionality
and responsibilities of the system were partitioned and then assigned to
subsystems or components. Don't go into too much detail about the individual
components themselves (there is a subsequent section for detailed component
descriptions). The main purpose here is to gain a general understanding of
how and why the system was decomposed, and how the individual parts work
together to provide the desired functionality.

At the top-most level, describe the major responsibilities that the software
must undertake and the various roles that the system (or portions of the
system) must play. Describe how the system was broken down into its
components/subsystems (identifying each top-level component/subsystem
and the roles/responsibilities assigned to it). Describe how the higher-level
components collaborate with each other in order to achieve the required
results. Don't forget to provide some sort of rationale for choosing this
particular decomposition of the system (perhaps discussing other proposed
decompositions and why they were rejected).

If there are any diagrams, models, flowcharts, documented scenarios or
use-cases of the system behavior and/or structure, they may be included here.
Detailed diagrams that describe a particular component or class should
be included within the particular subsection that describes that component
or subsystem.


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
