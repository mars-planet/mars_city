
.. sectnum:: :start: 3

============================================
Software Design Study for the Health Monitor
============================================

:Author: Mario Tambos

.. contents:: :local:

Change Record
=============

2014.05.16 - Document created.

Introduction
============

Scope
-----

This document describes the top level requirements for the Health Monitor
module, which in turn is part of the Crew Mission Assistant system.

Applicable Documents
--------------------

- [1] -- `C3 Prototype document v.4`_
- [2] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [3] -- `ERAS 2014 GSoC Strategic Plan`_
- [4] -- `Software Requirements Specification for the Health Monitor`_
- [5] -- `TANGO distributed control system`_
- [6] -- `PyTANGO - Python bindings for TANGO`_


.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`ERAS 2014 GSoC Strategic Plan`: <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202014>
.. _`Software Requirements Specification for the Health Monitor`: <https://eras.readthedocs.org/en/latest/servers/health_monitor/doc/swrs.html>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`PyTANGO - Python bindings for TANGO`: <http://www.tango-controls.org/static/PyTango/latest/doc/html/index.html>


Glossary
--------

.. glossary::

    ``AD``
        Anomaly Detection

    ``API``
        Application Programming Interface

    ``ERAS``
        European Mars Analog Station

    ``IMS``
        Italian Mars Society

    ``HRM``
        Health Monitor

    ``TBC``
        To Be Confirmed

    ``TBD``
        To Be Defined


Design Considerations
=====================

TBD

Assumptions and dependencies
----------------------------

The :term:`HRM` is to be programmed as a TANGO server, in the Python language.
As such its primary dependencies are:

* The Python language.
* The TANGO distributed control system (see [5]).
* The PyTango bindings (see [6]).
* NumPy and Pandas, respectively a scientific computation and a data analysis
  library for Python.
* SqlAlchemy, a ORM for Python.

General Constraints
-------------------

* Guidelines defined in [2].
* Requirements described in [4].

Objectives
----------

TBD


Software Architecture
=====================

TBD


Software Design
===============

TBD
