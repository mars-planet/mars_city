.. sectnum:: :start: 1

Mission Operations Control Center - High Level Design Documentation
===================================================================

:Author: Mario Tambos

.. contents::
   :local:
   :depth: 2

Change Record
=============

2015.06.16 - Document created.

Introduction
============

Purpose
-------

The Mission Operations Control Center (MOCC) is in charge of planning the
missions performed in European MaRs Analogue Station for Advanced Technologies
Integration (ERAS), and following said plan.

In this context, a **plan** is understood to be a sequence of activities,
together with the expected outcomes of these activities.
By **following a plan** is meant the execution of the plan, the control of its
progress as well as the counter-measures needed to correct eventual deviations
from the individual activity's expected outcomes.


Reference Documents
-------------------

- [1] -- `C3 Prototype document v.4`_
- [2] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [3] -- `V-ERAS-14 Mission Report`_

.. _`C3 Prototype document v.4`: http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148
.. _`Software Engineering Practices Guidelines for the ERAS Project`: http://erasproject.org/?wpdmdl=353
.. _`V-ERAS-14 Mission Report`: http://erasproject.org/?wpdmdl=353

Glossary
--------

.. glossary::

    ``ERAS``
        European Mars Analog Station

    ``EVA``
        Extra-Vehicular Activity

    ``GUI``
        Graphic User Interface

    ``IMS``
        Italian Mars Society

    ``MOCC``
        Mission Operations Control Center

    ``TBC``
        To Be confirmed

    ``TBD``
        To Be Defined

Overview
--------

The first section of this document describes at the highest level the
organization of the :term:`MOCC`, its communication channels, subsystems and
their general responsibilities, as well as the assumptions made during the
design process. The second section describes the documents associated with the
present one. Section 3. handles the MOCC's management structure.
Finally, Section 4. is concerned with the MOCC's infrastructure.


1. The :term:`MOCC` System and its Subsystems
=============================================

2. Documents
============

3. Management Structure
=======================

4. Infrastructure
=================
