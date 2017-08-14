Mission Operations Control Center - Implementation Documentation
================================================================

:Author: Mario Tambos

.. contents::
   :local:
   :depth: 2

Change Record
-------------

2015.06.22 - Document created.

Introduction
------------

The present document means to laid the foundations for building all the
:term:`MOCC`'s software. In doing so this document will specify how the
subsystems and components defined in :ref:`[1] <[1]>` are to be implemented.

How to use this document
++++++++++++++++++++++++

This document is meant primarily for software developers. ERAS's contributors
wanting to write code for the :term:`MOCC` should first read :ref:`[1] <[1]>`.
The contributor should then read this document, to familiarize him/her-self with
the general implementation rules of the :term:`MOCC`, and finally the
document corresponding to the subsystem he/she wants to contribute for, if
available.

If a detail is left out of this document, the reader should check the subsystem
documents. Failing that, those details are left to interpretation.

Reference Documents
+++++++++++++++++++

.. _[1]:

- [1] ++ `Mission Operations Control Center - High Level Design Documentation`_

.. _[2]:

- [2] ++ `C3 Prototype document v.4`_

.. _[3]:

- [3] ++ `Software Engineering Practices Guidelines for the ERAS Project`_

.. _[4]:

- [4] ++ `Dachstein 2012 Mission Report`_

.. _[5]:

- [5] ++ `Morocco MARS2013 Mission Report`_

.. _[6]:

- [6] ++ `V-ERAS Project Description (2014 Release)`_

.. _[7]:

- [7] ++ `V-ERAS-14 Mission Report`_

.. _[8]:

- [8] ++ `TANGO Controls`_

.. _[9]:

- [9] ++ `EUROPA framework`_

.. _[10]:

- [10] ++ `PANIC - The Package for Alarms and Notification of Incidents from Controls`_

.. _[11]:

- [11] ++ `ERAS' Tango Devices' JSON Interface Definition`_

.. _`Mission Operations Control Center - High Level Design Documentation`: ./MOCC_design.html
.. _`C3 Prototype document v.4`: http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148
.. _`Software Engineering Practices Guidelines for the ERAS Project`: ./guidelines.html
.. _`V-ERAS-14 Mission Report`: http://erasproject.org/?wpdmdl=353
.. _`Dachstein 2012 Mission Report`: http://www.oewf.org/dl/Dachstein2012_MissionReport_PUBLICv2.pdf
.. _`Morocco MARS2013 Mission Report`: http://www.oewf.org/dl/724b15d5b031dbd23fff2b5063903635.pdf
.. _`V-ERAS Project Description (2014 Release)`: http://erasproject.org/?wpdmdl=341
.. _`TANGO Controls`: http://www.tango-controls.org/
.. _`EUROPA framework`: https://code.google.com/p/europa-pso/wiki/EuropaWiki
.. _`PANIC - The Package for Alarms and Notification of Incidents from Controls`: http://plone.tango-controls.org/Members/srubio/panic
.. _`ERAS' Tango Devices' JSON Interface Definition`: ./ERAS_Tango_Devices_JSON_Interface_Definition.html

Glossary
++++++++

.. glossary::

    ``AI``
        Artificial Intelligence

    ``ERAS``
        European Mars Analog Station

    ``EVA``
        Extra-Vehicular Activity

    ``GUI``
        Graphic User Interface

    ``IMS``
        Italian Mars Society

    ``MARS``
        Mission Asset and Resource Simulation

    ``MOCC``
        Mission Operations Control Center

    ``NDDL``
        New Domain Description Language

    ``PANIC``
        Package for Alarms and Notification of Incidents from Controls

    ``TBC``
        To Be confirmed

    ``TBD``
        To Be Defined

Overview
++++++++

:ref:`Section 1 <sec_1>` deals with the choice of communication channels.
Sections :ref:`2 <sec_2>`, :ref:`3 <sec_3>`, :ref:`4 <sec_4>` and
:ref:`5 <sec_5>` explain implementation restrictions and considerations of
the Planning, Operations, Resources, Telemetry and Commands subsystems,
respectively. :ref:`Section 6. <sec_6>` covers the validation and verification
procedures. Finally, :ref:`Section 7. <sec_7>` deals with miscellaneous factors
that need to be addressed or acknowledged when implementing the :term:`MOCC`.

General considerations
++++++++++++++++++++++

As with any other part of the ERAS project, the guidelines in :ref:`[3] <[3]>`
must be followed. If the need for non-Python code arises, the coding style
recommended for the language should be used instead of PEP8.

.. _sec_1:

1. Communication channels
-------------------------

As explained in :ref:`[3] <[3]>`, the communication between the :term:`MOCC`'s
components is done via TANGO (see :ref:`[8] <[8]>`). All communication should be
done though the TANGO bus. In no scenario should a component communicate outside
the bus, with the only exceptions being components that act as a proxy to
external services (databases, hardware sensors, external data providers, etc.);
in these cases the communication between the proxy and the external service
should fulfill the external service's requirements.

In the case of databases, if the data store in the database can be accessed
through a TANGO device server, direct access to the database should be avoided.

Unless otherwise specified, all components should be implemented as TANGO device
servers.

.. _sec_2:

2. Planning Subsystem
---------------------

The framework chosen to implement planning related tasks is EUROPA (see
:ref:`[4] <[4]>`). Among other things, this means that templates, instantiations
as well as macros should be created using EUROPA's :term:`NDDL` and stored using
EUROPA's Plan Database, and that the :term:`AI` assistants and :term:`UI` s
should be developed using EUROPA's API.

The **Mission Asset and Resource Simulation** (MARS) (see :ref:`[9] <[9]>`,
Section 7.5) should be also taken into account, specially when developing
:term:`EVA`-related planning agents. In cases where EUROPA and MARS conflict,
EUROPA takes precedence.

The components in this Subsystem should present an abstraction layer over
EUROPA's and MARS's APIs. Direct access from other Subsystems to said APIs
should be avoided.

.. _sec_3:

3. Operations Subsystem
-----------------------

3.1. Execution Component
++++++++++++++++++++++++

The Execution component of this subsystem should use EUROPA's API to execute the
plans defined in the Planning Subsystem. However, no component from the
Operations Subsystem should get the plans using EUROPA's API directly, but
through the Planning Subsystem's components instead. This is done to maintain a
clear separation between planning and execution/control, and to facilitate an
eventual replacement of EUROPA, if necesary in the future.

3.2. Control Component
++++++++++++++++++++++

The Control component should be able to understand :term:`NDDL`,
since the plan step's expected outcomes will be defined in that language. This
component should get the telemetry readings through the Telemetry Subsystem's
Internal Delivery component(s), and not through accessing the telemetry storage
nor the sensors directly. Since the underlying framework is TANGO, this should
involve making a request to the TANGO device server in charge of managing the
sensor of interest.

3.3. AI assistants
++++++++++++++++++

At the time of this writing, two types of :term:`AI` assistants are foreseen:

* **Anomaly Detectors**: these assistants analyze telemetry from all sensors
  involved in a plan instance, in search for deviations that could compromise
  the plan. The method of analysis is sensor-dependant, but all anomaly
  detectors should use a uniform interface to provide their analysis. This means
  defining an appropriate "Alarm" data structure. The use of :term:`PANIC` (see
  :ref:`[10] <[10]>`) is advised whenever possible.

* **Corrective Measures Assistants**: these assistants help correct deviations
  in the plans. They should make use of the tools offered by EUROPA. No other
  requirements apply at the time.

3.5. User Interfaces
++++++++++++++++++++

The :term:`UI` s in the Operations Subsystem should be organized hierarchically
in a tree, as shown in `Figure 1`_. Each node in the tree should be a
:term:`UI` widget, which performs the following functions:

#. Gathers data from a data source.
#. Shows a user-defined representation of the data gathered.
#. Provides user-defined aggregations of the data gathered, which can function
   as a data source.
#. Shows a user-defined representation of the aggregations defined.

.. figure:: images/MOCC_Operations-UI.png
  :name: Figure 1

  Figure 1. The Operations UIs hierarchy


The aggregations are functions over one or all data sources queried by a node.
So, if a :term:`UI`-node **Z** gathers data from data sources *A*, *B*
and *C*, possible aggregation functions could be, for instance:

* Maximum over the last 30 seconds of *A*.
* Average over the last hour of *B*.
* Instantaneous minimum of *A* and *C*.
* Average over the last 45 minutes of the cross-entropy of *B* and *C*.

In the tree, the leaves should gather data exclusively from telemetry sources
and :term:`AI` assistants, whereas inner nodes should gather data
*preferentially* from lower tree nodes and :term:`AI` assistants. The idea is to
build abstraction layers to facilitate control and decision making by the
Operation Subsystem's users by giving enough information while avoiding
cluttering. This way, the users in charge of physical devices can see a detailed
view of their devices, users in charge of groups of devices can see the status
of the whole group, the the person in charge of the whole mission can see the
global status.

.. _sec_4:

4. Resources Subsystem
---------------------------

4.1. Devices
++++++++++++

Based on the services provided by TANGO, the Devices component should extend
TANGO's capabilities by offering an interface to find devices based on certain
criteria, e.g.:

* Devices able to go outside.
* Devices able to load cargo.
* Devices able to measure radiation.
* Etc.

Beyond the information about a device already provided by TANGO (type and name
of variables, signature of commands), in the case of devices that offer
JSON-encoded variables and/or commands (see :ref:`[11] <[11]>`), this component
should also make available the schemas of said variables and commands.

::

    NOTE: This last part could be implemented by requiring all ERAS' TANGO
    devices to implement a commands that provides the schema information, as
    suggested in [11].


4.2. Crew
+++++++++

Similarly to the Devices component, this component should offer information
about the crew members physical well as mental characteristics, e.g.:

* Personal information: name, nationality, place of origin, etc.
* Biomedical statistics: age, weight, medical conditions, etc.
* Areas of expertise: mechanical engineering, geology, medicine, etc.
* Psychological characteristics: leadership, patience, stress tolerance, etc.

4.3. Macros
+++++++++++

Combining TANGO and EUROPA, this component should offer a repository for macros.
A user should be able to retrieve the list of macros that involve a certain
device or crew member. A user should also be able to obtain the list of devices
and crew members involved in a certain macro.

To facilitate the process of macro building, a :term:`UI` should be developed.

.. _sec_5:

5. Telemetry and Commands Subsystems
------------------------------------

5.1. Telemetry Subsystem
++++++++++++++++++++++++

The Telemetry Subsystem consists of three components -- Acquisition, Storage
and Delivery -- in charge of reading telemetry from sensors, storing it in a
database and making it available to whoever needs it. In particular, the
delivery can be in real-time, in the case when the Subsystem delivers telemetry
as it is obtained; or historic, in the case when previously obtained and stored
telemetry is needed.

There are two types of subcomponents in each the Acquisition and Delivery
components. The first is the acquisition from and delivery to internal services,
i.e., services that are part of ERAS, which occurs using the TANGO bus.

The second is the acquisition from and delivery to *external* services, i.e.,
databases, hardware, third-party services, etc., which occurs using whatever
media the external services in question require. However, when the information
from the external service is requested by another Subsystem, the request is
processed through the TANGO bus, by developing a proxy (a TANGO device server)
for the external service needed.

Finally, the Storage component should provide an interface for saving telemetry
to a database, as well as retrieving it. Storage requests for this component
should not be directly made from outside the Telemetry Subsystem. One
possibility to achieve this is to implement it as a software library. However,
retrieval request should be accepted from outside sources, though the TANGO bus.

5.2. Commands Subsystem
+++++++++++++++++++++++

This Subsystem is very similar to the Telemetry Subsystem. The difference lies
in principle in the information flow; whereas the Telemetry Subsystems mainly
gathers information from external services (sensors) and delivers it to internal
services (chiefly the Operations Subsystem), the Commands Subsystem gathers
information only from the Operations Subsystem, and delivers it to internal as
well as external services (actuators, crew members, etc.).

5.3. Implementation
+++++++++++++++++++

`Figure 2`_ shows a diagram of the internal structure of a TANGO device server
that is able to gather and provide telemetry, as well as accepting and
delivering commands. Any devices involved in operations should follow this
structure.


.. figure:: images/MOCC_Telemetry-Commands_Implementation.png
  :name: Figure 2

  Figure 2. Telemetry and Commands� Modules Implementation

All commands received should be both stored in the Commands Database and
delivered to the appropriate external recipient (if needed).

Telemetry should be gathered at the instant the request is received, if the
request is for instant telemetry, or retrieved from the Telemetry Database or
TANGO attribute history buffer (whatever option is most appropriate).

.. _sec_6:

6. Validation and Verification Procedures
-----------------------------------------

:term:`TBD`

.. _sec_7:

7. Special Considerations
-------------------------

To avoid work duplication, a spreadsheet matching components to actual software
modules should be written and linked to this document. The spreadsheet should
also indicate whether a component is completely implemented, and if not, what is
missing.
