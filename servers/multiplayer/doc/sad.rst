========================================================================
Software Architecture Document for the Multi-user Integration for V-ERAS
========================================================================

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

The main goal of this project is to add multi-user support in the V-ERAS,
by integrating the Blender simulation with the Tango framework and with the
Oculus Rift VR headset.

Scope
-----

This document will provide an high-level overview of how the multi-user
integration for V-ERAS will work and outline the possible approaches that
are being considered while developing the networking sub-system of the
V-ERAS project.

Goals
-----

The two main goals of the project are:

1) enabling network communication among several machines;
2) ensure a coherent simulation among all the instances;

Components
----------

The V-ERAS system includes different components:

* the Tango bus (used to share data among the devices over the network);
* the devices:
    * Oculus Rift (used to display the VR simulation and for head tracking);
    * Kinect (used for body tracking);
    * possibly others;
* Blender (used to run the simulation);

In a typical test environment there will be 3 or 4 users.  For each user there
will be a computer running an instance of the Blender simulation, an Oculus
Rift and a Kinect.  The computers will be connected through a gigabit local
network.

Glossary
--------

.. To create a glossary use the following code (dedent it to make it work):

  .. glossary::

     ``Term``
        This is a sample term

.. Use the main :ref:`glossary` for general terms, and :term:`Term` to link
   to the glossary entries.

Goal 1: enabling network communication
======================================


Architecture
------------

The are two main architecture:

1) server-client;
2) peer to peer;

Server-client
~~~~~~~~~~~~~

The advantages of a server-client architecture are:

* easier to maintain coherence (the servers tell the clients what to do);
* easier to solve conflicts (the server takes the decisions in case
  of conflict);

However there are disadvantages as well:

* one of the machines has to act as a server:
  * adding an additional computer will result in additional costs and network
    overhead;
  * re-using an existing machine will result in additional overhead for that
    machine;
* additional network overhead and possibly lag;
* different software required for the server and the clients;

Peer to peer
~~~~~~~~~~~~

The advantages of a peer-to-peer network are:

* less network overhead, less lag;
* same software running on all the machines;

The disadvantages are:

* more difficult to solve conflicts (the peers have to reach the same decision
  indipentendly or have to communicate before proceeding);

Depending on how the communication happen, maintaining coherence might be more
or less difficult.  On one hand, divergence among the simulation might seem
more likely without a central server, on the other hand a simple architecture
may reduce lag and result in higher coherence.


Implementation
--------------

The possible implementation being considered are:

1) Tango;
2) sockets;

Tango
~~~~~

Tango is the most obvious choice for several reasons:

* it is already deeply integrated in V-ERAS;
* it already provides many facilities;
* it doesn't require writing/using a separate system;

The main drawback is that Tango is a "passive" component, i.e. rather than
broadcasting the data to all the clients, it relies on the clients requesting
them.  This means that if data are written on the Tango bus faster than the
clients are reading or if a client is busy and delays the reading, some
data might go missing as they are overwritten by most recent data

Sockets
~~~~~~~

Using sockets is the common approach in the majority of cases, where Tango is
not used.  Sockets can be used to exchange data between the machines directly
from the Blender instances, however Blender still needs to read data from
Tango.

If a server-client architecture is used, the server could be the only machine
reading data from Tango.  The server will then process the data and send
instructions to the other clients.

If a peer-to-peer architecture is used, an hybrid approach can be considered.
In this hybrid approach, all the peers read data from Tango and sockets are
used to ensure the consistency of the simulation by having the peer exchanging
information about the objects in the simulation and verifying their
correctness.

Summary
-------

In short, these are the possible options:

1) server-client, Tango-based: the Blender instance on the server reads data
   from Tango, decides what happens in the simulation, and communicates these
   decisions to the clients through Tango;
2) server-client, socket-based: as above, but the decisions are send to the
   clients using sockets;
3) peer-to-peer, Tango-based: all the Blender instances on the peers read the
   data from Tango and update the simulation accordingly.  The peers might
   also use Tango to exchange messages;
4) peer-to-peer, hybrid (Tango + sockets): as above, but communication between
   the peers happen via sockets;


Goal 2: ensuring coherence within the simulation
================================================

Approach
--------

This problem can be addressed in two ways:

1) preemptive;
2) corrective;

A preemptive approach aims at ensuring that no incoherences are introduced in
the simulation.  This can be done by ensuring that all the blender instances
start from the same state and that they all receive exactly the same inputs.
While this could in theory be done, it is not possible to ensure that the
same inputs are received at the same time, due to external factors (e.g.
network latency, machine performance).  The simulation could however be
designed so that delays in the inputs don't cause inconsistencies.

A corrective approach aims at correcting, rather than preventing, incoherences.
A way to correct incoherences is to broadcast at regular intervals the
absolute positions of the objects, so that the clients can update their
positions in case they are not correct.

