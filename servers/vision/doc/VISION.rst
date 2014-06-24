===========================================
Computer Vision on the Trevor Rover  
===========================================

:Author: Mathew Kallada


Change Record
=============

| 2014.06.24 - Document updated.
| 2014.05.21 - Document created.

Introduction
============

Purpose
-------

Computer vision capabilities are crucial in a rover helping it to analyze and 
understand its enviroment. This document outlines the key features of the 
computer vision features of the Italian Mars Society's Trevor Rover.


Applicable Documents
--------------------

- [1] -- `C3 Prototype document v.4`_
- [2] -- `OpenCV`_
- [3] -- `Software Engineering Practices Guidelines for the ERAS Project`_
- [4] -- `ERAS 2013 GSoC Strategic Plan`_
- [5] -- `Marscape Scenario`_
- [6] -- `TANGO distributed control system`_
- [7] -- `PyTANGO - Python bindings for TANGO`_
- [8] -- `Minoru 3D Webcam`_
- [9] -- `V-ERAS`_
- [10] -- `EUROPA Planning Software`_
- [11] -- `Histogram of Oriented Gradients`_
- [12] -- `Principal Component Analysis`_
- [13] -- `Denisty-based scan`_

.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`OpenCV`: <http://docs.opencv.org/modules/refman.html>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`ERAS 2013 GSoC Strategic Plan`: <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202013>
.. _`Marscape Scenario`: <http://code.google.com/p/europa-pso/wiki/ExampleRover>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`Minoru 3D Webcam`: <http://en.wikipedia.org/wiki/Minoru_3D_Webcam>
.. _`V-ERAS`: <http://www.spacerenaissance.it/wp-content/uploads/2014/03/DelMastro-VERAS.pdf>
.. _`EUROPA Planning Software`: <http://code.google.com/p/europa-pso/wiki/EuropaWiki>
.. _`Histogram of Oriented Gradients`: <http://www.vlfeat.org/overview/hog.html>
.. _`Principal Component Analysis`: <https://www.ce.yildiz.edu.tr/personal/songul/file/1097/principal_components.pdf>
.. _`Denisty-based scan`: <http://staffwww.itn.liu.se/~aidvi/courses/06/dm/Seminars2011/DBSCAN(4).pdf>

Glossary
--------

.. glossary::


    ``CV``
        Computer Vision

    ``API``
        Application Programming Interface

    ``ERAS``
        European Mars Analog Station

    ``IMS``
        Italian Mars Society

Overview
--------

This module provides a series of computer vision operations designed for navigation and exploration with the Trevor Rover. These operations include target recognition and hazard detection ultimately satisfing the requirements for the Marscape scenario described in [5].

Design Considerations
=====================

Hardware Requirements
----------------------------
We will be using the Minoru3D Webcam and the RaspberryPi. In case of fast 
moving objects, we will need to optimize the speed of the Minoru+RPi.

Software Requirements
----------------------------

Object Recognition and Tracking
--------
We will use scikit-learn for machine learning (predicting which objects 
it has previously seen), and OpenCV2 for image analysis (creating disparity 
fields, locating images).

Interface Requirements
----------------------------

Hardware Interfaces
--------

We will be using the Minoru 3D webcam and the RaspberryPi for computer vision
processing and AI planning and scheduling.

User Interfaces
--------

To add human reasoning into the rover's decision making abilities, 
there will be an interface to allow operators to specify properties of
previously seen objects.

Software Interfaces
--------

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/9d44b4992114703c17d527b2299413f5641ca9db/servers/vision/doc/Images/SA.png

An inputted image is sent to several tasks for processing. These tasks include 
object recognition and depth detection. Once we retrieve this information, we 
can infer conclusions such as hazards nearby, and finally send this data to 
the EUROPA system ([10]).

Performance Requirements
----------------------------

Ideally, the rover will want to interact and respond to it's enviroment in real 
time.


Software Design
===============

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/9d44b4992114703c17d527b2299413f5641ca9db/servers/vision/doc/Images/CD.png


High-level view of Object Recognition

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/a6a9815420161a89065421be5786981300a74be5/servers/vision/doc/Images/IR.png

This module takes a HOG representation ([11]) of each object on screen. Below, I
 have collected a series of objects and have shown ([12]) the dataset to 
 two-dimensions (with PCA).

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/4afa68b5bec747daa40b1cc18420f806cb6f1d74/servers/vision/doc/Images/IR_data.png

Each color represents a different cluster (found by DBSCAN as described in [13]).
Each cluster represents an object on screen. This way, we can recognize objects
we have seen earlier (the triangle is an object we are trying to predict).


Development and Progression
----------------------------

Standards Compliance
--------
The guidelines defined in [3] should be followed.


Planning
--------

A high level schedule is shown below.

- Milestone I: Finish Object Recognition & Target Tracking
- Milestone II: Enviroment Analysis

[Midterm Evaluation]

- Milestone II: Integrate with pyEUROPA
- Milestone IV: Integrate with the Waldo interface
