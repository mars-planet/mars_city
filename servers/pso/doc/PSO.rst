===========================================
Software Architecture Document for the Planning and Scheduling on the Trevor Rover  
===========================================

:Author: Mathew Kallada


Change Record
=============

2014.05.21 - Document created.

Introduction
============

Purpose
-------

This module provides critical computer vision functions for the Trevor Rover. This includes primal features such as hazard detection and target tracking.

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

Reference Documents
-------------------

Glossary
--------


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

To minimize

Software Requirements
----------------------------
We will use scikit-learn for machine learning, and OpenCV2 for computer vision.

Interface Requirements
----------------------------

To add human reasoning (supervision) into the rover's decision making abilities, 
there will be a web app to allow operators to include:
- samples of objects which are hazardous (and should be avoided)

Software Interfaces
--------

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/9d44b4992114703c17d527b2299413f5641ca9db/servers/vision/doc/Images/SA.png

An inputted image is sent to several tasks for processing. These tasks include 
object recognition and depth detection. Once we retrieve this information, we 
can infer conclusions such as hazard detection, and finally send this data to 
the EUROPA system ([10]).

Software Design
===============

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/9d44b4992114703c17d527b2299413f5641ca9db/servers/vision/doc/Images/CD.png


High-level view of Object Recognition

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/a6a9815420161a89065421be5786981300a74be5/servers/vision/doc/Images/IR.png

This module takes a HOG representation ([11]) of each object on screen. Below, I
 have collected a series of objects and have PCA'ed ([12]) the dataset to 
 two-dimensions.

.. image:: https://bytebucket.org/italianmarssociety/eras/raw/9d44b4992114703c17d527b2299413f5641ca9db/servers/vision/doc/Images/CD.png

Each color represents a different cluster (found by DBSCAN as described in [13]).
Each cluster represents an object on screen. This way, we can recognize objects
we have seen earlier (the triangle is an object we are trying to predict). The
triangle clearly belongs to the blue-labelled objects. 


Planning
=====================

- Milestone I: Finish Object Recognition & Target Tracking
- Milestone II: Path Travelling Module
- Milestone II: Integrate with PyEuropa
