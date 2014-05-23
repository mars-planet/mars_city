===========================================
Software Design Study for the Computer Vision on the Trevor Rover  
===========================================

:Author: Mathew Kallada


Change Record
=============

2014.05.21 - Document created.

Introduction
============

Purpose
-------

This document describes the installation, use and maintenance of the computer vision module for the Trevor Rover.

Scope
-----

The scope of the vision server is to be able to track and recognize nearby objects near the Waldo Rover.

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


.. _`C3 Prototype document v.4`: <http://www.erasproject.org/index.php?option=com_joomdoc&view=documents&path=C3+Subsystem/ERAS-C3Prototype_v4.pdf&Itemid=148>
.. _`OpenCV`: <http://docs.opencv.org/modules/refman.html>
.. _`Software Engineering Practices Guidelines for the ERAS Project`: <https://eras.readthedocs.org/en/latest/doc/guidelines.html>
.. _`ERAS 2013 GSoC Strategic Plan`: <https://bitbucket.org/italianmarssociety/eras/wiki/Google%20Summer%20of%20Code%202013>
.. _`Marscape Scenario`: <http://code.google.com/p/europa-pso/wiki/ExampleRover>
.. _`TANGO distributed control system`: <http://www.tango-controls.org/>
.. _`Minoru 3D Webcam`: <http://en.wikipedia.org/wiki/Minoru_3D_Webcam>
.. _`V-ERAS`: <http://www.spacerenaissance.it/wp-content/uploads/2014/03/DelMastro-VERAS.pdf>
.. _`EUROPA Planning Software`: <http://code.google.com/p/europa-pso/wiki/EuropaWiki>

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


Assumptions and dependencies
----------------------------

Some of the dependencies for the module are OpenCV in [2], OpenCV's Python Binding's and PyTango in [5]. The PyTango library is used for the Python bindings of the Tango distributed control system. We make use of the OpenCV library along with its Python bindings for its fast computer vision algorithms.

The Minoru 3D Webcam in [8] will be used with the Trevor Rover in order to provide depth sensing capabilities.

General Constraints
-------------------

One constraint for this module is the availablility of a strong network connection. If we do not have a strong network connection, then the rover will not be able to react accordingly. Furthermore, the integration with V-ERAS in [9], for controlling the rover with the Octulus Rift will not work well (as it will not have the real-time experience).

Since running complex computer vision algorithms is computationally exhaustive, the vision module will require a certain quality of high-power ressources on the server-side.

Since, we do not have images of the 'hazards', we will store attributes of nearby objects in a database such that we can specify what the objects are later.

Objectives
----------

* Fast real-time performance

In case a rock (or some hazardous object) abruptly appears, the rover needs to act accordingly.

* Keeping it simple and easy to maintain


Software Architecture
=====================

.. image:: images/SA.png

An inputted image is sent to several tasks for processing. These tasks include object recognition and depth detection. Once we retrieve this information, we can infer conclusions such as hazard detection, and finally send this data to the EUROPA system ([10]).

Software Design
===============

.. image:: images/CD.png


Unit CameraManager
------

Classification
--------------

Class

Definition
----------

This class manages an incoming image stream from a single camera. 

Responsibilities
----------------

To provide and capture images from a given camera.

Constraints
-----------

The component is constrained on the frame-rate of its specified camera.

Composition
-----------

A description of the use and meaning of the subcomponents that are a part
of this component.

Uses/Interactions
-----------------

We can access the current frame of the camera using the frame property.


Unit Cameo
------

Classification
--------------

Class

Definition
----------

This class manages the computer vision processes.

Responsibilities
----------------

Being run on a seperate thread, this class will manage the image processing.

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

This module sends data to Tango


Unit Cameo
------

Classification
--------------

Class

Definition
----------

This class manages the computer vision processes.

Responsibilities
----------------

Being run on a seperate thread, this class will manage the image processing tasks.

Constraints
-----------

Speed will be an issue

Composition
-----------

* left_camera
** Type: class <<CameraManager>>
** Function: Control the left camera.
* right_camera
** Type: class <<CameraManager>>
** Function: Control the right camera.
* window
** Type: class <<WindowManager>>
** Function: Display images on screen for debugging/testing.

Uses/Interactions
-----------------

This module sends data to Tango


Unit WindowManager
------

Classification
--------------

Class

Definition
----------

This class manages displaying an image on a window.

Responsibilities
----------------

The primary responsibility of this class is to show an image; primarily for testing, debugging, and reporting.

Constraints
-----------



Composition
-----------

* show
** Type: function
** Function: Display the given image.
* create_window
** Type: function
** Function: Create a window.
* destroy_window
** Type: function
** Function: Destroy this window.

Uses/Interactions
-----------------

This class will have an interface such as the following

.. image:: http://robottini.altervista.org/wp-content/uploads/2012/05/bilinear1-640x279.png


Unit ObjectTrackingManager
------

Classification
--------------

Class

Definition
----------

This class manages displaying tracking the target on screen. 

Responsibilities
----------------

The primary responsibility of this class is to track the movement of a given series of regions on screen.

Constraints
-----------

Speed


Composition
-----------

* start_tracking([[x1,x2,y1,y2]...[x1,x2,y1,y2]])
** Type: function
** Function: Tracks the movements of the given regions on screen.
* stop_tracking
** Type: function
** Function: Stops tracking objects on screen.

Uses/Interactions
-----------------


Unit ObjectRecognizerManager
------

Classification
--------------

Class

Definition
----------

This class manages object recognition within the vision module. 

Responsibilities
----------------

The primary responsibility of this class is to recognize nearby objects.

Constraints
-----------


Composition
-----------

*add_new(image_of_object)
** Type: function
** Function: Pre-process and store an object using SIFT features
*recognize(image_of_object)
** Type: function
** Function: Return the cluster association of the given image
*recognize_semi(image,{labeled vectors})
** Type: function
** Function: Returns the label of the image (based on given labeled vectors).
*find_known_object(image,{labeled vectors})
** Type: function
** Function: In a large image, this function will try to recognize the smaller objects that we have previously seen.

Uses/Interactions
-----------------

Unit DepthTrackingManager
------

Classification
--------------

Class

Definition
----------

This class uses of the stereo camera to provide depth analysis for the vision module. 

Responsibilities
----------------

The primary responsibility of this class is to analyze objects on .

Constraints
-----------

Both camera's must be working in order for this class to work properly.

Composition
-----------

*compute_disparity_map(image_r_image_l)
** Type: function
** Function: Returns the disparity map given stereo images
*get_nearby_objects(distance)
** Type: function
** Function: Returns an array of images that are on screen using the disparity map

Uses/Interactions
-----------------

We use the stereo camera for two purposes:
* Understand the proximity of objects nearby
* Eventually, look around with the Octulus Rift