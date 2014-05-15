#Software Architecture Document for ROS, Gazebo integration with Tango Controls

> __Author:__ Kunal Tyagi

***
##Change Record

***
##Introduction

###Purpose
This document is intended to detail for developers and Users of the ERAS Tango Controls and other ERAS C3 components what changes the control system would undergo during integration with ROS and Gazebo, and what new features it has (or will have) as well as known issues and work left. 

###Scope
This requirements specification is intended to cover a software library and associated documentation.

###Applicable Documents

###Reference Documents

###Glossary
####TANGO
TANGO Control, an Object Oriented Distributed Control System using CORBA and Zeromq, used as primary Control System by ERAS
####ROS
Robot Operating System, a set of software libraries and tools very useful in building and controlling devices, especially Robots
####Gazebo
A robust physics engine with high-quality graphics, useful for more realistic simulations to test the behaviour of robot in different settings

###Overview
This document is divided into several parts.

1. For a typical __User__, the sections [Interface Requirements](#interface-requirements), and [Performance Requirements](#performance-requirements) are of primary interest
2. For a __Beginner__, the section [Logical View](#logical-view) covers most of the information required to get started on their contributions.
3. For a __Developer__, [Architectural Requirements](#architectural-requirements) and [Implementation View](#implementation-view) are of high importance
4. For __Maintainers__, [Deployment View](#deployment-view) and [Development & Test Factors](development-test-factors) are a must-read apart from the aforementioned sections

PS: Start from 1 and make your way down towards any higher number

***
##Architectural Requirements
Working knowledge of [ROS](#ros), [TANGO](#tango), [Gazebo](#gazebo), as well as expertise in Build System, and Makefiles is a must

###Non-functional Requirements
* [ROS]() is independantly developed, and has an evolving build system. As a result, several features need to be modified with a new release of ROS, though most of the code is expected to work fine with only regular updates to API required to hanfle the upgrading process
* [Gazebo]() is independantly developed, and has undergone several changes in its API, and expected to go many more. It is mostly developed by the same community as ROS, so changes will be uniformly spread over these 2 softwares. Also, Python API is in developement, so currently, only C++ API would be used here
* Security: ROS uses no authentication methods, so, [TANGO](), developed in collboration with ESRF, would have to accomodate for this

###Functional requirements _(use case view)_
@TODO

***
##Interface Requirements

###User Interfaces
The user can(or rather would be able to) use ROS libraries with TANGO just as without TANGO, except with a few changes.

Similar usage with Gazebo is expected. It would likely be able to be used just like EUROPA, a standalone plugin to existing software stack

###GUI
No seperate GUI is provided except from the existing ones by ROS and Gazebo. Qt is heavily used by them

###CLI
No seperate CLI is to be created(as of now), though the CLI commands for ROS and Gazebo would be available for use, except for several commands which involve ROS_MASTER

###API
Maybe non-existant @TODO

###Hardware Interfaces
None

###Software Interfaces
None

###Communication Interfaces
None, apart from the exising interfaces for running TANGO

***
##Perfirmance Requirements

***
##Logical View

###Layers

###Sub-systems

###Use Case Realizations

***
##Implementation View

***
##Deployment View

***
##Development & Test Factors

###Hardware Limitations

###Software Validation & Verification

###Planning

***
##Notes

***
##Appendix A: Use Case template

###Use Case: <Name>

####Actors

####Priority

####Preconditions

####Basic Course

####Alternate Course

####Exception Course

####Postconditions

####Notes
